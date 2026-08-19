package edu.ftcphoenix.robots.phoenix;

import java.util.Objects;

import edu.ftcphoenix.fw.core.control.HysteresisBoolean;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveOverlay;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.drive.DriveOverlayStack;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidance;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.robots.phoenix.scoring.PhoenixTargeting;

/**
 * Robot-specific drive-assist service for Phoenix TeleOp.
 *
 * <p>
 * Phoenix keeps final drive hardware ownership in its {@code MecanumDrivebase} and keeps operator
 * input semantics in {@link PhoenixTeleOpControls}. This service lives between those layers. It owns
 * robot-specific drive-assist policy that combines manual drive, scoring state, and localization to
 * produce the final TeleOp drive source.
 * </p>
 *
 * <p>
 * Today that policy includes two assists:
 * </p>
 * <ul>
 *   <li>shoot-brace translation hold while the robot is actively shooting and the driver lets the
 *       translation stick settle near zero</li>
 *   <li>omega-only auto aim driven by {@link PhoenixTargeting}</li>
 * </ul>
 *
 * <p>
 * Keeping this logic out of {@link PhoenixRobot} preserves the composition root's role. The robot
 * container wires objects together and chooses loop order; this service owns the scoring-related
 * drive-assist behavior itself.
 * </p>
 *
 * <p>When Phoenix's localization calibration is not acknowledged, this service gates both assists
 * internally while preserving the base manual {@link DriveSource}. Availability is also retained
 * in {@link Status} for required Driver Station telemetry.</p>
 */
public final class PhoenixDriveAssistService {

    /** Mutable data-only tuning for Phoenix's shoot-brace drive-assist policy. */
    public static final class Config {

        /** Translation magnitude at or below which shoot-brace may latch on. */
        public double shootBraceEnterTranslateMagnitude = 0.06;

        /** Translation magnitude at or above which shoot-brace must drop out. */
        public double shootBraceExitTranslateMagnitude = 0.10;

        /** Translation proportional gain used by the shoot-brace pose lock. */
        public double shootBraceTranslateKp = 0.08;

        /** Maximum translation command magnitude contributed by shoot-brace. */
        public double shootBraceMaxTranslateCmd = 0.35;

        private Config() {
        }

        /**
         * Returns a fresh complete software-valid Phoenix drive-assist draft.
         *
         * @return fresh mutable drive-assist configuration
         */
        public static Config defaults() {
            return new Config();
        }
    }

    /**
     * Immutable status snapshot for Phoenix's robot-specific drive-assist service.
     */
    public static final class Status {
        /** Whether Phoenix's localization calibration permits pose-dependent assists. */
        public final boolean poseAssistsAvailable;
        /** Whether the driver requested auto-aim, even when readiness currently gates it off. */
        public final boolean autoAimRequested;
        public final boolean shootBraceEligible;
        public final boolean shootBraceEnabled;
        public final double manualTranslateMagnitude;

        /**
         * Creates an immutable drive-assist status snapshot.
         */
        public Status(boolean poseAssistsAvailable,
                      boolean autoAimRequested,
                      boolean shootBraceEligible,
                      boolean shootBraceEnabled,
                      double manualTranslateMagnitude) {
            this.poseAssistsAvailable = poseAssistsAvailable;
            this.autoAimRequested = autoAimRequested;
            this.shootBraceEligible = shootBraceEligible;
            this.shootBraceEnabled = shootBraceEnabled;
            this.manualTranslateMagnitude = manualTranslateMagnitude;
        }
    }

    private final ScalarSource manualTranslateMagnitude;
    private final Source<PhoenixCapabilities.ScoringStatus> scoringStatusSource;
    private final BooleanSource autoAimRequested;
    private final BooleanSource autoAimEnabled;
    private final boolean poseAssistsAvailable;
    private final HysteresisBoolean shootBraceLatch;
    private final Source<Status> statusSource;
    private final DriveSource driveSource;

    private Status lastStatus;

    /**
     * Creates the Phoenix drive-assist service.
     *
     * @param config                      robot-specific drive-assist draft whose retained scalar
     *                                    values are captured for local ownership
     * @param manualDrive                 base manual drive source from the controls owner
     * @param manualTranslateMagnitude    source describing the driver's current translation-stick magnitude
     * @param scoringStatusSource         current scoring snapshot source sampled as part of the final drive read
     * @param autoAimEnabled              source that requests omega-only auto aim when held
     * @param poseAssistsAvailable        whether checked-in localization calibration permits pose-dependent assists
     * @param globalAbsolutePoseEstimator shared global pose estimator used by the shoot-brace pose lock
     * @param autoAimOverlay              scoring-targeting overlay that controls robot omega while auto aim is active
     * @throws NullPointerException if a required object is {@code null}
     * @throws IllegalArgumentException if a Config scalar is non-finite or outside its documented
     *                                  domain, or the enter magnitude exceeds the exit magnitude
     */
    public PhoenixDriveAssistService(Config config,
                                     DriveSource manualDrive,
                                     ScalarSource manualTranslateMagnitude,
                                     Source<PhoenixCapabilities.ScoringStatus> scoringStatusSource,
                                     BooleanSource autoAimEnabled,
                                     boolean poseAssistsAvailable,
                                     AbsolutePoseEstimator globalAbsolutePoseEstimator,
                                     DriveOverlay autoAimOverlay) {
        Config draft = Objects.requireNonNull(config, "PhoenixDriveAssistService.Config");
        double shootBraceEnterTranslateMagnitude = requireFiniteUnitInterval(
                "shootBraceEnterTranslateMagnitude",
                draft.shootBraceEnterTranslateMagnitude
        );
        double shootBraceExitTranslateMagnitude = requireFiniteUnitInterval(
                "shootBraceExitTranslateMagnitude",
                draft.shootBraceExitTranslateMagnitude
        );
        if (shootBraceEnterTranslateMagnitude > shootBraceExitTranslateMagnitude) {
            throw new IllegalArgumentException(
                    "PhoenixDriveAssistService.Config.shootBraceEnterTranslateMagnitude must be "
                            + "<= PhoenixDriveAssistService.Config."
                            + "shootBraceExitTranslateMagnitude, got enter="
                            + shootBraceEnterTranslateMagnitude + " and exit="
                            + shootBraceExitTranslateMagnitude
            );
        }
        double shootBraceTranslateKp = requireFiniteNonnegative(
                "shootBraceTranslateKp",
                draft.shootBraceTranslateKp
        );
        double shootBraceMaxTranslateCmd = requireFiniteUnitInterval(
                "shootBraceMaxTranslateCmd",
                draft.shootBraceMaxTranslateCmd
        );

        Objects.requireNonNull(manualDrive, "manualDrive");
        this.manualTranslateMagnitude = Objects.requireNonNull(manualTranslateMagnitude, "manualTranslateMagnitude");
        this.scoringStatusSource = Objects.requireNonNull(
                scoringStatusSource,
                "scoringStatusSource"
        );
        this.poseAssistsAvailable = poseAssistsAvailable;
        this.autoAimRequested = Objects.requireNonNull(autoAimEnabled, "autoAimEnabled")
                .memoized();
        this.autoAimEnabled = this.autoAimRequested
                .and(BooleanSource.constant(poseAssistsAvailable))
                .memoized();
        Objects.requireNonNull(globalAbsolutePoseEstimator, "globalAbsolutePoseEstimator");
        Objects.requireNonNull(autoAimOverlay, "autoAimOverlay");

        this.shootBraceLatch = HysteresisBoolean.onWhenBelowOffWhenAbove(
                shootBraceEnterTranslateMagnitude,
                shootBraceExitTranslateMagnitude
        );

        this.statusSource = Source.of(this::calculateStatus).memoized();
        BooleanSource shootBraceEnabled = this.statusSource
                .mapToBoolean(status -> status.shootBraceEnabled);

        DriveSource assistedDrive = DriveOverlayStack.on(manualDrive)
                .add(
                        "shootBrace",
                        shootBraceEnabled,
                        DriveGuidance.poseLock(
                                globalAbsolutePoseEstimator,
                                DriveGuidancePlan.Tuning.defaults()
                                        .withTranslateKp(shootBraceTranslateKp)
                                        .withMaxTranslateCmd(shootBraceMaxTranslateCmd)
                        ),
                        DriveOverlayMask.TRANSLATION_ONLY
                )
                .add(
                        "autoAim",
                        this.autoAimEnabled,
                        autoAimOverlay,
                        DriveOverlayMask.OMEGA_ONLY
                )
                .build();
        this.lastStatus = new Status(poseAssistsAvailable, false, false, false, 0.0);

        this.driveSource = new DriveSource() {
            @Override
            public DriveSignal get(LoopClock clock) {
                // Publish the policy snapshot before any overlay samples its derived gates.
                statusSource.get(clock);
                return assistedDrive.get(clock);
            }

            @Override
            public void reset() {
                assistedDrive.reset();
                shootBraceLatch.reset(false);
                lastStatus = new Status(
                        poseAssistsAvailable,
                        false,
                        false,
                        false,
                        0.0
                );
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) {
                    return;
                }
                String p = (prefix == null || prefix.isEmpty()) ? "driveAssist" : prefix;
                Status status = lastStatus;
                dbg.addData(p + ".class", "PhoenixDriveAssistSource")
                        .addData(p + ".poseAssistsAvailable", status.poseAssistsAvailable)
                        .addData(p + ".autoAimRequested", status.autoAimRequested)
                        .addData(p + ".shootBraceEligible", status.shootBraceEligible)
                        .addData(p + ".shootBraceEnabled", status.shootBraceEnabled)
                        .addData(p + ".manualTranslateMagnitude", status.manualTranslateMagnitude);
                assistedDrive.debugDump(dbg, p + ".assisted");
            }
        };
    }

    private static double requireFiniteUnitInterval(String fieldName, double value) {
        if (!Double.isFinite(value) || value < 0.0 || value > 1.0) {
            throw new IllegalArgumentException(
                    "PhoenixDriveAssistService.Config." + fieldName
                            + " must be finite and in [0.0, 1.0], got " + value
            );
        }
        return value;
    }

    private static double requireFiniteNonnegative(String fieldName, double value) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(
                    "PhoenixDriveAssistService.Config." + fieldName
                            + " must be finite and >= 0.0, got " + value
            );
        }
        return value;
    }

    /**
     * Returns the final TeleOp drive source produced by this service.
     *
     * <p>
     * The returned source layers Phoenix's current drive assists on top of the manual driver
     * controls. Build the final drivebase once and sample this source each loop.
     * </p>
     *
     * @return drive source that applies shoot-brace and auto-aim overlays on top of manual drive
     */
    public DriveSource driveSource() {
        return driveSource;
    }

    private Status calculateStatus(LoopClock clock) {
        PhoenixCapabilities.ScoringStatus scoringStatus = Objects.requireNonNull(
                scoringStatusSource.get(clock),
                "scoringStatusSource returned null"
        );
        boolean autoAimRequested = this.autoAimRequested.getAsBoolean(clock);
        double manualTranslateMag = manualTranslateMagnitude.getAsDouble(clock);
        boolean shootBraceEligible = poseAssistsAvailable
                && scoringStatus.shootActive;
        boolean shootBraceEnabled;

        if (!shootBraceEligible) {
            shootBraceLatch.reset(false);
            shootBraceEnabled = false;
        } else {
            shootBraceEnabled = shootBraceLatch.update(manualTranslateMag);
        }

        Status calculated = new Status(
                poseAssistsAvailable,
                autoAimRequested,
                shootBraceEligible,
                shootBraceEnabled,
                manualTranslateMag
        );
        lastStatus = calculated;
        return calculated;
    }

    /**
     * Returns the most recently computed drive-assist snapshot.
     *
     * <p>The final source returned by {@link #driveSource()} publishes this snapshot once per loop
     * as part of the same read that produces the drive command. Downstream telemetry can therefore
     * consume one stable status without another imperative service heartbeat.</p>
     *
     * @return latest computed drive-assist status snapshot
     */
    public Status status() {
        return lastStatus;
    }
}
