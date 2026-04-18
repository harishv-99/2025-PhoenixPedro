package edu.ftcphoenix.robots.phoenix;

import java.util.Objects;

import edu.ftcphoenix.fw.core.control.HysteresisBoolean;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveOverlay;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.drive.DriveOverlayStack;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidance;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;

/**
 * Robot-specific drive-assist service for Phoenix TeleOp.
 *
 * <p>
 * Phoenix keeps stable drive hardware ownership in {@code FtcMecanumDriveLane} and keeps operator
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
 *   <li>omega-only auto aim driven by {@link ScoringTargeting}</li>
 * </ul>
 *
 * <p>
 * Keeping this logic out of {@link PhoenixRobot} preserves the composition root's role. The robot
 * container wires objects together and chooses loop order; this service owns the scoring-related
 * drive-assist behavior itself.
 * </p>
 */
public final class PhoenixDriveAssistService {

    private final ScalarSource manualTranslateMagnitude;
    private final BooleanSource autoAimEnabled;
    private final HysteresisBoolean shootBraceLatch;
    private final DriveSource driveSource;

    private long lastStatusCycle = Long.MIN_VALUE;
    private DriveAssistStatus lastStatus = new DriveAssistStatus(false, false, false, 0.0);

    /**
     * Creates the Phoenix drive-assist service.
     *
     * @param config                   robot-specific drive-assist tuning snapshot copied for local ownership
     * @param manualDrive              base manual drive source from the controls owner
     * @param manualTranslateMagnitude source describing the driver's current translation-stick magnitude
     * @param autoAimEnabled           source that requests omega-only auto aim when held
     * @param globalAbsolutePoseEstimator      shared global pose estimator used by the shoot-brace pose lock
     * @param autoAimOverlay           scoring-targeting overlay that controls robot omega while auto aim is active
     */
    public PhoenixDriveAssistService(PhoenixProfile.DriveAssistConfig config,
                                     DriveSource manualDrive,
                                     ScalarSource manualTranslateMagnitude,
                                     BooleanSource autoAimEnabled,
                                     AbsolutePoseEstimator globalAbsolutePoseEstimator,
                                     DriveOverlay autoAimOverlay) {
        PhoenixProfile.DriveAssistConfig cfg = Objects.requireNonNull(config, "config").copy();
        Objects.requireNonNull(manualDrive, "manualDrive");
        this.manualTranslateMagnitude = Objects.requireNonNull(manualTranslateMagnitude, "manualTranslateMagnitude");
        this.autoAimEnabled = Objects.requireNonNull(autoAimEnabled, "autoAimEnabled");
        Objects.requireNonNull(globalAbsolutePoseEstimator, "globalAbsolutePoseEstimator");
        Objects.requireNonNull(autoAimOverlay, "autoAimOverlay");

        PhoenixProfile.DriveAssistConfig.ShootBraceConfig shootBrace = cfg.shootBrace;
        this.shootBraceLatch = HysteresisBoolean.onWhenBelowOffWhenAbove(
                shootBrace.enterTranslateMagnitude,
                shootBrace.exitTranslateMagnitude
        );

        this.driveSource = DriveOverlayStack.on(manualDrive)
                .add(
                        "shootBrace",
                        BooleanSource.of(shootBraceLatch::get),
                        DriveGuidance.poseLock(
                                globalAbsolutePoseEstimator,
                                DriveGuidancePlan.Tuning.defaults()
                                        .withTranslateKp(shootBrace.translateKp)
                                        .withMaxTranslateCmd(shootBrace.maxTranslateCmd)
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
    }

    /**
     * Returns the final TeleOp drive source produced by this service.
     *
     * <p>
     * The returned source layers Phoenix's current drive assists on top of the manual driver
     * controls. Build the drive lane once and sample this source each loop.
     * </p>
     *
     * @return drive source that applies shoot-brace and auto-aim overlays on top of manual drive
     */
    public DriveSource driveSource() {
        return driveSource;
    }

    /**
     * Advances the service for the current loop.
     *
     * <p>
     * Call this after controls and scoring policy have updated for the loop so the service can
     * compute a consistent assist snapshot from current input semantics and scoring state.
     * </p>
     *
     * @param clock         shared loop clock for the active OpMode cycle
     * @param scoringStatus current scoring-policy snapshot that describes whether Phoenix is actively shooting
     */
    public void update(LoopClock clock, ScoringStatus scoringStatus) {
        Objects.requireNonNull(clock, "clock");
        long cycle = clock.cycle();
        if (cycle == lastStatusCycle) {
            return;
        }
        lastStatusCycle = cycle;

        boolean autoAimRequested = autoAimEnabled.getAsBoolean(clock);
        double manualTranslateMag = manualTranslateMagnitude.getAsDouble(clock);
        boolean shootBraceEligible = scoringStatus != null && scoringStatus.shootActive;
        boolean shootBraceEnabled;

        if (!shootBraceEligible) {
            shootBraceLatch.reset(false);
            shootBraceEnabled = false;
        } else {
            shootBraceEnabled = shootBraceLatch.update(manualTranslateMag);
        }

        lastStatus = new DriveAssistStatus(
                autoAimRequested,
                shootBraceEligible,
                shootBraceEnabled,
                manualTranslateMag
        );
    }

    /**
     * Returns the most recently computed drive-assist snapshot.
     *
     * <p>
     * The returned value is updated by {@link #update(LoopClock, ScoringStatus)}. Callers should
     * update the service once per loop and then treat this status object as the single shared
     * snapshot for telemetry and other read-only consumers.
     * </p>
     *
     * @return latest computed drive-assist status snapshot
     */
    public DriveAssistStatus status() {
        return lastStatus;
    }

    /**
     * Clears internal assist state so the next activation starts fresh.
     */
    public void reset() {
        shootBraceLatch.reset(false);
        lastStatusCycle = Long.MIN_VALUE;
        lastStatus = new DriveAssistStatus(false, false, false, 0.0);
    }
}
