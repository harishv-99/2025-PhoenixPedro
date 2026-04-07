package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

import edu.ftcphoenix.fw.core.control.HysteresisBoolean;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.drive.DriveOverlayStack;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidance;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceQuery;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceSpec;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceStatus;
import edu.ftcphoenix.fw.drive.guidance.References;
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.field.TagLayouts;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.FtcGameTagLayout;
import edu.ftcphoenix.fw.ftc.FtcVision;
import edu.ftcphoenix.fw.ftc.localization.PinpointPoseEstimator;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.apriltag.TagOnlyPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.OdometryTagEkfPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.OdometryTagFusionPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.VisionCorrectionPoseEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionPolicies;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionSource;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelections;

/**
 * Central robot container / composition root for Phoenix.
 *
 * <p>After the first-stage refactor this class focuses on wiring and loop order. TeleOp button
 * semantics live in {@link PhoenixTeleOpBindings}. Driver-facing telemetry formatting lives in
 * {@link PhoenixTelemetryPresenter}. Mechanism policy stays in {@link ShooterSupervisor} and plant
 * ownership stays in {@link Shooter}.</p>
 */
public final class PhoenixRobot {

    private final LoopClock clock = new LoopClock();
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepads gamepads;
    private final PhoenixProfile profile;
    private final HysteresisBoolean shootBraceLatch;
    private final PhoenixTelemetryPresenter telemetryPresenter;

    private Shooter shooter;
    private ShooterSupervisor shooterSupervisor;
    private PhoenixTeleOpBindings teleOpBindings;

    private MecanumDrivebase drivebase;
    private PinpointPoseEstimator pinpoint;
    private TagOnlyPoseEstimator tagLocalizer;
    private VisionCorrectionPoseEstimator globalLocalizer;

    private DriveSource stickDrive;
    private DriveSource driveWithAim;

    private CameraMountConfig cameraMountConfig;
    private AprilTagSensor tagSensor;
    private TagSelectionSource scoringSelection;
    private DriveGuidancePlan aimPlan;
    private DriveGuidanceQuery aimQuery;
    private BooleanSource aimReady = BooleanSource.constant(true);
    private BooleanSource aimOverride = BooleanSource.constant(false);
    private BooleanSource aimOkToShoot = BooleanSource.constant(true);
    private TagLayout gameTagLayout;

    /**
     * Creates a Phoenix robot container using the shared checked-in Phoenix profile.
     *
     * @param hardwareMap FTC hardware map used to create robot hardware owners
     * @param telemetry   FTC telemetry sink for driver-facing status output
     * @param gamepad1    first driver gamepad reference
     * @param gamepad2    second driver gamepad reference
     */
    public PhoenixRobot(HardwareMap hardwareMap,
                        Telemetry telemetry,
                        Gamepad gamepad1,
                        Gamepad gamepad2) {
        this(hardwareMap, telemetry, gamepad1, gamepad2, PhoenixProfile.current());
    }

    /**
     * Creates a Phoenix robot container using an explicit profile snapshot.
     *
     * @param hardwareMap FTC hardware map used to create robot hardware owners
     * @param telemetry FTC telemetry sink for driver-facing status output
     * @param gamepad1 first driver gamepad reference
     * @param gamepad2 second driver gamepad reference
     * @param profile Phoenix profile to copy and own for the lifetime of this robot container
     */
    public PhoenixRobot(HardwareMap hardwareMap,
                        Telemetry telemetry,
                        Gamepad gamepad1,
                        Gamepad gamepad2,
                        PhoenixProfile profile) {
        this.hardwareMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
        this.telemetry = Objects.requireNonNull(telemetry, "telemetry");
        this.gamepads = Gamepads.create(gamepad1, gamepad2);
        this.profile = Objects.requireNonNull(profile, "profile").copy();
        this.shootBraceLatch = HysteresisBoolean.onWhenBelowOffWhenAbove(
                this.profile.autoAim.shootBraceEnterMagnitude,
                this.profile.autoAim.shootBraceExitMagnitude
        );
        this.telemetryPresenter = new PhoenixTelemetryPresenter(telemetry, this.profile);
    }

    /**
     * Performs mode-agnostic initialization.
     *
     * <p>Phoenix currently performs all meaningful work in mode-specific initialization, so this is
     * intentionally a no-op hook that preserves the surrounding lifecycle shape.</p>
     */
    public void initAny() {
    }

    /**
     * Initializes the full Phoenix TeleOp runtime.
     *
     * <p>This method wires subsystems, localizers, drive overlays, driver bindings, and driver
     * telemetry text. Call it once while the OpMode is in INIT before any TeleOp updates begin.</p>
     */
    public void initTeleOp() {
        drivebase = FtcDrives.mecanum(
                hardwareMap,
                profile.driveTrain.mecanumWiring(),
                profile.driveTrain.drivebase.copy()
        );
        FtcDrives.setDriveBrake(hardwareMap, profile.driveTrain.mecanumWiring(), profile.driveTrain.zeroPowerBrake);

        shooter = new Shooter(hardwareMap, profile.shooter);
        stickDrive = GamepadDriveSource.teleOpMecanumSlowRb(gamepads);

        pinpoint = new PinpointPoseEstimator(hardwareMap, profile.localization.pinpoint.copy());

        cameraMountConfig = profile.vision.cameraMount;
        FtcVision.Config visionCfg = FtcVision.Config.defaults().withCameraMount(cameraMountConfig);
        tagSensor = FtcVision.aprilTags(hardwareMap, profile.vision.nameWebcam, visionCfg);

        gameTagLayout = FtcGameTagLayout.currentGameFieldFixed();
        TagLayout scoringTagLayout = TagLayouts.subsetOrSame(
                gameTagLayout,
                profile.autoAim.scoringTagIds(),
                "Phoenix scoring fixed-tag layout"
        );

        TagOnlyPoseEstimator.Config tagLocalizerCfg = profile.localization.aprilTags.copy()
                .withCameraMount(cameraMountConfig);
        tagLocalizer = new TagOnlyPoseEstimator(tagSensor, gameTagLayout, tagLocalizerCfg);
        globalLocalizer = createGlobalLocalizer(pinpoint, tagLocalizer);

        BooleanSource autoAimEnabled = gamepads.p2().leftBumper();

        Map<Integer, References.TagPointOffset> aimOffsetsByTag = new LinkedHashMap<Integer, References.TagPointOffset>();
        for (Map.Entry<Integer, PhoenixProfile.AutoAimConfig.AimOffset> entry : profile.autoAim.aimOffsetsByTag().entrySet()) {
            PhoenixProfile.AutoAimConfig.AimOffset offset = entry.getValue();
            aimOffsetsByTag.put(entry.getKey(), References.pointOffset(offset.forwardInches, offset.leftInches));
        }

        scoringSelection = TagSelections.from(tagSensor)
                .among(profile.autoAim.scoringTagIds())
                .freshWithin(profile.autoAim.selectionMaxAgeSec)
                .choose(TagSelectionPolicies.smallestAbsRobotBearing(cameraMountConfig))
                .stickyWhen(autoAimEnabled)
                .reacquireAfterLoss(profile.autoAim.selectionReacquireSec)
                .build();

        DriveGuidancePlan.Tuning aimTuning = DriveGuidancePlan.Tuning.defaults()
                .withAimKp(profile.autoAim.aimKp)
                .withMaxOmegaCmd(profile.autoAim.aimMaxOmegaCmd)
                .withMinOmegaCmd(profile.autoAim.aimMinOmegaCmd)
                .withAimDeadbandRad(Math.toRadians(profile.autoAim.aimToleranceDeg));

        aimPlan = DriveGuidance.plan()
                .aimTo()
                .point(References.relativeToSelectedTagPoint(scoringSelection, aimOffsetsByTag))
                .doneAimTo()
                .tuning(aimTuning)
                .resolveWith()
                .adaptive()
                .aprilTags(tagSensor, cameraMountConfig, profile.autoAim.selectionMaxAgeSec)
                .aprilTagFieldPoseConfig(profile.localization.aprilTags.toSolverConfig())
                .localization(globalLocalizer)
                .fixedAprilTagLayout(scoringTagLayout)
                .omegaPolicy(DriveGuidanceSpec.OmegaPolicy.PREFER_APRIL_TAGS_WHEN_VALID)
                .onLoss(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
                .doneResolveWith()
                .build();

        aimQuery = aimPlan.query();

        final double aimReadyTolRad = Math.toRadians(profile.autoAim.aimReadyToleranceDeg);
        BooleanSource rawAimReady = new BooleanSource() {
            private long lastCycle = Long.MIN_VALUE;
            private boolean last = true;

            /**
             * Samples aim readiness at most once per loop cycle.
             */
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return last;
                }
                lastCycle = cyc;

                DriveGuidanceStatus status = aimQuery != null
                        ? aimQuery.sample(clock, DriveOverlayMask.OMEGA_ONLY)
                        : null;
                if (status != null && status.hasOmegaError) {
                    last = status.omegaWithin(aimReadyTolRad);
                    return last;
                }

                last = true;
                return last;
            }
        };

        aimReady = rawAimReady.debouncedOn(profile.autoAim.aimReadyDebounceSec);
        aimOverride = gamepads.p2().y();
        aimOkToShoot = aimReady.or(aimOverride);

        shooterSupervisor = new ShooterSupervisor(shooter, profile.shooter, aimOkToShoot, aimOverride);
        teleOpBindings = new PhoenixTeleOpBindings(
                gamepads,
                shooter,
                shooterSupervisor,
                new Runnable() {
                    /**
                     * Captures a fresh range-based shot velocity from the selected target.
                     */
                    @Override
                    public void run() {
                        captureVelocityFromTarget(clock);
                    }
                }
        );

        driveWithAim = DriveOverlayStack.on(stickDrive)
                .add(
                        "shootBrace",
                        BooleanSource.of(shootBraceLatch::get),
                        DriveGuidance.poseLock(
                                globalLocalizer,
                                DriveGuidancePlan.Tuning.defaults()
                                        .withTranslateKp(profile.autoAim.shootBraceTranslateKp)
                                        .withMaxTranslateCmd(profile.autoAim.shootBraceMaxTranslateCmd)
                        ),
                        DriveOverlayMask.TRANSLATION_ONLY
                )
                .add(
                        "autoAim",
                        autoAimEnabled,
                        aimPlan.overlay(),
                        DriveOverlayMask.OMEGA_ONLY
                )
                .build();

        telemetry.addLine("Phoenix TeleOp (profile + bindings + presenter)");
        telemetry.addLine("P1: Left stick=drive, Right stick=turn, RB=slow mode");
        telemetry.addLine("P2: RB=toggle shooter flywheel (spins at selected velocity)");
        telemetry.addLine("P2: LB=auto aim + set velocity from AprilTag range");
        telemetry.addLine("P2: B=shoot (hold; release cancels)");
        telemetry.addLine("P2: Y=override shoot gates (hold; if flywheel ON, forces feed even if not ready)");
        telemetry.addLine("P2: A=toggle intake");
        telemetry.addLine("P2: X=eject / unjam (hold; reverse feeds)");
        telemetry.addLine("P2: DPad Up/Down=adjust selected velocity");
        telemetry.update();
    }

    private VisionCorrectionPoseEstimator createGlobalLocalizer(PinpointPoseEstimator odom,
                                                                TagOnlyPoseEstimator vision) {
        PhoenixProfile.LocalizationConfig.GlobalEstimatorMode mode = profile.localization.globalEstimatorMode;
        if (mode == PhoenixProfile.LocalizationConfig.GlobalEstimatorMode.EKF) {
            return new OdometryTagEkfPoseEstimator(
                    odom,
                    vision,
                    profile.localization.pinpointAprilTagEkf.validatedCopy(
                            "PhoenixProfile.current().localization.pinpointAprilTagEkf"
                    )
            );
        }
        return new OdometryTagFusionPoseEstimator(
                odom,
                vision,
                profile.localization.pinpointAprilTagFusion.validatedCopy(
                        "PhoenixProfile.current().localization.pinpointAprilTagFusion"
                )
        );
    }

    /**
     * Resets shared lifecycle state for any mode.
     *
     * @param runtime current FTC runtime in seconds
     */
    public void startAny(double runtime) {
        clock.reset(runtime);
    }

    /**
     * Starts TeleOp-specific runtime state.
     *
     * <p>Phoenix currently does not need an additional TeleOp start action beyond the shared clock
     * reset, so this remains an explicit no-op lifecycle hook.</p>
     */
    public void startTeleOp() {
    }

    /**
     * Updates shared lifecycle state for any mode.
     *
     * @param runtime current FTC runtime in seconds
     */
    public void updateAny(double runtime) {
        clock.update(runtime);
    }

    /**
     * Advances one TeleOp loop.
     *
     * <p>Loop order is intentionally explicit: bindings, localization, supervisor, drive, shooter,
     * then telemetry presentation.</p>
     */
    public void updateTeleOp() {
        if (drivebase == null || shooter == null || shooterSupervisor == null || driveWithAim == null) {
            return;
        }

        if (teleOpBindings != null) {
            teleOpBindings.update(clock);
        }

        if (globalLocalizer != null) {
            globalLocalizer.update(clock);
        } else if (pinpoint != null) {
            pinpoint.update(clock);
        }

        shooterSupervisor.update(clock);
        ScoringStatus scoringStatus = shooterSupervisor.status();

        updateShootBraceEnabled(scoringStatus);
        DriveSignal cmd = driveWithAim.get(clock).clamped();
        drivebase.update(clock);
        drivebase.drive(cmd);

        shooter.update(clock);
        ShooterStatus shooterStatus = shooter.status(clock);

        boolean aimReadyNow = aimReady.getAsBoolean(clock);
        boolean aimOkToShootNow = aimOkToShoot.getAsBoolean(clock);
        boolean aimOverrideNow = aimOverride.getAsBoolean(clock);
        TagSelectionResult selection = scoringSelection != null ? scoringSelection.get(clock) : null;
        DriveGuidanceStatus aimStatus = sampleCurrentAimStatus();
        PoseEstimate globalPose = globalLocalizer != null ? globalLocalizer.getEstimate() : null;
        PoseEstimate odomPose = pinpoint != null ? pinpoint.getEstimate() : null;

        telemetryPresenter.emitTeleOp(
                shooterStatus,
                scoringStatus,
                aimReadyNow,
                aimOkToShootNow,
                aimOverrideNow,
                shootBraceLatch.get(),
                globalPose,
                odomPose,
                selection,
                aimStatus,
                gameTagLayout
        );
    }

    private void captureVelocityFromTarget(LoopClock clock) {
        TagSelectionResult selection = scoringSelection != null ? scoringSelection.get(clock) : null;
        if (selection == null || !selection.hasFreshSelectedObservation) {
            return;
        }

        AprilTagObservation obs = selection.selectedObservation;
        if (!obs.hasTarget) {
            return;
        }

        shooter.setSelectedVelocity(shooter.velocityForRangeInches(obs.cameraRangeInches()));
    }

    private DriveGuidanceStatus sampleCurrentAimStatus() {
        return aimQuery != null
                ? aimQuery.sample(clock, DriveOverlayMask.OMEGA_ONLY)
                : null;
    }

    private void updateShootBraceEnabled(ScoringStatus scoringStatus) {
        if (scoringStatus == null || !scoringStatus.shootActive) {
            shootBraceLatch.reset(false);
            return;
        }

        double mag = gamepads.p1().leftStickMagnitude().getAsDouble(clock);
        shootBraceLatch.update(mag);
    }

    /**
     * Stops mode-agnostic hardware owners.
     */
    public void stopAny() {
        if (drivebase != null) {
            drivebase.stop();
        }
        if (shooter != null) {
            shooter.stop();
        }
    }

    /**
     * Stops TeleOp-specific resources and releases vision/localization helpers.
     */
    public void stopTeleOp() {
        if (tagSensor != null) {
            tagSensor.close();
            tagSensor = null;
        }
        if (teleOpBindings != null) {
            teleOpBindings.clear();
            teleOpBindings = null;
        }
        tagLocalizer = null;
        globalLocalizer = null;
        driveWithAim = null;
        aimPlan = null;
        aimQuery = null;
    }
}
