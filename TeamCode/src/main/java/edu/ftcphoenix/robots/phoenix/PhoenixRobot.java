package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.field.TagLayout;
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
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;

/**
 * Central robot container / composition root for Phoenix.
 *
 * <p>After the second-stage refactor this class focuses on wiring and loop order. TeleOp button
 * semantics live in {@link PhoenixTeleOpBindings}. Driver-facing telemetry formatting lives in
 * {@link PhoenixTelemetryPresenter}. Mechanism policy stays in {@link ShooterSupervisor}, plant
 * ownership stays in {@link Shooter}, and selected-tag/auto-aim policy lives in
 * {@link ScoringTargeting}.</p>
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
    private ScoringTargeting scoringTargeting;
    private PhoenixTeleOpBindings teleOpBindings;

    private MecanumDrivebase drivebase;
    private PinpointPoseEstimator pinpoint;
    private TagOnlyPoseEstimator tagLocalizer;
    private VisionCorrectionPoseEstimator globalLocalizer;

    private DriveSource stickDrive;
    private DriveSource driveWithAim;

    private AprilTagSensor tagSensor;

    /**
     * Creates a Phoenix robot container using the shared checked-in Phoenix profile.
     *
     * @param hardwareMap FTC hardware map used to create robot hardware owners
     * @param telemetry FTC telemetry sink for driver-facing status output
     * @param gamepad1 first driver gamepad reference
     * @param gamepad2 second driver gamepad reference
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

        CameraMountConfig cameraMountConfig = profile.vision.cameraMount;
        FtcVision.Config visionCfg = FtcVision.Config.defaults().withCameraMount(cameraMountConfig);
        tagSensor = FtcVision.aprilTags(hardwareMap, profile.vision.nameWebcam, visionCfg);

        TagLayout gameTagLayout = FtcGameTagLayout.currentGameFieldFixed();
        TagOnlyPoseEstimator.Config tagLocalizerCfg = profile.localization.aprilTags.copy()
                .withCameraMount(cameraMountConfig);
        tagLocalizer = new TagOnlyPoseEstimator(tagSensor, gameTagLayout, tagLocalizerCfg);
        globalLocalizer = createGlobalLocalizer(pinpoint, tagLocalizer);

        final BooleanSource autoAimEnabled = gamepads.p2().leftBumper().memoized();
        final BooleanSource aimOverride = gamepads.p2().y().memoized();

        scoringTargeting = new ScoringTargeting(
                profile.autoAim,
                profile.localization.aprilTags.copy(),
                tagSensor,
                cameraMountConfig,
                globalLocalizer,
                gameTagLayout,
                autoAimEnabled,
                aimOverride,
                profile.autoAim.shotVelocityModel()
        );

        shooterSupervisor = new ShooterSupervisor(
                shooter,
                profile.shooter,
                scoringTargeting.aimOkToShootSource(),
                scoringTargeting.aimOverrideSource()
        );

        teleOpBindings = new PhoenixTeleOpBindings(
                gamepads,
                shooter,
                shooterSupervisor,
                new Runnable() {
                    /**
                     * Captures a fresh range-based shot velocity suggestion from the targeting service.
                     */
                    @Override
                    public void run() {
                        shooter.setSelectedVelocity(
                                scoringTargeting.suggestedVelocityNative(clock, shooter.selectedVelocity())
                        );
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
                        scoringTargeting.aimOverlay(),
                        DriveOverlayMask.OMEGA_ONLY
                )
                .build();

        telemetry.addLine("Phoenix TeleOp (profile + targeting + bindings + presenter)");
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

    /**
     * Creates Phoenix's configured global localizer from odometry and AprilTag vision.
     *
     * @param odom   configured Pinpoint odometry estimator
     * @param vision configured AprilTag-only field-pose estimator
     * @return Phoenix's selected global pose estimator implementation
     */
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
     * <p>Loop order is intentionally explicit: localization, targeting, bindings, supervisor,
     * drive, shooter, then telemetry presentation.</p>
     */
    public void updateTeleOp() {
        if (drivebase == null
                || shooter == null
                || shooterSupervisor == null
                || driveWithAim == null
                || scoringTargeting == null) {
            return;
        }

        if (globalLocalizer != null) {
            globalLocalizer.update(clock);
        } else if (pinpoint != null) {
            pinpoint.update(clock);
        }

        scoringTargeting.update(clock);

        if (teleOpBindings != null) {
            teleOpBindings.update(clock);
        }

        shooterSupervisor.update(clock);
        ScoringStatus scoringStatus = shooterSupervisor.status();

        updateShootBraceEnabled(scoringStatus);
        DriveSignal cmd = driveWithAim.get(clock).clamped();
        drivebase.update(clock);
        drivebase.drive(cmd);

        shooter.update(clock);
        ShooterStatus shooterStatus = shooter.status(clock);

        TargetingStatus targetingStatus = scoringTargeting.status(clock);
        PoseEstimate globalPose = globalLocalizer != null ? globalLocalizer.getEstimate() : null;
        PoseEstimate odomPose = pinpoint != null ? pinpoint.getEstimate() : null;

        telemetryPresenter.emitTeleOp(
                shooterStatus,
                scoringStatus,
                targetingStatus,
                globalPose,
                odomPose
        );
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
        if (scoringTargeting != null) {
            scoringTargeting.reset();
            scoringTargeting = null;
        }
        tagLocalizer = null;
        globalLocalizer = null;
        driveWithAim = null;
    }
}
