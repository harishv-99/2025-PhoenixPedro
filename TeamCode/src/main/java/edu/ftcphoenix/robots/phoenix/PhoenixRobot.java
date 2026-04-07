package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

import edu.ftcphoenix.fw.core.control.HysteresisBoolean;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.drive.DriveOverlayStack;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidance;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcphoenix.fw.ftc.drive.FtcMecanumDriveLane;
import edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcphoenix.fw.ftc.vision.FtcAprilTagVisionLane;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.localization.PoseEstimate;

/**
 * Central robot container / composition root for Phoenix.
 *
 * <p>
 * At this stage Phoenix follows the framework's principle-driven split more explicitly:
 * </p>
 * <ul>
 *   <li>{@link FtcMecanumDriveLane} owns stable drive hardware/lifecycle concerns.</li>
 *   <li>{@link FtcAprilTagVisionLane} owns the stable AprilTag camera rig and vision resource lifecycle.</li>
 *   <li>{@link FtcOdometryAprilTagLocalizationLane} owns stable localization strategy and pose production.</li>
 *   <li>{@link PhoenixTeleOpControls} owns all TeleOp input semantics, including drive controls.</li>
 *   <li>{@link ScoringTargeting} owns target selection, aim status, and shot suggestions.</li>
 *   <li>{@link ShooterSupervisor} owns scoring policy.</li>
 *   <li>{@link Shooter} remains the single writer to the scoring-path plants.</li>
 *   <li>{@link PhoenixTelemetryPresenter} owns driver-facing telemetry formatting.</li>
 * </ul>
 */
public final class PhoenixRobot {

    private final LoopClock clock = new LoopClock();
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepads gamepads;
    private final PhoenixProfile profile;
    private final HysteresisBoolean shootBraceLatch;
    private final PhoenixTelemetryPresenter telemetryPresenter;

    private FtcMecanumDriveLane drive;
    private FtcAprilTagVisionLane vision;
    private FtcOdometryAprilTagLocalizationLane localization;
    private Shooter shooter;
    private ShooterSupervisor shooterSupervisor;
    private ScoringTargeting scoringTargeting;
    private PhoenixTeleOpControls teleOpControls;
    private DriveSource driveWithAim;

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
     * <p>
     * Phoenix currently performs all meaningful work in mode-specific initialization, so this is an
     * intentional no-op hook that preserves the surrounding lifecycle shape.
     * </p>
     */
    public void initAny() {
    }

    /**
     * Initializes the full Phoenix TeleOp runtime.
     *
     * <p>
     * This method wires the framework lanes, robot-specific controls, scoring services, drive
     * overlays, and driver-facing telemetry help text. Call it once while the OpMode is in INIT
     * before any TeleOp updates begin.
     * </p>
     */
    public void initTeleOp() {
        drive = new FtcMecanumDriveLane(hardwareMap, profile.drive);
        vision = new FtcAprilTagVisionLane(hardwareMap, profile.vision);
        localization = new FtcOdometryAprilTagLocalizationLane(
                hardwareMap,
                vision,
                profile.field.fixedAprilTagLayout,
                profile.localization
        );

        shooter = new Shooter(hardwareMap, profile.shooter);
        teleOpControls = new PhoenixTeleOpControls(gamepads, profile.controls);

        scoringTargeting = new ScoringTargeting(
                profile.autoAim,
                profile.localization.aprilTags.fieldPoseSolver.copy(),
                vision.tagSensor(),
                vision.cameraMountConfig(),
                localization.globalEstimator(),
                profile.field.fixedAprilTagLayout,
                teleOpControls.autoAimEnabledSource(),
                teleOpControls.aimOverrideSource(),
                profile.autoAim.shotVelocityModel()
        );

        shooterSupervisor = new ShooterSupervisor(
                shooter,
                profile.shooter,
                scoringTargeting.aimOkToShootSource(),
                scoringTargeting.aimOverrideSource()
        );

        teleOpControls.bindScoringControls(
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

        driveWithAim = DriveOverlayStack.on(teleOpControls.manualDriveSource())
                .add(
                        "shootBrace",
                        edu.ftcphoenix.fw.core.source.BooleanSource.of(shootBraceLatch::get),
                        DriveGuidance.poseLock(
                                localization.globalEstimator(),
                                DriveGuidancePlan.Tuning.defaults()
                                        .withTranslateKp(profile.autoAim.shootBraceTranslateKp)
                                        .withMaxTranslateCmd(profile.autoAim.shootBraceMaxTranslateCmd)
                        ),
                        DriveOverlayMask.TRANSLATION_ONLY
                )
                .add(
                        "autoAim",
                        teleOpControls.autoAimEnabledSource(),
                        scoringTargeting.aimOverlay(),
                        DriveOverlayMask.OMEGA_ONLY
                )
                .build();

        teleOpControls.emitInitHelp(telemetry);
        telemetry.update();
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
     * <p>
     * Phoenix currently does not need an additional TeleOp start action beyond the shared clock
     * reset, so this remains an explicit no-op lifecycle hook.
     * </p>
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
     * <p>
     * Loop order is intentionally explicit: localization lane, targeting, controls, supervisor,
     * drive lane, shooter subsystem, then telemetry presentation.
     * </p>
     */
    public void updateTeleOp() {
        if (drive == null
                || localization == null
                || shooter == null
                || shooterSupervisor == null
                || scoringTargeting == null
                || teleOpControls == null
                || driveWithAim == null) {
            return;
        }

        localization.update(clock);
        scoringTargeting.update(clock);
        teleOpControls.update(clock);

        shooterSupervisor.update(clock);
        ScoringStatus scoringStatus = shooterSupervisor.status();

        updateShootBraceEnabled(scoringStatus);
        DriveSignal cmd = driveWithAim.get(clock).clamped();
        drive.update(clock);
        drive.drive(cmd);

        shooter.update(clock);
        ShooterStatus shooterStatus = shooter.status(clock);
        TargetingStatus targetingStatus = scoringTargeting.status(clock);
        PoseEstimate globalPose = localization.globalPose();
        PoseEstimate odomPose = localization.odometryPose();

        telemetryPresenter.emitTeleOp(
                shooterStatus,
                scoringStatus,
                targetingStatus,
                globalPose,
                odomPose
        );
    }

    private void updateShootBraceEnabled(ScoringStatus scoringStatus) {
        if (scoringStatus == null || !scoringStatus.shootActive || teleOpControls == null) {
            shootBraceLatch.reset(false);
            return;
        }

        double mag = teleOpControls.manualTranslateMagnitude(clock);
        shootBraceLatch.update(mag);
    }

    /**
     * Stops mode-agnostic hardware owners.
     */
    public void stopAny() {
        if (drive != null) {
            drive.stop();
        }
        if (shooter != null) {
            shooter.stop();
        }
    }

    /**
     * Stops TeleOp-specific resources and releases vision/localization helpers.
     */
    public void stopTeleOp() {
        localization = null;
        shooterSupervisor = null;
        if (vision != null) {
            vision.close();
            vision = null;
        }
        if (teleOpControls != null) {
            teleOpControls.clear();
            teleOpControls = null;
        }
        if (scoringTargeting != null) {
            scoringTargeting.reset();
            scoringTargeting = null;
        }
        driveWithAim = null;
    }
}
