package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcphoenix.fw.ftc.drive.FtcMecanumDriveLane;
import edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcphoenix.fw.ftc.vision.FtcAprilTagVisionLane;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskRunner;
import edu.ftcphoenix.fw.task.Tasks;

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
 *   <li>{@link PhoenixDriveAssistService} owns robot-specific drive-assist policy layered on top of manual drive.</li>
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
    private final PhoenixTelemetryPresenter telemetryPresenter;

    private FtcMecanumDriveLane drive;
    private FtcAprilTagVisionLane vision;
    private FtcOdometryAprilTagLocalizationLane localization;
    private Shooter shooter;
    private ShooterSupervisor shooterSupervisor;
    private ScoringTargeting scoringTargeting;
    private PhoenixTeleOpControls teleOpControls;
    private PhoenixDriveAssistService driveAssists;
    private DriveSource teleOpDriveSource;
    private TaskRunner autoRunner;

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
        teleOpControls = new PhoenixTeleOpControls(gamepads, profile.controls);
        initSharedRuntime(
                teleOpControls.autoAimEnabledSource(),
                teleOpControls.aimOverrideSource()
        );

        teleOpControls.bindScoringControls(
                shooter,
                shooterSupervisor,
                this::captureSuggestedShotVelocity
        );

        driveAssists = new PhoenixDriveAssistService(
                profile.driveAssist,
                teleOpControls.manualDriveSource(),
                teleOpControls.manualTranslateMagnitudeSource(),
                teleOpControls.autoAimEnabledSource(),
                localization.globalEstimator(),
                scoringTargeting.aimOverlay()
        );
        teleOpDriveSource = driveAssists.driveSource();

        teleOpControls.emitInitHelp(telemetry);
        telemetry.update();
    }

    /**
     * Initializes the Phoenix autonomous runtime using always-on auto-aim and no manual override.
     *
     * <p>This mode intentionally omits a drivetrain lane so Phoenix Auto can be paired with an
     * external route package such as Pedro Pathing through the framework's small route and drive
     * seams.</p>
     */
    public void initAuto() {
        initAuto(BooleanSource.constant(true), BooleanSource.constant(false));
    }

    /**
     * Initializes the Phoenix autonomous runtime with explicit auto-aim enable / override sources.
     *
     * @param autoAimEnabledSource source that controls whether target selection + aim gating are active
     * @param aimOverrideSource source that bypasses aim-readiness gating when true
     */
    public void initAuto(BooleanSource autoAimEnabledSource,
                         BooleanSource aimOverrideSource) {
        initSharedRuntime(autoAimEnabledSource, aimOverrideSource);
        autoRunner = new TaskRunner();
        telemetry.addLine("Phoenix auto ready");
        telemetry.update();
    }

    private void initSharedRuntime(BooleanSource autoAimEnabledSource,
                                   BooleanSource aimOverrideSource) {
        vision = new FtcAprilTagVisionLane(hardwareMap, profile.vision);
        localization = new FtcOdometryAprilTagLocalizationLane(
                hardwareMap,
                vision,
                profile.field.fixedAprilTagLayout,
                profile.localization
        );

        shooter = new Shooter(hardwareMap, profile.shooter);
        scoringTargeting = new ScoringTargeting(
                profile.autoAim,
                profile.localization.aprilTags.fieldPoseSolver.copy(),
                vision.tagSensor(),
                vision.cameraMountConfig(),
                localization.globalEstimator(),
                profile.field.fixedAprilTagLayout,
                Objects.requireNonNull(autoAimEnabledSource, "autoAimEnabledSource"),
                Objects.requireNonNull(aimOverrideSource, "aimOverrideSource"),
                profile.autoAim.shotVelocityModel()
        );

        shooterSupervisor = new ShooterSupervisor(
                shooter,
                profile.shooter,
                scoringTargeting.aimOkToShootSource(),
                scoringTargeting.aimOverrideSource()
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
     * <p>
     * Phoenix currently does not need an additional TeleOp start action beyond the shared clock
     * reset, so this remains an explicit no-op lifecycle hook.
     * </p>
     */
    public void startTeleOp() {
    }

    /**
     * Starts autonomous-specific runtime state.
     *
     * <p>Phoenix Auto keeps its actual routine in the {@link #autoRunner()}, so there is no extra
     * start action beyond the shared clock reset.</p>
     */
    public void startAuto() {
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
     * Loop order is intentionally explicit: localization lane, targeting, controls, scoring
     * supervisor, drive-assist service, drive lane, shooter subsystem, then telemetry presentation.
     * </p>
     */
    public void updateTeleOp() {
        if (drive == null
                || localization == null
                || shooter == null
                || shooterSupervisor == null
                || scoringTargeting == null
                || teleOpControls == null
                || driveAssists == null
                || teleOpDriveSource == null) {
            return;
        }

        localization.update(clock);
        scoringTargeting.update(clock);
        teleOpControls.update(clock);

        shooterSupervisor.update(clock);
        ScoringStatus scoringStatus = shooterSupervisor.status();

        driveAssists.update(clock, scoringStatus);
        DriveSignal cmd = teleOpDriveSource.get(clock).clamped();
        drive.update(clock);
        drive.drive(cmd);

        shooter.update(clock);
        ShooterStatus shooterStatus = shooter.status(clock);
        TargetingStatus targetingStatus = scoringTargeting.status(clock);
        DriveAssistStatus driveAssistStatus = driveAssists.status();
        PoseEstimate globalPose = localization.globalPose();
        PoseEstimate odomPose = localization.odometryPose();

        telemetryPresenter.emitTeleOp(
                shooterStatus,
                scoringStatus,
                targetingStatus,
                driveAssistStatus,
                globalPose,
                odomPose
        );
    }

    /**
     * Advances one autonomous loop.
     *
     * <p>Loop order is explicit and matches Phoenix ownership boundaries: localization first, then
     * targeting, then queued autonomous tasks, then scoring policy, then the mechanism subsystem,
     * and finally telemetry.</p>
     */
    public void updateAuto() {
        if (localization == null
                || shooter == null
                || shooterSupervisor == null
                || scoringTargeting == null
                || autoRunner == null) {
            return;
        }

        localization.update(clock);
        scoringTargeting.update(clock);
        autoRunner.update(clock);

        shooterSupervisor.update(clock);
        ScoringStatus scoringStatus = shooterSupervisor.status();

        shooter.update(clock);
        ShooterStatus shooterStatus = shooter.status(clock);
        TargetingStatus targetingStatus = scoringTargeting.status(clock);
        PoseEstimate globalPose = localization.globalPose();
        PoseEstimate odomPose = localization.odometryPose();

        Task currentAutoTask = autoRunner.currentTaskOrNull();
        telemetry.addData("auto.currentTask", currentAutoTask != null ? currentAutoTask.getDebugName() : "<idle>");
        telemetry.addData("auto.currentOutcome", currentAutoTask != null ? currentAutoTask.getOutcome() : "IDLE");
        telemetry.addData("auto.queued", autoRunner.queuedCount());

        telemetryPresenter.emitTeleOp(
                shooterStatus,
                scoringStatus,
                targetingStatus,
                null,
                globalPose,
                odomPose
        );
    }

    /**
     * Enqueues one autonomous task into the shared Phoenix auto runner.
     *
     * @param task task to add to the end of the autonomous sequence
     */
    public void enqueueAuto(Task task) {
        requireAutoRunner().enqueue(Objects.requireNonNull(task, "task"));
    }

    /**
     * Returns the shared autonomous task runner.
     *
     * <p>This is mainly intended for advanced Auto integrations that want to inspect or cancel the
     * active task directly.</p>
     *
     * @return initialized autonomous task runner
     */
    public TaskRunner autoRunner() {
        return requireAutoRunner();
    }

    /**
     * Captures a fresh target-based flywheel velocity suggestion into the shooter subsystem.
     */
    public void captureSuggestedShotVelocity() {
        Shooter liveShooter = requireShooter();
        ScoringTargeting targeting = requireScoringTargeting();
        liveShooter.setSelectedVelocity(
                targeting.suggestedVelocityNative(clock, liveShooter.selectedVelocity())
        );
    }

    /**
     * Requests the scoring supervisor to enable or disable the flywheel.
     *
     * @param enabled {@code true} to spin up, {@code false} to spin down and clear pending shots
     */
    public void setFlywheelEnabled(boolean enabled) {
        requireShooterSupervisor().setFlywheelEnabled(enabled);
    }

    /**
     * Requests one autonomous shot through the scoring supervisor's intent API.
     */
    public void requestSingleShot() {
        requireShooterSupervisor().requestSingleShot();
    }

    /**
     * Returns whether Phoenix still has a shot request in flight.
     *
     * @return {@code true} while a requested shot is still pending or actively feeding
     */
    public boolean hasPendingShots() {
        return requireShooterSupervisor().hasPendingShots();
    }

    /**
     * Builds an autonomous aim task using Phoenix's scoring-target selection logic.
     *
     * @param driveSink sink used to apply the omega correction command
     * @param cfg task-level aim tolerances/timeouts; when {@code null}, defaults are used
     * @return task that turns the robot onto the selected Phoenix scoring target
     */
    public Task aimTask(DriveCommandSink driveSink,
                        DriveGuidanceTask.Config cfg) {
        return requireScoringTargeting().aimTask(driveSink, cfg);
    }

    /**
     * Creates a task that waits until Phoenix has a selected scoring target.
     *
     * @param timeoutSec timeout in seconds before the wait task reports {@code TIMEOUT}
     * @return wait task for target selection
     */
    public Task waitForTargetSelection(double timeoutSec) {
        final ScoringTargeting targeting = requireScoringTargeting();
        return Tasks.waitUntil(new BooleanSource() {
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                return targeting.status(clock).selection.hasSelection;
            }
        }, timeoutSec);
    }

    /**
     * Creates a task that waits until all pending shot requests have completed.
     *
     * @param timeoutSec timeout in seconds before the wait task reports {@code TIMEOUT}
     * @return wait task for shot completion
     */
    public Task waitForShotCompletion(double timeoutSec) {
        final ShooterSupervisor supervisor = requireShooterSupervisor();
        return Tasks.waitUntil(() -> !supervisor.hasPendingShots(), timeoutSec);
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
        if (teleOpControls != null) {
            teleOpControls.clear();
            teleOpControls = null;
        }
        if (driveAssists != null) {
            driveAssists.reset();
            driveAssists = null;
        }
        teleOpDriveSource = null;
        stopSharedRuntime();
    }

    /**
     * Stops autonomous-specific resources and cancels the shared autonomous task queue.
     */
    public void stopAuto() {
        if (autoRunner != null) {
            autoRunner.cancelAndClear();
            autoRunner = null;
        }
        stopSharedRuntime();
    }

    private void stopSharedRuntime() {
        localization = null;
        shooterSupervisor = null;
        if (vision != null) {
            vision.close();
            vision = null;
        }
        if (scoringTargeting != null) {
            scoringTargeting.reset();
            scoringTargeting = null;
        }
    }

    private Shooter requireShooter() {
        if (shooter == null) {
            throw new IllegalStateException("Phoenix shooter runtime is not initialized");
        }
        return shooter;
    }

    private ShooterSupervisor requireShooterSupervisor() {
        if (shooterSupervisor == null) {
            throw new IllegalStateException("Phoenix scoring supervisor is not initialized");
        }
        return shooterSupervisor;
    }

    private ScoringTargeting requireScoringTargeting() {
        if (scoringTargeting == null) {
            throw new IllegalStateException("Phoenix targeting runtime is not initialized");
        }
        return scoringTargeting;
    }

    private TaskRunner requireAutoRunner() {
        if (autoRunner == null) {
            throw new IllegalStateException("Phoenix auto runner is not initialized");
        }
        return autoRunner;
    }
}
