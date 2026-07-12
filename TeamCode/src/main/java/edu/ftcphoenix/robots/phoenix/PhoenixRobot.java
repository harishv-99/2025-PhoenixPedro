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
import edu.ftcphoenix.fw.ftc.drive.FtcMecanumDriveLane;
import edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskRunner;

/**
 * Central robot container / composition root for Phoenix.
 *
 * <p>
 * Phoenix follows the framework's principle-driven split explicitly:
 * </p>
 * <ul>
 *   <li>{@link FtcMecanumDriveLane} owns stable drive hardware/lifecycle concerns.</li>
 *   <li>{@link PhoenixVisionFactory} selects a concrete {@link AprilTagVisionLane} backend from the active profile.</li>
 *   <li>{@link FtcOdometryAprilTagLocalizationLane} owns stable localization strategy and pose production.</li>
 *   <li>{@link PhoenixCapabilities} exposes Phoenix's shared mode-neutral capability families.</li>
 *   <li>{@link PhoenixTeleOpControls} owns all TeleOp input semantics, including drive controls.</li>
 *   <li>{@link ScoringTargeting} owns target selection, aim status, and shot suggestions.</li>
 *   <li>{@link PhoenixDriveAssistService} owns robot-specific drive-assist policy layered on top of manual drive.</li>
 *   <li>The Auto {@link DriveCommandSink} stays vendor-neutral while Phoenix owns its loop and shutdown lifecycle.</li>
 *   <li>{@link ScoringPath} owns scoring policy, final target-source composition, and scoring-path Plant update order.</li>
 *   <li>{@link PhoenixTelemetryPresenter} owns driver-facing telemetry formatting.</li>
 * </ul>
 */
public final class PhoenixRobot {

    private final LoopClock clock = new LoopClock();
    private final PhoenixShutdown shutdown = new PhoenixShutdown();
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepads gamepads;
    private final PhoenixProfile profile;
    private final PhoenixTelemetryPresenter telemetryPresenter;

    private PhoenixCapabilities capabilities;
    private FtcMecanumDriveLane drive;
    private AprilTagVisionLane vision;
    private FtcOdometryAprilTagLocalizationLane localization;
    private ScoringPath scoringPath;
    private ScoringTargeting scoringTargeting;
    private PhoenixTeleOpControls teleOpControls;
    private PhoenixDriveAssistService driveAssists;
    private DriveSource teleOpDriveSource;
    private TaskRunner autoRunner;
    private DriveCommandSink autonomousDrive;

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
     * @param telemetry   FTC telemetry sink for driver-facing status output
     * @param gamepad1    first driver gamepad reference
     * @param gamepad2    second driver gamepad reference
     * @param profile     Phoenix profile to copy and own for the lifetime of this robot container
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
     * This method wires the framework lanes, shared Phoenix capability families, robot-specific
     * controls, scoring services, drive overlays, and driver-facing telemetry help text. Call it
     * once while the OpMode is in INIT before any TeleOp updates begin.
     * If construction fails, every Phoenix owner that was successfully created is stopped before
     * the original failure is rethrown.
     * </p>
     */
    public void initTeleOp() {
        shutdown.beginInitialization("initTeleOp");
        try {
            initTeleOpRuntime();
        } catch (RuntimeException initializationFailure) {
            stopAfterInitializationFailure(initializationFailure);
            throw initializationFailure;
        }
    }

    /** Build the TeleOp-owned and shared runtime after lifecycle validation. */
    private void initTeleOpRuntime() {
        drive = new FtcMecanumDriveLane(hardwareMap, profile.drive);
        teleOpControls = new PhoenixTeleOpControls(gamepads, profile.controls);
        initSharedRuntime(
                teleOpControls.autoAimEnabledSource(),
                teleOpControls.aimOverrideSource()
        );
        capabilities = createCapabilities();

        teleOpControls.bind(capabilities);

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
     * Initializes the Phoenix autonomous runtime using one owned external drive lifecycle,
     * always-on auto-aim, and no manual override.
     *
     * <p>Phoenix depends only on {@link DriveCommandSink}; a Pedro, Road Runner, or custom adapter
     * remains responsible for its vendor-specific behavior. Phoenix owns the supplied sink's
     * recurring {@link DriveCommandSink#update(LoopClock)} call and final
     * {@link DriveCommandSink#stop()} for this robot lifetime. Call one mode initialization exactly
     * once per {@code PhoenixRobot}; create a new robot container for another mode/runtime.</p>
     *
     * @param autonomousDrive external Auto drive owner; must not be null
     */
    public void initAuto(DriveCommandSink autonomousDrive) {
        initAuto(
                autonomousDrive,
                BooleanSource.constant(true),
                BooleanSource.constant(false)
        );
    }

    /**
     * Initializes the Phoenix autonomous runtime with an owned external drive lifecycle and
     * explicit auto-aim enable / override sources.
     *
     * <p>
     * This creates the same shared runtime and {@link PhoenixCapabilities} surface used in TeleOp,
     * owns the supplied drive sink's per-loop update and shutdown, and leaves route geometry plus
     * autonomous task composition outside the robot container.
     * If construction fails, every Phoenix owner that was successfully created is stopped before
     * the original failure is rethrown. Repeated or cross-mode initialization is rejected before
     * the active ownership graph can be overwritten.
     * </p>
     *
     * @param autonomousDrive     external Auto drive owner; must not be null
     * @param autoAimEnabledSource source that controls whether target selection + aim gating are active
     * @param aimOverrideSource    source that bypasses aim-readiness gating when true
     */
    public void initAuto(DriveCommandSink autonomousDrive,
                         BooleanSource autoAimEnabledSource,
                         BooleanSource aimOverrideSource) {
        shutdown.beginInitialization("initAuto");
        try {
            this.autonomousDrive = Objects.requireNonNull(
                    autonomousDrive,
                    "autonomousDrive"
            );
            initSharedRuntime(autoAimEnabledSource, aimOverrideSource);
            capabilities = createCapabilities();
            autoRunner = new TaskRunner();
            telemetry.addLine("Phoenix auto ready");
            telemetry.update();
        } catch (RuntimeException initializationFailure) {
            stopAfterInitializationFailure(initializationFailure);
            throw initializationFailure;
        }
    }

    private void initSharedRuntime(BooleanSource autoAimEnabledSource,
                                   BooleanSource aimOverrideSource) {
        vision = PhoenixVisionFactory.create(hardwareMap, profile.vision);
        localization = new FtcOdometryAprilTagLocalizationLane(
                hardwareMap,
                vision,
                profile.field.fixedAprilTagLayout,
                profile.localization
        );

        scoringTargeting = new ScoringTargeting(
                profile.autoAim,
                profile.localization.aprilTags.fieldPoseSolver.copy(),
                vision.tagSensor(),
                vision.cameraMountConfig(),
                localization.globalEstimator(),
                profile.field.fixedAprilTagLayout,
                Objects.requireNonNull(autoAimEnabledSource, "autoAimEnabledSource"),
                Objects.requireNonNull(aimOverrideSource, "aimOverrideSource"),
                profile.autoAim.shotVelocityTable
        );

        scoringPath = new ScoringPath(
                hardwareMap,
                profile.scoring,
                scoringTargeting,
                clock
        );
    }

    private PhoenixCapabilities createCapabilities() {
        return new PhoenixCapabilities(
                requireScoringPath(),
                requireScoringTargeting()
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
     * Loop order is intentionally explicit: localization lane, targeting, controls, scoring path, drive-assist service, drive lane, then telemetry presentation.
     * </p>
     */
    public void updateTeleOp() {
        if (drive == null
                || localization == null
                || scoringPath == null
                || scoringTargeting == null
                || teleOpControls == null
                || driveAssists == null
                || teleOpDriveSource == null) {
            return;
        }

        localization.update(clock);
        scoringTargeting.update(clock);
        teleOpControls.update(clock);

        scoringPath.update(clock);
        ScoringPath.Status scoringStatus = scoringPath.status();

        driveAssists.update(clock, scoringStatus);
        DriveSignal cmd = teleOpDriveSource.get(clock).clamped();
        drive.update(clock);
        drive.drive(cmd);

        ScoringTargeting.Status targetingStatus = scoringTargeting.status(clock);
        PhoenixDriveAssistService.Status driveAssistStatus = driveAssists.status();
        PoseEstimate globalPose = localization.globalEstimator().getEstimate();
        PoseEstimate odomPose = localization.predictor().getEstimate();

        telemetryPresenter.emitTeleOp(
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
     * targeting, the continuously owned external drive heartbeat, queued autonomous tasks, the
     * scoring path, and finally telemetry.</p>
     */
    public void updateAuto() {
        if (localization == null
                || scoringPath == null
                || scoringTargeting == null
                || autoRunner == null
                || autonomousDrive == null) {
            return;
        }

        localization.update(clock);
        scoringTargeting.update(clock);
        autonomousDrive.update(clock);
        autoRunner.update(clock);

        scoringPath.update(clock);
        ScoringPath.Status scoringStatus = scoringPath.status();

        ScoringTargeting.Status targetingStatus = scoringTargeting.status(clock);
        PoseEstimate globalPose = localization.globalEstimator().getEstimate();
        PoseEstimate odomPose = localization.predictor().getEstimate();

        Task currentAutoTask = autoRunner.currentTaskOrNull();
        telemetryPresenter.emitAuto(
                scoringStatus,
                targetingStatus,
                currentAutoTask,
                autoRunner.queuedCount(),
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
     * Returns Phoenix's shared mode-neutral capability families.
     *
     * @return initialized capability-family façade for the active mode
     */
    public PhoenixCapabilities capabilities() {
        return requireCapabilities();
    }

    /**
     * Stops the complete Phoenix runtime exactly once.
     *
     * <p>Behavior owners are canceled first, physical scoring and drive outputs are stopped next,
     * and supporting targeting/vision resources are released afterward. All ownership references
     * are detached before cleanup begins, making repeated or reentrant calls harmless.</p>
     *
     * <p>If one owner throws, the remaining cleanup actions still run. The first runtime failure is
     * rethrown after cleanup, with later failures attached as suppressed exceptions.</p>
     */
    public void stop() {
        TaskRunner runnerToStop = autoRunner;
        PhoenixTeleOpControls controlsToStop = teleOpControls;
        PhoenixDriveAssistService assistsToStop = driveAssists;
        ScoringPath scoringToStop = scoringPath;
        DriveCommandSink autonomousDriveToStop = autonomousDrive;
        FtcMecanumDriveLane driveToStop = drive;
        ScoringTargeting targetingToStop = scoringTargeting;
        AprilTagVisionLane visionToStop = vision;

        autoRunner = null;
        teleOpControls = null;
        driveAssists = null;
        teleOpDriveSource = null;
        autonomousDrive = null;
        scoringPath = null;
        drive = null;
        scoringTargeting = null;
        vision = null;
        capabilities = null;
        localization = null;

        Runnable cancelAuto = runnerToStop == null ? null : runnerToStop::cancelAndClear;
        Runnable clearControls = controlsToStop == null ? null : controlsToStop::clear;
        Runnable resetAssists = assistsToStop == null ? null : assistsToStop::reset;
        Runnable stopScoring = scoringToStop == null ? null : scoringToStop::stop;
        Runnable stopAutonomousDrive = autonomousDriveToStop == null
                ? null
                : autonomousDriveToStop::stop;
        Runnable stopDrive = driveToStop == null ? null : driveToStop::stop;
        Runnable resetTargeting = targetingToStop == null ? null : targetingToStop::reset;
        Runnable closeVision = visionToStop == null ? null : visionToStop::close;

        shutdown.run(
                cancelAuto,
                clearControls,
                resetAssists,
                stopScoring,
                stopAutonomousDrive,
                stopDrive,
                resetTargeting,
                closeVision
        );
    }

    /** Preserve an initialization failure while attaching any best-effort cleanup failure. */
    private void stopAfterInitializationFailure(RuntimeException initializationFailure) {
        try {
            stop();
        } catch (RuntimeException cleanupFailure) {
            if (cleanupFailure != initializationFailure) {
                initializationFailure.addSuppressed(cleanupFailure);
            }
        }
    }

    private PhoenixCapabilities requireCapabilities() {
        if (capabilities == null) {
            throw new IllegalStateException("Phoenix capabilities are not initialized");
        }
        return capabilities;
    }

    private ScoringPath requireScoringPath() {
        if (scoringPath == null) {
            throw new IllegalStateException("Phoenix scoring path is not initialized");
        }
        return scoringPath;
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
