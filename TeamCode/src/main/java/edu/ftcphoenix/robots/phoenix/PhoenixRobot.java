package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.ftc.drive.FtcMecanumDriveLane;
import edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.VisionReadiness;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.localization.MotionPredictor;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.PoseResetter;
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
 *   <li>The Auto {@link DriveCommandSink} and {@link MotionPredictor} stay vendor-neutral while
 *       Phoenix owns loop order and drive shutdown lifecycle.</li>
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
    private final TeleOpPoseRestoreLifecycle teleOpPoseRestore =
            new TeleOpPoseRestoreLifecycle();

    private PhoenixCapabilities capabilities;
    private FtcMecanumDriveLane drive;
    private AprilTagVisionLane vision;
    private FtcOdometryAprilTagLocalizationLane localization;
    private ScoringPath scoringPath;
    private ScoringTargeting scoringTargeting;
    private PhoenixTeleOpControls teleOpControls;
    private PhoenixDriveAssistService driveAssists;
    private DriveSource teleOpDriveSource;
    private PhoenixReadiness.Result teleOpPoseAssistReadiness;
    private AutoRoutineLifecycle autoRoutineLifecycle;
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
     * the original failure is rethrown. This stages the ordinary TeleOp help/readiness rows; the
     * mode client adds any mode-boundary status and commits the one complete INIT telemetry frame.
     * </p>
     */
    public void initTeleOp() {
        shutdown.beginInitialization("initTeleOp");
        try {
            initTeleOpRuntime();
        } catch (RuntimeException initializationFailure) {
            throw CleanupActions.attemptAllAfterFailure(
                    initializationFailure,
                    this::stop
            );
        }
    }

    /** Build the TeleOp-owned and shared runtime after lifecycle validation. */
    private void initTeleOpRuntime() {
        drive = new FtcMecanumDriveLane(hardwareMap, profile.drive);
        teleOpControls = new PhoenixTeleOpControls(gamepads, profile.controls);
        teleOpPoseAssistReadiness = PhoenixReadiness.teleOpPoseAssists(profile);
        BooleanSource enabledAutoAim = teleOpControls.autoAimEnabledSource()
                .and(BooleanSource.constant(teleOpPoseAssistReadiness.isAllowed()))
                .memoized();
        initSharedRuntimeWithOwnedPredictor(
                enabledAutoAim,
                teleOpControls.aimOverrideSource()
        );
        capabilities = createCapabilities();

        teleOpControls.bind(capabilities);

        driveAssists = new PhoenixDriveAssistService(
                profile.driveAssist,
                teleOpControls.manualDriveSource(),
                teleOpControls.manualTranslateMagnitudeSource(),
                teleOpControls.autoAimEnabledSource(),
                teleOpPoseAssistReadiness.isAllowed(),
                localization.globalEstimator(),
                scoringTargeting.aimOverlay()
        );
        teleOpDriveSource = driveAssists.driveSource();

        teleOpControls.emitInitHelp(telemetry);
        telemetryPresenter.emitTeleOpReadiness(teleOpPoseAssistReadiness);
        teleOpPoseRestore.initialize(localization.globalEstimator());
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
     * <p>The drive sink and predictor are supplied as an explicit pair because an external route
     * runtime may own both drivetrain actuation and the physical odometry resource. Requiring both
     * seams prevents Phoenix from silently constructing a second hardware predictor for Auto.</p>
     *
     * <p>Success means this composition root's shared services and routine lifecycle are available;
     * the Auto mode client must separately decide whether its exact calibration, field facts, and
     * route are ready before installing or starting behavior.</p>
     *
     * @param autonomousDrive external Auto drive owner; must not be null
     * @param motionPredictor predictor owned by the same Auto runtime; must not be null
     */
    public void initAuto(DriveCommandSink autonomousDrive,
                         MotionPredictor motionPredictor) {
        initAuto(
                autonomousDrive,
                motionPredictor,
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
     * @param motionPredictor     predictor owned by the same Auto runtime; must not be null
     * @param autoAimEnabledSource source that controls whether target selection + aim gating are active
     * @param aimOverrideSource    source that bypasses aim-readiness gating when true
     */
    public void initAuto(DriveCommandSink autonomousDrive,
                         MotionPredictor motionPredictor,
                         BooleanSource autoAimEnabledSource,
                         BooleanSource aimOverrideSource) {
        shutdown.beginInitialization("initAuto");
        try {
            this.autonomousDrive = Objects.requireNonNull(
                    autonomousDrive,
                    "autonomousDrive"
            );
            initSharedRuntimeWithPredictor(
                    Objects.requireNonNull(motionPredictor, "motionPredictor"),
                    autoAimEnabledSource,
                    aimOverrideSource
            );
            capabilities = createCapabilities();
            autoRoutineLifecycle = new AutoRoutineLifecycle();
        } catch (RuntimeException initializationFailure) {
            throw CleanupActions.attemptAllAfterFailure(
                    initializationFailure,
                    this::stop
            );
        }
    }

    /** Build the ordinary TeleOp localization lane, including its owned Pinpoint predictor. */
    private void initSharedRuntimeWithOwnedPredictor(BooleanSource autoAimEnabledSource,
                                                     BooleanSource aimOverrideSource) {
        vision = PhoenixVisionFactory.create(hardwareMap, profile.vision);
        localization = new FtcOdometryAprilTagLocalizationLane(
                hardwareMap,
                vision,
                profile.field.fixedAprilTagLayout,
                profile.localization
        );

        initSharedServices(autoAimEnabledSource, aimOverrideSource);
    }

    /** Build Auto localization around the predictor already owned by the external drive runtime. */
    private void initSharedRuntimeWithPredictor(MotionPredictor motionPredictor,
                                                BooleanSource autoAimEnabledSource,
                                                BooleanSource aimOverrideSource) {
        vision = PhoenixVisionFactory.create(hardwareMap, profile.vision);
        localization = FtcOdometryAprilTagLocalizationLane.withPredictor(
                Objects.requireNonNull(motionPredictor, "motionPredictor"),
                vision,
                profile.field.fixedAprilTagLayout,
                profile.localization
        );

        initSharedServices(autoAimEnabledSource, aimOverrideSource);
    }

    /** Build the mode-neutral Phoenix services after vision and localization ownership is fixed. */
    private void initSharedServices(BooleanSource autoAimEnabledSource,
                                    BooleanSource aimOverrideSource) {

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
     * <p>For Auto, this is the exact Phoenix lifecycle boundary for FTC START and closes the
     * pre-start routine installation window before {@link #startAuto()} begins the installed
     * root.</p>
     *
     * @param runtime current FTC runtime in seconds
     */
    public void startAny(double runtime) {
        clock.reset(runtime);
        teleOpPoseRestore.markStartBoundary();
        if (autoRoutineLifecycle != null) {
            autoRoutineLifecycle.markStartBoundary();
        }
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
     * <p>The one routine installed before {@link #startAny(double)} starts immediately on the shared
     * clock's exact FTC START boundary. This gives every match-time budget the same zero-delta
     * timestamp instead of delaying its start until the first regular loop.</p>
     *
     * @throws IllegalStateException if Auto is not initialized, {@link #startAny(double)} has not
     *                               established the FTC START boundary, no root is installed, or
     *                               this Auto runtime has already started
     */
    public void startAuto() {
        requireAutoRoutineLifecycle().start(clock);
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
     * Loop order is intentionally explicit: vision component readiness, localization lane,
     * targeting, controls, scoring path, drive-assist service, drive lane, then telemetry
     * presentation.
     * </p>
     */
    public void updateTeleOp() {
        if (drive == null
                || vision == null
                || localization == null
                || scoringPath == null
                || scoringTargeting == null
                || teleOpControls == null
                || driveAssists == null
                || teleOpDriveSource == null) {
            return;
        }

        VisionReadiness visionReadiness = vision.readiness(clock);
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
                teleOpPoseAssistReadiness,
                visionReadiness,
                globalPose,
                odomPose
        );
    }

    /**
     * Advances one autonomous loop.
     *
     * <p>Loop order is explicit and matches Phoenix ownership boundaries: vision component
     * readiness, localization, targeting, the continuously owned external drive heartbeat, the
     * installed autonomous root, the scoring path, and finally telemetry.</p>
     */
    public void updateAuto() {
        if (vision == null
                || localization == null
                || scoringPath == null
                || scoringTargeting == null
                || autoRoutineLifecycle == null
                || autonomousDrive == null) {
            return;
        }

        VisionReadiness visionReadiness = vision.readiness(clock);
        localization.update(clock);
        scoringTargeting.update(clock);
        autonomousDrive.update(clock);
        autoRoutineLifecycle.update(clock);

        scoringPath.update(clock);
        ScoringPath.Status scoringStatus = scoringPath.status();

        ScoringTargeting.Status targetingStatus = scoringTargeting.status(clock);
        PoseEstimate globalPose = localization.globalEstimator().getEstimate();
        PoseEstimate odomPose = localization.predictor().getEstimate();

        telemetryPresenter.emitAuto(
                scoringStatus,
                targetingStatus,
                autoRoutineLifecycle.installedRoutine(),
                visionReadiness,
                globalPose,
                odomPose
        );
    }

    /**
     * Installs the one root autonomous routine for this Phoenix runtime.
     *
     * <p>Call this exactly once after {@link #initAuto(DriveCommandSink, MotionPredictor)} and before
     * {@link #startAny(double)}. Normal Auto entries install during INIT; a guarded selector may
     * finish construction at the beginning of its FTC {@code start()} callback before forwarding
     * that lifecycle boundary. The root Task may internally compose every route, mechanism action,
     * deadline, and fallback needed by the selected strategy.</p>
     *
     * @param task fresh single-use root Task for the selected autonomous routine
     * @throws NullPointerException  if {@code task} is null
     * @throws IllegalStateException if Auto is not initialized, a root is already installed, or
     *                               {@link #startAny(double)} has closed the installation window
     */
    public void installAutoRoutine(Task task) {
        requireAutoRoutineLifecycle().install(task);
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
        AutoRoutineLifecycle autoRoutineToStop = autoRoutineLifecycle;
        PhoenixTeleOpControls controlsToStop = teleOpControls;
        PhoenixDriveAssistService assistsToStop = driveAssists;
        ScoringPath scoringToStop = scoringPath;
        DriveCommandSink autonomousDriveToStop = autonomousDrive;
        FtcMecanumDriveLane driveToStop = drive;
        ScoringTargeting targetingToStop = scoringTargeting;
        AprilTagVisionLane visionToStop = vision;

        autoRoutineLifecycle = null;
        teleOpControls = null;
        driveAssists = null;
        teleOpDriveSource = null;
        teleOpPoseAssistReadiness = null;
        autonomousDrive = null;
        scoringPath = null;
        drive = null;
        scoringTargeting = null;
        vision = null;
        capabilities = null;
        localization = null;
        teleOpPoseRestore.clear();

        Runnable cancelAuto = autoRoutineToStop == null
                ? null
                : autoRoutineToStop::cancelAndClear;
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

    /**
     * Applies one accepted Auto-to-TeleOp field pose before the FTC START boundary.
     *
     * <p>This package-private seam keeps the localization owner private. The robot-owned handoff
     * validates and consumes its process-local snapshot, while this composition root alone decides
     * how an accepted field pose rebases the active TeleOp estimator.</p>
     *
     * @param fieldToRobotPose accepted immutable pose in the Phoenix field frame
     * @throws NullPointerException  if {@code fieldToRobotPose} is null
     * @throws IllegalArgumentException if any planar pose component is not finite
     * @throws IllegalStateException if TeleOp localization is not initialized or FTC START has
     *                               already been reached
     */
    void restoreTeleOpPose(Pose2d fieldToRobotPose) {
        teleOpPoseRestore.restore(fieldToRobotPose);
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

    private AutoRoutineLifecycle requireAutoRoutineLifecycle() {
        if (autoRoutineLifecycle == null) {
            throw new IllegalStateException(
                    "Phoenix Auto is not initialized; call initAuto(...) before installing or "
                            + "starting an autonomous routine"
            );
        }
        return autoRoutineLifecycle;
    }

    /**
     * Package-private lifecycle owner behind Phoenix's one-root Auto API.
     *
     * <p>The {@link TaskRunner} remains an implementation detail. Retaining the installed root
     * separately keeps terminal phase/outcome telemetry available after the runner releases a
     * completed Task.</p>
     */
    static final class AutoRoutineLifecycle {
        private final TaskRunner runner = new TaskRunner();
        private Task installedRoutine;
        private boolean startBoundaryReached;
        private boolean started;

        /** Install exactly one fresh root before the Phoenix lifecycle reaches FTC START. */
        void install(Task task) {
            Task requiredTask = Objects.requireNonNull(
                    task,
                    "Phoenix auto routine is required"
            );
            if (startBoundaryReached || started) {
                throw new IllegalStateException(
                        "Phoenix auto routine installation is pre-start only; call "
                                + "installAutoRoutine(...) before startAny(...) and startAuto()"
                );
            }
            if (installedRoutine != null) {
                throw new IllegalStateException(
                        "Phoenix Auto already has an installed routine; install exactly one root "
                                + "Task for each PhoenixRobot"
                );
            }
            installedRoutine = requiredTask;
        }

        /** Record that FTC START has closed the pre-start installation window. */
        void markStartBoundary() {
            startBoundaryReached = true;
        }

        /** Start the installed root once at the current shared-clock boundary. */
        void start(LoopClock clock) {
            Objects.requireNonNull(clock, "Phoenix Auto start clock is required");
            if (started) {
                throw new IllegalStateException(
                        "Phoenix Auto has already started; create a new PhoenixRobot for another "
                                + "autonomous run"
                );
            }
            if (installedRoutine == null) {
                throw new IllegalStateException(
                        "Phoenix Auto cannot start without an installed routine; call "
                                + "installAutoRoutine(...) before startAny(runtime)"
                );
            }
            if (!startBoundaryReached) {
                throw new IllegalStateException(
                        "Phoenix Auto cannot start before startAny(runtime) resets the shared "
                                + "LoopClock at FTC START"
                );
            }

            // Publish the terminal lifecycle state before invoking arbitrary Task callbacks so a
            // reentrant or throwing start cannot schedule the root a second time.
            started = true;
            runner.enqueue(installedRoutine);
            runner.update(clock);
        }

        /** Advance the private one-root runner during the ordinary Phoenix Auto Task phase. */
        void update(LoopClock clock) {
            runner.update(clock);
        }

        /** Cancel active work and discard any pending runner state during total robot shutdown. */
        void cancelAndClear() {
            runner.cancelAndClear();
        }

        /** Return the retained root for active and terminal telemetry. */
        Task installedRoutine() {
            return installedRoutine;
        }

    }

    /**
     * Package-private lifecycle guard for the narrow pre-START TeleOp pose-reset seam.
     *
     * <p>Keeping this state separate from FTC hardware construction lets its boundary rules be
     * verified with a fake {@link PoseResetter} on the host JVM.</p>
     */
    static final class TeleOpPoseRestoreLifecycle {
        private PoseResetter poseResetter;
        private boolean startBoundaryReached;

        /** Install the TeleOp localization reset capability after initialization succeeds. */
        void initialize(PoseResetter poseResetter) {
            if (this.poseResetter != null) {
                throw new IllegalStateException(
                        "Phoenix TeleOp pose restore is already initialized; create a new "
                                + "PhoenixRobot for another mode"
                );
            }
            this.poseResetter = Objects.requireNonNull(
                    poseResetter,
                    "Phoenix TeleOp pose resetter is required"
            );
        }

        /** Close the restore window at the shared FTC START boundary. */
        void markStartBoundary() {
            startBoundaryReached = true;
        }

        /** Apply one validated field pose while TeleOp localization is initialized and pre-START. */
        void restore(Pose2d fieldToRobotPose) {
            Pose2d requiredPose = Objects.requireNonNull(
                    fieldToRobotPose,
                    "Auto-to-TeleOp field pose is required"
            );
            if (!Double.isFinite(requiredPose.xInches)
                    || !Double.isFinite(requiredPose.yInches)
                    || !Double.isFinite(requiredPose.headingRad)) {
                throw new IllegalArgumentException(
                        "Auto-to-TeleOp field pose must have finite xInches, yInches, and "
                                + "headingRad"
                );
            }
            if (poseResetter == null) {
                throw new IllegalStateException(
                        "Cannot restore the Auto pose because Phoenix TeleOp localization is not "
                                + "initialized; call initTeleOp() first"
                );
            }
            if (startBoundaryReached) {
                throw new IllegalStateException(
                        "Cannot restore the Auto pose after FTC START; call "
                                + "PhoenixMatchHandoff.restoreForTeleOp(...) during TeleOp INIT"
                );
            }
            poseResetter.setPose(requiredPose);
        }

        /** Detach the localization capability when the robot ownership graph stops. */
        void clear() {
            poseResetter = null;
            startBoundaryReached = false;
        }
    }
}
