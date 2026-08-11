package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;
import java.util.Set;

import edu.ftcphoenix.fw.core.debug.LoopPhaseProfiler;
import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.FtcTelemetryDebugSink;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.VisionReadiness;
import edu.ftcphoenix.fw.input.Gamepads;
import edu.ftcphoenix.fw.localization.MotionPredictor;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.PoseResetter;
import edu.ftcphoenix.fw.task.Task;

/**
 * Phoenix composition root.
 *
 * <p>Ordinary TeleOp uses one declarative grammar: construct this object, call
 * {@link #declareTeleOp(RobotProgram, Source)} with its frozen scoring-tag eligibility, restore
 * the optional match snapshot, and register
 * {@link #teleOpPresenter(PhoenixMatchHandoff.RestoreResult)}. The framework then owns the clock,
 * lifecycle callbacks, bindings, output heartbeat, telemetry commit, and fail-stop cleanup.</p>
 *
 * <p>Auto uses the same declaration grammar through {@link #declareAuto(RobotProgram,
 * DriveCommandSink, MotionPredictor, Source, BooleanSource, BooleanSource, Runnable)}. The managed
 * program owns FTC callbacks, the one clock, root Task, output order, telemetry commit, and
 * fail-stop cleanup in both modes. Phoenix still owns its hardware graph and service ordering;
 * alliance selection, Pedro route geometry, targeting policy, and routine composition remain
 * outside this mode-neutral composition root.</p>
 */
public final class PhoenixRobot {

    private static final boolean ENABLE_LOOP_PHASE_PROFILING = false;

    /**
     * Package-private construction seam for a hardware-neutral managed-TeleOp host test.
     *
     * <p>Ordinary robot code has only the public {@link HardwareMap} constructors. This seam is
     * instance-scoped, changes no lifecycle grammar, and still makes {@link PhoenixRobot} construct
     * and privately retain its complete mechanism graph.</p>
     */
    interface TeleOpHardwareAssembly {
        AprilTagVisionLane createVision(HardwareMap hardwareMap, PhoenixProfile profile);

        FtcOdometryAprilTagLocalizationLane createLocalization(
                HardwareMap hardwareMap,
                AprilTagVisionLane vision,
                PhoenixProfile profile
        );

        ScoringPath createScoring(
                HardwareMap hardwareMap,
                PhoenixProfile profile,
                ScoringTargeting targeting
        );

        DriveCommandSink createDrive(HardwareMap hardwareMap, PhoenixProfile profile);
    }

    /** Package-private hardware-neutral seam for managed-Auto lifecycle tests. */
    interface AutoHardwareAssembly {
        AprilTagVisionLane createVision(HardwareMap hardwareMap, PhoenixProfile profile);

        FtcOdometryAprilTagLocalizationLane createLocalization(
                MotionPredictor motionPredictor,
                AprilTagVisionLane vision,
                PhoenixProfile profile
        );

        ScoringPath createScoring(
                HardwareMap hardwareMap,
                PhoenixProfile profile,
                ScoringTargeting targeting
        );
    }

    private static final TeleOpHardwareAssembly FTC_TELEOP_HARDWARE =
            new TeleOpHardwareAssembly() {
                @Override
                public AprilTagVisionLane createVision(
                        HardwareMap hardwareMap,
                        PhoenixProfile profile
                ) {
                    return PhoenixVisionFactory.create(hardwareMap, profile.vision);
                }

                @Override
                public FtcOdometryAprilTagLocalizationLane createLocalization(
                        HardwareMap hardwareMap,
                        AprilTagVisionLane vision,
                        PhoenixProfile profile
                ) {
                    return new FtcOdometryAprilTagLocalizationLane(
                            hardwareMap,
                            vision,
                            profile.field.fixedAprilTagLayout,
                            profile.localization
                    );
                }

                @Override
                public ScoringPath createScoring(
                        HardwareMap hardwareMap,
                        PhoenixProfile profile,
                        ScoringTargeting targeting
                ) {
                    return new ScoringPath(hardwareMap, profile.scoring, targeting);
                }

                @Override
                public DriveCommandSink createDrive(
                        HardwareMap hardwareMap,
                        PhoenixProfile profile
                ) {
                    return FtcDrives.mecanum(hardwareMap, profile.drive);
                }
            };

    private static final AutoHardwareAssembly FTC_AUTO_HARDWARE =
            new AutoHardwareAssembly() {
                @Override
                public AprilTagVisionLane createVision(
                        HardwareMap hardwareMap,
                        PhoenixProfile profile
                ) {
                    return PhoenixVisionFactory.create(hardwareMap, profile.vision);
                }

                @Override
                public FtcOdometryAprilTagLocalizationLane createLocalization(
                        MotionPredictor motionPredictor,
                        AprilTagVisionLane vision,
                        PhoenixProfile profile
                ) {
                    return FtcOdometryAprilTagLocalizationLane.withPredictor(
                            motionPredictor,
                            vision,
                            profile.field.fixedAprilTagLayout,
                            profile.localization
                    );
                }

                @Override
                public ScoringPath createScoring(
                        HardwareMap hardwareMap,
                        PhoenixProfile profile,
                        ScoringTargeting targeting
                ) {
                    return new ScoringPath(hardwareMap, profile.scoring, targeting);
                }
            };

    private enum RuntimeMode {
        NEW,
        MANAGED_TELEOP,
        MANAGED_AUTO
    }

    private final HardwareMap hardwareMap;
    private final Gamepads gamepads;
    private final PhoenixProfile profile;
    private final TeleOpHardwareAssembly teleOpHardwareAssembly;
    private final AutoHardwareAssembly autoHardwareAssembly;
    private final PhoenixTelemetryPresenter telemetryPresenter;
    private final LoopPhaseProfiler loopPhaseProfiler;
    private final TeleOpPoseRestoreLifecycle teleOpPoseRestore =
            new TeleOpPoseRestoreLifecycle();

    private RuntimeMode mode = RuntimeMode.NEW;
    private PhoenixCapabilities capabilities;
    private AprilTagVisionLane vision;
    private FtcOdometryAprilTagLocalizationLane localization;
    private ScoringPath scoringPath;
    private ScoringTargeting scoringTargeting;

    // Managed-TeleOp graph and read-only presentation state.
    private PhoenixTeleOpControls teleOpControls;
    private PhoenixDriveAssistService driveAssists;
    private PhoenixReadiness.Result teleOpPoseAssistReadiness;
    private VisionReadiness teleOpVisionReadiness =
            VisionReadiness.notReady("Phoenix TeleOp has not reached FTC START");
    private boolean teleOpStartBoundaryReached;
    private boolean teleOpOrdinaryLoopReached;
    private boolean teleOpProfileCycleActive;
    private boolean teleOpPresenterCreated;

    // Managed-Auto presentation state.
    private VisionReadiness autoVisionReadiness =
            VisionReadiness.notReady("Phoenix Auto has not reached FTC START");
    private boolean autoStartBoundaryReached;
    private boolean autoProfileCycleActive;
    private boolean autoPresenterCreated;

    /** Create Phoenix using the checked-in profile snapshot. */
    public PhoenixRobot(HardwareMap hardwareMap,
                        Telemetry telemetry,
                        Gamepad gamepad1,
                        Gamepad gamepad2) {
        this(hardwareMap, telemetry, gamepad1, gamepad2, PhoenixProfile.current());
    }

    /** Create Phoenix using an explicit defensively copied profile. */
    public PhoenixRobot(HardwareMap hardwareMap,
                        Telemetry telemetry,
                        Gamepad gamepad1,
                        Gamepad gamepad2,
                        PhoenixProfile profile) {
        this(
                hardwareMap,
                telemetry,
                gamepad1,
                gamepad2,
                profile,
                FTC_TELEOP_HARDWARE,
                FTC_AUTO_HARDWARE
        );
    }

    /** Hardware-neutral host-test construction; not a robot-code construction path. */
    PhoenixRobot(HardwareMap hardwareMap,
                 Telemetry telemetry,
                 Gamepad gamepad1,
                 Gamepad gamepad2,
                 PhoenixProfile profile,
                 TeleOpHardwareAssembly teleOpHardwareAssembly) {
        this(
                hardwareMap,
                telemetry,
                gamepad1,
                gamepad2,
                profile,
                teleOpHardwareAssembly,
                FTC_AUTO_HARDWARE
        );
    }

    /** Hardware-neutral lifecycle-test construction; not an ordinary robot-code path. */
    PhoenixRobot(HardwareMap hardwareMap,
                 Telemetry telemetry,
                 Gamepad gamepad1,
                 Gamepad gamepad2,
                 PhoenixProfile profile,
                 TeleOpHardwareAssembly teleOpHardwareAssembly,
                 AutoHardwareAssembly autoHardwareAssembly) {
        this.hardwareMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
        Objects.requireNonNull(telemetry, "telemetry");
        this.gamepads = Gamepads.create(gamepad1, gamepad2);
        this.profile = Objects.requireNonNull(profile, "profile").copy();
        this.teleOpHardwareAssembly = Objects.requireNonNull(
                teleOpHardwareAssembly,
                "teleOpHardwareAssembly"
        );
        this.autoHardwareAssembly = Objects.requireNonNull(
                autoHardwareAssembly,
                "autoHardwareAssembly"
        );
        this.telemetryPresenter = new PhoenixTelemetryPresenter(this.profile);
        this.loopPhaseProfiler = LoopPhaseProfiler.create(ENABLE_LOOP_PHASE_PROFILING);
    }

    /**
     * Declare Phoenix's complete ordinary TeleOp program.
     *
     * <p>The declaration order is vision readiness, localization, targeting, managed bindings,
     * scoring realization, final drive, and presentation. Every hardware/resource owner is
     * registered immediately. If later configuration fails, {@link RobotProgram} performs the one
     * best-effort cleanup transaction.</p>
     *
     * <p>At FTC START, services refresh their snapshots, scoring realizes once, and the final drive
     * writes an explicit zero. Manual and assisted driving starts in the first ordinary active
     * loop. No TeleOp behavior or hardware is advanced during INIT.</p>
     *
     * @param program configuring managed program supplied by the FTC host
     * @param eligibleScoringTagIds non-empty configured scoring-AprilTag subset selected by the
     *                              mode client's frozen alliance policy
     */
    public void declareTeleOp(
            RobotProgram program,
            Source<Set<Integer>> eligibleScoringTagIds
    ) {
        RobotProgram requiredProgram = Objects.requireNonNull(
                program,
                "Phoenix TeleOp program is required"
        );
        Source<Set<Integer>> requiredEligibleScoringTagIds = Objects.requireNonNull(
                eligibleScoringTagIds,
                "Phoenix TeleOp eligibleScoringTagIds source is required"
        );
        beginMode(RuntimeMode.MANAGED_TELEOP, "declareTeleOp");
        loopPhaseProfiler.reset();

        teleOpControls = new PhoenixTeleOpControls(
                gamepads,
                profile.controls
        );
        teleOpPoseAssistReadiness = PhoenixReadiness.teleOpPoseAssists(profile);

        BooleanSource enabledAutoAim = teleOpControls.autoAimEnabledSource()
                .and(BooleanSource.constant(teleOpPoseAssistReadiness.isAllowed()))
                .memoized();

        vision = Objects.requireNonNull(
                teleOpHardwareAssembly.createVision(hardwareMap, profile),
                "TeleOp hardware assembly returned null vision"
        );
        ManagedTeleOpVisionService visionService = new ManagedTeleOpVisionService(vision);
        registerServiceOrClean(requiredProgram, visionService, vision::close);

        localization = Objects.requireNonNull(
                teleOpHardwareAssembly.createLocalization(
                        hardwareMap,
                        vision,
                        profile
                ),
                "TeleOp hardware assembly returned null localization"
        );
        requiredProgram.service(new ManagedTeleOpLocalizationService(localization));
        teleOpPoseRestore.initialize(localization.globalEstimator());

        scoringTargeting = createTargeting(
                requiredEligibleScoringTagIds,
                enabledAutoAim,
                teleOpControls.aimOverrideSource()
        );
        requiredProgram.service(new ManagedTeleOpTargetingService(scoringTargeting));

        scoringPath = Objects.requireNonNull(
                teleOpHardwareAssembly.createScoring(
                        hardwareMap,
                        profile,
                        scoringTargeting
                ),
                "TeleOp hardware assembly returned null scoring path"
        );
        ManagedTeleOpScoringOutput scoringOutput =
                new ManagedTeleOpScoringOutput(scoringPath);
        registerOutputOrClean(requiredProgram, scoringOutput, scoringPath::stop);

        capabilities = createCapabilities();
        teleOpControls.bind(requiredProgram.callbackBindings(), capabilities);

        Source<ScoringPath.Status> scoringStatus =
                Source.of(ignoredClock -> scoringPath.status());
        driveAssists = new PhoenixDriveAssistService(
                profile.driveAssist,
                teleOpControls.manualDriveSource(),
                teleOpControls.manualTranslateMagnitudeSource(),
                scoringStatus,
                teleOpControls.autoAimEnabledSource(),
                teleOpPoseAssistReadiness.isAllowed(),
                localization.globalEstimator(),
                scoringTargeting.aimOverlay()
        );

        DriveCommandSink drive = Objects.requireNonNull(
                teleOpHardwareAssembly.createDrive(hardwareMap, profile),
                "TeleOp hardware assembly returned null drive sink"
        );
        ManagedTeleOpDriveSink driveSink = new ManagedTeleOpDriveSink(drive);
        try {
            requiredProgram.drive(
                    new ManagedTeleOpDriveSource(driveAssists.driveSource()),
                    driveSink
            );
        } catch (RuntimeException registrationFailure) {
            throw CleanupActions.attemptAllAfterFailure(
                    registrationFailure,
                    driveSink::stop
            );
        }
    }

    /**
     * Return Phoenix's one additive TeleOp presenter after the match-handoff decision is known.
     *
     * <p>During INIT it repeats controls, pose-assist readiness, and handoff status. After START it
     * presents only already-published robot snapshots. It never clears or commits telemetry.</p>
     *
     * @param restoreResult result of consuming Phoenix's Auto-to-TeleOp match snapshot
     * @return presenter to register with the same managed program
     */
    public RobotProgram.Presenter teleOpPresenter(
            PhoenixMatchHandoff.RestoreResult restoreResult
    ) {
        requireMode(RuntimeMode.MANAGED_TELEOP, "create the TeleOp presenter");
        PhoenixMatchHandoff.RestoreResult requiredResult = Objects.requireNonNull(
                restoreResult,
                "Phoenix TeleOp handoff result is required"
        );
        if (teleOpPresenterCreated) {
            throw new IllegalStateException(
                    "Phoenix TeleOp already created its presenter; register exactly one presenter"
            );
        }
        teleOpPresenterCreated = true;
        return (clock, frameTelemetry) -> presentTeleOp(
                clock,
                frameTelemetry,
                requiredResult
        );
    }

    /**
     * Declare Phoenix's complete managed Auto hardware and service graph.
     *
     * <p>The caller retains strategy data and the root routine. Phoenix immediately transfers the
     * supplied drive owner into the program, then constructs vision, localization, targeting, and
     * scoring exactly once. At START the service applies the frozen Pedro pose before its first
     * vision, localization, targeting, and vendor-heartbeat updates. Ordinary Phoenix Auto passes
     * always-enabled auto aim and no override; a direct advanced assembly may supply other
     * clock-aware policies without taking lifecycle ownership away from the managed program.</p>
     *
     * @param program managed program that immediately owns every completed resource
     * @param autonomousDrive sole Auto drive heartbeat and stop owner
     * @param motionPredictor predictor owned by the same Auto runtime
     * @param eligibleScoringTagIds non-empty configured scoring-tag subset eligible in this mode
     * @param autoAimEnabledSource policy that enables target selection and aim gating
     * @param aimOverrideSource policy that bypasses aim-readiness gating while true
     * @param applyStartingPose exact-START action that applies the frozen Auto pose
     */
    public void declareAuto(
            RobotProgram program,
            DriveCommandSink autonomousDrive,
            MotionPredictor motionPredictor,
            Source<Set<Integer>> eligibleScoringTagIds,
            BooleanSource autoAimEnabledSource,
            BooleanSource aimOverrideSource,
            Runnable applyStartingPose
    ) {
        RobotProgram requiredProgram = Objects.requireNonNull(
                program,
                "Phoenix Auto program is required"
        );
        beginMode(RuntimeMode.MANAGED_AUTO, "declareAuto");
        loopPhaseProfiler.reset();

        ManagedAutoService autoService = new ManagedAutoService(
                Objects.requireNonNull(autonomousDrive, "autonomousDrive"),
                Objects.requireNonNull(applyStartingPose, "applyStartingPose")
        );
        registerServiceOrClean(requiredProgram, autoService, autoService::stop);

        vision = Objects.requireNonNull(
                autoHardwareAssembly.createVision(hardwareMap, profile),
                "Auto hardware assembly returned null vision"
        );
        autoService.attachVision(vision);

        localization = Objects.requireNonNull(
                autoHardwareAssembly.createLocalization(
                        Objects.requireNonNull(motionPredictor, "motionPredictor"),
                        vision,
                        profile
                ),
                "Auto hardware assembly returned null localization"
        );
        autoService.attachLocalization(localization);

        scoringTargeting = createTargeting(
                Objects.requireNonNull(
                        eligibleScoringTagIds,
                        "eligibleScoringTagIds"
                ),
                Objects.requireNonNull(
                        autoAimEnabledSource,
                        "autoAimEnabledSource"
                ),
                Objects.requireNonNull(
                        aimOverrideSource,
                        "aimOverrideSource"
                )
        );
        autoService.attachTargeting(scoringTargeting);

        scoringPath = Objects.requireNonNull(
                autoHardwareAssembly.createScoring(
                        hardwareMap,
                        profile,
                        scoringTargeting
                ),
                "Auto hardware assembly returned null scoring path"
        );
        ManagedAutoScoringOutput scoringOutput = new ManagedAutoScoringOutput(scoringPath);
        registerOutputOrClean(requiredProgram, scoringOutput, scoringPath::stop);
        capabilities = createCapabilities();
    }

    /** Return Phoenix's one additive managed-Auto presenter for the declared root routine. */
    public RobotProgram.Presenter autoPresenter(Task rootRoutine) {
        requireMode(RuntimeMode.MANAGED_AUTO, "create the Auto presenter");
        Task requiredRoot = Objects.requireNonNull(rootRoutine, "rootRoutine");
        if (autoPresenterCreated) {
            throw new IllegalStateException(
                    "Phoenix Auto already created its presenter; register exactly one presenter"
            );
        }
        autoPresenterCreated = true;
        return (clock, frameTelemetry) -> presentAuto(
                clock,
                frameTelemetry,
                requiredRoot
        );
    }

    /** Return Phoenix's initialized shared, mode-neutral capability vocabulary. */
    public PhoenixCapabilities capabilities() {
        if (capabilities == null) {
            throw new IllegalStateException("Phoenix capabilities are not initialized");
        }
        return capabilities;
    }

    /** Apply one accepted match pose while managed TeleOp is declared and still in INIT. */
    void restoreTeleOpPose(Pose2d fieldToRobotPose) {
        requireMode(RuntimeMode.MANAGED_TELEOP, "restore the Auto-to-TeleOp pose");
        teleOpPoseRestore.restore(fieldToRobotPose);
    }

    private ScoringTargeting createTargeting(Source<Set<Integer>> eligibleScoringTagIds,
                                              BooleanSource autoAimEnabledSource,
                                              BooleanSource aimOverrideSource) {
        return new ScoringTargeting(
                profile.autoAim,
                profile.localization.aprilTags.fieldPoseSolver.copy(),
                vision.tagSensor(),
                vision.cameraMountConfig(),
                localization.globalEstimator(),
                profile.field.fixedAprilTagLayout,
                eligibleScoringTagIds,
                autoAimEnabledSource,
                aimOverrideSource,
                profile.autoAim.shotVelocityTable
        );
    }

    private PhoenixCapabilities createCapabilities() {
        if (scoringPath == null || scoringTargeting == null) {
            throw new IllegalStateException(
                    "Phoenix cannot create capabilities before scoring and targeting exist"
            );
        }
        return new PhoenixCapabilities(scoringPath, scoringTargeting);
    }

    private void presentTeleOp(LoopClock clock,
                               Telemetry frameTelemetry,
                               PhoenixMatchHandoff.RestoreResult restoreResult) {
        Objects.requireNonNull(clock, "Phoenix TeleOp presentation clock is required");
        Telemetry requiredTelemetry = Objects.requireNonNull(
                frameTelemetry,
                "Phoenix TeleOp frame telemetry is required"
        );

        if (!teleOpStartBoundaryReached) {
            teleOpControls.emitInitHelp(requiredTelemetry);
            telemetryPresenter.emitTeleOpReadiness(
                    requiredTelemetry,
                    teleOpPoseAssistReadiness
            );
            requiredTelemetry.addData("teleop.matchHandoff", restoreResult);
            if (restoreResult != PhoenixMatchHandoff.RestoreResult.RESTORED) {
                requiredTelemetry.addLine(
                        "Match handoff was not restored; TeleOp keeps its normal pose and owns "
                                + "the displayed alliance selection."
                );
            }
            return;
        }

        ScoringPath.Status scoringStatus = scoringPath.status();
        ScoringTargeting.Status targetingStatus = scoringTargeting.status();
        PhoenixDriveAssistService.Status driveAssistStatus = driveAssists.status();
        PoseEstimate globalPose = localization.globalEstimator().getEstimate();
        PoseEstimate odomPose = localization.predictor().getEstimate();
        finishTeleOpProfilePhase("snapshots");

        if (ENABLE_LOOP_PHASE_PROFILING) {
            loopPhaseProfiler.debugDump(
                    new FtcTelemetryDebugSink(requiredTelemetry),
                    "loopProfile"
            );
        }
        telemetryPresenter.emitTeleOp(
                requiredTelemetry,
                scoringStatus,
                targetingStatus,
                driveAssistStatus,
                teleOpPoseAssistReadiness,
                teleOpVisionReadiness,
                globalPose,
                odomPose
        );
        finishTeleOpProfilePhase("presentation");
        finishTeleOpProfileCycle(clock);
    }

    private void presentAuto(LoopClock clock,
                             Telemetry frameTelemetry,
                             Task rootRoutine) {
        Objects.requireNonNull(clock, "Phoenix Auto presentation clock is required");
        Telemetry requiredTelemetry = Objects.requireNonNull(
                frameTelemetry,
                "Phoenix Auto frame telemetry is required"
        );
        if (!autoStartBoundaryReached) {
            requiredTelemetry.addLine(
                    "Phoenix Auto hardware is configured and remains inert until START is allowed."
            );
            return;
        }

        ScoringPath.Status scoringStatus = scoringPath.status();
        ScoringTargeting.Status targetingStatus = scoringTargeting.status();
        PoseEstimate globalPose = localization.globalEstimator().getEstimate();
        PoseEstimate odomPose = localization.predictor().getEstimate();
        finishAutoProfilePhase("snapshots");

        if (ENABLE_LOOP_PHASE_PROFILING) {
            loopPhaseProfiler.debugDump(
                    new FtcTelemetryDebugSink(requiredTelemetry),
                    "loopProfile"
            );
        }
        telemetryPresenter.emitAuto(
                requiredTelemetry,
                scoringStatus,
                targetingStatus,
                rootRoutine,
                autoVisionReadiness,
                globalPose,
                odomPose
        );
        finishAutoProfilePhase("presentation");
        finishAutoProfileCycle(clock);
    }

    private void beginMode(RuntimeMode selectedMode, String operation) {
        if (mode != RuntimeMode.NEW) {
            throw new IllegalStateException(
                    operation + " cannot run because this PhoenixRobot already selected " + mode
                            + "; create a new PhoenixRobot for another mode or runtime"
            );
        }
        mode = selectedMode;
    }

    private void requireMode(RuntimeMode requiredMode, String operation) {
        if (mode != requiredMode) {
            throw new IllegalStateException(
                    "PhoenixRobot cannot " + operation + " while its mode is " + mode
                            + "; expected " + requiredMode
            );
        }
    }

    private static <T extends RobotProgram.Service> T registerServiceOrClean(
            RobotProgram program,
            T service,
            Runnable cleanup
    ) {
        try {
            return program.service(service);
        } catch (RuntimeException registrationFailure) {
            throw CleanupActions.attemptAllAfterFailure(registrationFailure, cleanup);
        }
    }

    private static <T extends RobotProgram.Output> T registerOutputOrClean(
            RobotProgram program,
            T output,
            Runnable cleanup
    ) {
        try {
            return program.output(output);
        } catch (RuntimeException registrationFailure) {
            throw CleanupActions.attemptAllAfterFailure(registrationFailure, cleanup);
        }
    }

    private void finishTeleOpProfilePhase(String name) {
        if (teleOpProfileCycleActive) {
            loopPhaseProfiler.finishPhase(name);
        }
    }

    private void finishTeleOpProfileCycle(LoopClock clock) {
        if (!teleOpProfileCycleActive) {
            return;
        }
        loopPhaseProfiler.finishCycle(clock);
        teleOpProfileCycleActive = false;
    }

    private void finishAutoProfilePhase(String name) {
        if (autoProfileCycleActive) {
            loopPhaseProfiler.finishPhase(name);
        }
    }

    private void finishAutoProfileCycle(LoopClock clock) {
        if (!autoProfileCycleActive) {
            return;
        }
        loopPhaseProfiler.finishCycle(clock);
        autoProfileCycleActive = false;
    }

    /** Managed owner of vision readiness and its close lifecycle. */
    private final class ManagedTeleOpVisionService implements RobotProgram.Service {
        private final AprilTagVisionLane ownedVision;

        private ManagedTeleOpVisionService(AprilTagVisionLane ownedVision) {
            this.ownedVision = ownedVision;
        }

        @Override
        public void start(LoopClock clock) {
            loopPhaseProfiler.reset();
            teleOpStartBoundaryReached = true;
            teleOpOrdinaryLoopReached = false;
            teleOpPoseRestore.markStartBoundary();
            teleOpVisionReadiness = ownedVision.readiness(clock);
        }

        @Override
        public void update(LoopClock clock) {
            teleOpOrdinaryLoopReached = true;
            loopPhaseProfiler.startCycle(clock);
            teleOpProfileCycleActive = true;
            teleOpVisionReadiness = ownedVision.readiness(clock);
            finishTeleOpProfilePhase("visionReadiness");
        }

        @Override
        public void stop() {
            CleanupActions.attemptAll(ownedVision::close, teleOpPoseRestore::clear);
        }
    }

    /** Managed localization heartbeat; its hardware resource is owned by the vision/predictor graph. */
    private final class ManagedTeleOpLocalizationService implements RobotProgram.Service {
        private final FtcOdometryAprilTagLocalizationLane ownedLocalization;

        private ManagedTeleOpLocalizationService(
                FtcOdometryAprilTagLocalizationLane ownedLocalization
        ) {
            this.ownedLocalization = ownedLocalization;
        }

        @Override
        public void start(LoopClock clock) {
            ownedLocalization.update(clock);
        }

        @Override
        public void update(LoopClock clock) {
            ownedLocalization.update(clock);
            finishTeleOpProfilePhase("localization");
        }

        @Override
        public void stop() {
            // The lane's stable resources are stopped by their explicit vision/output owners.
        }
    }

    /** Managed targeting publisher and reset owner. */
    private final class ManagedTeleOpTargetingService implements RobotProgram.Service {
        private final ScoringTargeting ownedTargeting;

        private ManagedTeleOpTargetingService(ScoringTargeting ownedTargeting) {
            this.ownedTargeting = ownedTargeting;
        }

        @Override
        public void start(LoopClock clock) {
            ownedTargeting.update(clock);
        }

        @Override
        public void update(LoopClock clock) {
            ownedTargeting.update(clock);
            finishTeleOpProfilePhase("targeting");
        }

        @Override
        public void stop() {
            ownedTargeting.reset();
        }
    }

    /** Downstream scoring owner with transparent optional profiling boundaries. */
    private final class ManagedTeleOpScoringOutput implements RobotProgram.Output {
        private final ScoringPath ownedScoring;

        private ManagedTeleOpScoringOutput(ScoringPath ownedScoring) {
            this.ownedScoring = ownedScoring;
        }

        @Override
        public void update(LoopClock clock) {
            finishTeleOpProfilePhase("controls");
            ownedScoring.update(clock);
            finishTeleOpProfilePhase("scoring");
        }

        @Override
        public void stop() {
            ownedScoring.stop();
        }
    }

    /** Final source that makes exact START zero distinct from the first ordinary drive loop. */
    private final class ManagedTeleOpDriveSource implements DriveSource {
        private final DriveSource activeSource;

        private ManagedTeleOpDriveSource(DriveSource activeSource) {
            this.activeSource = activeSource;
        }

        @Override
        public DriveSignal get(LoopClock clock) {
            DriveSignal signal = teleOpOrdinaryLoopReached
                    ? activeSource.get(clock)
                    : DriveSignal.zero();
            finishTeleOpProfilePhase("driveAssist");
            return signal;
        }

        @Override
        public void reset() {
            activeSource.reset();
        }
    }

    /** Final drive sink wrapper that keeps physical write timing in Phoenix's diagnostics. */
    private final class ManagedTeleOpDriveSink implements DriveCommandSink {
        private final DriveCommandSink ownedSink;

        private ManagedTeleOpDriveSink(DriveCommandSink ownedSink) {
            this.ownedSink = ownedSink;
        }

        @Override
        public void update(LoopClock clock) {
            ownedSink.update(clock);
        }

        @Override
        public void drive(DriveSignal signal) {
            ownedSink.drive(signal);
            finishTeleOpProfilePhase("drive");
        }

        @Override
        public void stop() {
            ownedSink.stop();
        }
    }

    /** Stable managed owner of the complete upstream Auto update and cleanup order. */
    private final class ManagedAutoService implements RobotProgram.Service {
        private final DriveCommandSink autoDrive;
        private final Runnable applyStartingPose;
        private AprilTagVisionLane ownedVision;
        private FtcOdometryAprilTagLocalizationLane ownedLocalization;
        private ScoringTargeting ownedTargeting;
        private boolean stopped;

        private ManagedAutoService(DriveCommandSink autoDrive,
                                   Runnable applyStartingPose) {
            this.autoDrive = autoDrive;
            this.applyStartingPose = applyStartingPose;
        }

        private void attachVision(AprilTagVisionLane vision) {
            if (ownedVision != null) {
                throw new IllegalStateException("Phoenix Auto vision is already attached");
            }
            ownedVision = Objects.requireNonNull(vision, "vision");
        }

        private void attachLocalization(FtcOdometryAprilTagLocalizationLane localization) {
            if (ownedLocalization != null) {
                throw new IllegalStateException("Phoenix Auto localization is already attached");
            }
            ownedLocalization = Objects.requireNonNull(localization, "localization");
        }

        private void attachTargeting(ScoringTargeting targeting) {
            if (ownedTargeting != null) {
                throw new IllegalStateException("Phoenix Auto targeting is already attached");
            }
            ownedTargeting = Objects.requireNonNull(targeting, "targeting");
        }

        @Override
        public void start(LoopClock clock) {
            requireComplete();
            loopPhaseProfiler.reset();
            autoStartBoundaryReached = true;
            applyStartingPose.run();
            autoVisionReadiness = ownedVision.readiness(clock);
            ownedLocalization.update(clock);
            ownedTargeting.update(clock);
            autoDrive.update(clock);
        }

        @Override
        public void update(LoopClock clock) {
            requireComplete();
            loopPhaseProfiler.startCycle(clock);
            autoProfileCycleActive = true;

            autoVisionReadiness = ownedVision.readiness(clock);
            finishAutoProfilePhase("visionReadiness");
            ownedLocalization.update(clock);
            finishAutoProfilePhase("localization");
            ownedTargeting.update(clock);
            finishAutoProfilePhase("targeting");
            autoDrive.update(clock);
            finishAutoProfilePhase("drive");
        }

        @Override
        public void stop() {
            if (stopped) {
                return;
            }
            stopped = true;
            ScoringTargeting targetingToStop = ownedTargeting;
            AprilTagVisionLane visionToStop = ownedVision;
            ownedTargeting = null;
            ownedLocalization = null;
            ownedVision = null;
            autoProfileCycleActive = false;

            CleanupActions.attemptAll(
                    autoDrive::stop,
                    () -> {
                        if (targetingToStop != null) {
                            targetingToStop.reset();
                        }
                    },
                    () -> {
                        if (visionToStop != null) {
                            visionToStop.close();
                        }
                    }
            );
        }

        private void requireComplete() {
            if (stopped
                    || ownedVision == null
                    || ownedLocalization == null
                    || ownedTargeting == null) {
                throw new IllegalStateException(
                        "Phoenix managed Auto service graph is incomplete or already stopped"
                );
            }
        }
    }

    /** Downstream managed Auto scoring owner. */
    private final class ManagedAutoScoringOutput implements RobotProgram.Output {
        private final ScoringPath ownedScoring;

        private ManagedAutoScoringOutput(ScoringPath ownedScoring) {
            this.ownedScoring = ownedScoring;
        }

        @Override
        public void update(LoopClock clock) {
            finishAutoProfilePhase("tasks");
            ownedScoring.update(clock);
            finishAutoProfilePhase("scoring");
        }

        @Override
        public void stop() {
            ownedScoring.stop();
        }
    }

    /** Guard for the narrow managed-TeleOp pose-restore window. */
    static final class TeleOpPoseRestoreLifecycle {
        private PoseResetter poseResetter;
        private boolean startBoundaryReached;

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

        void markStartBoundary() {
            startBoundaryReached = true;
        }

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
                        "Cannot restore the Auto pose because Phoenix TeleOp is not declared; "
                                + "call declareTeleOp(program, eligibleScoringTagIds) first"
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

        void clear() {
            poseResetter = null;
            startBoundaryReached = false;
        }
    }
}
