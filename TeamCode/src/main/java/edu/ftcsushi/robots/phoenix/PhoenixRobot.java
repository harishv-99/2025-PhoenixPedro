package edu.ftcsushi.robots.phoenix;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;
import java.util.Set;

import edu.ftcsushi.fw.core.debug.LoopPhaseProfiler;
import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.lifecycle.CleanupActions;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.field.TagLayout;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcTelemetryDebugSink;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcsushi.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcsushi.fw.ftc.vision.VisionReadiness;
import edu.ftcsushi.fw.ftc.input.Gamepads;
import edu.ftcsushi.fw.localization.MotionPredictor;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.localization.PoseResetter;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.robots.phoenix.scoring.PhoenixScoring;
import edu.ftcsushi.robots.phoenix.scoring.PhoenixTargeting;

/**
 * Phoenix composition root.
 *
 * <p>Ordinary TeleOp uses one declarative grammar: construct this object, call
 * {@link #declareTeleOp(RobotProgram, PhoenixProfile, Gamepad, Gamepad, Source)} with its fresh
 * profile and frozen scoring-tag eligibility, restore
 * the optional match snapshot, and register
 * {@link #teleOpPresenter(PhoenixMatchHandoff.RestoreResult)}. The framework then owns the clock,
 * lifecycle callbacks, bindings, output heartbeat, telemetry commit, and fail-stop cleanup.</p>
 *
 * <p>Auto uses the same declaration grammar through {@link #declareAuto(RobotProgram,
 * PhoenixProfile, DriveCommandSink, MotionPredictor, Source, BooleanSource, BooleanSource,
 * Runnable)}. The managed
 * program owns FTC callbacks, the one clock, root Task, output order, telemetry commit, and
 * fail-stop cleanup in both modes. Phoenix still owns its hardware graph and service ordering;
 * alliance selection, Pedro route geometry, targeting policy, and routine composition remain
 * outside this mode-neutral composition root.</p>
 */
public final class PhoenixRobot {

    private static final boolean ENABLE_LOOP_PHASE_PROFILING = false;

    /**
     * Package-private FTC assembly seam for a managed-TeleOp lifecycle test.
     *
     * <p>Ordinary robot code uses the public FTC-resource constructor. This seam is
     * instance-scoped, changes no lifecycle grammar, and still makes {@link PhoenixRobot} construct
     * and privately retain its complete mechanism graph.</p>
     */
    interface TeleOpHardwareAssembly {
        AprilTagVisionLane createVision(
                HardwareMap hardwareMap,
                PhoenixVisionFactory.Config visionConfig
        );

        FtcOdometryAprilTagLocalizationLane createLocalization(
                HardwareMap hardwareMap,
                AprilTagVisionLane vision,
                TagLayout fixedAprilTagLayout,
                FtcOdometryAprilTagLocalizationLane.Config localizationConfig
        );

        PhoenixScoring createScoring(
                HardwareMap hardwareMap,
                PhoenixScoring.Config scoringConfig,
                PhoenixTargeting targeting
        );

        DriveCommandSink createDrive(
                HardwareMap hardwareMap,
                FtcDrives.MecanumConfig driveConfig
        );
    }

    /** Package-private FTC assembly seam for managed-Auto lifecycle tests. */
    interface AutoHardwareAssembly {
        AprilTagVisionLane createVision(
                HardwareMap hardwareMap,
                PhoenixVisionFactory.Config visionConfig
        );

        FtcOdometryAprilTagLocalizationLane createLocalization(
                MotionPredictor motionPredictor,
                AprilTagVisionLane vision,
                TagLayout fixedAprilTagLayout,
                FtcOdometryAprilTagLocalizationLane.EstimatorConfig estimationConfig
        );

        PhoenixScoring createScoring(
                HardwareMap hardwareMap,
                PhoenixScoring.Config scoringConfig,
                PhoenixTargeting targeting
        );
    }

    private static final TeleOpHardwareAssembly FTC_TELEOP_HARDWARE =
            new TeleOpHardwareAssembly() {
                @Override
                public AprilTagVisionLane createVision(
                        HardwareMap hardwareMap,
                        PhoenixVisionFactory.Config visionConfig
                ) {
                    return PhoenixVisionFactory.create(hardwareMap, visionConfig);
                }

                @Override
                public FtcOdometryAprilTagLocalizationLane createLocalization(
                        HardwareMap hardwareMap,
                        AprilTagVisionLane vision,
                        TagLayout fixedAprilTagLayout,
                        FtcOdometryAprilTagLocalizationLane.Config localizationConfig
                ) {
                    return new FtcOdometryAprilTagLocalizationLane(
                            hardwareMap,
                            vision,
                            fixedAprilTagLayout,
                            localizationConfig
                    );
                }

                @Override
                public PhoenixScoring createScoring(
                        HardwareMap hardwareMap,
                        PhoenixScoring.Config scoringConfig,
                        PhoenixTargeting targeting
                ) {
                    return new PhoenixScoring(hardwareMap, scoringConfig, targeting);
                }

                @Override
                public DriveCommandSink createDrive(
                        HardwareMap hardwareMap,
                        FtcDrives.MecanumConfig driveConfig
                ) {
                    return FtcDrives.mecanum(hardwareMap, driveConfig);
                }
            };

    private static final AutoHardwareAssembly FTC_AUTO_HARDWARE =
            new AutoHardwareAssembly() {
                @Override
                public AprilTagVisionLane createVision(
                        HardwareMap hardwareMap,
                        PhoenixVisionFactory.Config visionConfig
                ) {
                    return PhoenixVisionFactory.create(hardwareMap, visionConfig);
                }

                @Override
                public FtcOdometryAprilTagLocalizationLane createLocalization(
                        MotionPredictor motionPredictor,
                        AprilTagVisionLane vision,
                        TagLayout fixedAprilTagLayout,
                        FtcOdometryAprilTagLocalizationLane.EstimatorConfig estimationConfig
                ) {
                    return FtcOdometryAprilTagLocalizationLane.withPredictor(
                            motionPredictor,
                            vision,
                            fixedAprilTagLayout,
                            estimationConfig
                    );
                }

                @Override
                public PhoenixScoring createScoring(
                        HardwareMap hardwareMap,
                        PhoenixScoring.Config scoringConfig,
                        PhoenixTargeting targeting
                ) {
                    return new PhoenixScoring(hardwareMap, scoringConfig, targeting);
                }
            };

    private enum RuntimeMode {
        NEW,
        MANAGED_TELEOP,
        MANAGED_AUTO
    }

    private final HardwareMap hardwareMap;
    private final TeleOpHardwareAssembly teleOpHardwareAssembly;
    private final AutoHardwareAssembly autoHardwareAssembly;
    private final LoopPhaseProfiler loopPhaseProfiler;
    private final TeleOpPoseRestoreLifecycle teleOpPoseRestore =
            new TeleOpPoseRestoreLifecycle();

    private RuntimeMode mode = RuntimeMode.NEW;
    private PhoenixTelemetryPresenter telemetryPresenter;
    private AprilTagVisionLane vision;
    private FtcOdometryAprilTagLocalizationLane localization;
    private PhoenixScoring scoring;
    private PhoenixTargeting targeting;

    // Managed-TeleOp graph and read-only presentation state.
    private PhoenixTeleOpControls teleOpControls;
    private PhoenixDriveAssistService driveAssists;
    private PhoenixReadiness.Result teleOpPoseAssistReadiness;
    private VisionReadiness teleOpVisionReadiness =
            VisionReadiness.notReady("Phoenix TeleOp has not reached FTC START");
    private boolean teleOpStartBoundaryReached;
    private boolean teleOpOrdinaryLoopReached;
    private boolean teleOpDriveStartupReleased;
    private boolean teleOpProfileCycleActive;
    private boolean teleOpPresenterCreated;

    // Managed-Auto presentation state.
    private VisionReadiness autoVisionReadiness =
            VisionReadiness.notReady("Phoenix Auto has not reached FTC START");
    private boolean autoStartBoundaryReached;
    private boolean autoProfileCycleActive;
    private boolean autoPresenterCreated;

    /**
     * Create an unselected Phoenix composition root around the FTC hardware registry.
     *
     * <p>The active mode declaration supplies and synchronously routes one fresh profile. This
     * root never retains the aggregate or Auto-inactive Gamepad inputs.</p>
     */
    public PhoenixRobot(HardwareMap hardwareMap) {
        this(hardwareMap, FTC_TELEOP_HARDWARE, FTC_AUTO_HARDWARE);
    }

    /** FTC-resource host-test construction with replaceable TeleOp assembly; not robot code. */
    PhoenixRobot(HardwareMap hardwareMap,
                 TeleOpHardwareAssembly teleOpHardwareAssembly) {
        this(hardwareMap, teleOpHardwareAssembly, FTC_AUTO_HARDWARE);
    }

    /** FTC-resource lifecycle-test construction with replaceable assemblies; not robot code. */
    PhoenixRobot(HardwareMap hardwareMap,
                 TeleOpHardwareAssembly teleOpHardwareAssembly,
                 AutoHardwareAssembly autoHardwareAssembly) {
        this.hardwareMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
        this.teleOpHardwareAssembly = Objects.requireNonNull(
                teleOpHardwareAssembly,
                "teleOpHardwareAssembly"
        );
        this.autoHardwareAssembly = Objects.requireNonNull(
                autoHardwareAssembly,
                "autoHardwareAssembly"
        );
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
     * <p>This direct declaration is an advanced assembly seam. Its caller must establish exclusive
     * FTC hardware ownership before calling it, including trim-equivalent, case-sensitive motor
     * identity across {@code profile.drive} and {@code profile.scoring}. The ordinary Phoenix
     * TeleOp program performs that graph preflight before reaching this boundary.</p>
     *
     * @param program configuring managed program supplied by the FTC host
     * @param profile fresh complete Phoenix profile consumed only during this declaration
     * @param gamepad1 active driver Gamepad retained by the controls source graph
     * @param gamepad2 active operator Gamepad retained by the controls source graph
     * @param eligibleScoringTagIds non-empty configured scoring-AprilTag subset selected by the
     *                              mode client's frozen alliance policy
     */
    public void declareTeleOp(
            RobotProgram program,
            PhoenixProfile profile,
            Gamepad gamepad1,
            Gamepad gamepad2,
            Source<Set<Integer>> eligibleScoringTagIds
    ) {
        RobotProgram requiredProgram = Objects.requireNonNull(
                program,
                "Phoenix TeleOp program is required"
        );
        PhoenixProfile selectedProfile = Objects.requireNonNull(
                profile,
                "Phoenix TeleOp profile is required"
        );
        Source<Set<Integer>> requiredEligibleScoringTagIds = Objects.requireNonNull(
                eligibleScoringTagIds,
                "Phoenix TeleOp eligibleScoringTagIds source is required"
        );
        beginMode(RuntimeMode.MANAGED_TELEOP, "declareTeleOp");
        loopPhaseProfiler.reset();
        teleOpDriveStartupReleased = false;

        teleOpControls = new PhoenixTeleOpControls(
                Gamepads.create(gamepad1, gamepad2),
                selectedProfile.controls
        );
        teleOpPoseAssistReadiness = PhoenixReadiness.teleOpPoseAssists(
                selectedProfile.calibration
        );

        BooleanSource enabledAutoAim = teleOpControls.autoAimEnabledSource()
                .and(BooleanSource.constant(teleOpPoseAssistReadiness.isAllowed()))
                .memoized();

        vision = Objects.requireNonNull(
                teleOpHardwareAssembly.createVision(hardwareMap, selectedProfile.vision),
                "TeleOp hardware assembly returned null vision"
        );
        ManagedTeleOpVisionService visionService = new ManagedTeleOpVisionService(vision);
        registerServiceOrClean(requiredProgram, visionService, vision::close);

        localization = Objects.requireNonNull(
                teleOpHardwareAssembly.createLocalization(
                        hardwareMap,
                        vision,
                        selectedProfile.fixedAprilTagLayout,
                        selectedProfile.localization
                ),
                "TeleOp hardware assembly returned null localization"
        );
        requiredProgram.service(new ManagedTeleOpLocalizationService(localization));
        teleOpPoseRestore.initialize(localization.globalEstimator());
        telemetryPresenter = createTelemetryPresenter(selectedProfile.localization);

        targeting = createTargeting(
                selectedProfile.targeting,
                selectedProfile.localization,
                selectedProfile.fixedAprilTagLayout,
                requiredEligibleScoringTagIds,
                enabledAutoAim,
                teleOpControls.aimOverrideSource()
        );
        requiredProgram.service(new ManagedTeleOpTargetingService(targeting));

        scoring = Objects.requireNonNull(
                teleOpHardwareAssembly.createScoring(
                        hardwareMap,
                        selectedProfile.scoring,
                        targeting
                ),
                "TeleOp hardware assembly returned null scoring owner"
        );
        ManagedTeleOpScoringOutput scoringOutput =
                new ManagedTeleOpScoringOutput(scoring);
        registerOutputOrClean(requiredProgram, scoringOutput, scoring::stop);

        PhoenixCapabilities capabilities = createCapabilities();
        teleOpControls.bind(requiredProgram.callbackBindings(), capabilities);

        Source<PhoenixCapabilities.ScoringStatus> scoringStatus =
                Source.of(ignoredClock -> scoring.status());
        driveAssists = new PhoenixDriveAssistService(
                selectedProfile.driveAssist,
                teleOpControls.manualDriveSource(),
                teleOpControls.manualTranslateMagnitudeSource(),
                scoringStatus,
                teleOpControls.autoAimEnabledSource(),
                teleOpPoseAssistReadiness.isAllowed(),
                localization.globalEstimator(),
                targeting.aimOverlay()
        );

        DriveCommandSink drive = Objects.requireNonNull(
                teleOpHardwareAssembly.createDrive(hardwareMap, selectedProfile.drive),
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
     * <p>This direct declaration is an advanced assembly seam. The caller owns exclusive hardware
     * assignment for the supplied drive and the Phoenix owners constructed here. In particular,
     * this method cannot infer the FTC motor identities hidden behind an arbitrary
     * {@code autonomousDrive}. The ordinary Phoenix Auto program performs its known-profile
     * scoring-versus-Pedro motor preflight before it creates the Pedro runtime.</p>
     *
     * @param program managed program that immediately owns every completed resource
     * @param profile fresh complete Phoenix profile consumed only during this declaration
     * @param autonomousDrive sole Auto drive heartbeat and stop owner
     * @param motionPredictor predictor owned by the same Auto runtime
     * @param eligibleScoringTagIds non-empty configured scoring-tag subset eligible in this mode
     * @param autoAimEnabledSource policy that enables target selection and aim gating
     * @param aimOverrideSource policy that bypasses aim-readiness gating while true
     * @param applyStartingPose exact-START action that applies the frozen Auto pose
     * @return completed shared capability vocabulary used by the Auto routine builder
     */
    public PhoenixCapabilities declareAuto(
            RobotProgram program,
            PhoenixProfile profile,
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
        PhoenixProfile selectedProfile = Objects.requireNonNull(
                profile,
                "Phoenix Auto profile is required"
        );
        beginMode(RuntimeMode.MANAGED_AUTO, "declareAuto");
        loopPhaseProfiler.reset();

        ManagedAutoService autoService = new ManagedAutoService(
                Objects.requireNonNull(autonomousDrive, "autonomousDrive"),
                Objects.requireNonNull(applyStartingPose, "applyStartingPose")
        );
        registerServiceOrClean(requiredProgram, autoService, autoService::stop);

        vision = Objects.requireNonNull(
                autoHardwareAssembly.createVision(hardwareMap, selectedProfile.vision),
                "Auto hardware assembly returned null vision"
        );
        autoService.attachVision(vision);

        localization = Objects.requireNonNull(
                autoHardwareAssembly.createLocalization(
                        Objects.requireNonNull(motionPredictor, "motionPredictor"),
                        vision,
                        selectedProfile.fixedAprilTagLayout,
                        selectedProfile.localization == null
                                ? null
                                : selectedProfile.localization.estimation
                ),
                "Auto hardware assembly returned null localization"
        );
        autoService.attachLocalization(localization);
        telemetryPresenter = createTelemetryPresenter(selectedProfile.localization);

        targeting = createTargeting(
                selectedProfile.targeting,
                selectedProfile.localization,
                selectedProfile.fixedAprilTagLayout,
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
        autoService.attachTargeting(targeting);

        scoring = Objects.requireNonNull(
                autoHardwareAssembly.createScoring(
                        hardwareMap,
                        selectedProfile.scoring,
                        targeting
                ),
                "Auto hardware assembly returned null scoring owner"
        );
        ManagedAutoScoringOutput scoringOutput = new ManagedAutoScoringOutput(scoring);
        registerOutputOrClean(requiredProgram, scoringOutput, scoring::stop);
        return createCapabilities();
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

    /** Apply one accepted match pose while managed TeleOp is declared and still in INIT. */
    void restoreTeleOpPose(Pose2d fieldToRobotPose) {
        requireMode(RuntimeMode.MANAGED_TELEOP, "restore the Auto-to-TeleOp pose");
        teleOpPoseRestore.restore(fieldToRobotPose);
    }

    private PhoenixTargeting createTargeting(PhoenixTargeting.Config targetingConfig,
                                             FtcOdometryAprilTagLocalizationLane.Config localizationConfig,
                                             TagLayout fixedAprilTagLayout,
                                             Source<Set<Integer>> eligibleScoringTagIds,
                                             BooleanSource autoAimEnabledSource,
                                             BooleanSource aimOverrideSource) {
        FtcOdometryAprilTagLocalizationLane.Config requiredLocalization = Objects.requireNonNull(
                localizationConfig,
                "PhoenixProfile.localization is required for targeting"
        );
        return new PhoenixTargeting(
                targetingConfig,
                requiredLocalization.estimation.aprilTags.fieldPoseSolver,
                vision.tagSensor(),
                vision.cameraMountConfig(),
                localization.globalEstimator(),
                fixedAprilTagLayout,
                eligibleScoringTagIds,
                autoAimEnabledSource,
                aimOverrideSource
        );
    }

    private static PhoenixTelemetryPresenter createTelemetryPresenter(
            FtcOdometryAprilTagLocalizationLane.Config localizationConfig
    ) {
        FtcOdometryAprilTagLocalizationLane.EstimatorConfig estimation = Objects.requireNonNull(
                Objects.requireNonNull(
                        localizationConfig,
                        "PhoenixProfile.localization is required for presentation"
                ).estimation,
                "PhoenixProfile.localization.estimation is required for presentation"
        );
        return new PhoenixTelemetryPresenter(
                estimation.correctedEstimatorMode,
                Objects.requireNonNull(
                        estimation.correctionSource,
                        "PhoenixProfile.localization.estimation.correctionSource is required "
                                + "for presentation"
                ).mode
        );
    }

    private PhoenixCapabilities createCapabilities() {
        if (scoring == null || targeting == null) {
            throw new IllegalStateException(
                    "Phoenix cannot create capabilities before scoring and targeting exist"
            );
        }
        return new PhoenixCapabilities(scoring, targeting);
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

        PhoenixCapabilities.ScoringStatus scoringStatus = scoring.status();
        PhoenixCapabilities.TargetingStatus targetingStatus = targeting.status();
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

        PhoenixCapabilities.ScoringStatus scoringStatus = scoring.status();
        PhoenixCapabilities.TargetingStatus targetingStatus = targeting.status();
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
            updateTeleOpDriveStartupGate(clock);
        }

        @Override
        public void update(LoopClock clock) {
            ownedLocalization.update(clock);
            updateTeleOpDriveStartupGate(clock);
            finishTeleOpProfilePhase("localization");
        }

        @Override
        public void stop() {
            // The lane's stable resources are stopped by their explicit vision/output owners.
        }
    }

    /** Managed targeting publisher and reset owner. */
    private final class ManagedTeleOpTargetingService implements RobotProgram.Service {
        private final PhoenixTargeting ownedTargeting;

        private ManagedTeleOpTargetingService(PhoenixTargeting ownedTargeting) {
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
        private final PhoenixScoring ownedScoring;

        private ManagedTeleOpScoringOutput(PhoenixScoring ownedScoring) {
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
            ownedSink.drive(teleOpDriveStartupReleased ? signal : DriveSignal.zero());
            finishTeleOpProfilePhase("drive");
        }

        @Override
        public void stop() {
            ownedSink.stop();
        }
    }

    /**
     * Releases ordinary TeleOp drive once after the owned Pinpoint startup reset publishes one
     * current finite measured pose. Later localization loss disables localization-dependent
     * assists through their own evidence checks, but does not take robot-centric manual drive away.
     */
    private void updateTeleOpDriveStartupGate(LoopClock clock) {
        if (teleOpDriveStartupReleased) {
            return;
        }
        PoseEstimate estimate = localization.predictor().getEstimate();
        if (estimate == null || !estimate.hasPose || estimate.fieldToRobotPose == null) {
            return;
        }
        if (!Double.isFinite(estimate.fieldToRobotPose.xInches)
                || !Double.isFinite(estimate.fieldToRobotPose.yInches)
                || !Double.isFinite(estimate.fieldToRobotPose.yawRad)) {
            return;
        }
        try {
            teleOpDriveStartupReleased = estimate.timestamp != null
                    && estimate.timestamp.isFresh(clock, 0.0);
        } catch (IllegalArgumentException incompatibleTimestamp) {
            teleOpDriveStartupReleased = false;
        }
    }

    /** Stable managed owner of the complete upstream Auto update and cleanup order. */
    private final class ManagedAutoService implements RobotProgram.Service {
        private final DriveCommandSink autoDrive;
        private final Runnable applyStartingPose;
        private AprilTagVisionLane ownedVision;
        private FtcOdometryAprilTagLocalizationLane ownedLocalization;
        private PhoenixTargeting ownedTargeting;
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

        private void attachTargeting(PhoenixTargeting targeting) {
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
            PhoenixTargeting targetingToStop = ownedTargeting;
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
        private final PhoenixScoring ownedScoring;

        private ManagedAutoScoringOutput(PhoenixScoring ownedScoring) {
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
                                + "call declareTeleOp(program, profile, gamepad1, gamepad2, "
                                + "eligibleScoringTagIds) first"
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
