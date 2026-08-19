package edu.ftcphoenix.robots.phoenix.scoring;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.math.InterpolatingTable1D;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveOverlay;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.drive.DriveOverlayOutput;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidance;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceQuery;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceSpec;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceStatus;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcphoenix.fw.field.SimpleTagLayout;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.apriltag.FixedTagFieldPoseSolver;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionPolicies;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionSource;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelections;
import edu.ftcphoenix.fw.spatial.References;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.robots.phoenix.PhoenixAlliance;
import edu.ftcphoenix.robots.phoenix.PhoenixCapabilities;

/**
 * Shared targeting service for Phoenix scoring.
 *
 * <p>This class owns selected-tag policy, the auto-aim guidance query, and range-based shot
 * suggestions. Higher-level code reads one cached {@link PhoenixCapabilities.TargetingStatus}
 * snapshot per loop instead
 * of re-sampling the stateful guidance query in multiple places. The robot supplies which
 * configured scoring tags are eligible; that set is sampled and frozen before this service's
 * first detection selection.</p>
 */
public final class PhoenixTargeting implements PhoenixCapabilities.Targeting {

    /** Mutable data-only alliance targeting, aiming, and shot-selection configuration. */
    public static final class Config {
        public int redAllianceScoringTagId;
        public int blueAllianceScoringTagId;
        public LinkedHashMap<Integer, ScoringTarget> scoringTargets;
        public double aimToleranceDeg;
        public double aimKp;
        public double aimMaxOmegaCmd;
        public double aimReadyToleranceDeg;
        public double aimReadyDebounceSec;
        public double aimMinOmegaCmd;
        public double selectionMaxAgeSec;
        public double selectionReacquireSec;
        public AimOffset defaultAimOffset;
        public InterpolatingTable1D shotVelocityTable;

        private Config() {
            // Use defaults() to start from the complete Phoenix software baseline.
        }

        /** Returns a fresh software baseline; this does not establish field calibration. */
        public static Config defaults() {
            Config config = new Config();
            config.redAllianceScoringTagId = 24;
            config.blueAllianceScoringTagId = 20;
            config.scoringTargets = new LinkedHashMap<Integer, ScoringTarget>();
            config.scoringTargets.put(
                    20,
                    new ScoringTarget("Blue scoring target", new AimOffset(0.0, 0.0))
            );
            config.scoringTargets.put(
                    24,
                    new ScoringTarget("Red scoring target", new AimOffset(0.0, 0.0))
            );
            config.aimToleranceDeg = 0.25;
            config.aimKp = 1.5;
            config.aimMaxOmegaCmd = 0.80;
            config.aimReadyToleranceDeg = 0.50;
            config.aimReadyDebounceSec = 0.05;
            config.aimMinOmegaCmd = 0.05;
            config.selectionMaxAgeSec = 0.50;
            config.selectionReacquireSec = 0.20;
            config.defaultAimOffset = new AimOffset(0.0, 0.0);
            config.shotVelocityTable = PhoenixShotVelocityCalibration.currentTable();
            return config;
        }

        /** Returns the configured scoring tag for the selected match alliance. */
        public int scoringTagIdFor(PhoenixAlliance alliance) {
            switch (Objects.requireNonNull(alliance, "alliance")) {
                case RED:
                    return redAllianceScoringTagId;
                case BLUE:
                    return blueAllianceScoringTagId;
                default:
                    throw new IllegalArgumentException(
                            "Unsupported Phoenix alliance: " + alliance
                    );
            }
        }

        private static Config rawCopyOf(Config source) {
            Config copy = new Config();
            copy.redAllianceScoringTagId = source.redAllianceScoringTagId;
            copy.blueAllianceScoringTagId = source.blueAllianceScoringTagId;
            copy.scoringTargets = copyScoringTargets(source.scoringTargets);
            copy.aimToleranceDeg = source.aimToleranceDeg;
            copy.aimKp = source.aimKp;
            copy.aimMaxOmegaCmd = source.aimMaxOmegaCmd;
            copy.aimReadyToleranceDeg = source.aimReadyToleranceDeg;
            copy.aimReadyDebounceSec = source.aimReadyDebounceSec;
            copy.aimMinOmegaCmd = source.aimMinOmegaCmd;
            copy.selectionMaxAgeSec = source.selectionMaxAgeSec;
            copy.selectionReacquireSec = source.selectionReacquireSec;
            copy.defaultAimOffset = copyAimOffset(source.defaultAimOffset);
            copy.shotVelocityTable = source.shotVelocityTable;
            return copy;
        }
    }

    /** Raw target-catalog value; the map key is the target's sole AprilTag identity. */
    public static final class ScoringTarget {
        public String label;
        public AimOffset aimOffset;

        /** Creates one raw target definition without replacing invalid authoring evidence. */
        public ScoringTarget(String label, AimOffset aimOffset) {
            this.label = label;
            this.aimOffset = aimOffset;
        }
    }

    /** Raw tag-local forward/left aim offset in inches. */
    public static final class AimOffset {
        public double forwardInches;
        public double leftInches;

        /** Creates one raw tag-local offset without validating or normalizing its values. */
        public AimOffset(double forwardInches, double leftInches) {
            this.forwardInches = forwardInches;
            this.leftInches = leftInches;
        }
    }

    /** One session's exact eligible selector and guidance graph. */
    private static final class AimRuntime {
        private final Set<Integer> eligibleTagIds;
        private final TagLayout fieldTagLayout;
        private final Map<Integer, Pose2d> fieldToAimPoints;
        private final TagSelectionSource selection;
        private final DriveGuidancePlan plan;
        private final DriveGuidanceQuery query;

        private AimRuntime(Set<Integer> eligibleTagIds,
                           TagLayout fieldTagLayout,
                           Map<Integer, Pose2d> fieldToAimPoints,
                           TagSelectionSource selection,
                           DriveGuidancePlan plan) {
            this.eligibleTagIds = Objects.requireNonNull(eligibleTagIds, "eligibleTagIds");
            this.fieldTagLayout = Objects.requireNonNull(fieldTagLayout, "fieldTagLayout");
            this.fieldToAimPoints = Objects.requireNonNull(
                    fieldToAimPoints,
                    "fieldToAimPoints"
            );
            this.selection = Objects.requireNonNull(selection, "selection");
            this.plan = Objects.requireNonNull(plan, "plan");
            this.query = plan.query();
        }
    }

    /**
     * Complete immutable result of the fallible targeting calculation for one loop.
     *
     * <p>Keeping this separate from readiness debounce means every query, lookup, interpolation,
     * and pose calculation succeeds before readiness state advances or this service publishes a
     * new {@link PhoenixCapabilities.TargetingStatus}. Upstream children still own their
     * independently completed
     * observations.</p>
     */
    private static final class TargetingCalculation {
        final boolean autoAimEnabled;
        final boolean aimOverride;
        final boolean rawAimReady;
        final TagSelectionResult selection;
        final DriveGuidanceStatus aimStatus;
        final String targetLabel;
        final double aimOffsetForwardInches;
        final double aimOffsetLeftInches;
        final boolean hasSuggestedVelocity;
        final double suggestedVelocityNative;
        final Pose3d fieldToSelectedTag;
        final Pose2d fieldToAimPoint;

        TargetingCalculation(boolean autoAimEnabled,
                             boolean aimOverride,
                             boolean rawAimReady,
                             TagSelectionResult selection,
                             DriveGuidanceStatus aimStatus,
                             String targetLabel,
                             double aimOffsetForwardInches,
                             double aimOffsetLeftInches,
                             boolean hasSuggestedVelocity,
                             double suggestedVelocityNative,
                             Pose3d fieldToSelectedTag,
                             Pose2d fieldToAimPoint) {
            this.autoAimEnabled = autoAimEnabled;
            this.aimOverride = aimOverride;
            this.rawAimReady = rawAimReady;
            this.selection = selection;
            this.aimStatus = aimStatus;
            this.targetLabel = targetLabel;
            this.aimOffsetForwardInches = aimOffsetForwardInches;
            this.aimOffsetLeftInches = aimOffsetLeftInches;
            this.hasSuggestedVelocity = hasSuggestedVelocity;
            this.suggestedVelocityNative = suggestedVelocityNative;
            this.fieldToSelectedTag = fieldToSelectedTag;
            this.fieldToAimPoint = fieldToAimPoint;
        }

        PhoenixCapabilities.TargetingStatus toStatus(double aimToleranceDeg,
                                                     double aimReadyToleranceDeg,
                                                     boolean aimReady) {
            return new PhoenixCapabilities.TargetingStatus(
                    autoAimEnabled,
                    aimReady,
                    aimReady || aimOverride,
                    aimOverride,
                    aimToleranceDeg,
                    aimReadyToleranceDeg,
                    selection,
                    aimStatus,
                    targetLabel,
                    aimOffsetForwardInches,
                    aimOffsetLeftInches,
                    hasSuggestedVelocity,
                    suggestedVelocityNative,
                    fieldToSelectedTag,
                    fieldToAimPoint
            );
        }
    }

    private final Config cfg;
    private final FixedTagFieldPoseSolver fieldPoseSolver;
    private final AprilTagSensor tagSensor;
    private final CameraMountConfig cameraMountConfig;
    private final AbsolutePoseEstimator globalAbsolutePoseEstimator;
    private final TagLayout fieldTagLayout;
    private final Source<Set<Integer>> eligibleScoringTagIds;
    private final BooleanSource autoAimEnabled;
    private final BooleanSource aimOverrideInput;
    private final double aimReadyToleranceRad;
    private final Source<TargetingCalculation> targetingCalculation;
    private final BooleanSource stableAimReady;
    private final Source<PhoenixCapabilities.TargetingStatus> statusSource;
    private final BooleanSource aimOkToShootSource;
    private final BooleanSource aimOverrideSource;
    private AimRuntime aimRuntime;
    private long aimRuntimeGeneration;
    private boolean targetingCalculationInProgress;
    private PhoenixCapabilities.TargetingStatus latestStatus;

    /**
     * Creates the shared Phoenix scoring-targeting service.
     *
     * @param config                      auto-aim configuration snapshot copied for local ownership
     * @param aprilTagFieldPoseConfig     AprilTag field-pose solve configuration captured by this
     *                                    service before it builds guidance plans
     * @param tagSensor                   shared AprilTag sensor used for selection and guidance
     * @param cameraMountConfig           fixed camera extrinsics for the current robot profile
     * @param globalAbsolutePoseEstimator current global pose-estimator lane used by adaptive guidance
     * @param fieldTagLayout              fixed field tag layout for the current game
     * @param eligibleScoringTagIds       robot-owned source of the non-empty configured scoring-tag
     *                                    subset eligible in the current mode; sampled once and
     *                                    defensively frozen before the first detection selection
     * @param autoAimEnabled              driver enable source that activates sticky target selection and the aim overlay
     * @param aimOverrideInput            driver override source that bypasses aim readiness gates when held
     */
    public PhoenixTargeting(Config config,
                            FixedTagFieldPoseSolver.Config aprilTagFieldPoseConfig,
                            AprilTagSensor tagSensor,
                            CameraMountConfig cameraMountConfig,
                            AbsolutePoseEstimator globalAbsolutePoseEstimator,
                            TagLayout fieldTagLayout,
                            Source<Set<Integer>> eligibleScoringTagIds,
                            BooleanSource autoAimEnabled,
                            BooleanSource aimOverrideInput) {
        this.cfg = captureConfig(config);
        this.fieldPoseSolver = new FixedTagFieldPoseSolver(
                Objects.requireNonNull(aprilTagFieldPoseConfig, "aprilTagFieldPoseConfig")
        );
        this.tagSensor = Objects.requireNonNull(tagSensor, "tagSensor");
        this.cameraMountConfig = Objects.requireNonNull(cameraMountConfig, "cameraMountConfig");
        this.globalAbsolutePoseEstimator = Objects.requireNonNull(
                globalAbsolutePoseEstimator,
                "globalAbsolutePoseEstimator"
        );
        this.fieldTagLayout = Objects.requireNonNull(fieldTagLayout, "fieldTagLayout");
        this.eligibleScoringTagIds = Objects.requireNonNull(
                eligibleScoringTagIds,
                "PhoenixTargeting eligibleScoringTagIds source is required"
        );
        this.autoAimEnabled = Objects.requireNonNull(autoAimEnabled, "autoAimEnabled").memoized();
        this.aimOverrideInput = Objects.requireNonNull(aimOverrideInput, "aimOverrideInput").memoized();
        this.aimReadyToleranceRad = Math.toRadians(this.cfg.aimReadyToleranceDeg);

        targetingCalculation = Source.of(this::calculateTargeting).memoized();
        stableAimReady = targetingCalculation
                .mapToBoolean(calculation -> calculation.rawAimReady)
                .debouncedOn(this.cfg.aimReadyDebounceSec);
        statusSource = Source.of(clock -> {
            TargetingCalculation calculation = targetingCalculation.get(clock);
            boolean aimReady = !calculation.autoAimEnabled
                    || stableAimReady.getAsBoolean(clock);
            return calculation.toStatus(
                    this.cfg.aimToleranceDeg,
                    this.cfg.aimReadyToleranceDeg,
                    aimReady
            );
        }).memoized();

        latestStatus = initialStatus();
        aimOkToShootSource = BooleanSource.of(() -> status().aimOkToShoot);
        aimOverrideSource = BooleanSource.of(() -> status().aimOverride);
    }

    /**
     * Returns a fresh auto-aim overlay built from this service's shared plan.
     *
     * <p>Call this during initialization and keep the returned overlay for the lifetime of the
     * owning drive stack. The wrapper resolves its private guidance delegate after the managed
     * targeting service has frozen eligibility on its first active update. Each overlay has its
     * own controller runtime state.</p>
     *
     * @return new omega-only/plan-configured drive overlay for scoring auto-aim
     */
    public DriveOverlay aimOverlay() {
        return new DeferredAimOverlay();
    }

    /**
     * Returns a task wrapper around Phoenix's shared aim plan.
     *
     * <p>This is the autonomous counterpart to {@link #aimOverlay()}: it reuses the exact same
     * selected-target policy, AprilTag/localization resolution, and controller tuning, but drives a
     * supplied {@link DriveCommandSink} directly until the aim task reaches its tolerance.</p>
     *
     * <p>The mutable task configuration is copied when this method is called, before construction
     * of the deferred inner task. Later caller mutation therefore cannot change the task that will
     * start. {@link DriveGuidanceTask} remains the owner of configuration validation and performs
     * it at the inner aim Task's own start boundary, before that inner Task invokes the drive sink.
     * In a larger routine this boundary may follow earlier route phases that used the same sink.</p>
     *
     * @param driveSink sink used to apply the aim command
     * @param cfg       task-level tolerances/timeouts; when {@code null}, defaults are used
     * @return task that turns the robot toward the currently selected Phoenix scoring target
     */
    @Override
    public Task aimTask(DriveCommandSink driveSink, DriveGuidanceTask.Config cfg) {
        DriveCommandSink requiredDriveSink = Objects.requireNonNull(driveSink, "driveSink");
        final DriveGuidanceTask.Config taskConfig = copyAimTaskConfig(cfg);
        return Tasks.buildAtStart(
                "Phoenix scoring aim",
                () -> requireAimRuntime("start the scoring aim Task")
                        .plan
                        .task(requiredDriveSink, taskConfig)
        );
    }

    /**
     * Returns a private field-for-field snapshot for the deferred aim-task factory.
     *
     * <p>Validation deliberately remains at the framework task-construction boundary. This helper
     * only prevents the mutable caller object from becoming live configuration retained by the
     * start-time supplier.</p>
     */
    private static DriveGuidanceTask.Config copyAimTaskConfig(DriveGuidanceTask.Config cfg) {
        DriveGuidanceTask.Config source = cfg != null ? cfg : new DriveGuidanceTask.Config();
        DriveGuidanceTask.Config copy = new DriveGuidanceTask.Config();
        copy.positionTolInches = source.positionTolInches;
        copy.headingTolRad = source.headingTolRad;
        copy.timeoutSec = source.timeoutSec;
        copy.maxNoGuidanceSec = source.maxNoGuidanceSec;
        copy.requestedMask = source.requestedMask;
        return copy;
    }

    /**
     * Returns a boolean source that reflects whether targeting policy currently allows feeding.
     *
     * @return source that becomes true when aim is ready or the driver is overriding the gate
     */
    public BooleanSource aimOkToShootSource() {
        return aimOkToShootSource;
    }

    /**
     * Returns a boolean source that reflects the driver's current aim-override request.
     *
     * @return source that is true while override is being held
     */
    public BooleanSource aimOverrideSource() {
        return aimOverrideSource;
    }

    /**
     * Updates the successfully published targeting snapshot for the current loop cycle.
     *
     * @param clock shared loop clock for the active OpMode cycle
     */
    public void update(LoopClock clock) {
        PhoenixCapabilities.TargetingStatus nextStatus =
                statusSource.get(Objects.requireNonNull(clock, "clock"));
        latestStatus = nextStatus;
    }

    /**
     * Returns the latest targeting snapshot successfully published by {@link #update(LoopClock)}.
     *
     * <p>This read is clockless and side-effect-free. If an update fails, the preceding published
     * snapshot remains available and the transactional source graph remains eligible for a
     * same-cycle retry.</p>
     *
     * @return latest successfully published targeting snapshot, or the conservative initial
     * snapshot before the first successful update
     */
    @Override
    public PhoenixCapabilities.TargetingStatus status() {
        return latestStatus;
    }

    private TargetingCalculation calculateTargeting(LoopClock clock) {
        if (targetingCalculationInProgress) {
            throw new IllegalStateException(
                    "PhoenixTargeting cannot calculate or reset reentrantly while its eligibility, "
                            + "selection, or guidance graph is being evaluated."
            );
        }
        targetingCalculationInProgress = true;
        try {
            boolean autoAimNow = autoAimEnabled.getAsBoolean(clock);
            boolean aimOverrideNow = aimOverrideInput.getAsBoolean(clock);
            AimRuntime runtime = aimRuntime;
            if (runtime == null) {
                AimRuntime candidate = buildAimRuntime(clock);

                // Commit the complete immutable runtime before the first stateful sensor/selector
                // read. If that later read fails, a same-cycle retry keeps the exact frozen
                // eligibility and selector instead of rebuilding from a possibly changed source.
                aimRuntime = candidate;
                aimRuntimeGeneration++;
                runtime = candidate;
            }
            TagSelectionResult selection = Objects.requireNonNull(
                    runtime.selection.get(clock),
                    "Phoenix scoring tag selector returned null"
            );
            return calculateTargeting(
                    clock,
                    runtime,
                    selection,
                    autoAimNow,
                    aimOverrideNow
            );
        } finally {
            targetingCalculationInProgress = false;
        }
    }

    private TargetingCalculation calculateTargeting(
            LoopClock clock,
            AimRuntime runtime,
            TagSelectionResult selection,
            boolean autoAimNow,
            boolean aimOverrideNow
    ) {
        if (selection == null) {
            selection = TagSelectionResult.none(Collections.emptySet());
        }

        DriveGuidanceStatus aimStatus = runtime.query.sample(clock, DriveOverlayMask.OMEGA_ONLY);
        boolean hasAimReference = !autoAimNow || selection.hasSelection;
        boolean rawAimReady = !autoAimNow || (
                hasAimReference
                        && aimStatus != null
                        && aimStatus.hasOmegaError
                        && aimStatus.omegaWithin(aimReadyToleranceRad)
        );

        ScoringTarget target = selection.hasSelection
                ? cfg.scoringTargets.get(selection.selectedTagId)
                : defaultTarget();

        boolean hasFreshTargetObservation = selection.hasFreshSelectedObservation
                && selection.selectedObservation.hasTarget;
        double candidateVelocityNative = hasFreshTargetObservation
                ? cfg.shotVelocityTable.interpolate(
                        selection.selectedObservation.cameraRangeInches()
                )
                : Double.NaN;
        boolean hasSuggestedVelocity = Double.isFinite(candidateVelocityNative);
        double suggestedVelocityNative = hasSuggestedVelocity
                ? candidateVelocityNative
                : Double.NaN;

        Pose3d fieldToSelectedTag = null;
        Pose2d fieldToAimPoint = null;
        if (selection.hasSelection) {
            fieldToSelectedTag = runtime.fieldTagLayout.requireFieldToTagPose(
                    selection.selectedTagId
            );
            fieldToAimPoint = runtime.fieldToAimPoints.get(selection.selectedTagId);
        }

        return new TargetingCalculation(
                autoAimNow,
                aimOverrideNow,
                rawAimReady,
                selection,
                aimStatus,
                target.label,
                target.aimOffset.forwardInches,
                target.aimOffset.leftInches,
                hasSuggestedVelocity,
                suggestedVelocityNative,
                fieldToSelectedTag,
                fieldToAimPoint
        );
    }

    /**
     * Clears owned selector, query, readiness, and publication state so the next loop starts a
     * fresh targeting session.
     *
     * <p>The owning robot calls this only after detaching the complete graph during shutdown;
     * sampling and reset must not overlap.</p>
     */
    public void reset() {
        if (targetingCalculationInProgress) {
            throw new IllegalStateException(
                    "PhoenixTargeting cannot reset while its eligibility, selection, or guidance "
                            + "graph is being evaluated. Detach the complete drive/targeting graph "
                            + "before shutdown reset."
            );
        }

        AimRuntime runtime = aimRuntime;
        if (runtime != null) {
            runtime.selection.reset();
            runtime.query.reset();
        } else {
            tagSensor.reset();
            autoAimEnabled.reset();
        }
        eligibleScoringTagIds.reset();
        aimRuntime = null;
        aimRuntimeGeneration++;
        aimOverrideInput.reset();
        stableAimReady.reset();
        statusSource.reset();
        latestStatus = initialStatus();
    }

    private PhoenixCapabilities.TargetingStatus initialStatus() {
        ScoringTarget target = defaultTarget();
        return new PhoenixCapabilities.TargetingStatus(
                false,
                false,
                false,
                false,
                cfg.aimToleranceDeg,
                cfg.aimReadyToleranceDeg,
                TagSelectionResult.none(Collections.<Integer>emptySet()),
                null,
                target.label,
                target.aimOffset.forwardInches,
                target.aimOffset.leftInches,
                false,
                Double.NaN,
                null,
                null
        );
    }

    private ScoringTarget defaultTarget() {
        return new ScoringTarget("No target", cfg.defaultAimOffset);
    }

    /** Captures raw catalog evidence, then validates only the always-active targeting policy. */
    private static Config captureConfig(Config config) {
        Config source = Objects.requireNonNull(
                config,
                "PhoenixTargeting.Config is required"
        );
        Config copy = Config.rawCopyOf(source);

        requireFiniteRange("aimToleranceDeg", copy.aimToleranceDeg, 0.0, 180.0);
        requireFiniteNonNegative("aimKp", copy.aimKp);
        requireFiniteRange("aimMaxOmegaCmd", copy.aimMaxOmegaCmd, 0.0, 1.0);
        if (!Double.isFinite(copy.aimReadyToleranceDeg)
                || copy.aimReadyToleranceDeg < copy.aimToleranceDeg
                || copy.aimReadyToleranceDeg > 180.0) {
            throw invalidConfig(
                    "aimReadyToleranceDeg",
                    "finite, >= aimToleranceDeg, and <= 180.0",
                    copy.aimReadyToleranceDeg
            );
        }
        requireFiniteNonNegative("aimReadyDebounceSec", copy.aimReadyDebounceSec);
        if (!Double.isFinite(copy.aimMinOmegaCmd)
                || copy.aimMinOmegaCmd < 0.0
                || copy.aimMinOmegaCmd > copy.aimMaxOmegaCmd) {
            throw invalidConfig(
                    "aimMinOmegaCmd",
                    "finite and in [0.0, aimMaxOmegaCmd]",
                    copy.aimMinOmegaCmd
            );
        }
        if (copy.aimMinOmegaCmd > 0.0 && !(copy.aimKp > 0.0)) {
            throw new IllegalArgumentException(
                    "PhoenixTargeting.Config.aimKp must be > 0 when aimMinOmegaCmd is > 0, got "
                            + copy.aimKp + " and " + copy.aimMinOmegaCmd + "."
            );
        }
        requireFiniteNonNegative("selectionMaxAgeSec", copy.selectionMaxAgeSec);
        requireFiniteNonNegative("selectionReacquireSec", copy.selectionReacquireSec);
        if (copy.defaultAimOffset == null) {
            throw invalidConfig("defaultAimOffset", "non-null", null);
        }
        requireFinite(
                "defaultAimOffset.forwardInches",
                copy.defaultAimOffset.forwardInches
        );
        requireFinite(
                "defaultAimOffset.leftInches",
                copy.defaultAimOffset.leftInches
        );
        if (copy.shotVelocityTable == null) {
            throw invalidConfig("shotVelocityTable", "non-null", null);
        }
        return copy;
    }

    private static LinkedHashMap<Integer, ScoringTarget> copyScoringTargets(
            LinkedHashMap<Integer, ScoringTarget> source
    ) {
        if (source == null) {
            return null;
        }
        LinkedHashMap<Integer, ScoringTarget> copy =
                new LinkedHashMap<Integer, ScoringTarget>();
        for (Map.Entry<Integer, ScoringTarget> entry : source.entrySet()) {
            ScoringTarget target = entry.getValue();
            copy.put(
                    entry.getKey(),
                    target == null
                            ? null
                            : new ScoringTarget(target.label, copyAimOffset(target.aimOffset))
            );
        }
        return copy;
    }

    private static AimOffset copyAimOffset(AimOffset source) {
        return source == null
                ? null
                : new AimOffset(source.forwardInches, source.leftInches);
    }

    private static void requireFiniteRange(String fieldName,
                                           double value,
                                           double minimum,
                                           double maximum) {
        if (!Double.isFinite(value) || value < minimum || value > maximum) {
            throw invalidConfig(
                    fieldName,
                    "finite and in [" + minimum + ", " + maximum + "]",
                    value
            );
        }
    }

    private static void requireFiniteNonNegative(String fieldName, double value) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw invalidConfig(fieldName, "finite and >= 0", value);
        }
    }

    private static void requireFinite(String fieldName, double value) {
        if (!Double.isFinite(value)) {
            throw invalidConfig(fieldName, "finite", value);
        }
    }

    private static void requireFiniteFieldPose(int tagId, Pose3d pose) {
        if (pose == null) {
            throw new IllegalArgumentException(
                    "PhoenixTargeting eligibleScoringTagIds contains tag id " + tagId
                            + " without a pose in the fixed field layout. Managed readiness must "
                            + "block START until the selected alliance scoring tag is fixed."
            );
        }
        requireFinite("fixedFieldPose[" + tagId + "].xInches", pose.xInches);
        requireFinite("fixedFieldPose[" + tagId + "].yInches", pose.yInches);
        requireFinite("fixedFieldPose[" + tagId + "].zInches", pose.zInches);
        requireFinite("fixedFieldPose[" + tagId + "].yawRad", pose.yawRad);
        requireFinite("fixedFieldPose[" + tagId + "].pitchRad", pose.pitchRad);
        requireFinite("fixedFieldPose[" + tagId + "].rollRad", pose.rollRad);
    }

    private static IllegalArgumentException invalidConfig(String fieldName,
                                                          String requirement,
                                                          Object value) {
        return new IllegalArgumentException(
                "PhoenixTargeting.Config." + fieldName + " must be " + requirement + ", got "
                        + value + "."
        );
    }

    private AimRuntime buildAimRuntime(LoopClock clock) {
        Set<Integer> suppliedEligibleTagIds = eligibleScoringTagIds.get(clock);
        if (suppliedEligibleTagIds == null) {
            throw new IllegalArgumentException(
                    "PhoenixTargeting eligibleScoringTagIds source returned null; return the "
                            + "non-empty selected-mode subset of PhoenixTargeting.Config."
            );
        }
        if (suppliedEligibleTagIds.isEmpty()) {
            throw new IllegalArgumentException(
                    "PhoenixTargeting eligibleScoringTagIds must contain at least one configured "
                            + "scoring tag."
            );
        }

        LinkedHashSet<Integer> eligibleSnapshot = new LinkedHashSet<Integer>();
        for (Integer tagId : suppliedEligibleTagIds) {
            if (tagId == null) {
                throw new IllegalArgumentException(
                        "PhoenixTargeting eligibleScoringTagIds must not contain null."
                );
            }
            if (tagId < 0) {
                throw new IllegalArgumentException(
                        "PhoenixTargeting eligibleScoringTagIds must contain only non-negative "
                                + "tag ids, got " + tagId + "."
                );
            }
            eligibleSnapshot.add(tagId);
        }

        if (cfg.scoringTargets == null) {
            throw new IllegalArgumentException(
                    "PhoenixTargeting.Config.scoringTargets is required for the selected "
                            + "eligibleScoringTagIds. Managed readiness must block START until "
                            + "the selected alliance target exists."
            );
        }

        SimpleTagLayout selectedFieldLayout = new SimpleTagLayout();
        LinkedHashMap<Integer, References.TagPointOffset> aimOffsets =
                new LinkedHashMap<Integer, References.TagPointOffset>();
        LinkedHashMap<Integer, Pose2d> fieldToAimPoints =
                new LinkedHashMap<Integer, Pose2d>();
        for (Integer tagId : eligibleSnapshot) {
            ScoringTarget target = cfg.scoringTargets.get(tagId);
            if (target == null) {
                throw new IllegalArgumentException(
                        "PhoenixTargeting eligibleScoringTagIds contains tag id " + tagId
                                + " without a target in PhoenixTargeting.Config.scoringTargets."
                );
            }
            if (target.label == null || target.label.trim().isEmpty()) {
                throw new IllegalArgumentException(
                        "PhoenixTargeting.Config.scoringTargets[" + tagId
                                + "].label must be non-blank, got " + target.label + "."
                );
            }
            AimOffset aimOffset = target.aimOffset;
            if (aimOffset == null) {
                throw new IllegalArgumentException(
                        "PhoenixTargeting.Config.scoringTargets[" + tagId
                                + "].aimOffset is required, got null."
                );
            }
            requireFinite(
                    "scoringTargets[" + tagId + "].aimOffset.forwardInches",
                    aimOffset.forwardInches
            );
            requireFinite(
                    "scoringTargets[" + tagId + "].aimOffset.leftInches",
                    aimOffset.leftInches
            );

            Pose3d fieldToTag = fieldTagLayout.getFieldToTagPose(tagId);
            requireFiniteFieldPose(tagId, fieldToTag);
            Pose2d fieldToAimPoint = new Pose2d(
                    fieldToTag.xInches,
                    fieldToTag.yInches,
                    fieldToTag.yawRad
            ).then(new Pose2d(aimOffset.forwardInches, aimOffset.leftInches, 0.0));
            requireFinite(
                    "composed fieldToAimPoint[" + tagId + "].xInches",
                    fieldToAimPoint.xInches
            );
            requireFinite(
                    "composed fieldToAimPoint[" + tagId + "].yInches",
                    fieldToAimPoint.yInches
            );
            requireFinite(
                    "composed fieldToAimPoint[" + tagId + "].headingRad",
                    fieldToAimPoint.headingRad
            );

            selectedFieldLayout.addPose(tagId, fieldToTag);
            aimOffsets.put(
                    tagId,
                    References.pointOffset(aimOffset.forwardInches, aimOffset.leftInches)
            );
            fieldToAimPoints.put(tagId, fieldToAimPoint);
        }

        Set<Integer> frozenEligibleTagIds = Collections.unmodifiableSet(eligibleSnapshot);
        TagLayout scoringTagLayout = selectedFieldLayout;
        TagSelectionSource scoringSelection = TagSelections.from(tagSensor)
                .among(frozenEligibleTagIds)
                .freshWithinSec(cfg.selectionMaxAgeSec)
                .choose(TagSelectionPolicies.smallestAbsRobotBearing(cameraMountConfig))
                .stickyWhen(autoAimEnabled)
                .reacquireAfterLossSec(cfg.selectionReacquireSec)
                .build();

        DriveGuidancePlan.Tuning aimTuning = DriveGuidancePlan.Tuning.defaults()
                .withAimKp(cfg.aimKp)
                .withMaxOmegaCmd(cfg.aimMaxOmegaCmd)
                .withMinOmegaCmd(cfg.aimMinOmegaCmd)
                .withAimDeadbandRad(Math.toRadians(cfg.aimToleranceDeg));

        DriveGuidancePlan aimPlan = DriveGuidance.plan()
                .faceTo()
                .point(References.relativeToSelectedTagPoint(
                        scoringSelection,
                        aimOffsets
                ))
                .solveWith()
                .adaptive()
                .localization(globalAbsolutePoseEstimator)
                .aprilTags(tagSensor, cameraMountConfig)
                .aprilTagMaxAgeSec(cfg.selectionMaxAgeSec)
                .aprilTagFieldPoseSolver(fieldPoseSolver)
                .fixedAprilTagLayout(scoringTagLayout)
                .omegaPolicy(DriveGuidanceSpec.OmegaPolicy.PREFER_APRIL_TAGS_WHEN_VALID)
                .onLoss(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
                .doneAdaptive()
                .driveTuning()
                .use(aimTuning)
                .doneDriveTuning()
                .build();

        return new AimRuntime(
                frozenEligibleTagIds,
                scoringTagLayout,
                Collections.unmodifiableMap(fieldToAimPoints),
                scoringSelection,
                aimPlan
        );
    }

    private AimRuntime requireAimRuntime(String operation) {
        AimRuntime runtime = aimRuntime;
        if (runtime == null) {
            throw new IllegalStateException(
                    "Cannot " + operation + " before PhoenixTargeting.update(clock) freezes the "
                            + "selected alliance's eligible scoring target. Managed TeleOp/Auto "
                            + "starts targeting before drive overlays and Tasks; custom hosts must "
                            + "preserve that lifecycle order."
            );
        }
        return runtime;
    }

    /** One activation-owned wrapper that resolves its plan after managed targeting start. */
    private final class DeferredAimOverlay implements DriveOverlay {
        private DriveOverlay delegate;
        private AimRuntime installedRuntime;
        private long installedGeneration = Long.MIN_VALUE;
        private boolean enabled;

        @Override
        public DriveOverlayOutput get(LoopClock clock) {
            return requireDelegate(clock, "sample the Phoenix scoring aim overlay").get(clock);
        }

        @Override
        public void onEnable(LoopClock clock) {
            DriveOverlay requiredDelegate =
                    requireDelegate(clock, "enable the Phoenix scoring aim overlay");
            requiredDelegate.onEnable(clock);
            enabled = true;
        }

        @Override
        public void onDisable(LoopClock clock) {
            if (delegate != null && enabled) {
                delegate.onDisable(clock);
            }
            enabled = false;
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) {
                return;
            }
            String p = (prefix == null || prefix.isEmpty()) ? "phoenixAim" : prefix;
            AimRuntime current = aimRuntime;
            dbg.addData(p + ".runtimeReady", current != null)
                    .addData(p + ".delegateCreated", delegate != null)
                    .addData(p + ".enabled", enabled)
                    .addData(p + ".runtimeGeneration", aimRuntimeGeneration);
            if (current != null) {
                dbg.addData(p + ".eligibleTagIds", current.eligibleTagIds.toString());
            }
            if (delegate != null) {
                delegate.debugDump(dbg, p + ".delegate");
            }
        }

        private DriveOverlay requireDelegate(LoopClock clock, String operation) {
            Objects.requireNonNull(clock, "clock");
            AimRuntime current = requireAimRuntime(operation);
            if (delegate == null) {
                delegate = current.plan.overlay();
                installedRuntime = current;
                installedGeneration = aimRuntimeGeneration;
            } else if (installedRuntime != current
                    || installedGeneration != aimRuntimeGeneration) {
                throw new IllegalStateException(
                        "A Phoenix scoring aim overlay cannot be reused after "
                                + "PhoenixTargeting.reset(). Detach the old drive graph and obtain "
                                + "a fresh aimOverlay() for the next managed session."
                );
            }
            return delegate;
        }
    }
}
