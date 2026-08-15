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
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.field.TagLayouts;
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
import edu.ftcphoenix.robots.phoenix.PhoenixCapabilities;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;

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

    /** One session's exact eligible selector and guidance graph. */
    private static final class AimRuntime {
        private final Set<Integer> eligibleTagIds;
        private final TagSelectionSource selection;
        private final DriveGuidancePlan plan;
        private final DriveGuidanceQuery query;

        private AimRuntime(Set<Integer> eligibleTagIds,
                           TagSelectionSource selection,
                           DriveGuidancePlan plan) {
            this.eligibleTagIds = Objects.requireNonNull(eligibleTagIds, "eligibleTagIds");
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

    private final PhoenixProfile.AutoAimConfig cfg;
    private final FixedTagFieldPoseSolver.Config fieldPoseCfg;
    private final AprilTagSensor tagSensor;
    private final CameraMountConfig cameraMountConfig;
    private final AbsolutePoseEstimator globalAbsolutePoseEstimator;
    private final TagLayout fieldTagLayout;
    private final Source<Set<Integer>> eligibleScoringTagIds;
    private final BooleanSource autoAimEnabled;
    private final BooleanSource aimOverrideInput;
    private final InterpolatingTable1D shotVelocityTable;
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
     * @param aprilTagFieldPoseConfig     AprilTag field-pose solve config used by the guidance plan
     * @param tagSensor                   shared AprilTag sensor used for selection and guidance
     * @param cameraMountConfig           fixed camera extrinsics for the current robot profile
     * @param globalAbsolutePoseEstimator current global pose-estimator lane used by adaptive guidance
     * @param fieldTagLayout              fixed field tag layout for the current game
     * @param eligibleScoringTagIds       robot-owned source of the non-empty configured scoring-tag
     *                                    subset eligible in the current mode; sampled once and
     *                                    defensively frozen before the first detection selection
     * @param autoAimEnabled              driver enable source that activates sticky target selection and the aim overlay
     * @param aimOverrideInput            driver override source that bypasses aim readiness gates when held
     * @param shotVelocityTable           range-to-velocity table used for fresh target-based shot suggestions
     */
    public PhoenixTargeting(PhoenixProfile.AutoAimConfig config,
                            FixedTagFieldPoseSolver.Config aprilTagFieldPoseConfig,
                            AprilTagSensor tagSensor,
                            CameraMountConfig cameraMountConfig,
                            AbsolutePoseEstimator globalAbsolutePoseEstimator,
                            TagLayout fieldTagLayout,
                            Source<Set<Integer>> eligibleScoringTagIds,
                            BooleanSource autoAimEnabled,
                            BooleanSource aimOverrideInput,
                            InterpolatingTable1D shotVelocityTable) {
        this.cfg = Objects.requireNonNull(config, "config").copy();
        this.fieldPoseCfg =
                FixedTagFieldPoseSolver.Config.normalizedValidatedCopyOf(
                        Objects.requireNonNull(aprilTagFieldPoseConfig, "aprilTagFieldPoseConfig"),
                        "PhoenixTargeting.aprilTagFieldPoseConfig"
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
        this.shotVelocityTable = Objects.requireNonNull(shotVelocityTable, "shotVelocityTable");
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

        PhoenixProfile.AutoAimConfig.ScoringTarget target = selection.hasSelection
                ? cfg.targetProfileForTag(selection.selectedTagId)
                : cfg.defaultTargetProfile(-1);

        boolean hasSuggestedVelocity = selection.hasFreshSelectedObservation && selection.selectedObservation.hasTarget;
        double suggestedVelocityNative = hasSuggestedVelocity
                ? shotVelocityTable.interpolate(selection.selectedObservation.cameraRangeInches())
                : Double.NaN;

        Pose3d fieldToSelectedTag = null;
        Pose2d fieldToAimPoint = null;
        if (selection.hasSelection && fieldTagLayout.has(selection.selectedTagId)) {
            fieldToSelectedTag = fieldTagLayout.requireFieldToTagPose(selection.selectedTagId);
            fieldToAimPoint = new Pose2d(
                    fieldToSelectedTag.xInches,
                    fieldToSelectedTag.yInches,
                    fieldToSelectedTag.yawRad
            ).then(new Pose2d(target.aimOffset.forwardInches, target.aimOffset.leftInches, 0.0));
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
        PhoenixProfile.AutoAimConfig.ScoringTarget target = cfg.defaultTargetProfile(-1);
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

    private AimRuntime buildAimRuntime(LoopClock clock) {
        if (cfg.scoringTargets == null) {
            throw new IllegalArgumentException(
                    "PhoenixProfile.autoAim.scoringTargets is required before Phoenix targeting "
                            + "can freeze eligibleScoringTagIds. The managed prestart readiness "
                            + "screen must block START until the selected alliance target exists."
            );
        }

        Set<Integer> configuredTagIds =
                new LinkedHashSet<Integer>(cfg.scoringTargets.keySet());
        if (configuredTagIds.isEmpty()) {
            throw new IllegalArgumentException(
                    "Phoenix auto-aim requires at least one entry in "
                            + "PhoenixProfile.autoAim.scoringTargets before targeting starts."
            );
        }

        Set<Integer> suppliedEligibleTagIds = eligibleScoringTagIds.get(clock);
        if (suppliedEligibleTagIds == null) {
            throw new IllegalArgumentException(
                    "PhoenixTargeting eligibleScoringTagIds source returned null; return the "
                            + "non-empty selected-mode subset of PhoenixProfile.autoAim.scoringTargets."
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
            if (!configuredTagIds.contains(tagId)
                    || cfg.scoringTargets.get(tagId) == null) {
                throw new IllegalArgumentException(
                        "PhoenixTargeting eligibleScoringTagIds contains tag id " + tagId
                                + " that is not a usable entry in "
                                + "PhoenixProfile.autoAim.scoringTargets. Configured tag ids: "
                                + configuredTagIds + "."
                );
            }
            if (!fieldTagLayout.has(tagId)) {
                throw new IllegalArgumentException(
                        "PhoenixTargeting eligibleScoringTagIds contains tag id " + tagId
                                + " that is not in PhoenixProfile.field.fixedAprilTagLayout. "
                                + "Managed TeleOp/Auto readiness must block START until the "
                                + "selected alliance scoring tag has a fixed field pose."
                );
            }
            eligibleSnapshot.add(tagId);
        }

        Set<Integer> frozenEligibleTagIds = Collections.unmodifiableSet(eligibleSnapshot);
        TagLayout scoringTagLayout = TagLayouts.subsetOrSame(
                fieldTagLayout,
                frozenEligibleTagIds,
                "Phoenix eligible scoring fixed-tag layout"
        );
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
                        buildAimOffsetsByTag(frozenEligibleTagIds)
                ))
                .solveWith()
                .adaptive()
                .localization(globalAbsolutePoseEstimator)
                .aprilTags(tagSensor, cameraMountConfig)
                .aprilTagMaxAgeSec(cfg.selectionMaxAgeSec)
                .aprilTagFieldPoseConfig(fieldPoseCfg)
                .fixedAprilTagLayout(scoringTagLayout)
                .omegaPolicy(DriveGuidanceSpec.OmegaPolicy.PREFER_APRIL_TAGS_WHEN_VALID)
                .onLoss(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
                .doneAdaptive()
                .driveTuning()
                .use(aimTuning)
                .doneDriveTuning()
                .build();

        return new AimRuntime(frozenEligibleTagIds, scoringSelection, aimPlan);
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

    private Map<Integer, References.TagPointOffset> buildAimOffsetsByTag(
            Set<Integer> eligibleTagIds
    ) {
        LinkedHashMap<Integer, References.TagPointOffset> offsets = new LinkedHashMap<Integer, References.TagPointOffset>();
        for (Integer tagId : eligibleTagIds) {
            PhoenixProfile.AutoAimConfig.ScoringTarget target = cfg.scoringTargets.get(tagId);
            PhoenixProfile.AutoAimConfig.AimOffset aimOffset = target.aimOffset;
            offsets.put(tagId, References.pointOffset(
                    aimOffset.forwardInches,
                    aimOffset.leftInches
            ));
        }
        return offsets;
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
