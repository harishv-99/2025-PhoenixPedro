package edu.ftcsushi.robots.phoenix.scoring;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.math.InterpolatingTable1D;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveOverlay;
import edu.ftcsushi.fw.drive.DriveOverlayMask;
import edu.ftcsushi.fw.drive.DriveOverlayOutput;
import edu.ftcsushi.fw.drive.guidance.DriveGuidance;
import edu.ftcsushi.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceQuery;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceSpec;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceStatus;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcsushi.fw.field.TagLayout;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.spatial.References;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.Tasks;
import edu.ftcsushi.robots.phoenix.PhoenixAlliance;
import edu.ftcsushi.robots.phoenix.PhoenixCapabilities;

/**
 * Shared targeting service for Phoenix scoring.
 *
 * <p>The robot supplies one alliance-selected tag id, frozen at the first managed update. This
 * service uses corrected field-pose evidence for both aiming and camera-to-tag-center 3D range.
 * It neither reads camera observations nor solves another robot pose. Mode clients read one
 * retained {@link PhoenixCapabilities.TargetingStatus}; visibility is not a separate gate.</p>
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
        /** Inclusive maximum corrected-pose evidence age, in seconds, shared by aim and range. */
        public double poseMaxAgeSec;
        /** Minimum finite producer score in [0,1]; not a physical accuracy probability. */
        public double poseMinQuality;
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
            config.poseMaxAgeSec = 0.50;
            config.poseMinQuality = 0.10;
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
            copy.poseMaxAgeSec = source.poseMaxAgeSec;
            copy.poseMinQuality = source.poseMinQuality;
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

    /** One session's frozen target facts and corrected-pose guidance graph. */
    private static final class AimRuntime {
        private final int tagId;
        private final ScoringTarget target;
        private final Pose3d fieldToTag;
        private final Pose2d fieldToAimPoint;
        private final DriveGuidancePlan plan;
        private final DriveGuidanceQuery query;

        private AimRuntime(int tagId, ScoringTarget target, Pose3d fieldToTag,
                           Pose2d fieldToAimPoint, DriveGuidancePlan plan) {
            this.tagId = tagId;
            this.target = target;
            this.fieldToTag = fieldToTag;
            this.fieldToAimPoint = fieldToAimPoint;
            this.plan = plan;
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
        final int configuredTagId;
        final boolean hasUsablePose;
        final LoopTimestamp poseTimestamp;
        final double cameraToTagRange3dInches;
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
                             int configuredTagId,
                             boolean hasUsablePose,
                             LoopTimestamp poseTimestamp,
                             double cameraToTagRange3dInches,
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
            this.configuredTagId = configuredTagId;
            this.hasUsablePose = hasUsablePose;
            this.poseTimestamp = poseTimestamp;
            this.cameraToTagRange3dInches = cameraToTagRange3dInches;
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
                    configuredTagId,
                    hasUsablePose,
                    poseTimestamp,
                    cameraToTagRange3dInches,
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
    private final CameraMountConfig cameraMountConfig;
    private final AbsolutePoseEstimator globalAbsolutePoseEstimator;
    private final Source<PoseEstimate> poseSnapshot;
    private PoseEstimate capturedPose = PoseEstimate.noPose(LoopTimestamp.unavailable());
    /** Borrowed, read-only view: the managed localization owner alone advances the estimator. */
    private final AbsolutePoseEstimator aimPose = new AbsolutePoseEstimator() {
        @Override public void update(LoopClock clock) { Objects.requireNonNull(clock, "clock"); }
        @Override public PoseEstimate getEstimate() { return capturedPose; }
    };
    private final TagLayout fieldTagLayout;
    private final Source<Integer> selectedScoringTagId;
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
     * @param cameraMountConfig           fixed camera extrinsics for the current robot profile
     * @param globalAbsolutePoseEstimator borrowed corrected pose; its lifecycle owner updates first
     * @param fieldTagLayout              fixed field tag layout for the current game
     * @param selectedScoringTagId        robot-owned configured id, frozen at the first managed
     *                                    update before any pose query
     * @param autoAimEnabled              driver enable source for aim readiness and the aim overlay
     * @param aimOverrideInput            driver override source that bypasses aim readiness gates when held
     */
    public PhoenixTargeting(Config config,
                            CameraMountConfig cameraMountConfig,
                            AbsolutePoseEstimator globalAbsolutePoseEstimator,
                            TagLayout fieldTagLayout,
                            Source<Integer> selectedScoringTagId,
                            BooleanSource autoAimEnabled,
                            BooleanSource aimOverrideInput) {
        this.cfg = captureConfig(config);
        this.cameraMountConfig = Objects.requireNonNull(cameraMountConfig, "cameraMountConfig");
        this.globalAbsolutePoseEstimator = Objects.requireNonNull(
                globalAbsolutePoseEstimator,
                "globalAbsolutePoseEstimator"
        );
        this.fieldTagLayout = Objects.requireNonNull(fieldTagLayout, "fieldTagLayout");
        this.poseSnapshot = Source.of(clock -> {
            PoseEstimate estimate = this.globalAbsolutePoseEstimator.getEstimate();
            return estimate != null ? estimate : PoseEstimate.noPose(LoopTimestamp.unavailable());
        }).memoized();
        this.selectedScoringTagId = Objects.requireNonNull(
                selectedScoringTagId,
                "PhoenixTargeting selectedScoringTagId source is required"
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
     * targeting service has frozen the selected id on its first active update. Each overlay has its
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
     * configured target, corrected-pose resolution, and controller tuning, but drives a
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
                    "PhoenixTargeting cannot calculate or reset reentrantly while its target, "
                            + "pose, or guidance graph is being evaluated."
            );
        }
        targetingCalculationInProgress = true;
        try {
            boolean autoAimNow = autoAimEnabled.getAsBoolean(clock);
            boolean aimOverrideNow = aimOverrideInput.getAsBoolean(clock);
            AimRuntime runtime = aimRuntime;
            if (runtime == null) {
                AimRuntime candidate = buildAimRuntime(clock);

                // Freeze before fallible pose/guidance reads. A same-cycle retry retains this
                // target even if the caller changes its selected-id source after the failure.
                aimRuntime = candidate;
                aimRuntimeGeneration++;
                runtime = candidate;
            }
            return calculateTargeting(clock, runtime, autoAimNow, aimOverrideNow);
        } finally {
            targetingCalculationInProgress = false;
        }
    }

    private TargetingCalculation calculateTargeting(
            LoopClock clock, AimRuntime runtime, boolean autoAimNow, boolean aimOverrideNow) {
        // The managed localization service publishes first. Range and every aim consumer use
        // this one captured pose; rejected evidence is unavailable through the private view.
        PoseEstimate estimate = poseSnapshot.get(clock);
        boolean hasUsablePose = isUsablePose(estimate, clock);
        capturedPose = hasUsablePose ? estimate : PoseEstimate.noPose(
                estimate == null ? LoopTimestamp.unavailable() : estimate.timestamp);
        DriveGuidanceStatus aimStatus = runtime.query.sample(clock, DriveOverlayMask.OMEGA_ONLY);
        boolean rawAimReady = !autoAimNow || (hasUsablePose
                && aimStatus != null && aimStatus.omegaWithin(aimReadyToleranceRad));

        double range = hasUsablePose
                ? cameraToTagRange3dInches(estimate.fieldToRobotPose, runtime.fieldToTag)
                : Double.NaN;
        double candidateVelocityNative = Double.isFinite(range)
                ? cfg.shotVelocityTable.interpolate(range) : Double.NaN;
        boolean hasSuggestedVelocity = Double.isFinite(candidateVelocityNative);
        return new TargetingCalculation(
                autoAimNow, aimOverrideNow, rawAimReady, runtime.tagId, hasUsablePose,
                estimate == null ? LoopTimestamp.unavailable() : estimate.timestamp,
                range, aimStatus, runtime.target.label,
                runtime.target.aimOffset.forwardInches, runtime.target.aimOffset.leftInches,
                hasSuggestedVelocity, hasSuggestedVelocity ? candidateVelocityNative : Double.NaN,
                runtime.fieldToTag, runtime.fieldToAimPoint);
    }

    /** Applies the same action-specific admission requirements as the absolute-pose aim plan. */
    private boolean isUsablePose(PoseEstimate estimate, LoopClock clock) {
        return estimate != null && estimate.hasPose && finitePose(estimate.fieldToRobotPose)
                && Double.isFinite(estimate.quality) && estimate.quality >= cfg.poseMinQuality
                && estimate.quality <= 1.0 && estimate.timestamp.isFresh(clock, cfg.poseMaxAgeSec);
    }

    /** Uses the camera origin and tag center, not floor-plane or shooter-to-basket distance. */
    private double cameraToTagRange3dInches(Pose3d fieldToRobot, Pose3d fieldToTag) {
        Pose3d fieldToCamera = fieldToRobot.then(cameraMountConfig.robotToCameraPose());
        double range = Math.hypot(Math.hypot(
                fieldToTag.xInches - fieldToCamera.xInches,
                fieldToTag.yInches - fieldToCamera.yInches),
                fieldToTag.zInches - fieldToCamera.zInches);
        return Double.isFinite(range) ? range : Double.NaN;
    }

    private static boolean finitePose(Pose3d pose) {
        return pose != null && Double.isFinite(pose.xInches) && Double.isFinite(pose.yInches)
                && Double.isFinite(pose.zInches) && Double.isFinite(pose.yawRad)
                && Double.isFinite(pose.pitchRad) && Double.isFinite(pose.rollRad);
    }

    /**
     * Clears owned target, query, readiness, and publication state so the next loop starts a
     * fresh targeting session.
     *
     * <p>The owning robot calls this only after detaching the complete graph during shutdown;
     * sampling and reset must not overlap.</p>
     */
    public void reset() {
        if (targetingCalculationInProgress) {
            throw new IllegalStateException(
                    "PhoenixTargeting cannot reset while its target, pose, or guidance "
                            + "graph is being evaluated. Detach the complete drive/targeting graph "
                            + "before shutdown reset."
            );
        }

        AimRuntime runtime = aimRuntime;
        if (runtime != null) {
            runtime.query.reset();
        }
        autoAimEnabled.reset();
        aimRuntime = null;
        poseSnapshot.reset();
        capturedPose = PoseEstimate.noPose(LoopTimestamp.unavailable());
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
                -1,
                false,
                LoopTimestamp.unavailable(),
                Double.NaN,
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
        requireFiniteNonNegative("poseMaxAgeSec", copy.poseMaxAgeSec);
        requireFiniteRange("poseMinQuality", copy.poseMinQuality, 0.0, 1.0);
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
                    "PhoenixTargeting selectedScoringTagId contains tag id " + tagId
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
        Integer tagId = selectedScoringTagId.get(clock);
        if (tagId == null || tagId < 0) {
            throw new IllegalArgumentException(
                    "PhoenixTargeting selectedScoringTagId must supply one non-negative configured id.");
        }
        ScoringTarget target = cfg.scoringTargets == null ? null : cfg.scoringTargets.get(tagId);
        if (target == null) {
            throw new IllegalArgumentException("PhoenixTargeting selectedScoringTagId " + tagId
                    + " has no target in Config.scoringTargets; block START until configured.");
        }
        if (target.label == null || target.label.trim().isEmpty()) {
            throw invalidConfig("scoringTargets[" + tagId + "].label", "non-blank", target.label);
        }
        AimOffset offset = target.aimOffset;
        if (offset == null) {
            throw invalidConfig("scoringTargets[" + tagId + "].aimOffset", "non-null", null);
        }
        requireFinite("scoringTargets[" + tagId + "].aimOffset.forwardInches", offset.forwardInches);
        requireFinite("scoringTargets[" + tagId + "].aimOffset.leftInches", offset.leftInches);
        Pose3d fieldToTag = fieldTagLayout.getFieldToTagPose(tagId);
        requireFiniteFieldPose(tagId, fieldToTag);
        Pose2d fieldToAimPoint = fieldToTag.toPose2d()
                .then(new Pose2d(offset.forwardInches, offset.leftInches, 0.0));
        requireFinite("composed fieldToAimPoint[" + tagId + "].xInches", fieldToAimPoint.xInches);
        requireFinite("composed fieldToAimPoint[" + tagId + "].yInches", fieldToAimPoint.yInches);
        requireFinite("composed fieldToAimPoint[" + tagId + "].headingRad", fieldToAimPoint.headingRad);

        DriveGuidancePlan.Tuning aimTuning = DriveGuidancePlan.Tuning.defaults()
                .withAimKp(cfg.aimKp)
                .withMaxOmegaCmd(cfg.aimMaxOmegaCmd)
                .withMinOmegaCmd(cfg.aimMinOmegaCmd)
                .withAimDeadbandRad(Math.toRadians(cfg.aimToleranceDeg));
        // The fixed target is frozen into the plan; the field layout is not a sensor owner.
        TagLayout targetLayout = new edu.ftcsushi.fw.field.SimpleTagLayout().addPose(tagId, fieldToTag);
        DriveGuidancePlan aimPlan = DriveGuidance.plan()
                .faceTo()
                .point(References.relativeToTagPoint(tagId, offset.forwardInches, offset.leftInches))
                .solveWith()
                .absolutePose(aimPose)
                .maxAgeSec(cfg.poseMaxAgeSec)
                .minQuality(cfg.poseMinQuality)
                .fixedAprilTagLayout(targetLayout)
                .onLoss(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
                .doneAbsolutePose()
                .driveTuning()
                .use(aimTuning)
                .doneDriveTuning()
                .build();
        return new AimRuntime(tagId, target, fieldToTag, fieldToAimPoint, aimPlan);
    }

    private AimRuntime requireAimRuntime(String operation) {
        AimRuntime runtime = aimRuntime;
        if (runtime == null) {
            throw new IllegalStateException(
                    "Cannot " + operation + " before PhoenixTargeting.update(clock) freezes the "
                            + "selected alliance's scoring target. Managed TeleOp/Auto "
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
                dbg.addData(p + ".configuredTagId", current.tagId);
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
