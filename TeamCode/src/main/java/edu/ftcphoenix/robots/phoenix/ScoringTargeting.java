package edu.ftcphoenix.robots.phoenix;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

import edu.ftcphoenix.fw.core.control.DebounceBoolean;
import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.math.InterpolatingTable1D;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveOverlay;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
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

/**
 * Shared targeting service for Phoenix scoring.
 *
 * <p>This class owns selected-tag policy, the auto-aim guidance query, and range-based shot
 * suggestions. Higher-level code reads one cached {@link Status} snapshot per loop instead
 * of re-sampling the stateful guidance query in multiple places.</p>
 */
public final class ScoringTargeting implements PhoenixCapabilities.Targeting {

    /**
     * Immutable status snapshot for Phoenix scoring-target selection and auto-aim.
     */
    public static final class Status {
        public final boolean autoAimEnabled;
        public final boolean aimReady;
        public final boolean aimOkToShoot;
        public final boolean aimOverride;
        public final double aimToleranceDeg;
        public final double aimReadyToleranceDeg;
        public final TagSelectionResult selection;
        public final DriveGuidanceStatus aimStatus;
        public final String targetLabel;
        public final double aimOffsetForwardInches;
        public final double aimOffsetLeftInches;
        public final boolean hasSuggestedVelocity;
        public final double suggestedVelocityNative;
        public final Pose3d fieldToSelectedTag;
        public final Pose2d fieldToAimPoint;

        /**
         * Creates an immutable targeting snapshot.
         */
        public Status(boolean autoAimEnabled,
                      boolean aimReady,
                      boolean aimOkToShoot,
                      boolean aimOverride,
                      double aimToleranceDeg,
                      double aimReadyToleranceDeg,
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
            this.aimReady = aimReady;
            this.aimOkToShoot = aimOkToShoot;
            this.aimOverride = aimOverride;
            this.aimToleranceDeg = aimToleranceDeg;
            this.aimReadyToleranceDeg = aimReadyToleranceDeg;
            this.selection = selection;
            this.aimStatus = aimStatus;
            this.targetLabel = targetLabel != null ? targetLabel : "";
            this.aimOffsetForwardInches = aimOffsetForwardInches;
            this.aimOffsetLeftInches = aimOffsetLeftInches;
            this.hasSuggestedVelocity = hasSuggestedVelocity;
            this.suggestedVelocityNative = suggestedVelocityNative;
            this.fieldToSelectedTag = fieldToSelectedTag;
            this.fieldToAimPoint = fieldToAimPoint;
        }
    }

    private final PhoenixProfile.AutoAimConfig cfg;
    private final CameraMountConfig cameraMountConfig;
    private final TagLayout fieldTagLayout;
    private final BooleanSource autoAimEnabled;
    private final BooleanSource aimOverrideInput;
    private final InterpolatingTable1D shotVelocityTable;
    private final DebounceBoolean aimReadyDebouncer;
    private final TagSelectionSource scoringSelection;
    private final DriveGuidancePlan aimPlan;
    private final DriveGuidanceQuery aimQuery;
    private final double aimReadyToleranceRad;

    private final BooleanSource aimOkToShootSource = new BooleanSource() {
        /**
         * {@inheritDoc}
         */
        @Override
        public boolean getAsBoolean(LoopClock clock) {
            return status(clock).aimOkToShoot;
        }
    };

    private final BooleanSource aimOverrideSource = new BooleanSource() {
        /**
         * {@inheritDoc}
         */
        @Override
        public boolean getAsBoolean(LoopClock clock) {
            return status(clock).aimOverride;
        }
    };

    private long lastStatusCycle = Long.MIN_VALUE;
    private Status lastStatus = new Status(
            false,
            true,
            true,
            false,
            0.0,
            0.0,
            TagSelectionResult.none(Collections.emptySet()),
            null,
            "",
            0.0,
            0.0,
            false,
            Double.NaN,
            null,
            null
    );

    /**
     * Creates the shared Phoenix scoring-targeting service.
     *
     * @param config                      auto-aim configuration snapshot copied for local ownership
     * @param aprilTagFieldPoseConfig     AprilTag field-pose solve config used by the guidance plan
     * @param tagSensor                   shared AprilTag sensor used for selection and guidance
     * @param cameraMountConfig           fixed camera extrinsics for the current robot profile
     * @param globalAbsolutePoseEstimator current global pose-estimator lane used by adaptive guidance
     * @param fieldTagLayout              fixed field tag layout for the current game
     * @param autoAimEnabled              driver enable source that activates sticky target selection and the aim overlay
     * @param aimOverrideInput            driver override source that bypasses aim readiness gates when held
     * @param shotVelocityTable           range-to-velocity table used for fresh target-based shot suggestions
     */
    public ScoringTargeting(PhoenixProfile.AutoAimConfig config,
                            FixedTagFieldPoseSolver.Config aprilTagFieldPoseConfig,
                            AprilTagSensor tagSensor,
                            CameraMountConfig cameraMountConfig,
                            AbsolutePoseEstimator globalAbsolutePoseEstimator,
                            TagLayout fieldTagLayout,
                            BooleanSource autoAimEnabled,
                            BooleanSource aimOverrideInput,
                            InterpolatingTable1D shotVelocityTable) {
        this.cfg = Objects.requireNonNull(config, "config").copy();
        FixedTagFieldPoseSolver.Config fieldPoseCfg =
                FixedTagFieldPoseSolver.Config.normalizedValidatedCopyOf(
                        Objects.requireNonNull(aprilTagFieldPoseConfig, "aprilTagFieldPoseConfig"),
                        "ScoringTargeting.aprilTagFieldPoseConfig"
                );
        Objects.requireNonNull(tagSensor, "tagSensor");
        this.cameraMountConfig = Objects.requireNonNull(cameraMountConfig, "cameraMountConfig");
        Objects.requireNonNull(globalAbsolutePoseEstimator, "globalAbsolutePoseEstimator");
        this.fieldTagLayout = Objects.requireNonNull(fieldTagLayout, "fieldTagLayout");
        this.autoAimEnabled = Objects.requireNonNull(autoAimEnabled, "autoAimEnabled").memoized();
        this.aimOverrideInput = Objects.requireNonNull(aimOverrideInput, "aimOverrideInput").memoized();
        this.shotVelocityTable = Objects.requireNonNull(shotVelocityTable, "shotVelocityTable");
        this.aimReadyDebouncer = DebounceBoolean.onAfterOffImmediately(this.cfg.aimReadyDebounceSec);
        this.aimReadyToleranceRad = Math.toRadians(this.cfg.aimReadyToleranceDeg);

        if (this.cfg.scoringTagIds().isEmpty()) {
            throw new IllegalArgumentException(
                    "Phoenix auto-aim requires at least one scoring target in PhoenixProfile.autoAim.scoringTargets"
            );
        }

        TagLayout scoringTagLayout = TagLayouts.subsetOrSame(
                this.fieldTagLayout,
                this.cfg.scoringTagIds(),
                "Phoenix scoring fixed-tag layout"
        );

        scoringSelection = TagSelections.from(tagSensor)
                .among(this.cfg.scoringTagIds())
                .freshWithinSec(this.cfg.selectionMaxAgeSec)
                .choose(TagSelectionPolicies.smallestAbsRobotBearing(this.cameraMountConfig))
                .stickyWhen(this.autoAimEnabled)
                .reacquireAfterLossSec(this.cfg.selectionReacquireSec)
                .build();

        DriveGuidancePlan.Tuning aimTuning = DriveGuidancePlan.Tuning.defaults()
                .withAimKp(this.cfg.aimKp)
                .withMaxOmegaCmd(this.cfg.aimMaxOmegaCmd)
                .withMinOmegaCmd(this.cfg.aimMinOmegaCmd)
                .withAimDeadbandRad(Math.toRadians(this.cfg.aimToleranceDeg));

        aimPlan = DriveGuidance.plan()
                .faceTo()
                .point(References.relativeToSelectedTagPoint(scoringSelection, buildAimOffsetsByTag()))
                .solveWith()
                .adaptive()
                .localization(globalAbsolutePoseEstimator)
                .aprilTags(tagSensor, this.cameraMountConfig)
                .aprilTagMaxAgeSec(this.cfg.selectionMaxAgeSec)
                .aprilTagFieldPoseConfig(fieldPoseCfg)
                .fixedAprilTagLayout(scoringTagLayout)
                .omegaPolicy(DriveGuidanceSpec.OmegaPolicy.PREFER_APRIL_TAGS_WHEN_VALID)
                .onLoss(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
                .doneAdaptive()
                .driveTuning()
                .use(aimTuning)
                .doneDriveTuning()
                .build();

        aimQuery = aimPlan.query();
    }

    /**
     * Returns a fresh auto-aim overlay built from this service's shared plan.
     *
     * <p>Call this during initialization and keep the returned overlay for the lifetime of the
     * owning drive stack. Each overlay has its own runtime state.</p>
     *
     * @return new omega-only/plan-configured drive overlay for scoring auto-aim
     */
    public DriveOverlay aimOverlay() {
        return aimPlan.overlay();
    }

    /**
     * Returns a task wrapper around Phoenix's shared aim plan.
     *
     * <p>This is the autonomous counterpart to {@link #aimOverlay()}: it reuses the exact same
     * selected-target policy, AprilTag/localization resolution, and controller tuning, but drives a
     * supplied {@link DriveCommandSink} directly until the aim task reaches its tolerance.</p>
     *
     * @param driveSink sink used to apply the aim command
     * @param cfg       task-level tolerances/timeouts; when {@code null}, defaults are used
     * @return task that turns the robot toward the currently selected Phoenix scoring target
     */
    @Override
    public Task aimTask(DriveCommandSink driveSink, DriveGuidanceTask.Config cfg) {
        return aimPlan.task(driveSink, cfg);
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
     * Updates the cached targeting snapshot for the current loop cycle.
     *
     * @param clock shared loop clock for the active OpMode cycle
     */
    public void update(LoopClock clock) {
        status(clock);
    }

    /**
     * Returns the targeting snapshot for the current loop, sampling the underlying query at most
     * once per cycle.
     *
     * @param clock shared loop clock for the active OpMode cycle
     * @return cached targeting snapshot for the current cycle
     */
    @Override
    public Status status(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        long cyc = clock.cycle();
        if (cyc == lastStatusCycle) {
            return lastStatus;
        }
        lastStatusCycle = cyc;

        boolean autoAimNow = autoAimEnabled.getAsBoolean(clock);
        boolean aimOverrideNow = aimOverrideInput.getAsBoolean(clock);
        TagSelectionResult selection = scoringSelection.get(clock);
        if (selection == null) {
            selection = TagSelectionResult.none(Collections.emptySet());
        }

        DriveGuidanceStatus aimStatus = aimQuery.sample(clock, DriveOverlayMask.OMEGA_ONLY);
        boolean hasAimReference = !autoAimNow || selection.hasSelection;
        boolean rawAimReady = !autoAimNow || (
                hasAimReference
                        && aimStatus != null
                        && aimStatus.hasOmegaError
                        && aimStatus.omegaWithin(aimReadyToleranceRad)
        );
        boolean aimReadyNow = !autoAimNow || aimReadyDebouncer.update(clock, rawAimReady);
        boolean aimOkToShootNow = aimReadyNow || aimOverrideNow;

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

        lastStatus = new Status(
                autoAimNow,
                aimReadyNow,
                aimOkToShootNow,
                aimOverrideNow,
                cfg.aimToleranceDeg,
                cfg.aimReadyToleranceDeg,
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
        return lastStatus;
    }

    /**
     * Returns a target-derived velocity suggestion when a fresh selected observation exists.
     *
     * @param clock                  shared loop clock for the active OpMode cycle
     * @param fallbackVelocityNative value to return when no fresh range observation is available
     * @return fresh target-derived velocity recommendation, or {@code fallbackVelocityNative} when unavailable
     */
    public double suggestedVelocityNative(LoopClock clock, double fallbackVelocityNative) {
        Status status = status(clock);
        return status.hasSuggestedVelocity ? status.suggestedVelocityNative : fallbackVelocityNative;
    }

    /**
     * Clears cached selector/query state so the next loop starts a fresh targeting session.
     */
    public void reset() {
        scoringSelection.reset();
        aimQuery.reset();
        autoAimEnabled.reset();
        aimOverrideInput.reset();
        aimReadyDebouncer.reset(false);
        lastStatusCycle = Long.MIN_VALUE;
        lastStatus = new Status(
                false,
                true,
                true,
                false,
                cfg.aimToleranceDeg,
                cfg.aimReadyToleranceDeg,
                TagSelectionResult.none(Collections.emptySet()),
                null,
                "",
                0.0,
                0.0,
                false,
                Double.NaN,
                null,
                null
        );
    }

    private Map<Integer, References.TagPointOffset> buildAimOffsetsByTag() {
        LinkedHashMap<Integer, References.TagPointOffset> offsets = new LinkedHashMap<Integer, References.TagPointOffset>();
        for (Map.Entry<Integer, PhoenixProfile.AutoAimConfig.ScoringTarget> entry : cfg.scoringTargetsById().entrySet()) {
            PhoenixProfile.AutoAimConfig.AimOffset aimOffset = entry.getValue().aimOffset;
            offsets.put(entry.getKey(), References.pointOffset(aimOffset.forwardInches, aimOffset.leftInches));
        }
        return offsets;
    }
}
