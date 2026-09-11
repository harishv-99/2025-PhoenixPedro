package edu.ftcsushi.fw.drive.guidance;

import java.util.ArrayList;
import java.util.Objects;
import java.util.Set;

import edu.ftcsushi.fw.drive.DriveOverlay;
import edu.ftcsushi.fw.field.TagLayout;
import edu.ftcsushi.fw.field.TagLayouts;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcsushi.fw.spatial.FacingTarget2d;
import edu.ftcsushi.fw.spatial.ReferenceFrame2d;
import edu.ftcsushi.fw.spatial.ReferencePoint2d;
import edu.ftcsushi.fw.spatial.References;
import edu.ftcsushi.fw.spatial.SpatialControlFrames;
import edu.ftcsushi.fw.spatial.SpatialQuerySpec;
import edu.ftcsushi.fw.spatial.SpatialSolveSet;
import edu.ftcsushi.fw.spatial.SpatialTargets;
import edu.ftcsushi.fw.spatial.TranslationTarget2d;

/**
 * Guided builders and helpers for creating {@link DriveGuidanceSpec}s and {@link DriveGuidancePlan}s.
 *
 * <p>DriveGuidance is intentionally split into two public objects:</p>
 * <ul>
 *   <li>{@link DriveGuidanceSpec}: controller-neutral <b>what</b> (targets, control frames, solve lanes)</li>
 *   <li>{@link DriveGuidancePlan}: spec + {@link DriveGuidancePlan.Tuning} (<b>how strongly</b>)</li>
 * </ul>
 *
 * <p>The staged builder follows the same principle as {@code PlantTargets.plan(request)}: answer
 * required conceptual questions in order, then expose optional tuning branches:</p>
 * <ol>
 *   <li>choose the first translation or facing target, then optionally add the other channel,</li>
 *   <li>optionally choose controlled robot frames,</li>
 *   <li>choose the solve mode and required solve lanes,</li>
 *   <li>optionally enter drive tuning,</li>
 *   <li>build the reusable plan.</li>
 * </ol>
 *
 * <p>Build is not visible until a target and solve mode have been configured. Target choice methods
 * such as {@code point(...)}, {@code fieldPointInches(...)}, and {@code frameHeading(...)} return
 * directly to the parent stage because they answer one required choice. Solver policy knobs only
 * appear inside the selected solve-mode branch, and drive-controller tuning only appears after
 * entering {@link PlanOptionalTuningStage#driveTuning()}.</p>
 *
 * <h2>Common usage</h2>
 *
 * <pre>{@code
 * ReferenceFrame2d slotFace = References.fieldFrame(48.0, 24.0, Math.PI);
 *
 * DriveGuidancePlan alignPlan = DriveGuidance.plan()
 *         .translateTo()
 *             .point(References.framePoint(slotFace, -6.0, 0.0))
 *         .andFaceTo()
 *             .frameHeading(slotFace)
 *         .solveWith()
 *             .absolutePose(poseEstimator)
 *                 .doneAbsolutePose()
 *         .driveTuning()
 *             .aimKp(2.8)
 *             .doneDriveTuning()
 *         .build();
 * }</pre>
 *
 * <p>This reference-first API describes targets as semantic points or frames. The evaluation layer
 * uses the explicitly selected evidence authority; it never blends or switches pose sources.</p>
 */
public final class DriveGuidance {

    private DriveGuidance() {
        // static utility
    }

    /**
     * Starts building a controller-neutral {@link DriveGuidanceSpec}.
     *
     * <p>Use this when you want to reuse the same targets/solve-lane configuration with different
     * controller tunings, for example gentler TeleOp assist and stronger autonomous tuning.</p>
     */
    public static SpecBuilder0 spec() {
        return new Spec0(new State());
    }

    /**
     * Starts building a complete {@link DriveGuidancePlan} in one pass.
     */
    public static PlanBuilder0 plan() {
        return new Builder0(new State());
    }

    /**
     * Starts building a {@link DriveGuidancePlan} from a pre-built spec.
     *
     * <p>The spec has already answered the target and solve-mode questions, so this builder exposes
     * only optional drive tuning and build.</p>
     */
    public static PlanFromSpecBuilder plan(DriveGuidanceSpec spec) {
        return new PlanFromSpecBuilderImpl(spec);
    }

    /**
     * Minimal builder that combines a pre-built spec with optional drive tuning.
     */
    public interface PlanFromSpecBuilder extends PlanOptionalTuningStage {
        // marker interface for readability
    }

    private static final class PlanFromSpecBuilderImpl implements PlanFromSpecBuilder {
        private final DriveGuidanceSpec spec;
        private DriveGuidancePlan.Tuning tuning = DriveGuidancePlan.Tuning.defaults();

        PlanFromSpecBuilderImpl(DriveGuidanceSpec spec) {
            this.spec = Objects.requireNonNull(spec, "spec");
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public DriveTuningBranch driveTuning() {
            return new PlanFromSpecTuningStep(this);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public DriveGuidancePlan build() {
            return new DriveGuidancePlan(spec, tuning);
        }
    }

    /**
     * Creates a pose-lock overlay that holds the current field pose using default tuning.
     *
     * <p>Activation captures only a finite, available pose no older than 0.50 seconds with finite
     * quality in [0.10, 1]. The same gate applies to active feedback. Invalid initial evidence
     * leaves the overlay inactive until re-enabled; temporary loss after a valid capture passes
     * manual commands through without changing that captured target.</p>
     */
    public static DriveOverlay poseLock(AbsolutePoseEstimator poseEstimator) {
        return poseLock(poseEstimator, DriveGuidancePlan.Tuning.defaults());
    }

    /**
     * Creates a pose-lock overlay with custom tuning.
     * Evidence admission and enable-only target capture are the same as {@link #poseLock(AbsolutePoseEstimator)}.
     */
    public static DriveOverlay poseLock(AbsolutePoseEstimator poseEstimator, DriveGuidancePlan.Tuning tuning) {
        return new PoseLockOverlay(poseEstimator, tuning);
    }

    // ------------------------------------------------------------------------
    // Spec builder staging
    // ------------------------------------------------------------------------

    /**
     * Initial spec stage: choose the first translation or facing target.
     */
    public interface SpecBuilder0 {
        /**
         * Begins configuring the translation target.
         */
        TranslateToBuilder<SpecBuilder1> translateTo();

        /**
         * Begins configuring the facing target.
         */
        FaceToBuilder<SpecBuilder2> faceTo();
    }

    /**
     * Shared spec stage after at least one target has been configured.
     */
    public interface SpecConfiguredStage<SELF> {
        /**
         * Chooses which point(s) on the robot guidance should translate / face with respect to.
         */
        SELF controlFrames(SpatialControlFrames frames);

        /**
         * Begins choosing the solve mode and solve lanes for this spec.
         */
        ResolveModeChoice<SpecBuildStage> solveWith();
    }

    /**
     * Spec stage after translation has been configured.
     */
    public interface SpecBuilder1 extends SpecConfiguredStage<SpecBuilder1> {
        /**
         * Adds a facing target to a spec that already has translation.
         */
        FaceToBuilder<SpecBuilder3> andFaceTo();
    }

    /**
     * Spec stage after facing has been configured.
     */
    public interface SpecBuilder2 extends SpecConfiguredStage<SpecBuilder2> {
        /**
         * Adds a translation target to a spec that already has facing.
         */
        TranslateToBuilder<SpecBuilder3> andTranslateTo();
    }

    /**
     * Final target stage after both translation and facing have been configured.
     */
    public interface SpecBuilder3 extends SpecConfiguredStage<SpecBuilder3> {
        // no-op
    }

    /**
     * Terminal spec stage after targets and solve mode have both been configured.
     */
    public interface SpecBuildStage {
        /**
         * Builds the immutable controller-neutral spec.
         */
        DriveGuidanceSpec build();
    }

    // ------------------------------------------------------------------------
    // Plan builder staging
    // ------------------------------------------------------------------------

    /**
     * Initial plan stage: choose the first translation or facing target.
     */
    public interface PlanBuilder0 {
        /**
         * Begins configuring the translation target.
         */
        TranslateToBuilder<PlanBuilder1> translateTo();

        /**
         * Begins configuring the facing target.
         */
        FaceToBuilder<PlanBuilder2> faceTo();
    }

    /**
     * Shared plan stage after at least one target has been configured.
     */
    public interface PlanConfiguredStage<SELF> {
        /**
         * Chooses which point(s) on the robot guidance should translate / face with respect to.
         */
        SELF controlFrames(SpatialControlFrames frames);

        /**
         * Begins choosing the solve mode and solve lanes for this plan.
         */
        ResolveModeChoice<PlanOptionalTuningStage> solveWith();
    }

    /**
     * Plan stage after translation has been configured.
     */
    public interface PlanBuilder1 extends PlanConfiguredStage<PlanBuilder1> {
        /**
         * Adds a facing target to a plan that already has translation.
         */
        FaceToBuilder<PlanBuilder3> andFaceTo();
    }

    /**
     * Plan stage after facing has been configured.
     */
    public interface PlanBuilder2 extends PlanConfiguredStage<PlanBuilder2> {
        /**
         * Adds a translation target to a plan that already has facing.
         */
        TranslateToBuilder<PlanBuilder3> andTranslateTo();
    }

    /**
     * Final target stage after both translation and facing have been configured.
     */
    public interface PlanBuilder3 extends PlanConfiguredStage<PlanBuilder3> {
        // no-op
    }

    /**
     * Optional-tuning stage after target and solve mode have been chosen.
     */
    public interface PlanOptionalTuningStage extends PlanBuildStage {
        /**
         * Enters optional controller tuning. Build immediately to use {@link DriveGuidancePlan.Tuning#defaults()}.
         */
        DriveTuningBranch driveTuning();
    }

    /**
     * Terminal plan stage that can build the immutable plan.
     */
    public interface PlanBuildStage {
        /**
         * Builds the immutable plan.
         */
        DriveGuidancePlan build();
    }

    /**
     * Optional branch for drivetrain-controller gains and command caps.
     */
    public interface DriveTuningBranch {
        /**
         * Replaces the current tuning bundle.
         */
        DriveTuningBranch use(DriveGuidancePlan.Tuning tuning);

        /**
         * Sets translation proportional gain, in drive command per inch of translation error.
         */
        DriveTuningBranch translateKp(double kPTranslate);

        /**
         * Sets maximum translation command magnitude.
         */
        DriveTuningBranch maxTranslateCmd(double maxTranslateCmd);

        /**
         * Sets facing proportional gain, in omega command per radian of facing error.
         */
        DriveTuningBranch aimKp(double kPAim);

        /**
         * Sets maximum omega command magnitude.
         */
        DriveTuningBranch maxOmegaCmd(double maxOmegaCmd);

        /**
         * Sets minimum omega command magnitude outside the aim deadband.
         */
        DriveTuningBranch minOmegaCmd(double minOmegaCmd);

        /**
         * Sets the aim deadband in radians.
         */
        DriveTuningBranch aimDeadbandRad(double aimDeadbandRad);

        /**
         * Returns to the main plan builder after drive tuning.
         */
        PlanOptionalTuningStage doneDriveTuning();
    }

    // ------------------------------------------------------------------------
    // Target builders
    // ------------------------------------------------------------------------

    /**
     * Nested builder used to describe the translation goal.
     */
    public interface TranslateToBuilder<RETURN> {
        /**
         * Translates toward a field-fixed point, in field inches, then returns to the parent stage.
         */
        RETURN fieldPointInches(double xInches, double yInches);

        /**
         * Captures a robot-relative delta, in inches, when guidance enables, then returns to the parent stage.
         */
        RETURN robotRelativePointInches(double forwardInches, double leftInches);

        /**
         * Translates toward a semantic point reference, then returns to the parent stage.
         */
        RETURN point(ReferencePoint2d reference);

    }

    /**
     * Nested builder used to describe the facing / heading goal.
     */
    public interface FaceToBuilder<RETURN> {
        /**
         * Faces a field-fixed point, in field inches, then returns to the parent stage.
         */
        RETURN fieldPointInches(double xInches, double yInches);

        /**
         * Aligns to an absolute field heading in radians, then returns to the parent stage.
         */
        RETURN fieldHeadingRad(double fieldHeadingRad);

        /**
         * Faces a semantic point reference, then returns to the parent stage.
         */
        RETURN point(ReferencePoint2d reference);

        /**
         * Aligns to the heading of a semantic reference frame, then returns to the parent stage.
         */
        RETURN frameHeading(ReferenceFrame2d reference);

        /**
         * Aligns to the heading of a semantic reference frame plus an additional offset in radians,
         * then returns to the parent stage.
         */
        RETURN frameHeading(ReferenceFrame2d reference, double headingOffsetRad);

    }

    // ------------------------------------------------------------------------
    // Resolve / solve-mode builders
    // ------------------------------------------------------------------------

    /** Choose one evidence authority and supply its required source in the same answer. */
    public interface ResolveModeChoice<RETURN> {
        /**
         * Reads one already-updated absolute pose. Defaults: age at most 0.50 seconds, quality
         * at least 0.10, and pass-through on loss. Neither the estimator nor its sensors are updated.
         */
        AbsolutePoseTuningStage<RETURN> absolutePose(AbsolutePoseEstimator poseEstimator);

        /**
         * Uses the actual observed tag-relative target with a fixed camera mount. Defaults:
         * frame age at most 0.50 seconds and pass-through on loss. Field-only targets are rejected.
         */
        RelativeAprilTagsTuningStage<RETURN> relativeAprilTags(
                AprilTagSensor aprilTags, CameraMountConfig cameraMount);

        /**
         * Uses an observed-point reference as delayed robot-at-capture feedback. The reference owns
         * freshness; this answer chooses loss behavior. No current-motion compensation is implied.
         */
        RETURN observedPoints(DriveGuidanceSpec.LossPolicy onLoss);
    }

    /** Optional settings for the one borrowed absolute-pose authority. */
    public interface AbsolutePoseTuningStage<RETURN> {
        /** Maximum accepted pose age, finite non-negative seconds, inclusive. */
        AbsolutePoseTuningStage<RETURN> maxAgeSec(double maxAgeSec);
        /** Minimum accepted producer quality, finite in [0, 1], not an accuracy guarantee. */
        AbsolutePoseTuningStage<RETURN> minQuality(double minQuality);
        /** Supplies fixed tag metadata; the completed spec validates and snapshots it. */
        AbsolutePoseTuningStage<RETURN> fixedAprilTagLayout(TagLayout tagLayout);
        /** Chooses the output behavior independently for each unsolved requested channel. */
        AbsolutePoseTuningStage<RETURN> onLoss(DriveGuidanceSpec.LossPolicy onLoss);
        /** Closes this multi-setting branch. */
        RETURN doneAbsolutePose();
    }

    /** Optional settings for direct observed tag geometry, never a field-pose estimator. */
    public interface RelativeAprilTagsTuningStage<RETURN> {
        /** Maximum accepted camera-frame age, finite non-negative seconds, inclusive. */
        RelativeAprilTagsTuningStage<RETURN> maxAgeSec(double maxAgeSec);
        /** Chooses the output behavior independently for each unsolved requested channel. */
        RelativeAprilTagsTuningStage<RETURN> onLoss(DriveGuidanceSpec.LossPolicy onLoss);
        /** Closes this multi-setting branch. */
        RETURN doneRelativeAprilTags();
    }
    // ------------------------------------------------------------------------
    // Implementation
    // ------------------------------------------------------------------------

    private static final class State {
        TranslationTarget2d translationTarget;
        FacingTarget2d facingTarget;

        SpatialControlFrames controlFrames = SpatialControlFrames.robotCenter();
        DriveGuidancePlan.Tuning tuning = DriveGuidancePlan.Tuning.defaults();

        AprilTagSensor aprilTagSensor;
        CameraMountConfig cameraMount;
        double tagsMaxAgeSec = DriveGuidanceSpec.RelativeAprilTags.DEFAULT_MAX_AGE_SEC;

        AbsolutePoseEstimator poseEstimator;
        double poseMaxAgeSec = DriveGuidanceSpec.AbsolutePose.DEFAULT_MAX_AGE_SEC;
        double poseMinQuality = DriveGuidanceSpec.AbsolutePose.DEFAULT_MIN_QUALITY;

        DriveGuidanceSpec.SolveMode solveMode;
        TagLayout fixedAprilTagLayout;
        DriveGuidanceSpec.LossPolicy onLoss = DriveGuidanceSpec.LossPolicy.PASS_THROUGH;
    }

    /**
     * Validates the staged builder state and produces an immutable spec snapshot.
     */
    private static DriveGuidanceSpec buildSpec(State s) {
        if (s.translationTarget == null && s.facingTarget == null) {
            throw new IllegalStateException("DriveGuidance spec needs translateTo() and/or faceTo() configured");
        }

        TagLayout fixedAprilTagLayout = s.fixedAprilTagLayout != null
                ? TagLayouts.snapshot(s.fixedAprilTagLayout)
                : null;
        validateCapabilitiesOrThrow(s, fixedAprilTagLayout);

        DriveGuidanceSpec.RelativeAprilTags tags = s.aprilTagSensor != null
                ? new DriveGuidanceSpec.RelativeAprilTags(s.aprilTagSensor, s.cameraMount, s.tagsMaxAgeSec)
                : null;
        DriveGuidanceSpec.AbsolutePose absolutePose = s.poseEstimator != null
                ? new DriveGuidanceSpec.AbsolutePose(s.poseEstimator, s.poseMaxAgeSec, s.poseMinQuality)
                : null;
        DriveGuidanceSpec.ResolveWith rw = DriveGuidanceSpec.ResolveWith.create(
                s.solveMode, tags, absolutePose, fixedAprilTagLayout, s.onLoss);

        SpatialSolveSet solveSet;
        switch (s.solveMode) {
            case ABSOLUTE_POSE:
                solveSet = SpatialSolveSet.builder()
                        .absolutePose(absolutePose.poseEstimator, absolutePose.maxAgeSec, absolutePose.minQuality)
                        .build();
                break;
            case RELATIVE_APRIL_TAGS:
                solveSet = SpatialSolveSet.builder()
                        .relativeAprilTags(tags.sensor, tags.cameraMount, tags.maxAgeSec).build();
                break;
            case OBSERVED_POINTS:
                solveSet = SpatialSolveSet.builder().observedPoints().build();
                break;
            default:
                throw new IllegalStateException("A supported solveWith() mode is required");
        }

        SpatialQuerySpec spatialQuerySpec = null;
        TranslationTarget2d spatialTranslationTarget = (s.translationTarget instanceof DriveGuidanceSpec.RobotRelativePoint)
                ? null
                : s.translationTarget;
        if (spatialTranslationTarget != null && s.facingTarget != null) {
            spatialQuerySpec = SpatialQuerySpec.builder()
                    .translateTo(spatialTranslationTarget)
                    .andFaceTo(s.facingTarget)
                    .controlFrames(s.controlFrames)
                    .solveWith(solveSet)
                    .fixedAprilTagLayout(fixedAprilTagLayout)
                    .build();
        } else if (spatialTranslationTarget != null) {
            spatialQuerySpec = SpatialQuerySpec.builder()
                    .translateTo(spatialTranslationTarget)
                    .controlFrames(s.controlFrames)
                    .solveWith(solveSet)
                    .fixedAprilTagLayout(fixedAprilTagLayout)
                    .build();
        } else if (s.facingTarget != null) {
            spatialQuerySpec = SpatialQuerySpec.builder()
                    .faceTo(s.facingTarget)
                    .controlFrames(s.controlFrames)
                    .solveWith(solveSet)
                    .fixedAprilTagLayout(fixedAprilTagLayout)
                    .build();
        }

        return new DriveGuidanceSpec(
                s.translationTarget,
                s.facingTarget,
                s.controlFrames,
                rw,
                spatialQuerySpec
        );
    }

    /**
     * Builds a plan directly from the staged builder state.
     */
    private static DriveGuidancePlan buildPlan(State s) {
        return new DriveGuidancePlan(buildSpec(s), s.tuning);
    }

    /**
     * Ensures the configured targets can be solved by the configured solve lanes.
     */
    private static void validateCapabilitiesOrThrow(State s, TagLayout fixedAprilTagLayout) {
        ArrayList<String> errors = new ArrayList<String>();
        if (s.solveMode == null) {
            errors.add("solveWith() requires absolutePose(...), relativeAprilTags(...), or observedPoints(...)");
        } else if (s.solveMode == DriveGuidanceSpec.SolveMode.ABSOLUTE_POSE) {
            if (s.poseEstimator == null) errors.add("absolutePose(...) requires a pose estimator");
            if (!Double.isFinite(s.poseMaxAgeSec) || s.poseMaxAgeSec < 0.0) {
                errors.add("absolutePose(...): maxAgeSec must be finite and >= 0");
            }
            if (!Double.isFinite(s.poseMinQuality) || s.poseMinQuality < 0.0 || s.poseMinQuality > 1.0) {
                errors.add("absolutePose(...): minQuality must be finite and in [0, 1]");
            }
            if (s.translationTarget != null
                    && !canSolveTranslationWithLocalization(s.translationTarget, fixedAprilTagLayout)) {
                errors.add(localizationFailureForTranslationTarget(s.translationTarget, fixedAprilTagLayout));
            }
            if (s.facingTarget != null
                    && !canSolveAimWithLocalization(s.facingTarget, fixedAprilTagLayout)) {
                errors.add(localizationFailureForFacingTarget(s.facingTarget, fixedAprilTagLayout));
            }
        } else if (s.solveMode == DriveGuidanceSpec.SolveMode.RELATIVE_APRIL_TAGS) {
            if (s.aprilTagSensor == null || s.cameraMount == null) {
                errors.add("relativeAprilTags(...) requires a sensor and camera mount");
            }
            if (!Double.isFinite(s.tagsMaxAgeSec) || s.tagsMaxAgeSec < 0.0) {
                errors.add("relativeAprilTags(...): maxAgeSec must be finite and >= 0");
            }
            if (s.translationTarget != null && !canSolveTranslationWithAprilTags(s.translationTarget)) {
                errors.add("relativeAprilTags(...) translateTo() requires a direct or selected tag-relative "
                        + "point; field-fixed, remembered and robotRelativePointInches(...) targets require absolutePose(...)");
            }
            if (s.facingTarget != null && !canSolveAimWithAprilTags(s.facingTarget)) {
                errors.add("relativeAprilTags(...) faceTo() requires a direct or selected tag-relative "
                        + "point/frame; field-fixed and remembered targets require absolutePose(...)");
            }
        } else {
            if (s.translationTarget != null && !isObservedPointTarget(s.translationTarget)) {
                errors.add("observedPoints() translateTo() requires References.selectedTargetPoint(...); "
                        + "remembered field targets require absolutePose(...)");
            }
            if (s.facingTarget != null && !isObservedPointTarget(s.facingTarget)) {
                errors.add("observedPoints() faceTo() requires References.selectedTargetPoint(...); "
                        + "remembered field targets require absolutePose(...)");
            }
        }
        if (!errors.isEmpty()) {
            StringBuilder message = new StringBuilder("Invalid DriveGuidance plan:\n");
            for (String error : errors) message.append(" - ").append(error).append('\n');
            throw new IllegalStateException(message.toString());
        }
    }

    private static String localizationFailureForTranslationTarget(TranslationTarget2d target,
                                                                  TagLayout layout) {
        String base = "translateTo() target cannot be solved from absolutePose(...)";
        if (target instanceof SpatialTargets.ReferencePointTarget) {
            return explainLocalizationPointFailure(
                    ((SpatialTargets.ReferencePointTarget) target).reference,
                    layout,
                    base
            );
        }
        return base + "; add fixedAprilTagLayout(...) for fixed-tag references or choose relativeAprilTags(...) for direct observed-tag geometry";
    }

    private static String localizationFailureForFacingTarget(FacingTarget2d target,
                                                             TagLayout layout) {
        String base = "faceTo() target cannot be solved from absolutePose(...)";
        if (target instanceof SpatialTargets.ReferencePointTarget) {
            return explainLocalizationPointFailure(
                    ((SpatialTargets.ReferencePointTarget) target).reference,
                    layout,
                    base
            );
        }
        if (target instanceof SpatialTargets.ReferenceFrameHeadingTarget) {
            return explainLocalizationFrameFailure(
                    ((SpatialTargets.ReferenceFrameHeadingTarget) target).reference,
                    layout,
                    base
            );
        }
        return base + "; add fixedAprilTagLayout(...) for fixed-tag references or choose relativeAprilTags(...) for direct observed-tag geometry";
    }

    private static String explainLocalizationPointFailure(ReferencePoint2d ref,
                                                          TagLayout layout,
                                                          String base) {
        if (ref == null) {
            return base;
        }
        if (References.isFramePoint(ref)) {
            return explainLocalizationFrameFailure(References.framePointBaseFrame(ref), layout, base);
        }
        if (References.isDirectTagPoint(ref) || References.isSelectedTagPoint(ref)) {
            if (layout == null) {
                return base + "; fixedAprilTagLayout(...) is required for fixed-tag / selected-tag localization";
            }

            Set<Integer> missing = References.missingCandidateTagIds(ref, layout);
            if (!missing.isEmpty()) {
                return base + "; localization requires every candidate tag ID to be present in fixedAprilTagLayout(...); missing " + missing;
            }
        }
        return base + "; add fixedAprilTagLayout(...) for fixed-tag references or choose relativeAprilTags(...) for direct observed-tag geometry";
    }

    private static String explainLocalizationFrameFailure(ReferenceFrame2d ref,
                                                          TagLayout layout,
                                                          String base) {
        if (ref == null) {
            return base;
        }
        if (References.isDirectTagFrame(ref) || References.isSelectedTagFrame(ref)) {
            if (layout == null) {
                return base + "; fixedAprilTagLayout(...) is required for fixed-tag / selected-tag localization";
            }

            Set<Integer> missing = References.missingCandidateTagIds(ref, layout);
            if (!missing.isEmpty()) {
                return base + "; localization requires every candidate tag ID to be present in fixedAprilTagLayout(...); missing " + missing;
            }
        }
        return base + "; add fixedAprilTagLayout(...) for fixed-tag references or choose relativeAprilTags(...) for direct observed-tag geometry";
    }

    private static boolean canSolveTranslationWithLocalization(TranslationTarget2d target,
                                                               TagLayout layout) {
        if (target instanceof SpatialTargets.FieldPoint) {
            return true;
        }
        if (target instanceof DriveGuidanceSpec.RobotRelativePoint) {
            return true;
        }
        if (target instanceof SpatialTargets.ReferencePointTarget) {
            return canResolvePointWithLocalization(((SpatialTargets.ReferencePointTarget) target).reference, layout);
        }
        return false;
    }

    private static boolean canSolveTranslationWithAprilTags(TranslationTarget2d target) {
        return target instanceof SpatialTargets.ReferencePointTarget
                && canResolvePointWithAprilTags(((SpatialTargets.ReferencePointTarget) target).reference);
    }

    private static boolean canSolveAimWithLocalization(FacingTarget2d target,
                                                       TagLayout layout) {
        if (target instanceof SpatialTargets.FieldPoint) {
            return true;
        }
        if (target instanceof SpatialTargets.FieldHeading) {
            return true;
        }
        if (target instanceof SpatialTargets.ReferencePointTarget) {
            return canResolvePointWithLocalization(((SpatialTargets.ReferencePointTarget) target).reference, layout);
        }
        if (target instanceof SpatialTargets.ReferenceFrameHeadingTarget) {
            return canResolveFrameWithLocalization(((SpatialTargets.ReferenceFrameHeadingTarget) target).reference, layout);
        }
        return false;
    }

    private static boolean canSolveAimWithAprilTags(FacingTarget2d target) {
        if (target instanceof SpatialTargets.ReferencePointTarget) {
            return canResolvePointWithAprilTags(((SpatialTargets.ReferencePointTarget) target).reference);
        }
        return target instanceof SpatialTargets.ReferenceFrameHeadingTarget
                && canResolveFrameWithAprilTags(((SpatialTargets.ReferenceFrameHeadingTarget) target).reference);
    }

    private static boolean canResolvePointWithLocalization(ReferencePoint2d ref, TagLayout layout) {
        if (References.isFieldPoint(ref) || References.isObservedPoint(ref) || References.isRememberedPoint(ref)) {
            return true;
        }
        if (References.isDirectTagPoint(ref) || References.isSelectedTagPoint(ref)) {
            return References.allCandidateTagsAreFixed(ref, layout);
        }
        if (References.isFramePoint(ref)) {
            return canResolveFrameWithLocalization(References.framePointBaseFrame(ref), layout);
        }
        return false;
    }

    private static boolean canResolvePointWithAprilTags(ReferencePoint2d ref) {
        if (References.isDirectTagPoint(ref) || References.isSelectedTagPoint(ref)) return true;
        return References.isFramePoint(ref)
                && canResolveFrameWithAprilTags(References.framePointBaseFrame(ref));
    }

    private static boolean canResolveFrameWithLocalization(ReferenceFrame2d ref, TagLayout layout) {
        if (References.isFieldFrame(ref) || References.isApproachFrame(ref)) {
            return true;
        }
        if (References.isDirectTagFrame(ref) || References.isSelectedTagFrame(ref)) {
            return References.allCandidateTagsAreFixed(ref, layout);
        }
        return false;
    }

    private static boolean canResolveFrameWithAprilTags(ReferenceFrame2d ref) {
        return References.isDirectTagFrame(ref) || References.isSelectedTagFrame(ref);
    }

    // ------------------------------------------------------------------------
    // Builder implementations
    // ------------------------------------------------------------------------

    private static boolean isObservedPointTarget(Object target) {
        return target instanceof SpatialTargets.ReferencePointTarget
                && References.isObservedPoint(((SpatialTargets.ReferencePointTarget) target).reference);
    }

    /** Clears old builder branch answers before choosing a single new authority. */
    private static void resetSolve(State s, DriveGuidanceSpec.SolveMode mode) {
        s.solveMode = mode;
        s.aprilTagSensor = null;
        s.cameraMount = null;
        s.tagsMaxAgeSec = DriveGuidanceSpec.RelativeAprilTags.DEFAULT_MAX_AGE_SEC;
        s.poseEstimator = null;
        s.poseMaxAgeSec = DriveGuidanceSpec.AbsolutePose.DEFAULT_MAX_AGE_SEC;
        s.poseMinQuality = DriveGuidanceSpec.AbsolutePose.DEFAULT_MIN_QUALITY;
        s.fixedAprilTagLayout = null;
        s.onLoss = DriveGuidanceSpec.LossPolicy.PASS_THROUGH;
    }

    private static abstract class ConfiguredTargetBuilder<SELF, AFTER_SOLVE> {
        final State s;
        final AFTER_SOLVE afterSolve;

        ConfiguredTargetBuilder(State s, AFTER_SOLVE afterSolve) {
            this.s = s;
            this.afterSolve = afterSolve;
        }

        @SuppressWarnings("unchecked")
        final SELF self() {
            return (SELF) this;
        }

        public final SELF controlFrames(SpatialControlFrames frames) {
            s.controlFrames = Objects.requireNonNull(frames, "frames");
            return self();
        }

        public final ResolveModeChoice<AFTER_SOLVE> solveWith() {
            return new ResolveModeChoiceStep<AFTER_SOLVE>(s, afterSolve);
        }
    }

    private static final class Spec0 implements SpecBuilder0 {
        private final State s;

        Spec0(State s) {
            this.s = s;
        }

        @Override
        public TranslateToBuilder<SpecBuilder1> translateTo() {
            return new TranslateToStep<SpecBuilder1>(s, new Spec1(s));
        }

        @Override
        public FaceToBuilder<SpecBuilder2> faceTo() {
            return new FaceToStep<SpecBuilder2>(s, new Spec2(s));
        }
    }

    private static final class Spec1 extends ConfiguredTargetBuilder<SpecBuilder1, SpecBuildStage> implements SpecBuilder1 {
        Spec1(State s) {
            super(s, new SpecTerminal(s));
        }

        @Override
        public FaceToBuilder<SpecBuilder3> andFaceTo() {
            return new FaceToStep<SpecBuilder3>(s, new Spec3(s));
        }
    }

    private static final class Spec2 extends ConfiguredTargetBuilder<SpecBuilder2, SpecBuildStage> implements SpecBuilder2 {
        Spec2(State s) {
            super(s, new SpecTerminal(s));
        }

        @Override
        public TranslateToBuilder<SpecBuilder3> andTranslateTo() {
            return new TranslateToStep<SpecBuilder3>(s, new Spec3(s));
        }
    }

    private static final class Spec3 extends ConfiguredTargetBuilder<SpecBuilder3, SpecBuildStage> implements SpecBuilder3 {
        Spec3(State s) {
            super(s, new SpecTerminal(s));
        }
    }

    private static final class SpecTerminal implements SpecBuildStage {
        private final State s;

        SpecTerminal(State s) {
            this.s = s;
        }

        @Override
        public DriveGuidanceSpec build() {
            return buildSpec(s);
        }
    }

    private static final class Builder0 implements PlanBuilder0 {
        private final State s;

        Builder0(State s) {
            this.s = s;
        }

        @Override
        public TranslateToBuilder<PlanBuilder1> translateTo() {
            return new TranslateToStep<PlanBuilder1>(s, new Builder1(s));
        }

        @Override
        public FaceToBuilder<PlanBuilder2> faceTo() {
            return new FaceToStep<PlanBuilder2>(s, new Builder2(s));
        }
    }

    private static final class Builder1 extends ConfiguredTargetBuilder<PlanBuilder1, PlanOptionalTuningStage> implements PlanBuilder1 {
        Builder1(State s) {
            super(s, new PlanTerminal(s));
        }

        @Override
        public FaceToBuilder<PlanBuilder3> andFaceTo() {
            return new FaceToStep<PlanBuilder3>(s, new Builder3(s));
        }
    }

    private static final class Builder2 extends ConfiguredTargetBuilder<PlanBuilder2, PlanOptionalTuningStage> implements PlanBuilder2 {
        Builder2(State s) {
            super(s, new PlanTerminal(s));
        }

        @Override
        public TranslateToBuilder<PlanBuilder3> andTranslateTo() {
            return new TranslateToStep<PlanBuilder3>(s, new Builder3(s));
        }
    }

    private static final class Builder3 extends ConfiguredTargetBuilder<PlanBuilder3, PlanOptionalTuningStage> implements PlanBuilder3 {
        Builder3(State s) {
            super(s, new PlanTerminal(s));
        }
    }

    private static final class PlanTerminal implements PlanOptionalTuningStage {
        private final State s;

        PlanTerminal(State s) {
            this.s = s;
        }

        @Override
        public DriveTuningBranch driveTuning() {
            return new DriveTuningStep(s, this);
        }

        @Override
        public DriveGuidancePlan build() {
            return buildPlan(s);
        }
    }

    private static final class TranslateToStep<RETURN> implements TranslateToBuilder<RETURN> {
        private final State s;
        private final RETURN ret;

        TranslateToStep(State s, RETURN ret) {
            this.s = s;
            this.ret = ret;
        }

        @Override
        public RETURN fieldPointInches(double xInches, double yInches) {
            if (s.translationTarget != null) {
                throw new IllegalStateException("translateTo() target already configured; choose only one target method");
            }
            s.translationTarget = SpatialTargets.fieldPoint(xInches, yInches);
            return ret;
        }

        @Override
        public RETURN robotRelativePointInches(double forwardInches, double leftInches) {
            if (s.translationTarget != null) {
                throw new IllegalStateException("translateTo() target already configured; choose only one target method");
            }
            s.translationTarget = new DriveGuidanceSpec.RobotRelativePoint(forwardInches, leftInches);
            return ret;
        }

        @Override
        public RETURN point(ReferencePoint2d reference) {
            if (s.translationTarget != null) {
                throw new IllegalStateException("translateTo() target already configured; choose only one target method");
            }
            s.translationTarget = SpatialTargets.point(Objects.requireNonNull(reference, "reference"));
            return ret;
        }

    }

    private static final class FaceToStep<RETURN> implements FaceToBuilder<RETURN> {
        private final State s;
        private final RETURN ret;

        FaceToStep(State s, RETURN ret) {
            this.s = s;
            this.ret = ret;
        }

        @Override
        public RETURN fieldPointInches(double xInches, double yInches) {
            if (s.facingTarget != null) {
                throw new IllegalStateException("faceTo() target already configured; choose only one target method");
            }
            s.facingTarget = SpatialTargets.fieldPoint(xInches, yInches);
            return ret;
        }

        @Override
        public RETURN fieldHeadingRad(double fieldHeadingRad) {
            if (s.facingTarget != null) {
                throw new IllegalStateException("faceTo() target already configured; choose only one target method");
            }
            s.facingTarget = SpatialTargets.fieldHeading(fieldHeadingRad);
            return ret;
        }

        @Override
        public RETURN point(ReferencePoint2d reference) {
            if (s.facingTarget != null) {
                throw new IllegalStateException("faceTo() target already configured; choose only one target method");
            }
            s.facingTarget = SpatialTargets.point(Objects.requireNonNull(reference, "reference"));
            return ret;
        }

        @Override
        public RETURN frameHeading(ReferenceFrame2d reference) {
            return frameHeading(reference, 0.0);
        }

        @Override
        public RETURN frameHeading(ReferenceFrame2d reference, double headingOffsetRad) {
            if (s.facingTarget != null) {
                throw new IllegalStateException("faceTo() target already configured; choose only one target method");
            }
            s.facingTarget = SpatialTargets.frameHeading(Objects.requireNonNull(reference, "reference"), headingOffsetRad);
            return ret;
        }

    }

    private static final class ResolveModeChoiceStep<RETURN> implements ResolveModeChoice<RETURN> {
        private final State s;
        private final RETURN ret;

        ResolveModeChoiceStep(State s, RETURN ret) {
            this.s = s;
            this.ret = ret;
        }

        @Override
        public AbsolutePoseTuningStage<RETURN> absolutePose(AbsolutePoseEstimator poseEstimator) {
            Objects.requireNonNull(poseEstimator, "poseEstimator");
            resetSolve(s, DriveGuidanceSpec.SolveMode.ABSOLUTE_POSE);
            s.poseEstimator = poseEstimator;
            return new AbsolutePoseStep<RETURN>(s, ret);
        }

        @Override
        public RelativeAprilTagsTuningStage<RETURN> relativeAprilTags(
                AprilTagSensor aprilTags, CameraMountConfig cameraMount) {
            Objects.requireNonNull(aprilTags, "aprilTags");
            Objects.requireNonNull(cameraMount, "cameraMount");
            resetSolve(s, DriveGuidanceSpec.SolveMode.RELATIVE_APRIL_TAGS);
            s.aprilTagSensor = aprilTags;
            s.cameraMount = cameraMount;
            return new RelativeAprilTagsStep<RETURN>(s, ret);
        }

        @Override
        public RETURN observedPoints(DriveGuidanceSpec.LossPolicy onLoss) {
            Objects.requireNonNull(onLoss, "onLoss");
            resetSolve(s, DriveGuidanceSpec.SolveMode.OBSERVED_POINTS);
            s.onLoss = onLoss;
            return ret;
        }
    }

    private static final class AbsolutePoseStep<RETURN> implements AbsolutePoseTuningStage<RETURN> {
        private final State s;
        private final RETURN ret;

        AbsolutePoseStep(State s, RETURN ret) {
            this.s = s;
            this.ret = ret;
        }

        @Override public AbsolutePoseTuningStage<RETURN> maxAgeSec(double value) {
            s.poseMaxAgeSec = value;
            return this;
        }
        @Override public AbsolutePoseTuningStage<RETURN> minQuality(double value) {
            s.poseMinQuality = value;
            return this;
        }
        @Override public AbsolutePoseTuningStage<RETURN> fixedAprilTagLayout(TagLayout layout) {
            s.fixedAprilTagLayout = Objects.requireNonNull(layout, "tagLayout");
            return this;
        }
        @Override public AbsolutePoseTuningStage<RETURN> onLoss(DriveGuidanceSpec.LossPolicy value) {
            s.onLoss = Objects.requireNonNull(value, "onLoss");
            return this;
        }
        @Override public RETURN doneAbsolutePose() { return ret; }
    }

    private static final class RelativeAprilTagsStep<RETURN>
            implements RelativeAprilTagsTuningStage<RETURN> {
        private final State s;
        private final RETURN ret;

        RelativeAprilTagsStep(State s, RETURN ret) {
            this.s = s;
            this.ret = ret;
        }

        @Override public RelativeAprilTagsTuningStage<RETURN> maxAgeSec(double value) {
            s.tagsMaxAgeSec = value;
            return this;
        }
        @Override public RelativeAprilTagsTuningStage<RETURN> onLoss(DriveGuidanceSpec.LossPolicy value) {
            s.onLoss = Objects.requireNonNull(value, "onLoss");
            return this;
        }
        @Override public RETURN doneRelativeAprilTags() { return ret; }
    }

    private static final class DriveTuningStep implements DriveTuningBranch {
        private final State s;
        private final PlanOptionalTuningStage ret;

        DriveTuningStep(State s, PlanOptionalTuningStage ret) {
            this.s = s;
            this.ret = ret;
        }

        @Override
        public DriveTuningBranch use(DriveGuidancePlan.Tuning tuning) {
            s.tuning = Objects.requireNonNull(tuning, "tuning");
            return this;
        }

        @Override
        public DriveTuningBranch translateKp(double kPTranslate) {
            s.tuning = s.tuning.withTranslateKp(kPTranslate);
            return this;
        }

        @Override
        public DriveTuningBranch maxTranslateCmd(double maxTranslateCmd) {
            s.tuning = s.tuning.withMaxTranslateCmd(maxTranslateCmd);
            return this;
        }

        @Override
        public DriveTuningBranch aimKp(double kPAim) {
            s.tuning = s.tuning.withAimKp(kPAim);
            return this;
        }

        @Override
        public DriveTuningBranch maxOmegaCmd(double maxOmegaCmd) {
            s.tuning = s.tuning.withMaxOmegaCmd(maxOmegaCmd);
            return this;
        }

        @Override
        public DriveTuningBranch minOmegaCmd(double minOmegaCmd) {
            s.tuning = s.tuning.withMinOmegaCmd(minOmegaCmd);
            return this;
        }

        @Override
        public DriveTuningBranch aimDeadbandRad(double aimDeadbandRad) {
            s.tuning = s.tuning.withAimDeadbandRad(aimDeadbandRad);
            return this;
        }

        @Override
        public PlanOptionalTuningStage doneDriveTuning() {
            return ret;
        }
    }

    private static final class PlanFromSpecTuningStep implements DriveTuningBranch {
        private final PlanFromSpecBuilderImpl parent;

        PlanFromSpecTuningStep(PlanFromSpecBuilderImpl parent) {
            this.parent = parent;
        }

        @Override
        public DriveTuningBranch use(DriveGuidancePlan.Tuning tuning) {
            parent.tuning = Objects.requireNonNull(tuning, "tuning");
            return this;
        }

        @Override
        public DriveTuningBranch translateKp(double kPTranslate) {
            parent.tuning = parent.tuning.withTranslateKp(kPTranslate);
            return this;
        }

        @Override
        public DriveTuningBranch maxTranslateCmd(double maxTranslateCmd) {
            parent.tuning = parent.tuning.withMaxTranslateCmd(maxTranslateCmd);
            return this;
        }

        @Override
        public DriveTuningBranch aimKp(double kPAim) {
            parent.tuning = parent.tuning.withAimKp(kPAim);
            return this;
        }

        @Override
        public DriveTuningBranch maxOmegaCmd(double maxOmegaCmd) {
            parent.tuning = parent.tuning.withMaxOmegaCmd(maxOmegaCmd);
            return this;
        }

        @Override
        public DriveTuningBranch minOmegaCmd(double minOmegaCmd) {
            parent.tuning = parent.tuning.withMinOmegaCmd(minOmegaCmd);
            return this;
        }

        @Override
        public DriveTuningBranch aimDeadbandRad(double aimDeadbandRad) {
            parent.tuning = parent.tuning.withAimDeadbandRad(aimDeadbandRad);
            return this;
        }

        @Override
        public PlanOptionalTuningStage doneDriveTuning() {
            return parent;
        }
    }
}
