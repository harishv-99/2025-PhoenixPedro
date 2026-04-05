package edu.ftcphoenix.fw.drive.guidance;

import java.util.ArrayList;
import java.util.Objects;

import edu.ftcphoenix.fw.drive.DriveOverlay;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.PoseEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;

/**
 * Builder + helpers for creating {@link DriveGuidanceSpec}s and {@link DriveGuidancePlan}s.
 *
 * <p>DriveGuidance is intentionally split into two public objects:</p>
 * <ul>
 *   <li>{@link DriveGuidanceSpec}: controller-neutral <b>what</b> (targets, control frames, feedback)</li>
 *   <li>{@link DriveGuidancePlan}: spec + {@link DriveGuidancePlan.Tuning} (<b>how strongly</b>)</li>
 * </ul>
 *
 * <p>The same plan can be reused three ways:</p>
 * <ul>
 *   <li>{@link DriveGuidancePlan#overlay()} for TeleOp assists</li>
 *   <li>{@link DriveGuidancePlan#task(edu.ftcphoenix.fw.drive.DriveCommandSink, DriveGuidanceTask.Config)}
 *       for autonomous motion</li>
 *   <li>{@link DriveGuidancePlan#query()} for “ready?” checks and telemetry</li>
 * </ul>
 *
 * <h2>Common usage</h2>
 *
 * <pre>{@code
 * ReferenceFrame2d slotFace = References.fieldFrame(48.0, 24.0, Math.PI);
 *
 * DriveGuidancePlan alignPlan = DriveGuidance.plan()
 *         .translateTo()
 *             .referenceFrameOffsetInches(slotFace, -6.0, 0.0)
 *             .doneTranslateTo()
 *         .aimTo()
 *             .referenceFrameHeading(slotFace)
 *             .doneAimTo()
 *         .feedback()
 *             .fieldPose(poseEstimator)
 *             .aprilTags(tagSensor, cameraMount, 0.25)
 *             .fixedTagLayout(tagLayout)
 *             .doneFeedback()
 *         .build();
 * }</pre>
 *
 * <p>This reference-first API replaces older tag-specific public target nouns. Guidance now talks
 * about semantic points / frames, while the evaluation layer decides whether those are solved from
 * field pose, live AprilTags, or both.</p>
 */
public final class DriveGuidance {

    private DriveGuidance() {
        // static utility
    }

    /**
     * Starts building a controller-neutral {@link DriveGuidanceSpec}.
     *
     * <p>Use this when you want to reuse the same targets/feedback with different controller
     * tunings, for example gentler TeleOp assist and stronger autonomous tuning.</p>
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
     */
    public static PlanFromSpecBuilder plan(DriveGuidanceSpec spec) {
        return new PlanFromSpecBuilderImpl(spec);
    }

    /**
     * Minimal builder that combines a pre-built spec with tuning.
     */
    public interface PlanFromSpecBuilder {

        /**
         * Supplies controller tuning for the final plan.
         */
        PlanFromSpecBuilder tuning(DriveGuidancePlan.Tuning tuning);

        /**
         * Builds the final immutable plan.
         */
        DriveGuidancePlan build();
    }

    private static final class PlanFromSpecBuilderImpl implements PlanFromSpecBuilder {
        private final DriveGuidanceSpec spec;
        private DriveGuidancePlan.Tuning tuning = DriveGuidancePlan.Tuning.defaults();

        PlanFromSpecBuilderImpl(DriveGuidanceSpec spec) {
            this.spec = Objects.requireNonNull(spec, "spec");
        }

        @Override
        public PlanFromSpecBuilder tuning(DriveGuidancePlan.Tuning tuning) {
            this.tuning = Objects.requireNonNull(tuning, "tuning");
            return this;
        }

        @Override
        public DriveGuidancePlan build() {
            return new DriveGuidancePlan(spec, tuning);
        }
    }

    /**
     * Creates a pose-lock overlay that holds the current field pose using default tuning.
     */
    public static DriveOverlay poseLock(PoseEstimator poseEstimator) {
        return poseLock(poseEstimator, DriveGuidancePlan.Tuning.defaults());
    }

    /**
     * Creates a pose-lock overlay with custom tuning.
     */
    public static DriveOverlay poseLock(PoseEstimator poseEstimator, DriveGuidancePlan.Tuning tuning) {
        return new PoseLockOverlay(poseEstimator, tuning);
    }

    // ------------------------------------------------------------------------
    // Spec builder staging
    // ------------------------------------------------------------------------

    /**
     * Common methods shared by all <b>spec</b> builder stages.
     *
     * <p>A spec answers “what should guidance do?” rather than “how aggressively should it drive?”
     * so tuning is intentionally absent here.</p>
     */
    public interface SpecBuilderCommon<SELF> {

        /**
         * Configures the feedback lanes guidance may use to solve the requested targets.
         *
         * <p>At least one active lane is required:</p>
         * <ul>
         *   <li>field pose via {@link FeedbackBuilder#fieldPose(PoseEstimator)}</li>
         *   <li>live AprilTags via {@link FeedbackBuilder#aprilTags(AprilTagSensor, CameraMountConfig)}</li>
         *   <li>or both for adaptive behavior</li>
         * </ul>
         */
        FeedbackBuilder<SELF> feedback();

        /**
         * Chooses which point(s) on the robot guidance should translate / aim with respect to.
         */
        SELF controlFrames(ControlFrames frames);

        /**
         * Finishes the builder and returns an immutable {@link DriveGuidanceSpec}.
         */
        DriveGuidanceSpec build();
    }

    /**
     * Initial spec stage: configure translation, aim, or both.
     */
    public interface SpecBuilder0 extends SpecBuilderCommon<SpecBuilder0> {

        /**
         * Begins configuring the translation target.
         */
        TranslateToBuilder<SpecBuilder1> translateTo();

        /**
         * Begins configuring the aim target.
         */
        AimToBuilder<SpecBuilder2> aimTo();
    }

    /**
     * Spec stage after translation has been configured.
     */
    public interface SpecBuilder1 extends SpecBuilderCommon<SpecBuilder1> {

        /**
         * Adds an aim target to the spec.
         */
        AimToBuilder<SpecBuilder3> aimTo();
    }

    /**
     * Spec stage after aim has been configured.
     */
    public interface SpecBuilder2 extends SpecBuilderCommon<SpecBuilder2> {

        /**
         * Adds a translation target to the spec.
         */
        TranslateToBuilder<SpecBuilder3> translateTo();
    }

    /**
     * Final spec stage after both translation and aim have been configured.
     */
    public interface SpecBuilder3 extends SpecBuilderCommon<SpecBuilder3> {
        // no-op
    }

    // ------------------------------------------------------------------------
    // Plan builder staging
    // ------------------------------------------------------------------------

    /**
     * Common methods shared by all plan builder stages.
     *
     * <p>Most one-shot plans follow this pattern:</p>
     * <pre>{@code
     * DriveGuidancePlan plan = DriveGuidance.plan()
     *         .translateTo()...doneTranslateTo()
     *         .aimTo()...doneAimTo()
     *         .feedback()...doneFeedback()
     *         .tuning(DriveGuidancePlan.Tuning.defaults())
     *         .build();
     * }</pre>
     */
    public interface PlanBuilderCommon<SELF> {

        /**
         * Configures the feedback lanes guidance may use to solve the requested targets.
         */
        FeedbackBuilder<SELF> feedback();

        /**
         * Chooses which point(s) on the robot guidance should translate / aim with respect to.
         */
        SELF controlFrames(ControlFrames frames);

        /**
         * Supplies controller tuning for the final plan.
         */
        SELF tuning(DriveGuidancePlan.Tuning tuning);

        /**
         * Builds the immutable guidance plan.
         */
        DriveGuidancePlan build();
    }

    /**
     * Initial plan stage: configure translation, aim, or both.
     */
    public interface PlanBuilder0 extends PlanBuilderCommon<PlanBuilder0> {

        /**
         * Begins configuring the translation target.
         */
        TranslateToBuilder<PlanBuilder1> translateTo();

        /**
         * Begins configuring the aim target.
         */
        AimToBuilder<PlanBuilder2> aimTo();
    }

    /**
     * Plan stage after translation has been configured.
     */
    public interface PlanBuilder1 extends PlanBuilderCommon<PlanBuilder1> {

        /**
         * Adds an aim target to the plan.
         */
        AimToBuilder<PlanBuilder3> aimTo();
    }

    /**
     * Plan stage after aim has been configured.
     */
    public interface PlanBuilder2 extends PlanBuilderCommon<PlanBuilder2> {

        /**
         * Adds a translation target to the plan.
         */
        TranslateToBuilder<PlanBuilder3> translateTo();
    }

    /**
     * Final plan stage after both translation and aim have been configured.
     */
    public interface PlanBuilder3 extends PlanBuilderCommon<PlanBuilder3> {
        // no-op
    }

    // ------------------------------------------------------------------------
    // Nested staged builders
    // ------------------------------------------------------------------------

    /**
     * Nested builder used to describe the translation goal.
     */
    public interface TranslateToBuilder<RETURN> {

        /**
         * Translates toward a field-fixed point.
         */
        TranslateToBuilder<RETURN> fieldPointInches(double xInches, double yInches);

        /**
         * Captures a robot-relative delta when guidance enables.
         *
         * <p>This means “move this far from where the translation control frame is right now”, not
         * “move this far every loop”.</p>
         */
        TranslateToBuilder<RETURN> robotRelativePointInches(double forwardInches, double leftInches);

        /**
         * Translates toward a semantic point reference.
         *
         * <p>The reference may be field-fixed or tag-relative; guidance resolves the appropriate
         * solve path from the configured feedback lanes.</p>
         */
        TranslateToBuilder<RETURN> referencePoint(ReferencePoint2d reference);

        /**
         * Translates toward the origin point of a semantic frame.
         */
        TranslateToBuilder<RETURN> referenceFrameOrigin(ReferenceFrame2d reference);

        /**
         * Translates toward a point expressed as an offset in a semantic reference frame.
         *
         * <p>Example: “go 6 inches in front of the reference frame origin”.</p>
         */
        TranslateToBuilder<RETURN> referenceFrameOffsetInches(ReferenceFrame2d reference,
                                                              double forwardInches,
                                                              double leftInches);

        /**
         * Returns to the parent builder stage.
         */
        RETURN doneTranslateTo();
    }

    /**
     * Nested builder used to describe the aim / heading goal.
     */
    public interface AimToBuilder<RETURN> {

        /**
         * Aims at a field-fixed point.
         */
        AimToBuilder<RETURN> fieldPointInches(double xInches, double yInches);

        /**
         * Aligns to an absolute field heading in radians.
         */
        AimToBuilder<RETURN> fieldHeadingRad(double fieldHeadingRad);

        /**
         * Aligns to an absolute field heading in degrees.
         */
        AimToBuilder<RETURN> fieldHeadingDeg(double fieldHeadingDeg);

        /**
         * Aims at a semantic point reference.
         */
        AimToBuilder<RETURN> referencePoint(ReferencePoint2d reference);

        /**
         * Aims at the origin point of a semantic frame.
         */
        AimToBuilder<RETURN> referenceFrameOrigin(ReferenceFrame2d reference);

        /**
         * Aims at a point expressed as an offset in a semantic reference frame.
         */
        AimToBuilder<RETURN> referenceFrameOffsetInches(ReferenceFrame2d reference,
                                                        double forwardInches,
                                                        double leftInches);

        /**
         * Aligns to the heading of a semantic reference frame.
         */
        AimToBuilder<RETURN> referenceFrameHeading(ReferenceFrame2d reference);

        /**
         * Aligns to the heading of a semantic reference frame plus an additional offset.
         */
        AimToBuilder<RETURN> referenceFrameHeading(ReferenceFrame2d reference, double headingOffsetRad);

        /**
         * Returns to the parent builder stage.
         */
        RETURN doneAimTo();
    }

    /**
     * Nested builder used to describe which feedback lanes guidance may use.
     */
    public interface FeedbackBuilder<RETURN> {

        /**
         * Adds a live AprilTag solve path with default freshness bounds.
         */
        FeedbackBuilder<RETURN> aprilTags(AprilTagSensor aprilTags, CameraMountConfig cameraMount);

        /**
         * Adds a live AprilTag solve path with an explicit freshness bound.
         *
         * <p>This lane is the direct visual solve path. It does not require a localizer.</p>
         */
        FeedbackBuilder<RETURN> aprilTags(AprilTagSensor aprilTags,
                                          CameraMountConfig cameraMount,
                                          double maxAgeSec);

        /**
         * Adds a field-pose solve path with default age/quality gates.
         */
        FeedbackBuilder<RETURN> fieldPose(PoseEstimator poseEstimator);

        /**
         * Adds a field-pose solve path with explicit age/quality gates.
         */
        FeedbackBuilder<RETURN> fieldPose(PoseEstimator poseEstimator,
                                          double maxAgeSec,
                                          double minQuality);

        /**
         * Supplies fixed field metadata for AprilTags.
         *
         * <p>This enables two important behaviors:</p>
         * <ul>
         *   <li>field-fixed references can be solved from visible fixed tags even without a localizer</li>
         *   <li>single-tag-relative references can be promoted into field coordinates, enabling
         *       localizer fallback when that fixed tag is not currently visible</li>
         * </ul>
         */
        FeedbackBuilder<RETURN> fixedTagLayout(TagLayout tagLayout);

        /**
         * Configures adaptive takeover hysteresis and blend timing when both field pose and
         * AprilTags are present.
         */
        FeedbackBuilder<RETURN> gates(double enterRangeInches, double exitRangeInches, double takeoverBlendSec);

        /**
         * Whether omega should prefer the live AprilTag lane whenever it is valid.
         */
        FeedbackBuilder<RETURN> preferAprilTagsForOmega(boolean prefer);

        /**
         * Chooses what happens when guidance cannot solve the requested channels.
         */
        FeedbackBuilder<RETURN> lossPolicy(DriveGuidanceSpec.LossPolicy lossPolicy);

        /**
         * Returns to the parent builder stage.
         */
        RETURN doneFeedback();
    }

    // ------------------------------------------------------------------------
    // Implementation
    // ------------------------------------------------------------------------

    private static final class State {
        DriveGuidanceSpec.TranslationTarget translationTarget;
        DriveGuidanceSpec.AimTarget aimTarget;

        ControlFrames controlFrames = ControlFrames.robotCenter();
        DriveGuidancePlan.Tuning tuning = DriveGuidancePlan.Tuning.defaults();

        AprilTagSensor aprilTagSensor;
        CameraMountConfig cameraMount;
        double tagsMaxAgeSec = DriveGuidanceSpec.AprilTags.DEFAULT_MAX_AGE_SEC;

        PoseEstimator poseEstimator;
        double poseMaxAgeSec = DriveGuidanceSpec.FieldPose.DEFAULT_MAX_AGE_SEC;
        double poseMinQuality = DriveGuidanceSpec.FieldPose.DEFAULT_MIN_QUALITY;

        TagLayout fixedTagLayout;
        DriveGuidanceSpec.Gates gates;
        boolean preferAprilTagsForOmega = true;
        DriveGuidanceSpec.LossPolicy lossPolicy = DriveGuidanceSpec.LossPolicy.PASS_THROUGH;
    }

    /**
     * Validates the staged builder state and produces an immutable spec snapshot.
     */
    private static DriveGuidanceSpec buildSpec(State s) {
        if (s.translationTarget == null && s.aimTarget == null) {
            throw new IllegalStateException("DriveGuidance spec needs translateTo() and/or aimTo() configured");
        }

        validateCapabilitiesOrThrow(s);

        DriveGuidanceSpec.AprilTags tags = null;
        if (s.aprilTagSensor != null && s.cameraMount != null) {
            tags = new DriveGuidanceSpec.AprilTags(s.aprilTagSensor, s.cameraMount, s.tagsMaxAgeSec);
        }

        DriveGuidanceSpec.FieldPose fieldPose = null;
        if (s.poseEstimator != null) {
            fieldPose = new DriveGuidanceSpec.FieldPose(s.poseEstimator, s.poseMaxAgeSec, s.poseMinQuality);
        }

        DriveGuidanceSpec.Feedback fb = DriveGuidanceSpec.Feedback.create(
                tags,
                fieldPose,
                s.fixedTagLayout,
                s.gates,
                s.preferAprilTagsForOmega,
                s.lossPolicy
        );

        return new DriveGuidanceSpec(s.translationTarget, s.aimTarget, s.controlFrames, fb);
    }

    /**
     * Builds a plan directly from the staged builder state.
     */
    private static DriveGuidancePlan buildPlan(State s) {
        return new DriveGuidancePlan(buildSpec(s), s.tuning);
    }

    /**
     * Ensures the configured targets can be solved by the configured feedback lanes.
     *
     * <p>These checks intentionally fail fast with actionable messages so students find
     * misconfigurations while wiring the plan rather than later during runtime debugging.</p>
     */
    private static void validateCapabilitiesOrThrow(State s) {
        ArrayList<String> errors = new ArrayList<>();

        boolean hasAprilTags = s.aprilTagSensor != null && s.cameraMount != null;
        boolean hasFieldPose = s.poseEstimator != null;
        boolean hasLayout = s.fixedTagLayout != null;

        if (!hasAprilTags && !hasFieldPose) {
            errors.add("feedback() must configure aprilTags(...) and/or fieldPose(...)");
        }

        if ((s.aprilTagSensor != null) ^ (s.cameraMount != null)) {
            errors.add("aprilTags(...) requires both an AprilTagSensor and a CameraMountConfig");
        }

        if (hasAprilTags && (!Double.isFinite(s.tagsMaxAgeSec) || s.tagsMaxAgeSec < 0.0)) {
            errors.add("aprilTags(...): maxAgeSec must be >= 0");
        }

        if (hasFieldPose) {
            if (!Double.isFinite(s.poseMaxAgeSec) || s.poseMaxAgeSec < 0.0) {
                errors.add("fieldPose(...): maxAgeSec must be >= 0");
            }
            if (!Double.isFinite(s.poseMinQuality) || s.poseMinQuality < 0.0 || s.poseMinQuality > 1.0) {
                errors.add("fieldPose(...): minQuality must be in [0, 1]");
            }
        }

        if (s.gates != null) {
            if (!Double.isFinite(s.gates.enterRangeInches) || s.gates.enterRangeInches < 0.0) {
                errors.add("gates(...): enterRangeInches must be >= 0");
            }
            if (!Double.isFinite(s.gates.exitRangeInches) || s.gates.exitRangeInches < 0.0) {
                errors.add("gates(...): exitRangeInches must be >= 0");
            }
            if (!Double.isFinite(s.gates.takeoverBlendSec) || s.gates.takeoverBlendSec < 0.0) {
                errors.add("gates(...): takeoverBlendSec must be >= 0");
            }
            if (Double.isFinite(s.gates.enterRangeInches)
                    && Double.isFinite(s.gates.exitRangeInches)
                    && s.gates.exitRangeInches < s.gates.enterRangeInches) {
                errors.add("gates(...): exitRangeInches must be >= enterRangeInches");
            }
            if (!(hasAprilTags && hasFieldPose)) {
                errors.add("gates(...) is only used when both aprilTags(...) and fieldPose(...) are configured");
            }
        }

        if (s.translationTarget instanceof DriveGuidanceSpec.RobotRelativePoint && !hasFieldPose) {
            errors.add("robotRelativePointInches(...) requires fieldPose(...) feedback");
        }

        if (s.translationTarget != null) {
            boolean canField = hasFieldPose && canSolveTranslationWithFieldPose(s.translationTarget, s.fixedTagLayout);
            boolean canTags = hasAprilTags && canSolveTranslationWithAprilTags(s.translationTarget, hasLayout);
            if (!canField && !canTags) {
                errors.add("translateTo() target cannot be solved with the configured feedback; add fieldPose(...), aprilTags(...), and/or fixedTagLayout(...) as appropriate");
            }
        }

        if (s.aimTarget != null) {
            boolean canField = hasFieldPose && canSolveAimWithFieldPose(s.aimTarget, s.fixedTagLayout);
            boolean canTags = hasAprilTags && canSolveAimWithAprilTags(s.aimTarget, hasLayout);
            if (!canField && !canTags) {
                errors.add("aimTo() target cannot be solved with the configured feedback; add fieldPose(...), aprilTags(...), and/or fixedTagLayout(...) as appropriate");
            }
        }

        if (!errors.isEmpty()) {
            StringBuilder msg = new StringBuilder();
            msg.append("Invalid DriveGuidance plan:\n");
            for (String error : errors) {
                msg.append(" - ").append(error).append('\n');
            }
            throw new IllegalStateException(msg.toString());
        }
    }

    /**
     * Returns whether the translation target can be solved from the field-pose lane.
     */
    private static boolean canSolveTranslationWithFieldPose(DriveGuidanceSpec.TranslationTarget target,
                                                            TagLayout layout) {
        if (target instanceof DriveGuidanceSpec.FieldPoint) {
            return true;
        }
        if (target instanceof DriveGuidanceSpec.RobotRelativePoint) {
            return true;
        }
        if (target instanceof DriveGuidanceSpec.ReferencePointTarget) {
            ReferencePoint2d ref = ((DriveGuidanceSpec.ReferencePointTarget) target).reference;
            return References.isFieldPoint(ref) || References.isSingleFixedTagCandidate(ref, layout);
        }
        if (target instanceof DriveGuidanceSpec.ReferenceFrameOriginTarget) {
            ReferenceFrame2d ref = ((DriveGuidanceSpec.ReferenceFrameOriginTarget) target).reference;
            return References.isFieldFrame(ref) || References.isSingleFixedTagCandidate(ref, layout);
        }
        if (target instanceof DriveGuidanceSpec.ReferenceFrameOffsetTarget) {
            ReferenceFrame2d ref = ((DriveGuidanceSpec.ReferenceFrameOffsetTarget) target).reference;
            return References.isFieldFrame(ref) || References.isSingleFixedTagCandidate(ref, layout);
        }
        return false;
    }

    /**
     * Returns whether the translation target can be solved from the live AprilTag lane.
     */
    private static boolean canSolveTranslationWithAprilTags(DriveGuidanceSpec.TranslationTarget target,
                                                            boolean hasLayout) {
        if (target instanceof DriveGuidanceSpec.RobotRelativePoint) {
            return false;
        }
        if (target instanceof DriveGuidanceSpec.FieldPoint) {
            return hasLayout;
        }
        if (target instanceof DriveGuidanceSpec.ReferencePointTarget) {
            ReferencePoint2d ref = ((DriveGuidanceSpec.ReferencePointTarget) target).reference;
            return References.isTagRelativePoint(ref) || (References.isFieldPoint(ref) && hasLayout);
        }
        if (target instanceof DriveGuidanceSpec.ReferenceFrameOriginTarget) {
            ReferenceFrame2d ref = ((DriveGuidanceSpec.ReferenceFrameOriginTarget) target).reference;
            return References.isTagRelativeFrame(ref) || (References.isFieldFrame(ref) && hasLayout);
        }
        if (target instanceof DriveGuidanceSpec.ReferenceFrameOffsetTarget) {
            ReferenceFrame2d ref = ((DriveGuidanceSpec.ReferenceFrameOffsetTarget) target).reference;
            return References.isTagRelativeFrame(ref) || (References.isFieldFrame(ref) && hasLayout);
        }
        return false;
    }

    /**
     * Returns whether the aim target can be solved from the field-pose lane.
     */
    private static boolean canSolveAimWithFieldPose(DriveGuidanceSpec.AimTarget target,
                                                    TagLayout layout) {
        if (target instanceof DriveGuidanceSpec.FieldPoint) {
            return true;
        }
        if (target instanceof DriveGuidanceSpec.FieldHeading) {
            return true;
        }
        if (target instanceof DriveGuidanceSpec.ReferencePointTarget) {
            ReferencePoint2d ref = ((DriveGuidanceSpec.ReferencePointTarget) target).reference;
            return References.isFieldPoint(ref) || References.isSingleFixedTagCandidate(ref, layout);
        }
        if (target instanceof DriveGuidanceSpec.ReferenceFrameOriginTarget) {
            ReferenceFrame2d ref = ((DriveGuidanceSpec.ReferenceFrameOriginTarget) target).reference;
            return References.isFieldFrame(ref) || References.isSingleFixedTagCandidate(ref, layout);
        }
        if (target instanceof DriveGuidanceSpec.ReferenceFrameOffsetTarget) {
            ReferenceFrame2d ref = ((DriveGuidanceSpec.ReferenceFrameOffsetTarget) target).reference;
            return References.isFieldFrame(ref) || References.isSingleFixedTagCandidate(ref, layout);
        }
        if (target instanceof DriveGuidanceSpec.ReferenceFrameHeadingTarget) {
            ReferenceFrame2d ref = ((DriveGuidanceSpec.ReferenceFrameHeadingTarget) target).reference;
            return References.isFieldFrame(ref) || References.isSingleFixedTagCandidate(ref, layout);
        }
        return false;
    }

    /**
     * Returns whether the aim target can be solved from the live AprilTag lane.
     */
    private static boolean canSolveAimWithAprilTags(DriveGuidanceSpec.AimTarget target,
                                                    boolean hasLayout) {
        if (target instanceof DriveGuidanceSpec.FieldPoint) {
            return hasLayout;
        }
        if (target instanceof DriveGuidanceSpec.FieldHeading) {
            return hasLayout;
        }
        if (target instanceof DriveGuidanceSpec.ReferencePointTarget) {
            ReferencePoint2d ref = ((DriveGuidanceSpec.ReferencePointTarget) target).reference;
            return References.isTagRelativePoint(ref) || (References.isFieldPoint(ref) && hasLayout);
        }
        if (target instanceof DriveGuidanceSpec.ReferenceFrameOriginTarget) {
            ReferenceFrame2d ref = ((DriveGuidanceSpec.ReferenceFrameOriginTarget) target).reference;
            return References.isTagRelativeFrame(ref) || (References.isFieldFrame(ref) && hasLayout);
        }
        if (target instanceof DriveGuidanceSpec.ReferenceFrameOffsetTarget) {
            ReferenceFrame2d ref = ((DriveGuidanceSpec.ReferenceFrameOffsetTarget) target).reference;
            return References.isTagRelativeFrame(ref) || (References.isFieldFrame(ref) && hasLayout);
        }
        if (target instanceof DriveGuidanceSpec.ReferenceFrameHeadingTarget) {
            ReferenceFrame2d ref = ((DriveGuidanceSpec.ReferenceFrameHeadingTarget) target).reference;
            return References.isTagRelativeFrame(ref) || (References.isFieldFrame(ref) && hasLayout);
        }
        return false;
    }

    /**
     * Shared stateful base for the staged builders.
     */
    private static abstract class CommonBuilder<SELF> {
        final State s;

        CommonBuilder(State s) {
            this.s = s;
        }

        @SuppressWarnings("unchecked")
        final SELF self() {
            return (SELF) this;
        }

        public final SELF controlFrames(ControlFrames frames) {
            s.controlFrames = Objects.requireNonNull(frames, "frames");
            return self();
        }

        public final FeedbackBuilder<SELF> feedback() {
            return new FeedbackStep<>(s, self());
        }
    }

    /**
     * Base implementation for spec builder stages.
     */
    private static abstract class SpecBaseBuilder<SELF> extends CommonBuilder<SELF> {
        SpecBaseBuilder(State s) {
            super(s);
        }

        public final DriveGuidanceSpec build() {
            return buildSpec(s);
        }
    }

    /**
     * Base implementation for plan builder stages.
     */
    private static abstract class PlanBaseBuilder<SELF> extends CommonBuilder<SELF> {
        PlanBaseBuilder(State s) {
            super(s);
        }

        public final SELF tuning(DriveGuidancePlan.Tuning tuning) {
            s.tuning = Objects.requireNonNull(tuning, "tuning");
            return self();
        }

        public final DriveGuidancePlan build() {
            return buildPlan(s);
        }
    }

    private static final class Spec0 extends SpecBaseBuilder<SpecBuilder0> implements SpecBuilder0 {
        Spec0(State s) {
            super(s);
        }

        @Override
        public TranslateToBuilder<SpecBuilder1> translateTo() {
            return new TranslateToStep<>(s, new Spec1(s));
        }

        @Override
        public AimToBuilder<SpecBuilder2> aimTo() {
            return new AimToStep<>(s, new Spec2(s));
        }
    }

    private static final class Spec1 extends SpecBaseBuilder<SpecBuilder1> implements SpecBuilder1 {
        Spec1(State s) {
            super(s);
        }

        @Override
        public AimToBuilder<SpecBuilder3> aimTo() {
            return new AimToStep<>(s, new Spec3(s));
        }
    }

    private static final class Spec2 extends SpecBaseBuilder<SpecBuilder2> implements SpecBuilder2 {
        Spec2(State s) {
            super(s);
        }

        @Override
        public TranslateToBuilder<SpecBuilder3> translateTo() {
            return new TranslateToStep<>(s, new Spec3(s));
        }
    }

    private static final class Spec3 extends SpecBaseBuilder<SpecBuilder3> implements SpecBuilder3 {
        Spec3(State s) {
            super(s);
        }
    }

    private static final class Builder0 extends PlanBaseBuilder<PlanBuilder0> implements PlanBuilder0 {
        Builder0(State s) {
            super(s);
        }

        @Override
        public TranslateToBuilder<PlanBuilder1> translateTo() {
            return new TranslateToStep<>(s, new Builder1(s));
        }

        @Override
        public AimToBuilder<PlanBuilder2> aimTo() {
            return new AimToStep<>(s, new Builder2(s));
        }
    }

    private static final class Builder1 extends PlanBaseBuilder<PlanBuilder1> implements PlanBuilder1 {
        Builder1(State s) {
            super(s);
        }

        @Override
        public AimToBuilder<PlanBuilder3> aimTo() {
            return new AimToStep<>(s, new Builder3(s));
        }
    }

    private static final class Builder2 extends PlanBaseBuilder<PlanBuilder2> implements PlanBuilder2 {
        Builder2(State s) {
            super(s);
        }

        @Override
        public TranslateToBuilder<PlanBuilder3> translateTo() {
            return new TranslateToStep<>(s, new Builder3(s));
        }
    }

    private static final class Builder3 extends PlanBaseBuilder<PlanBuilder3> implements PlanBuilder3 {
        Builder3(State s) {
            super(s);
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
        public TranslateToBuilder<RETURN> fieldPointInches(double xInches, double yInches) {
            if (s.translationTarget != null) {
                throw new IllegalStateException("translateTo() target already configured; choose only one target method");
            }
            s.translationTarget = new DriveGuidanceSpec.FieldPoint(xInches, yInches);
            return this;
        }

        @Override
        public TranslateToBuilder<RETURN> robotRelativePointInches(double forwardInches, double leftInches) {
            if (s.translationTarget != null) {
                throw new IllegalStateException("translateTo() target already configured; choose only one target method");
            }
            s.translationTarget = new DriveGuidanceSpec.RobotRelativePoint(forwardInches, leftInches);
            return this;
        }

        @Override
        public TranslateToBuilder<RETURN> referencePoint(ReferencePoint2d reference) {
            if (s.translationTarget != null) {
                throw new IllegalStateException("translateTo() target already configured; choose only one target method");
            }
            s.translationTarget = new DriveGuidanceSpec.ReferencePointTarget(reference);
            return this;
        }

        @Override
        public TranslateToBuilder<RETURN> referenceFrameOrigin(ReferenceFrame2d reference) {
            if (s.translationTarget != null) {
                throw new IllegalStateException("translateTo() target already configured; choose only one target method");
            }
            s.translationTarget = new DriveGuidanceSpec.ReferenceFrameOriginTarget(reference);
            return this;
        }

        @Override
        public TranslateToBuilder<RETURN> referenceFrameOffsetInches(ReferenceFrame2d reference,
                                                                     double forwardInches,
                                                                     double leftInches) {
            if (s.translationTarget != null) {
                throw new IllegalStateException("translateTo() target already configured; choose only one target method");
            }
            s.translationTarget = new DriveGuidanceSpec.ReferenceFrameOffsetTarget(reference, forwardInches, leftInches);
            return this;
        }

        @Override
        public RETURN doneTranslateTo() {
            if (s.translationTarget == null) {
                throw new IllegalStateException("translateTo() requires a target before doneTranslateTo()");
            }
            return ret;
        }
    }

    private static final class AimToStep<RETURN> implements AimToBuilder<RETURN> {
        private final State s;
        private final RETURN ret;

        AimToStep(State s, RETURN ret) {
            this.s = s;
            this.ret = ret;
        }

        @Override
        public AimToBuilder<RETURN> fieldPointInches(double xInches, double yInches) {
            if (s.aimTarget != null) {
                throw new IllegalStateException("aimTo() target already configured; choose only one target method");
            }
            s.aimTarget = new DriveGuidanceSpec.FieldPoint(xInches, yInches);
            return this;
        }

        @Override
        public AimToBuilder<RETURN> fieldHeadingRad(double fieldHeadingRad) {
            if (s.aimTarget != null) {
                throw new IllegalStateException("aimTo() target already configured; choose only one target method");
            }
            s.aimTarget = new DriveGuidanceSpec.FieldHeading(fieldHeadingRad);
            return this;
        }

        @Override
        public AimToBuilder<RETURN> fieldHeadingDeg(double fieldHeadingDeg) {
            return fieldHeadingRad(Math.toRadians(fieldHeadingDeg));
        }

        @Override
        public AimToBuilder<RETURN> referencePoint(ReferencePoint2d reference) {
            if (s.aimTarget != null) {
                throw new IllegalStateException("aimTo() target already configured; choose only one target method");
            }
            s.aimTarget = new DriveGuidanceSpec.ReferencePointTarget(reference);
            return this;
        }

        @Override
        public AimToBuilder<RETURN> referenceFrameOrigin(ReferenceFrame2d reference) {
            if (s.aimTarget != null) {
                throw new IllegalStateException("aimTo() target already configured; choose only one target method");
            }
            s.aimTarget = new DriveGuidanceSpec.ReferenceFrameOriginTarget(reference);
            return this;
        }

        @Override
        public AimToBuilder<RETURN> referenceFrameOffsetInches(ReferenceFrame2d reference,
                                                               double forwardInches,
                                                               double leftInches) {
            if (s.aimTarget != null) {
                throw new IllegalStateException("aimTo() target already configured; choose only one target method");
            }
            s.aimTarget = new DriveGuidanceSpec.ReferenceFrameOffsetTarget(reference, forwardInches, leftInches);
            return this;
        }

        @Override
        public AimToBuilder<RETURN> referenceFrameHeading(ReferenceFrame2d reference) {
            return referenceFrameHeading(reference, 0.0);
        }

        @Override
        public AimToBuilder<RETURN> referenceFrameHeading(ReferenceFrame2d reference, double headingOffsetRad) {
            if (s.aimTarget != null) {
                throw new IllegalStateException("aimTo() target already configured; choose only one target method");
            }
            s.aimTarget = new DriveGuidanceSpec.ReferenceFrameHeadingTarget(reference, headingOffsetRad);
            return this;
        }

        @Override
        public RETURN doneAimTo() {
            if (s.aimTarget == null) {
                throw new IllegalStateException("aimTo() requires a target before doneAimTo()");
            }
            return ret;
        }
    }

    private static final class FeedbackStep<RETURN> implements FeedbackBuilder<RETURN> {
        private final State s;
        private final RETURN ret;

        FeedbackStep(State s, RETURN ret) {
            this.s = s;
            this.ret = ret;
        }

        @Override
        public FeedbackBuilder<RETURN> aprilTags(AprilTagSensor aprilTags, CameraMountConfig cameraMount) {
            s.aprilTagSensor = Objects.requireNonNull(aprilTags, "aprilTags");
            s.cameraMount = Objects.requireNonNull(cameraMount, "cameraMount");
            s.tagsMaxAgeSec = DriveGuidanceSpec.AprilTags.DEFAULT_MAX_AGE_SEC;
            return this;
        }

        @Override
        public FeedbackBuilder<RETURN> aprilTags(AprilTagSensor aprilTags,
                                                 CameraMountConfig cameraMount,
                                                 double maxAgeSec) {
            s.aprilTagSensor = Objects.requireNonNull(aprilTags, "aprilTags");
            s.cameraMount = Objects.requireNonNull(cameraMount, "cameraMount");
            s.tagsMaxAgeSec = maxAgeSec;
            return this;
        }

        @Override
        public FeedbackBuilder<RETURN> fieldPose(PoseEstimator poseEstimator) {
            s.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator");
            s.poseMaxAgeSec = DriveGuidanceSpec.FieldPose.DEFAULT_MAX_AGE_SEC;
            s.poseMinQuality = DriveGuidanceSpec.FieldPose.DEFAULT_MIN_QUALITY;
            return this;
        }

        @Override
        public FeedbackBuilder<RETURN> fieldPose(PoseEstimator poseEstimator,
                                                 double maxAgeSec,
                                                 double minQuality) {
            s.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator");
            s.poseMaxAgeSec = maxAgeSec;
            s.poseMinQuality = minQuality;
            return this;
        }

        @Override
        public FeedbackBuilder<RETURN> fixedTagLayout(TagLayout tagLayout) {
            s.fixedTagLayout = tagLayout;
            return this;
        }

        @Override
        public FeedbackBuilder<RETURN> gates(double enterRangeInches, double exitRangeInches, double takeoverBlendSec) {
            s.gates = new DriveGuidanceSpec.Gates(enterRangeInches, exitRangeInches, takeoverBlendSec);
            return this;
        }

        @Override
        public FeedbackBuilder<RETURN> preferAprilTagsForOmega(boolean prefer) {
            s.preferAprilTagsForOmega = prefer;
            return this;
        }

        @Override
        public FeedbackBuilder<RETURN> lossPolicy(DriveGuidanceSpec.LossPolicy lossPolicy) {
            s.lossPolicy = Objects.requireNonNull(lossPolicy, "lossPolicy");
            return this;
        }

        @Override
        public RETURN doneFeedback() {
            return ret;
        }
    }
}
