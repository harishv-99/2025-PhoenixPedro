package edu.ftcphoenix.fw.drive.guidance;

import java.util.ArrayList;
import java.util.Objects;
import java.util.Set;

import edu.ftcphoenix.fw.drive.DriveOverlay;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.apriltag.FixedTagFieldPoseSolver;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;

/**
 * Builder + helpers for creating {@link DriveGuidanceSpec}s and {@link DriveGuidancePlan}s.
 *
 * <p>DriveGuidance is intentionally split into two public objects:</p>
 * <ul>
 *   <li>{@link DriveGuidanceSpec}: controller-neutral <b>what</b> (targets, control frames, solve lanes)</li>
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
 *             .point(References.framePoint(slotFace, -6.0, 0.0))
 *             .doneTranslateTo()
 *         .aimTo()
 *             .frameHeading(slotFace)
 *             .doneAimTo()
 *         .resolveWith()
 *             .localization(poseEstimator)
 *             .aprilTags(tagSensor, cameraMount, 0.25)
 *             .fixedAprilTagLayout(tagLayout)
 *             .doneResolveWith()
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
     * <p>Use this when you want to reuse the same targets/solve-lane configuration with different controller
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

        /**
         * {@inheritDoc}
         */
        @Override
        public PlanFromSpecBuilder tuning(DriveGuidancePlan.Tuning tuning) {
            this.tuning = Objects.requireNonNull(tuning, "tuning");
            return this;
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
     */
    public static DriveOverlay poseLock(AbsolutePoseEstimator poseEstimator) {
        return poseLock(poseEstimator, DriveGuidancePlan.Tuning.defaults());
    }

    /**
     * Creates a pose-lock overlay with custom tuning.
     */
    public static DriveOverlay poseLock(AbsolutePoseEstimator poseEstimator, DriveGuidancePlan.Tuning tuning) {
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
         *   <li>field pose via {@link ResolveWithBuilder#localization(AbsolutePoseEstimator)}</li>
         *   <li>live AprilTags via {@link ResolveWithBuilder#aprilTags(AprilTagSensor, CameraMountConfig)}</li>
         *   <li>or both for adaptive behavior</li>
         * </ul>
         */
        ResolveWithBuilder<SELF> resolveWith();

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
     *         .resolveWith()...doneResolveWith()
     *         .tuning(DriveGuidancePlan.Tuning.defaults())
     *         .build();
     * }</pre>
     */
    public interface PlanBuilderCommon<SELF> {

        /**
         * Configures the feedback lanes guidance may use to solve the requested targets.
         */
        ResolveWithBuilder<SELF> resolveWith();

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
         */
        TranslateToBuilder<RETURN> robotRelativePointInches(double forwardInches, double leftInches);

        /**
         * Translates toward a semantic point reference.
         */
        TranslateToBuilder<RETURN> point(ReferencePoint2d reference);

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
         * Aims at a semantic point reference.
         */
        AimToBuilder<RETURN> point(ReferencePoint2d reference);

        /**
         * Aligns to the heading of a semantic reference frame.
         */
        AimToBuilder<RETURN> frameHeading(ReferenceFrame2d reference);

        /**
         * Aligns to the heading of a semantic reference frame plus an additional offset.
         */
        AimToBuilder<RETURN> frameHeading(ReferenceFrame2d reference, double headingOffsetRad);

        /**
         * Returns to the parent builder stage.
         */
        RETURN doneAimTo();
    }

    /**
     * Nested builder used to describe which solve lanes guidance may use.
     */
    public interface ResolveWithBuilder<RETURN> {

        /**
         * Explicitly declare that guidance should use localization only.
         */
        ResolveWithBuilder<RETURN> localizationOnly();

        /**
         * Explicitly declare that guidance should use AprilTags only.
         */
        ResolveWithBuilder<RETURN> aprilTagsOnly();

        /**
         * Explicitly declare that guidance should use adaptive arbitration between localization and
         * live AprilTags.
         */
        ResolveWithBuilder<RETURN> adaptive();

        /**
         * Adds a live AprilTag solve lane with default freshness bounds.
         */
        ResolveWithBuilder<RETURN> aprilTags(AprilTagSensor aprilTags, CameraMountConfig cameraMount);

        /**
         * Adds a live AprilTag solve lane with an explicit freshness bound.
         */
        ResolveWithBuilder<RETURN> aprilTags(AprilTagSensor aprilTags,
                                             CameraMountConfig cameraMount,
                                             double maxAgeSec);

        /**
         * Adds a localization solve lane with default age/quality bounds.
         */
        ResolveWithBuilder<RETURN> localization(AbsolutePoseEstimator poseEstimator);

        /**
         * Adds a localization solve lane with explicit age/quality bounds.
         */
        ResolveWithBuilder<RETURN> localization(AbsolutePoseEstimator poseEstimator,
                                                double maxAgeSec,
                                                double minQuality);

        /**
         * Supplies fixed field metadata for field-fixed AprilTags.
         */
        ResolveWithBuilder<RETURN> fixedAprilTagLayout(TagLayout tagLayout);

        /**
         * Overrides the shared multi-tag field-pose solver configuration used by the live
         * AprilTag guidance lane when it temporarily promotes fixed tags into a field pose.
         *
         * <p>This lets guidance use the same weighting / outlier / plausibility policy as an
         * AprilTag-only localizer instead of silently falling back to helper defaults.</p>
         *
         * <p>The framework normalizes this to the shared base solver config at the API boundary.
         * Subclass-specific extras (for example {@code AprilTagPoseEstimator.Config.cameraMount})
         * are intentionally ignored here because they belong to other APIs.</p>
         */
        ResolveWithBuilder<RETURN> aprilTagFieldPoseConfig(FixedTagFieldPoseSolver.Config cfg);

        /**
         * Configures adaptive translation takeover hysteresis and blend timing.
         */
        ResolveWithBuilder<RETURN> translationTakeover(double enterRangeInches,
                                                       double exitRangeInches,
                                                       double blendSec);

        /**
         * Configures adaptive omega arbitration.
         */
        ResolveWithBuilder<RETURN> omegaPolicy(DriveGuidanceSpec.OmegaPolicy omegaPolicy);

        /**
         * Chooses what happens when guidance cannot solve the requested channels.
         */
        ResolveWithBuilder<RETURN> onLoss(DriveGuidanceSpec.LossPolicy onLoss);

        /**
         * Returns to the parent builder stage.
         */
        RETURN doneResolveWith();
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
        FixedTagFieldPoseSolver.Config aprilTagFieldPoseConfig = FixedTagFieldPoseSolver.Config.defaults();

        AbsolutePoseEstimator poseEstimator;
        double poseMaxAgeSec = DriveGuidanceSpec.Localization.DEFAULT_MAX_AGE_SEC;
        double poseMinQuality = DriveGuidanceSpec.Localization.DEFAULT_MIN_QUALITY;

        DriveGuidanceSpec.SolveMode solveMode;
        TagLayout fixedAprilTagLayout;
        DriveGuidanceSpec.TranslationTakeover translationTakeover;
        DriveGuidanceSpec.OmegaPolicy omegaPolicy = DriveGuidanceSpec.OmegaPolicy.PREFER_APRIL_TAGS_WHEN_VALID;
        boolean omegaPolicyExplicit = false;
        DriveGuidanceSpec.LossPolicy onLoss = DriveGuidanceSpec.LossPolicy.PASS_THROUGH;
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
            tags = new DriveGuidanceSpec.AprilTags(
                    s.aprilTagSensor,
                    s.cameraMount,
                    s.tagsMaxAgeSec,
                    s.aprilTagFieldPoseConfig
            );
        }

        DriveGuidanceSpec.Localization localization = null;
        if (s.poseEstimator != null) {
            localization = new DriveGuidanceSpec.Localization(s.poseEstimator, s.poseMaxAgeSec, s.poseMinQuality);
        }

        DriveGuidanceSpec.SolveMode mode = effectiveSolveMode(s, tags != null, localization != null);
        DriveGuidanceSpec.ResolveWith rw = DriveGuidanceSpec.ResolveWith.create(
                mode,
                tags,
                localization,
                s.fixedAprilTagLayout,
                s.translationTakeover,
                s.omegaPolicy,
                s.onLoss
        );

        return new DriveGuidanceSpec(s.translationTarget, s.aimTarget, s.controlFrames, rw);
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
    private static void validateCapabilitiesOrThrow(State s) {
        ArrayList<String> errors = new ArrayList<String>();

        boolean hasAprilTags = s.aprilTagSensor != null && s.cameraMount != null;
        boolean hasLocalization = s.poseEstimator != null;
        boolean hasLayout = s.fixedAprilTagLayout != null;

        if (!hasAprilTags && !hasLocalization) {
            errors.add("resolveWith() must configure localization(...), aprilTags(...), or both");
        }

        if ((s.aprilTagSensor != null) ^ (s.cameraMount != null)) {
            errors.add("aprilTags(...) requires both an AprilTagSensor and a CameraMountConfig");
        }

        if (hasAprilTags && (!Double.isFinite(s.tagsMaxAgeSec) || s.tagsMaxAgeSec < 0.0)) {
            errors.add("aprilTags(...): maxAgeSec must be >= 0");
        }

        if (hasLocalization) {
            if (!Double.isFinite(s.poseMaxAgeSec) || s.poseMaxAgeSec < 0.0) {
                errors.add("localization(...): maxAgeSec must be >= 0");
            }
            if (!Double.isFinite(s.poseMinQuality) || s.poseMinQuality < 0.0 || s.poseMinQuality > 1.0) {
                errors.add("localization(...): minQuality must be in [0, 1]");
            }
        }

        DriveGuidanceSpec.SolveMode mode = effectiveSolveMode(s, hasAprilTags, hasLocalization);
        if (mode == DriveGuidanceSpec.SolveMode.LOCALIZATION_ONLY) {
            if (!hasLocalization) {
                errors.add("localizationOnly() requires localization(...)");
            }
            if (hasAprilTags) {
                errors.add("localizationOnly() does not accept aprilTags(...); use adaptive() when both lanes are intended");
            }
        } else if (mode == DriveGuidanceSpec.SolveMode.APRIL_TAGS_ONLY) {
            if (!hasAprilTags) {
                errors.add("aprilTagsOnly() requires aprilTags(...)");
            }
            if (hasLocalization) {
                errors.add("aprilTagsOnly() does not accept localization(...); use adaptive() when both lanes are intended");
            }
        } else if (mode == DriveGuidanceSpec.SolveMode.ADAPTIVE) {
            if (!hasAprilTags || !hasLocalization) {
                errors.add("adaptive() requires both localization(...) and aprilTags(...)");
            }
        }

        if (s.translationTakeover != null) {
            if (!Double.isFinite(s.translationTakeover.enterRangeInches) || s.translationTakeover.enterRangeInches < 0.0) {
                errors.add("translationTakeover(...): enterRangeInches must be >= 0");
            }
            if (!Double.isFinite(s.translationTakeover.exitRangeInches) || s.translationTakeover.exitRangeInches < 0.0) {
                errors.add("translationTakeover(...): exitRangeInches must be >= 0");
            }
            if (!Double.isFinite(s.translationTakeover.blendSec) || s.translationTakeover.blendSec < 0.0) {
                errors.add("translationTakeover(...): blendSec must be >= 0");
            }
            if (Double.isFinite(s.translationTakeover.enterRangeInches)
                    && Double.isFinite(s.translationTakeover.exitRangeInches)
                    && s.translationTakeover.exitRangeInches < s.translationTakeover.enterRangeInches) {
                errors.add("translationTakeover(...): exitRangeInches must be >= enterRangeInches");
            }
            if (mode != DriveGuidanceSpec.SolveMode.ADAPTIVE) {
                errors.add("translationTakeover(...) is only used in adaptive() mode");
            }
        }

        boolean canLocT = s.translationTarget != null && hasLocalization
                && canSolveTranslationWithLocalization(s.translationTarget, s.fixedAprilTagLayout);
        boolean canTagsT = s.translationTarget != null && hasAprilTags
                && canSolveTranslationWithAprilTags(s.translationTarget, hasLayout);
        boolean canLocO = s.aimTarget != null && hasLocalization
                && canSolveAimWithLocalization(s.aimTarget, s.fixedAprilTagLayout);
        boolean canTagsO = s.aimTarget != null && hasAprilTags
                && canSolveAimWithAprilTags(s.aimTarget, hasLayout);

        if (s.translationTarget instanceof DriveGuidanceSpec.RobotRelativePoint && !hasLocalization) {
            errors.add("robotRelativePointInches(...) requires localization(...)");
        }

        if (s.translationTarget != null) {
            if (mode == DriveGuidanceSpec.SolveMode.LOCALIZATION_ONLY && !canLocT) {
                errors.add(localizationFailureForTranslationTarget(s.translationTarget, s.fixedAprilTagLayout));
            }
            if (mode == DriveGuidanceSpec.SolveMode.APRIL_TAGS_ONLY && !canTagsT) {
                errors.add("translateTo() target cannot be solved from aprilTags(...); add fixedAprilTagLayout(...) for field-fixed references or choose localization()/adaptive() as appropriate");
            }
            if (mode == DriveGuidanceSpec.SolveMode.ADAPTIVE && !canLocT && !canTagsT) {
                errors.add("translateTo() target cannot be solved by either adaptive lane; check localization(...), aprilTags(...), and fixedAprilTagLayout(...)");
            }
        }

        if (s.aimTarget != null) {
            if (mode == DriveGuidanceSpec.SolveMode.LOCALIZATION_ONLY && !canLocO) {
                errors.add(localizationFailureForAimTarget(s.aimTarget, s.fixedAprilTagLayout));
            }
            if (mode == DriveGuidanceSpec.SolveMode.APRIL_TAGS_ONLY && !canTagsO) {
                errors.add("aimTo() target cannot be solved from aprilTags(...); add fixedAprilTagLayout(...) for field-fixed references or choose localization()/adaptive() as appropriate");
            }
            if (mode == DriveGuidanceSpec.SolveMode.ADAPTIVE && !canLocO && !canTagsO) {
                errors.add("aimTo() target cannot be solved by either adaptive lane; check localization(...), aprilTags(...), and fixedAprilTagLayout(...)");
            }
        }

        if (mode == DriveGuidanceSpec.SolveMode.ADAPTIVE) {
            boolean dualT = s.translationTarget != null && canLocT && canTagsT;
            boolean dualO = s.aimTarget != null && canLocO && canTagsO;
            if (!dualT && !dualO) {
                errors.add("adaptive() requires at least one requested channel to be solvable by both lanes; otherwise choose localizationOnly() or aprilTagsOnly()");
            }
            if (s.translationTakeover != null && s.translationTarget != null && !dualT) {
                errors.add("translationTakeover(...) is not applicable because translation cannot be solved by both adaptive lanes");
            }
            if (s.omegaPolicyExplicit && s.aimTarget != null && !dualO) {
                errors.add("omegaPolicy(...) is not applicable because omega cannot be solved by both adaptive lanes");
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

    private static String localizationFailureForTranslationTarget(DriveGuidanceSpec.TranslationTarget target,
                                                                  TagLayout layout) {
        String base = "translateTo() target cannot be solved from localization(...)";
        if (target instanceof DriveGuidanceSpec.ReferencePointTarget) {
            return explainLocalizationPointFailure(
                    ((DriveGuidanceSpec.ReferencePointTarget) target).reference,
                    layout,
                    base
            );
        }
        return base + "; add fixedAprilTagLayout(...) for fixed-tag references or choose aprilTags()/adaptive() as appropriate";
    }

    private static String localizationFailureForAimTarget(DriveGuidanceSpec.AimTarget target,
                                                          TagLayout layout) {
        String base = "aimTo() target cannot be solved from localization(...)";
        if (target instanceof DriveGuidanceSpec.ReferencePointTarget) {
            return explainLocalizationPointFailure(
                    ((DriveGuidanceSpec.ReferencePointTarget) target).reference,
                    layout,
                    base
            );
        }
        if (target instanceof DriveGuidanceSpec.ReferenceFrameHeadingTarget) {
            return explainLocalizationFrameFailure(
                    ((DriveGuidanceSpec.ReferenceFrameHeadingTarget) target).reference,
                    layout,
                    base
            );
        }
        return base + "; add fixedAprilTagLayout(...) for fixed-tag references or choose aprilTags()/adaptive() as appropriate";
    }

    private static String explainLocalizationPointFailure(ReferencePoint2d ref,
                                                          TagLayout layout,
                                                          String base) {
        if (ref == null) {
            return base;
        }
        if (References.isFramePoint(ref)) {
            return explainLocalizationFrameFailure(((References.FramePointRef) ref).frame, layout, base);
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
        return base + "; add fixedAprilTagLayout(...) for fixed-tag references or choose aprilTags()/adaptive() as appropriate";
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
        return base + "; add fixedAprilTagLayout(...) for fixed-tag references or choose aprilTags()/adaptive() as appropriate";
    }

    private static DriveGuidanceSpec.SolveMode effectiveSolveMode(State s,
                                                                  boolean hasAprilTags,
                                                                  boolean hasLocalization) {
        if (s.solveMode != null) {
            return s.solveMode;
        }
        if (hasAprilTags && hasLocalization) {
            return DriveGuidanceSpec.SolveMode.ADAPTIVE;
        }
        return hasAprilTags
                ? DriveGuidanceSpec.SolveMode.APRIL_TAGS_ONLY
                : DriveGuidanceSpec.SolveMode.LOCALIZATION_ONLY;
    }

    private static boolean canSolveTranslationWithLocalization(DriveGuidanceSpec.TranslationTarget target,
                                                               TagLayout layout) {
        if (target instanceof DriveGuidanceSpec.FieldPoint) {
            return true;
        }
        if (target instanceof DriveGuidanceSpec.RobotRelativePoint) {
            return true;
        }
        if (target instanceof DriveGuidanceSpec.ReferencePointTarget) {
            return canResolvePointWithLocalization(((DriveGuidanceSpec.ReferencePointTarget) target).reference, layout);
        }
        return false;
    }

    private static boolean canSolveTranslationWithAprilTags(DriveGuidanceSpec.TranslationTarget target,
                                                            boolean hasLayout) {
        if (target instanceof DriveGuidanceSpec.RobotRelativePoint) {
            return false;
        }
        if (target instanceof DriveGuidanceSpec.FieldPoint) {
            return hasLayout;
        }
        if (target instanceof DriveGuidanceSpec.ReferencePointTarget) {
            return canResolvePointWithAprilTags(((DriveGuidanceSpec.ReferencePointTarget) target).reference, hasLayout);
        }
        return false;
    }

    private static boolean canSolveAimWithLocalization(DriveGuidanceSpec.AimTarget target,
                                                       TagLayout layout) {
        if (target instanceof DriveGuidanceSpec.FieldPoint) {
            return true;
        }
        if (target instanceof DriveGuidanceSpec.FieldHeading) {
            return true;
        }
        if (target instanceof DriveGuidanceSpec.ReferencePointTarget) {
            return canResolvePointWithLocalization(((DriveGuidanceSpec.ReferencePointTarget) target).reference, layout);
        }
        if (target instanceof DriveGuidanceSpec.ReferenceFrameHeadingTarget) {
            return canResolveFrameWithLocalization(((DriveGuidanceSpec.ReferenceFrameHeadingTarget) target).reference, layout);
        }
        return false;
    }

    private static boolean canSolveAimWithAprilTags(DriveGuidanceSpec.AimTarget target,
                                                    boolean hasLayout) {
        if (target instanceof DriveGuidanceSpec.FieldPoint) {
            return hasLayout;
        }
        if (target instanceof DriveGuidanceSpec.FieldHeading) {
            return hasLayout;
        }
        if (target instanceof DriveGuidanceSpec.ReferencePointTarget) {
            return canResolvePointWithAprilTags(((DriveGuidanceSpec.ReferencePointTarget) target).reference, hasLayout);
        }
        if (target instanceof DriveGuidanceSpec.ReferenceFrameHeadingTarget) {
            return canResolveFrameWithAprilTags(((DriveGuidanceSpec.ReferenceFrameHeadingTarget) target).reference, hasLayout);
        }
        return false;
    }

    private static boolean canResolvePointWithLocalization(ReferencePoint2d ref, TagLayout layout) {
        if (References.isFieldPoint(ref)) {
            return true;
        }
        if (References.isDirectTagPoint(ref) || References.isSelectedTagPoint(ref)) {
            return References.allCandidateTagsAreFixed(ref, layout);
        }
        if (References.isFramePoint(ref)) {
            return canResolveFrameWithLocalization(((References.FramePointRef) ref).frame, layout);
        }
        return false;
    }

    private static boolean canResolvePointWithAprilTags(ReferencePoint2d ref, boolean hasLayout) {
        if (References.isFieldPoint(ref)) {
            return hasLayout;
        }
        if (References.isDirectTagPoint(ref) || References.isSelectedTagPoint(ref)) {
            return true;
        }
        if (References.isFramePoint(ref)) {
            return canResolveFrameWithAprilTags(((References.FramePointRef) ref).frame, hasLayout);
        }
        return false;
    }

    private static boolean canResolveFrameWithLocalization(ReferenceFrame2d ref, TagLayout layout) {
        if (References.isFieldFrame(ref)) {
            return true;
        }
        if (References.isDirectTagFrame(ref) || References.isSelectedTagFrame(ref)) {
            return References.allCandidateTagsAreFixed(ref, layout);
        }
        return false;
    }

    private static boolean canResolveFrameWithAprilTags(ReferenceFrame2d ref, boolean hasLayout) {
        if (References.isFieldFrame(ref)) {
            return hasLayout;
        }
        return References.isDirectTagFrame(ref) || References.isSelectedTagFrame(ref);
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

        /**
         * Applies the requested robot control frames to the current staged builder.
         *
         * <p>This implementation backs the public builder interfaces and simply stores the chosen
         * frames into shared builder state.</p>
         */
        public final SELF controlFrames(ControlFrames frames) {
            s.controlFrames = Objects.requireNonNull(frames, "frames");
            return self();
        }

        /**
         * Starts configuring the solve lanes for the current staged builder.
         */
        public final ResolveWithBuilder<SELF> resolveWith() {
            return new ResolveWithStep<>(s, self());
        }
    }

    /**
     * Base implementation for spec builder stages.
     */
    private static abstract class SpecBaseBuilder<SELF> extends CommonBuilder<SELF> {
        SpecBaseBuilder(State s) {
            super(s);
        }

        /**
         * Builds the immutable spec represented by the current staged builder state.
         */
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

        /**
         * Applies controller tuning to the staged plan builder.
         */
        public final SELF tuning(DriveGuidancePlan.Tuning tuning) {
            s.tuning = Objects.requireNonNull(tuning, "tuning");
            return self();
        }

        /**
         * Builds the immutable plan represented by the current staged builder state.
         */
        public final DriveGuidancePlan build() {
            return buildPlan(s);
        }
    }

    private static final class Spec0 extends SpecBaseBuilder<SpecBuilder0> implements SpecBuilder0 {
        Spec0(State s) {
            super(s);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public TranslateToBuilder<SpecBuilder1> translateTo() {
            return new TranslateToStep<>(s, new Spec1(s));
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public AimToBuilder<SpecBuilder2> aimTo() {
            return new AimToStep<>(s, new Spec2(s));
        }
    }

    private static final class Spec1 extends SpecBaseBuilder<SpecBuilder1> implements SpecBuilder1 {
        Spec1(State s) {
            super(s);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public AimToBuilder<SpecBuilder3> aimTo() {
            return new AimToStep<>(s, new Spec3(s));
        }
    }

    private static final class Spec2 extends SpecBaseBuilder<SpecBuilder2> implements SpecBuilder2 {
        Spec2(State s) {
            super(s);
        }

        /**
         * {@inheritDoc}
         */
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

        /**
         * {@inheritDoc}
         */
        @Override
        public TranslateToBuilder<PlanBuilder1> translateTo() {
            return new TranslateToStep<>(s, new Builder1(s));
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public AimToBuilder<PlanBuilder2> aimTo() {
            return new AimToStep<>(s, new Builder2(s));
        }
    }

    private static final class Builder1 extends PlanBaseBuilder<PlanBuilder1> implements PlanBuilder1 {
        Builder1(State s) {
            super(s);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public AimToBuilder<PlanBuilder3> aimTo() {
            return new AimToStep<>(s, new Builder3(s));
        }
    }

    private static final class Builder2 extends PlanBaseBuilder<PlanBuilder2> implements PlanBuilder2 {
        Builder2(State s) {
            super(s);
        }

        /**
         * {@inheritDoc}
         */
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

        /**
         * {@inheritDoc}
         */
        @Override
        public TranslateToBuilder<RETURN> fieldPointInches(double xInches, double yInches) {
            if (s.translationTarget != null) {
                throw new IllegalStateException("translateTo() target already configured; choose only one target method");
            }
            s.translationTarget = new DriveGuidanceSpec.FieldPoint(xInches, yInches);
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public TranslateToBuilder<RETURN> robotRelativePointInches(double forwardInches, double leftInches) {
            if (s.translationTarget != null) {
                throw new IllegalStateException("translateTo() target already configured; choose only one target method");
            }
            s.translationTarget = new DriveGuidanceSpec.RobotRelativePoint(forwardInches, leftInches);
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public TranslateToBuilder<RETURN> point(ReferencePoint2d reference) {
            if (s.translationTarget != null) {
                throw new IllegalStateException("translateTo() target already configured; choose only one target method");
            }
            s.translationTarget = new DriveGuidanceSpec.ReferencePointTarget(Objects.requireNonNull(reference, "reference"));
            return this;
        }

        /**
         * {@inheritDoc}
         */
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

        /**
         * {@inheritDoc}
         */
        @Override
        public AimToBuilder<RETURN> fieldPointInches(double xInches, double yInches) {
            if (s.aimTarget != null) {
                throw new IllegalStateException("aimTo() target already configured; choose only one target method");
            }
            s.aimTarget = new DriveGuidanceSpec.FieldPoint(xInches, yInches);
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public AimToBuilder<RETURN> fieldHeadingRad(double fieldHeadingRad) {
            if (s.aimTarget != null) {
                throw new IllegalStateException("aimTo() target already configured; choose only one target method");
            }
            s.aimTarget = new DriveGuidanceSpec.FieldHeading(fieldHeadingRad);
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public AimToBuilder<RETURN> point(ReferencePoint2d reference) {
            if (s.aimTarget != null) {
                throw new IllegalStateException("aimTo() target already configured; choose only one target method");
            }
            s.aimTarget = new DriveGuidanceSpec.ReferencePointTarget(Objects.requireNonNull(reference, "reference"));
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public AimToBuilder<RETURN> frameHeading(ReferenceFrame2d reference) {
            return frameHeading(reference, 0.0);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public AimToBuilder<RETURN> frameHeading(ReferenceFrame2d reference, double headingOffsetRad) {
            if (s.aimTarget != null) {
                throw new IllegalStateException("aimTo() target already configured; choose only one target method");
            }
            s.aimTarget = new DriveGuidanceSpec.ReferenceFrameHeadingTarget(Objects.requireNonNull(reference, "reference"), headingOffsetRad);
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public RETURN doneAimTo() {
            if (s.aimTarget == null) {
                throw new IllegalStateException("aimTo() requires a target before doneAimTo()");
            }
            return ret;
        }
    }

    private static final class ResolveWithStep<RETURN> implements ResolveWithBuilder<RETURN> {
        private final State s;
        private final RETURN ret;

        ResolveWithStep(State s, RETURN ret) {
            this.s = s;
            this.ret = ret;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ResolveWithBuilder<RETURN> localizationOnly() {
            s.solveMode = DriveGuidanceSpec.SolveMode.LOCALIZATION_ONLY;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ResolveWithBuilder<RETURN> aprilTagsOnly() {
            s.solveMode = DriveGuidanceSpec.SolveMode.APRIL_TAGS_ONLY;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ResolveWithBuilder<RETURN> adaptive() {
            s.solveMode = DriveGuidanceSpec.SolveMode.ADAPTIVE;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ResolveWithBuilder<RETURN> aprilTags(AprilTagSensor aprilTags, CameraMountConfig cameraMount) {
            s.aprilTagSensor = Objects.requireNonNull(aprilTags, "aprilTags");
            s.cameraMount = Objects.requireNonNull(cameraMount, "cameraMount");
            s.tagsMaxAgeSec = DriveGuidanceSpec.AprilTags.DEFAULT_MAX_AGE_SEC;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ResolveWithBuilder<RETURN> aprilTags(AprilTagSensor aprilTags,
                                                    CameraMountConfig cameraMount,
                                                    double maxAgeSec) {
            s.aprilTagSensor = Objects.requireNonNull(aprilTags, "aprilTags");
            s.cameraMount = Objects.requireNonNull(cameraMount, "cameraMount");
            s.tagsMaxAgeSec = maxAgeSec;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ResolveWithBuilder<RETURN> localization(AbsolutePoseEstimator poseEstimator) {
            s.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator");
            s.poseMaxAgeSec = DriveGuidanceSpec.Localization.DEFAULT_MAX_AGE_SEC;
            s.poseMinQuality = DriveGuidanceSpec.Localization.DEFAULT_MIN_QUALITY;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ResolveWithBuilder<RETURN> localization(AbsolutePoseEstimator poseEstimator,
                                                       double maxAgeSec,
                                                       double minQuality) {
            s.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator");
            s.poseMaxAgeSec = maxAgeSec;
            s.poseMinQuality = minQuality;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ResolveWithBuilder<RETURN> fixedAprilTagLayout(TagLayout tagLayout) {
            s.fixedAprilTagLayout = tagLayout;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ResolveWithBuilder<RETURN> aprilTagFieldPoseConfig(FixedTagFieldPoseSolver.Config cfg) {
            s.aprilTagFieldPoseConfig = (cfg != null)
                    ? FixedTagFieldPoseSolver.Config.normalizedValidatedCopyOf(
                    cfg,
                    "DriveGuidance.resolveWith().aprilTagFieldPoseConfig"
            )
                    : FixedTagFieldPoseSolver.Config.defaults();
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ResolveWithBuilder<RETURN> translationTakeover(double enterRangeInches, double exitRangeInches, double blendSec) {
            s.translationTakeover = new DriveGuidanceSpec.TranslationTakeover(enterRangeInches, exitRangeInches, blendSec);
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ResolveWithBuilder<RETURN> omegaPolicy(DriveGuidanceSpec.OmegaPolicy omegaPolicy) {
            s.omegaPolicy = Objects.requireNonNull(omegaPolicy, "omegaPolicy");
            s.omegaPolicyExplicit = true;
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public ResolveWithBuilder<RETURN> onLoss(DriveGuidanceSpec.LossPolicy onLoss) {
            s.onLoss = Objects.requireNonNull(onLoss, "onLoss");
            return this;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public RETURN doneResolveWith() {
            return ret;
        }
    }

}