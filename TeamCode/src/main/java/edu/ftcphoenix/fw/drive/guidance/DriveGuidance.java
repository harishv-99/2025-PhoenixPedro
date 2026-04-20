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
import edu.ftcphoenix.fw.spatial.FacingTarget2d;
import edu.ftcphoenix.fw.spatial.ReferenceFrame2d;
import edu.ftcphoenix.fw.spatial.ReferencePoint2d;
import edu.ftcphoenix.fw.spatial.References;
import edu.ftcphoenix.fw.spatial.SpatialControlFrames;
import edu.ftcphoenix.fw.spatial.SpatialQuerySpec;
import edu.ftcphoenix.fw.spatial.SpatialSolveSet;
import edu.ftcphoenix.fw.spatial.SpatialTargets;
import edu.ftcphoenix.fw.spatial.TranslationTarget2d;

/**
 * Guided builders and helpers for creating {@link DriveGuidanceSpec}s and {@link DriveGuidancePlan}s.
 *
 * <p>DriveGuidance is intentionally split into two public objects:</p>
 * <ul>
 *   <li>{@link DriveGuidanceSpec}: controller-neutral <b>what</b> (targets, control frames, solve lanes)</li>
 *   <li>{@link DriveGuidancePlan}: spec + {@link DriveGuidancePlan.Tuning} (<b>how strongly</b>)</li>
 * </ul>
 *
 * <p>The staged builder mirrors {@code ScalarSetpoints.plan()}: it asks the required conceptual
 * questions first, then exposes optional tuning branches:</p>
 * <ol>
 *   <li>choose the first translation or facing target, then optionally add the other channel,</li>
 *   <li>optionally choose controlled robot frames,</li>
 *   <li>choose the solve mode and required solve lanes,</li>
 *   <li>optionally enter drive tuning,</li>
 *   <li>build the reusable plan.</li>
 * </ol>
 *
 * <p>Build is not visible until a target and solve mode have been configured. Solver policy knobs
 * only appear inside the selected solve-mode branch, and drive-controller tuning only appears after
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
 *             .doneTranslateTo()
 *         .andFaceTo()
 *             .frameHeading(slotFace)
 *             .doneFaceTo()
 *         .solveWith()
 *             .adaptive()
 *                 .localization(poseEstimator)
 *                 .aprilTags(tagSensor, cameraMount)
 *                 .fixedAprilTagLayout(tagLayout)
 *                 .doneAdaptive()
 *         .driveTuning()
 *             .aimKp(2.8)
 *             .doneDriveTuning()
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
         * Translates toward a field-fixed point, in field inches.
         */
        TranslateToBuilder<RETURN> fieldPointInches(double xInches, double yInches);

        /**
         * Captures a robot-relative delta, in inches, when guidance enables.
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
     * Nested builder used to describe the facing / heading goal.
     */
    public interface FaceToBuilder<RETURN> {
        /**
         * Faces a field-fixed point, in field inches.
         */
        FaceToBuilder<RETURN> fieldPointInches(double xInches, double yInches);

        /**
         * Aligns to an absolute field heading in radians.
         */
        FaceToBuilder<RETURN> fieldHeadingRad(double fieldHeadingRad);

        /**
         * Faces a semantic point reference.
         */
        FaceToBuilder<RETURN> point(ReferencePoint2d reference);

        /**
         * Aligns to the heading of a semantic reference frame.
         */
        FaceToBuilder<RETURN> frameHeading(ReferenceFrame2d reference);

        /**
         * Aligns to the heading of a semantic reference frame plus an additional offset in radians.
         */
        FaceToBuilder<RETURN> frameHeading(ReferenceFrame2d reference, double headingOffsetRad);

        /**
         * Returns to the parent builder stage.
         */
        RETURN doneFaceTo();
    }

    // ------------------------------------------------------------------------
    // Resolve / solve-mode builders
    // ------------------------------------------------------------------------

    /**
     * First solve-mode stage: choose one explicit solve mode.
     *
     * <p>Use the {@code ...WithDefaults(...)} methods when the default freshness/policy settings are
     * acceptable. Enter the named branch when you need to tune mode-specific options such as max age,
     * field layout, adaptive takeover, or loss policy.</p>
     */
    public interface ResolveModeChoice<RETURN> {
        /**
         * Uses localization only with default lane bounds and loss policy.
         */
        RETURN localizationOnlyWithDefaults(AbsolutePoseEstimator poseEstimator);

        /**
         * Enters the localization-only branch to configure lane bounds or loss policy.
         */
        LocalizationOnlyEstimatorStage<RETURN> localizationOnly();

        /**
         * Uses live AprilTags only with default lane bounds and loss policy.
         */
        RETURN aprilTagsOnlyWithDefaults(AprilTagSensor aprilTags, CameraMountConfig cameraMount);

        /**
         * Enters the AprilTag-only branch to configure lane bounds, field layout, or loss policy.
         */
        AprilTagsOnlySensorStage<RETURN> aprilTagsOnly();

        /**
         * Uses adaptive localization + AprilTag arbitration with default lane bounds and loss policy.
         */
        RETURN adaptiveWithDefaults(AbsolutePoseEstimator poseEstimator,
                                    AprilTagSensor aprilTags,
                                    CameraMountConfig cameraMount);

        /**
         * Enters the adaptive branch to configure both lanes and adaptive policy.
         */
        AdaptiveLocalizationStage<RETURN> adaptive();
    }

    /**
     * Localization-only branch stage: provide the required pose estimator.
     */
    public interface LocalizationOnlyEstimatorStage<RETURN> {
        /**
         * Supplies the localization lane used by localization-only guidance.
         */
        LocalizationOnlyTuningStage<RETURN> localization(AbsolutePoseEstimator poseEstimator);
    }

    /**
     * Localization-only optional tuning branch.
     */
    public interface LocalizationOnlyTuningStage<RETURN> {
        /**
         * Sets maximum accepted pose age in seconds.
         */
        LocalizationOnlyTuningStage<RETURN> maxAgeSec(double maxAgeSec);

        /**
         * Sets minimum accepted pose quality in [0, 1].
         */
        LocalizationOnlyTuningStage<RETURN> minQuality(double minQuality);

        /**
         * Supplies fixed field metadata so localization can resolve fixed-tag references.
         */
        LocalizationOnlyTuningStage<RETURN> fixedAprilTagLayout(TagLayout tagLayout);

        /**
         * Chooses what guidance outputs when the requested channels cannot be solved.
         */
        LocalizationOnlyTuningStage<RETURN> onLoss(DriveGuidanceSpec.LossPolicy onLoss);

        /**
         * Returns to the parent builder after localization-only configuration.
         */
        RETURN doneLocalizationOnly();
    }

    /**
     * AprilTag-only branch stage: provide the required live AprilTag lane.
     */
    public interface AprilTagsOnlySensorStage<RETURN> {
        /**
         * Supplies the AprilTag sensor and the camera mount used by AprilTag-only guidance.
         */
        AprilTagsOnlyTuningStage<RETURN> aprilTags(AprilTagSensor aprilTags, CameraMountConfig cameraMount);
    }

    /**
     * AprilTag-only optional tuning branch.
     */
    public interface AprilTagsOnlyTuningStage<RETURN> {
        /**
         * Sets maximum accepted AprilTag frame age in seconds.
         */
        AprilTagsOnlyTuningStage<RETURN> maxAgeSec(double maxAgeSec);

        /**
         * Supplies fixed field metadata for field-fixed AprilTag solving.
         */
        AprilTagsOnlyTuningStage<RETURN> fixedAprilTagLayout(TagLayout tagLayout);

        /**
         * Overrides the shared multi-tag field-pose solver config for fixed-tag solving.
         */
        AprilTagsOnlyTuningStage<RETURN> aprilTagFieldPoseConfig(FixedTagFieldPoseSolver.Config cfg);

        /**
         * Chooses what guidance outputs when the requested channels cannot be solved.
         */
        AprilTagsOnlyTuningStage<RETURN> onLoss(DriveGuidanceSpec.LossPolicy onLoss);

        /**
         * Returns to the parent builder after AprilTag-only configuration.
         */
        RETURN doneAprilTagsOnly();
    }

    /**
     * Adaptive branch stage: provide the required localization lane first.
     */
    public interface AdaptiveLocalizationStage<RETURN> {
        /**
         * Supplies the localization lane used by adaptive guidance.
         */
        AdaptiveAprilTagsStage<RETURN> localization(AbsolutePoseEstimator poseEstimator);
    }

    /**
     * Adaptive branch stage: provide the required AprilTag lane.
     */
    public interface AdaptiveAprilTagsStage<RETURN> {
        /**
         * Supplies the AprilTag sensor and camera mount used by adaptive guidance.
         */
        AdaptiveTuningStage<RETURN> aprilTags(AprilTagSensor aprilTags, CameraMountConfig cameraMount);
    }

    /**
     * Adaptive optional tuning branch.
     */
    public interface AdaptiveTuningStage<RETURN> {
        /**
         * Sets maximum accepted localization pose age in seconds.
         */
        AdaptiveTuningStage<RETURN> localizationMaxAgeSec(double maxAgeSec);

        /**
         * Sets minimum accepted localization pose quality in [0, 1].
         */
        AdaptiveTuningStage<RETURN> localizationMinQuality(double minQuality);

        /**
         * Sets maximum accepted AprilTag frame age in seconds.
         */
        AdaptiveTuningStage<RETURN> aprilTagMaxAgeSec(double maxAgeSec);

        /**
         * Supplies fixed field metadata for field-fixed AprilTag/localization reference solving.
         */
        AdaptiveTuningStage<RETURN> fixedAprilTagLayout(TagLayout tagLayout);

        /**
         * Overrides the shared multi-tag field-pose solver config for fixed-tag solving.
         */
        AdaptiveTuningStage<RETURN> aprilTagFieldPoseConfig(FixedTagFieldPoseSolver.Config cfg);

        /**
         * Configures adaptive translation takeover hysteresis and blend timing, in inches/seconds.
         */
        AdaptiveTuningStage<RETURN> translationTakeover(double enterRangeInches,
                                                        double exitRangeInches,
                                                        double blendSec);

        /**
         * Configures adaptive omega arbitration.
         */
        AdaptiveTuningStage<RETURN> omegaPolicy(DriveGuidanceSpec.OmegaPolicy omegaPolicy);

        /**
         * Chooses what guidance outputs when the requested channels cannot be solved.
         */
        AdaptiveTuningStage<RETURN> onLoss(DriveGuidanceSpec.LossPolicy onLoss);

        /**
         * Returns to the parent builder after adaptive configuration.
         */
        RETURN doneAdaptive();
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
        if (s.translationTarget == null && s.facingTarget == null) {
            throw new IllegalStateException("DriveGuidance spec needs translateTo() and/or faceTo() configured");
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

        SpatialSolveSet.MoreLanesStep solveSetBuilder = null;
        int localizationLaneIndex = -1;
        int aprilTagsLaneIndex = -1;
        int nextLaneIndex = 0;

        if (localization != null) {
            localizationLaneIndex = nextLaneIndex++;
            solveSetBuilder = SpatialSolveSet.builder()
                    .absolutePose(localization.poseEstimator, localization.maxAgeSec, localization.minQuality);
        }
        if (tags != null) {
            aprilTagsLaneIndex = nextLaneIndex++;
            if (solveSetBuilder == null) {
                solveSetBuilder = SpatialSolveSet.builder()
                        .aprilTags(tags.sensor, tags.cameraMount, tags.maxAgeSec, tags.fieldPoseSolverConfig);
            } else {
                solveSetBuilder.aprilTags(tags.sensor, tags.cameraMount, tags.maxAgeSec, tags.fieldPoseSolverConfig);
            }
        }
        if (solveSetBuilder == null) {
            throw new IllegalStateException("DriveGuidance solveWith() must provide at least one solve lane");
        }
        SpatialSolveSet solveSet = solveSetBuilder.build();

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
                    .fixedAprilTagLayout(s.fixedAprilTagLayout)
                    .build();
        } else if (spatialTranslationTarget != null) {
            spatialQuerySpec = SpatialQuerySpec.builder()
                    .translateTo(spatialTranslationTarget)
                    .controlFrames(s.controlFrames)
                    .solveWith(solveSet)
                    .fixedAprilTagLayout(s.fixedAprilTagLayout)
                    .build();
        } else if (s.facingTarget != null) {
            spatialQuerySpec = SpatialQuerySpec.builder()
                    .faceTo(s.facingTarget)
                    .controlFrames(s.controlFrames)
                    .solveWith(solveSet)
                    .fixedAprilTagLayout(s.fixedAprilTagLayout)
                    .build();
        }

        return new DriveGuidanceSpec(
                s.translationTarget,
                s.facingTarget,
                s.controlFrames,
                rw,
                spatialQuerySpec,
                localizationLaneIndex,
                aprilTagsLaneIndex
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
    private static void validateCapabilitiesOrThrow(State s) {
        ArrayList<String> errors = new ArrayList<String>();

        boolean hasAprilTags = s.aprilTagSensor != null && s.cameraMount != null;
        boolean hasLocalization = s.poseEstimator != null;
        boolean hasLayout = s.fixedAprilTagLayout != null;

        if (!hasAprilTags && !hasLocalization) {
            errors.add("solveWith() must choose localizationOnlyWithDefaults(...), aprilTagsOnlyWithDefaults(...), adaptiveWithDefaults(...), or enter one of the solve-mode branches");
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

        boolean canLocT = hasLocalization
                && canSolveTranslationWithLocalization(s.translationTarget, s.fixedAprilTagLayout);
        boolean canTagsT = hasAprilTags
                && canSolveTranslationWithAprilTags(s.translationTarget, hasLayout);
        boolean canLocO = hasLocalization
                && canSolveAimWithLocalization(s.facingTarget, s.fixedAprilTagLayout);
        boolean canTagsO = hasAprilTags
                && canSolveAimWithAprilTags(s.facingTarget, hasLayout);

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

        if (s.facingTarget != null) {
            if (mode == DriveGuidanceSpec.SolveMode.LOCALIZATION_ONLY && !canLocO) {
                errors.add(localizationFailureForFacingTarget(s.facingTarget, s.fixedAprilTagLayout));
            }
            if (mode == DriveGuidanceSpec.SolveMode.APRIL_TAGS_ONLY && !canTagsO) {
                errors.add("faceTo() target cannot be solved from aprilTags(...); add fixedAprilTagLayout(...) for field-fixed references or choose localization()/adaptive() as appropriate");
            }
            if (mode == DriveGuidanceSpec.SolveMode.ADAPTIVE && !canLocO && !canTagsO) {
                errors.add("faceTo() target cannot be solved by either adaptive lane; check localization(...), aprilTags(...), and fixedAprilTagLayout(...)");
            }
        }

        if (mode == DriveGuidanceSpec.SolveMode.ADAPTIVE) {
            boolean dualT = s.translationTarget != null && canLocT && canTagsT;
            boolean dualO = s.facingTarget != null && canLocO && canTagsO;
            if (!dualT && !dualO) {
                errors.add("adaptive() requires at least one requested channel to be solvable by both lanes; otherwise choose localizationOnly() or aprilTagsOnly()");
            }
            if (s.translationTakeover != null && s.translationTarget != null && !dualT) {
                errors.add("translationTakeover(...) is not applicable because translation cannot be solved by both adaptive lanes");
            }
            if (s.omegaPolicyExplicit && s.facingTarget != null && !dualO) {
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

    private static String localizationFailureForTranslationTarget(TranslationTarget2d target,
                                                                  TagLayout layout) {
        String base = "translateTo() target cannot be solved from localization(...)";
        if (target instanceof SpatialTargets.ReferencePointTarget) {
            return explainLocalizationPointFailure(
                    ((SpatialTargets.ReferencePointTarget) target).reference,
                    layout,
                    base
            );
        }
        return base + "; add fixedAprilTagLayout(...) for fixed-tag references or choose aprilTags()/adaptive() as appropriate";
    }

    private static String localizationFailureForFacingTarget(FacingTarget2d target,
                                                             TagLayout layout) {
        String base = "faceTo() target cannot be solved from localization(...)";
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
        return base + "; add fixedAprilTagLayout(...) for fixed-tag references or choose aprilTags()/adaptive() as appropriate";
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

    private static boolean canSolveTranslationWithAprilTags(TranslationTarget2d target,
                                                            boolean hasLayout) {
        if (target instanceof DriveGuidanceSpec.RobotRelativePoint) {
            return false;
        }
        if (target instanceof SpatialTargets.FieldPoint) {
            return hasLayout;
        }
        if (target instanceof SpatialTargets.ReferencePointTarget) {
            return canResolvePointWithAprilTags(((SpatialTargets.ReferencePointTarget) target).reference, hasLayout);
        }
        return false;
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

    private static boolean canSolveAimWithAprilTags(FacingTarget2d target,
                                                    boolean hasLayout) {
        if (target instanceof SpatialTargets.FieldPoint) {
            return hasLayout;
        }
        if (target instanceof SpatialTargets.FieldHeading) {
            return hasLayout;
        }
        if (target instanceof SpatialTargets.ReferencePointTarget) {
            return canResolvePointWithAprilTags(((SpatialTargets.ReferencePointTarget) target).reference, hasLayout);
        }
        if (target instanceof SpatialTargets.ReferenceFrameHeadingTarget) {
            return canResolveFrameWithAprilTags(((SpatialTargets.ReferenceFrameHeadingTarget) target).reference, hasLayout);
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
            return canResolveFrameWithLocalization(References.framePointBaseFrame(ref), layout);
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
            return canResolveFrameWithAprilTags(References.framePointBaseFrame(ref), hasLayout);
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


    // ------------------------------------------------------------------------
    // Builder implementations
    // ------------------------------------------------------------------------

    private static void resetSolve(State s, DriveGuidanceSpec.SolveMode mode) {
        s.solveMode = mode;
        s.aprilTagSensor = null;
        s.cameraMount = null;
        s.tagsMaxAgeSec = DriveGuidanceSpec.AprilTags.DEFAULT_MAX_AGE_SEC;
        s.aprilTagFieldPoseConfig = FixedTagFieldPoseSolver.Config.defaults();
        s.poseEstimator = null;
        s.poseMaxAgeSec = DriveGuidanceSpec.Localization.DEFAULT_MAX_AGE_SEC;
        s.poseMinQuality = DriveGuidanceSpec.Localization.DEFAULT_MIN_QUALITY;
        s.fixedAprilTagLayout = null;
        s.translationTakeover = null;
        s.omegaPolicy = DriveGuidanceSpec.OmegaPolicy.PREFER_APRIL_TAGS_WHEN_VALID;
        s.omegaPolicyExplicit = false;
        s.onLoss = DriveGuidanceSpec.LossPolicy.PASS_THROUGH;
    }

    private static void setAprilTagFieldPoseConfig(State s, FixedTagFieldPoseSolver.Config cfg) {
        s.aprilTagFieldPoseConfig = (cfg != null)
                ? FixedTagFieldPoseSolver.Config.normalizedValidatedCopyOf(
                cfg,
                "DriveGuidance.solveWith().aprilTagFieldPoseConfig"
        )
                : FixedTagFieldPoseSolver.Config.defaults();
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
        public TranslateToBuilder<RETURN> fieldPointInches(double xInches, double yInches) {
            if (s.translationTarget != null) {
                throw new IllegalStateException("translateTo() target already configured; choose only one target method");
            }
            s.translationTarget = SpatialTargets.fieldPoint(xInches, yInches);
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
        public TranslateToBuilder<RETURN> point(ReferencePoint2d reference) {
            if (s.translationTarget != null) {
                throw new IllegalStateException("translateTo() target already configured; choose only one target method");
            }
            s.translationTarget = SpatialTargets.point(Objects.requireNonNull(reference, "reference"));
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

    private static final class FaceToStep<RETURN> implements FaceToBuilder<RETURN> {
        private final State s;
        private final RETURN ret;

        FaceToStep(State s, RETURN ret) {
            this.s = s;
            this.ret = ret;
        }

        @Override
        public FaceToBuilder<RETURN> fieldPointInches(double xInches, double yInches) {
            if (s.facingTarget != null) {
                throw new IllegalStateException("faceTo() target already configured; choose only one target method");
            }
            s.facingTarget = SpatialTargets.fieldPoint(xInches, yInches);
            return this;
        }

        @Override
        public FaceToBuilder<RETURN> fieldHeadingRad(double fieldHeadingRad) {
            if (s.facingTarget != null) {
                throw new IllegalStateException("faceTo() target already configured; choose only one target method");
            }
            s.facingTarget = SpatialTargets.fieldHeading(fieldHeadingRad);
            return this;
        }

        @Override
        public FaceToBuilder<RETURN> point(ReferencePoint2d reference) {
            if (s.facingTarget != null) {
                throw new IllegalStateException("faceTo() target already configured; choose only one target method");
            }
            s.facingTarget = SpatialTargets.point(Objects.requireNonNull(reference, "reference"));
            return this;
        }

        @Override
        public FaceToBuilder<RETURN> frameHeading(ReferenceFrame2d reference) {
            return frameHeading(reference, 0.0);
        }

        @Override
        public FaceToBuilder<RETURN> frameHeading(ReferenceFrame2d reference, double headingOffsetRad) {
            if (s.facingTarget != null) {
                throw new IllegalStateException("faceTo() target already configured; choose only one target method");
            }
            s.facingTarget = SpatialTargets.frameHeading(Objects.requireNonNull(reference, "reference"), headingOffsetRad);
            return this;
        }

        @Override
        public RETURN doneFaceTo() {
            if (s.facingTarget == null) {
                throw new IllegalStateException("faceTo() requires a target before doneFaceTo()");
            }
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
        public RETURN localizationOnlyWithDefaults(AbsolutePoseEstimator poseEstimator) {
            resetSolve(s, DriveGuidanceSpec.SolveMode.LOCALIZATION_ONLY);
            s.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator");
            return ret;
        }

        @Override
        public LocalizationOnlyEstimatorStage<RETURN> localizationOnly() {
            resetSolve(s, DriveGuidanceSpec.SolveMode.LOCALIZATION_ONLY);
            return new LocalizationOnlyStep<RETURN>(s, ret);
        }

        @Override
        public RETURN aprilTagsOnlyWithDefaults(AprilTagSensor aprilTags, CameraMountConfig cameraMount) {
            resetSolve(s, DriveGuidanceSpec.SolveMode.APRIL_TAGS_ONLY);
            s.aprilTagSensor = Objects.requireNonNull(aprilTags, "aprilTags");
            s.cameraMount = Objects.requireNonNull(cameraMount, "cameraMount");
            return ret;
        }

        @Override
        public AprilTagsOnlySensorStage<RETURN> aprilTagsOnly() {
            resetSolve(s, DriveGuidanceSpec.SolveMode.APRIL_TAGS_ONLY);
            return new AprilTagsOnlyStep<RETURN>(s, ret);
        }

        @Override
        public RETURN adaptiveWithDefaults(AbsolutePoseEstimator poseEstimator,
                                           AprilTagSensor aprilTags,
                                           CameraMountConfig cameraMount) {
            resetSolve(s, DriveGuidanceSpec.SolveMode.ADAPTIVE);
            s.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator");
            s.aprilTagSensor = Objects.requireNonNull(aprilTags, "aprilTags");
            s.cameraMount = Objects.requireNonNull(cameraMount, "cameraMount");
            return ret;
        }

        @Override
        public AdaptiveLocalizationStage<RETURN> adaptive() {
            resetSolve(s, DriveGuidanceSpec.SolveMode.ADAPTIVE);
            return new AdaptiveStep<RETURN>(s, ret);
        }
    }

    private static final class LocalizationOnlyStep<RETURN>
            implements LocalizationOnlyEstimatorStage<RETURN>, LocalizationOnlyTuningStage<RETURN> {
        private final State s;
        private final RETURN ret;

        LocalizationOnlyStep(State s, RETURN ret) {
            this.s = s;
            this.ret = ret;
        }

        @Override
        public LocalizationOnlyTuningStage<RETURN> localization(AbsolutePoseEstimator poseEstimator) {
            s.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator");
            return this;
        }

        @Override
        public LocalizationOnlyTuningStage<RETURN> maxAgeSec(double maxAgeSec) {
            s.poseMaxAgeSec = maxAgeSec;
            return this;
        }

        @Override
        public LocalizationOnlyTuningStage<RETURN> minQuality(double minQuality) {
            s.poseMinQuality = minQuality;
            return this;
        }

        @Override
        public LocalizationOnlyTuningStage<RETURN> fixedAprilTagLayout(TagLayout tagLayout) {
            s.fixedAprilTagLayout = tagLayout;
            return this;
        }

        @Override
        public LocalizationOnlyTuningStage<RETURN> onLoss(DriveGuidanceSpec.LossPolicy onLoss) {
            s.onLoss = Objects.requireNonNull(onLoss, "onLoss");
            return this;
        }

        @Override
        public RETURN doneLocalizationOnly() {
            if (s.poseEstimator == null) {
                throw new IllegalStateException("localizationOnly() requires localization(...) before doneLocalizationOnly()");
            }
            return ret;
        }
    }

    private static final class AprilTagsOnlyStep<RETURN>
            implements AprilTagsOnlySensorStage<RETURN>, AprilTagsOnlyTuningStage<RETURN> {
        private final State s;
        private final RETURN ret;

        AprilTagsOnlyStep(State s, RETURN ret) {
            this.s = s;
            this.ret = ret;
        }

        @Override
        public AprilTagsOnlyTuningStage<RETURN> aprilTags(AprilTagSensor aprilTags, CameraMountConfig cameraMount) {
            s.aprilTagSensor = Objects.requireNonNull(aprilTags, "aprilTags");
            s.cameraMount = Objects.requireNonNull(cameraMount, "cameraMount");
            return this;
        }

        @Override
        public AprilTagsOnlyTuningStage<RETURN> maxAgeSec(double maxAgeSec) {
            s.tagsMaxAgeSec = maxAgeSec;
            return this;
        }

        @Override
        public AprilTagsOnlyTuningStage<RETURN> fixedAprilTagLayout(TagLayout tagLayout) {
            s.fixedAprilTagLayout = tagLayout;
            return this;
        }

        @Override
        public AprilTagsOnlyTuningStage<RETURN> aprilTagFieldPoseConfig(FixedTagFieldPoseSolver.Config cfg) {
            setAprilTagFieldPoseConfig(s, cfg);
            return this;
        }

        @Override
        public AprilTagsOnlyTuningStage<RETURN> onLoss(DriveGuidanceSpec.LossPolicy onLoss) {
            s.onLoss = Objects.requireNonNull(onLoss, "onLoss");
            return this;
        }

        @Override
        public RETURN doneAprilTagsOnly() {
            if (s.aprilTagSensor == null || s.cameraMount == null) {
                throw new IllegalStateException("aprilTagsOnly() requires aprilTags(...) before doneAprilTagsOnly()");
            }
            return ret;
        }
    }

    private static final class AdaptiveStep<RETURN>
            implements AdaptiveLocalizationStage<RETURN>, AdaptiveAprilTagsStage<RETURN>, AdaptiveTuningStage<RETURN> {
        private final State s;
        private final RETURN ret;

        AdaptiveStep(State s, RETURN ret) {
            this.s = s;
            this.ret = ret;
        }

        @Override
        public AdaptiveAprilTagsStage<RETURN> localization(AbsolutePoseEstimator poseEstimator) {
            s.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator");
            return this;
        }

        @Override
        public AdaptiveTuningStage<RETURN> aprilTags(AprilTagSensor aprilTags, CameraMountConfig cameraMount) {
            s.aprilTagSensor = Objects.requireNonNull(aprilTags, "aprilTags");
            s.cameraMount = Objects.requireNonNull(cameraMount, "cameraMount");
            return this;
        }

        @Override
        public AdaptiveTuningStage<RETURN> localizationMaxAgeSec(double maxAgeSec) {
            s.poseMaxAgeSec = maxAgeSec;
            return this;
        }

        @Override
        public AdaptiveTuningStage<RETURN> localizationMinQuality(double minQuality) {
            s.poseMinQuality = minQuality;
            return this;
        }

        @Override
        public AdaptiveTuningStage<RETURN> aprilTagMaxAgeSec(double maxAgeSec) {
            s.tagsMaxAgeSec = maxAgeSec;
            return this;
        }

        @Override
        public AdaptiveTuningStage<RETURN> fixedAprilTagLayout(TagLayout tagLayout) {
            s.fixedAprilTagLayout = tagLayout;
            return this;
        }

        @Override
        public AdaptiveTuningStage<RETURN> aprilTagFieldPoseConfig(FixedTagFieldPoseSolver.Config cfg) {
            setAprilTagFieldPoseConfig(s, cfg);
            return this;
        }

        @Override
        public AdaptiveTuningStage<RETURN> translationTakeover(double enterRangeInches,
                                                               double exitRangeInches,
                                                               double blendSec) {
            s.translationTakeover = new DriveGuidanceSpec.TranslationTakeover(enterRangeInches, exitRangeInches, blendSec);
            return this;
        }

        @Override
        public AdaptiveTuningStage<RETURN> omegaPolicy(DriveGuidanceSpec.OmegaPolicy omegaPolicy) {
            s.omegaPolicy = Objects.requireNonNull(omegaPolicy, "omegaPolicy");
            s.omegaPolicyExplicit = true;
            return this;
        }

        @Override
        public AdaptiveTuningStage<RETURN> onLoss(DriveGuidanceSpec.LossPolicy onLoss) {
            s.onLoss = Objects.requireNonNull(onLoss, "onLoss");
            return this;
        }

        @Override
        public RETURN doneAdaptive() {
            if (s.poseEstimator == null) {
                throw new IllegalStateException("adaptive() requires localization(...) before aprilTags(...)");
            }
            if (s.aprilTagSensor == null || s.cameraMount == null) {
                throw new IllegalStateException("adaptive() requires aprilTags(...) before doneAdaptive()");
            }
            return ret;
        }
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
