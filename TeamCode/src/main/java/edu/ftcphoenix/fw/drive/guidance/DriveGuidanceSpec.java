package edu.ftcphoenix.fw.drive.guidance;

import java.util.Objects;

import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.PoseEstimator;
import edu.ftcphoenix.fw.localization.apriltag.FixedTagFieldPoseSolver;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;

/**
 * Controller-neutral configuration for {@link DriveGuidance}.
 *
 * <p>A {@link DriveGuidanceSpec} answers four questions:</p>
 * <ol>
 *   <li><b>What is the robot trying to do?</b> translate, aim, or both</li>
 *   <li><b>How is the goal expressed?</b> field point / heading, robot-relative nudge, or semantic
 *       reference point / frame</li>
 *   <li><b>Which point on the robot is controlled?</b> the robot center or an offset mechanism frame</li>
 *   <li><b>How will that goal be solved?</b> localization, live AprilTags, or explicit adaptive
 *       arbitration between both lanes</li>
 * </ol>
 *
 * <p>A spec intentionally contains <strong>no controller tuning</strong>. That keeps “what I want”
 * reusable across TeleOp overlays, autonomous tasks, and readiness queries.</p>
 */
public final class DriveGuidanceSpec {

    public interface TranslationTarget {
        // Marker.
    }

    public interface AimTarget {
        // Marker.
    }

    /**
     * Explicit solve mode for guidance.
     */
    public enum SolveMode {
        LOCALIZATION_ONLY,
        APRIL_TAGS_ONLY,
        ADAPTIVE
    }

    /**
     * Omega arbitration policy when adaptive solve mode is active.
     */
    public enum OmegaPolicy {
        /**
         * Whenever live AprilTag omega is valid, prefer it over localization.
         */
        PREFER_APRIL_TAGS_WHEN_VALID,
        /**
         * Keep omega on the same side of the takeover decision as translation.
         */
        FOLLOW_TRANSLATION_TAKEOVER
    }

    /**
     * Behavior when guidance cannot produce a valid command from the available solve lanes.
     */
    public enum LossPolicy {
        PASS_THROUGH,
        ZERO_OUTPUT
    }

    /**
     * Field point target used for translation or point-at aiming.
     */
    public static final class FieldPoint implements TranslationTarget, AimTarget {
        public final double xInches;
        public final double yInches;

        /**
         * Creates a field-fixed point target in inches.
         */
        public FieldPoint(double xInches, double yInches) {
            this.xInches = xInches;
            this.yInches = yInches;
        }
    }

    /**
     * Absolute field heading target.
     */
    public static final class FieldHeading implements AimTarget {
        public final double fieldHeadingRad;

        /**
         * Creates an absolute field heading target in radians.
         */
        public FieldHeading(double fieldHeadingRad) {
            this.fieldHeadingRad = fieldHeadingRad;
        }
    }

    /**
     * Robot-relative movement captured from the translation control frame when guidance enables.
     */
    public static final class RobotRelativePoint implements TranslationTarget {
        public final double forwardInches;
        public final double leftInches;

        /**
         * Creates a robot-relative translation target captured from the translation control frame
         * when guidance enables.
         */
        public RobotRelativePoint(double forwardInches, double leftInches) {
            this.forwardInches = forwardInches;
            this.leftInches = leftInches;
        }
    }

    /**
     * Semantic reference point target.
     */
    public static final class ReferencePointTarget implements TranslationTarget, AimTarget {
        public final ReferencePoint2d reference;

        /**
         * Wraps a semantic reference point as a guidance target.
         */
        public ReferencePointTarget(ReferencePoint2d reference) {
            this.reference = Objects.requireNonNull(reference, "reference");
        }
    }

    /**
     * Aligns the aim control frame to a semantic reference frame heading.
     */
    public static final class ReferenceFrameHeadingTarget implements AimTarget {
        public final ReferenceFrame2d reference;
        public final double headingOffsetRad;

        /**
         * Wraps a semantic reference frame heading as an aim target.
         *
         * @param reference frame whose heading should be matched
         * @param headingOffsetRad additional heading offset applied on top of the frame heading
         */
        public ReferenceFrameHeadingTarget(ReferenceFrame2d reference, double headingOffsetRad) {
            this.reference = Objects.requireNonNull(reference, "reference");
            this.headingOffsetRad = headingOffsetRad;
        }
    }

    /**
     * Live AprilTag solve lane used directly by guidance.
     */
    public static final class AprilTags {
        public static final double DEFAULT_MAX_AGE_SEC = 0.50;

        public final AprilTagSensor sensor;
        public final CameraMountConfig cameraMount;
        public final double maxAgeSec;
        public final FixedTagFieldPoseSolver.Config fieldPoseSolverConfig;

        /**
         * Creates a live AprilTag solve lane using the default freshness window and default
         * multi-tag field-pose solver configuration.
         */
        public AprilTags(AprilTagSensor sensor, CameraMountConfig cameraMount) {
            this(sensor, cameraMount, DEFAULT_MAX_AGE_SEC, FixedTagFieldPoseSolver.Config.defaults());
        }

        /**
         * Creates a live AprilTag solve lane using an explicit freshness window and the default
         * multi-tag field-pose solver configuration.
         *
         * @param sensor shared AprilTag detections source
         * @param cameraMount camera extrinsics in the robot frame
         * @param maxAgeSec maximum accepted detection age in seconds
         */
        public AprilTags(AprilTagSensor sensor, CameraMountConfig cameraMount, double maxAgeSec) {
            this(sensor, cameraMount, maxAgeSec, FixedTagFieldPoseSolver.Config.defaults());
        }

        /**
         * Creates a live AprilTag solve lane.
         *
         * @param sensor                shared AprilTag detections source
         * @param cameraMount           camera extrinsics in the robot frame
         * @param maxAgeSec             maximum accepted detection age in seconds
         * @param fieldPoseSolverConfig shared multi-tag field-pose solver configuration used when
         *                              guidance temporarily promotes fixed tags into a field pose;
         *                              normalized to the shared base solver config at this API
         *                              boundary so subtype-only extras do not leak through
         */
        public AprilTags(AprilTagSensor sensor,
                         CameraMountConfig cameraMount,
                         double maxAgeSec,
                         FixedTagFieldPoseSolver.Config fieldPoseSolverConfig) {
            this.sensor = Objects.requireNonNull(sensor, "sensor");
            this.cameraMount = Objects.requireNonNull(cameraMount, "cameraMount");
            this.maxAgeSec = maxAgeSec;
            this.fieldPoseSolverConfig = fieldPoseSolverConfig != null
                    ? FixedTagFieldPoseSolver.Config.normalizedValidatedCopyOf(
                    fieldPoseSolverConfig,
                    "DriveGuidanceSpec.AprilTags.fieldPoseSolverConfig"
            )
                    : FixedTagFieldPoseSolver.Config.defaults();
        }
    }

    /**
     * Localization solve lane parameters.
     */
    public static final class Localization {
        public static final double DEFAULT_MAX_AGE_SEC = 0.50;
        public static final double DEFAULT_MIN_QUALITY = 0.10;

        public final PoseEstimator poseEstimator;
        public final double maxAgeSec;
        public final double minQuality;

        /**
         * Creates a localization solve lane using the default freshness and quality gates.
         */
        public Localization(PoseEstimator poseEstimator) {
            this(poseEstimator, DEFAULT_MAX_AGE_SEC, DEFAULT_MIN_QUALITY);
        }

        /**
         * Creates a localization solve lane.
         *
         * @param poseEstimator field-centric pose estimator to sample
         * @param maxAgeSec maximum accepted estimate age in seconds
         * @param minQuality minimum accepted estimate quality
         */
        public Localization(PoseEstimator poseEstimator, double maxAgeSec, double minQuality) {
            this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator");
            this.maxAgeSec = maxAgeSec;
            this.minQuality = minQuality;
        }
    }

    /**
     * Hysteresis + blending parameters for adaptive translation takeover.
     */
    public static final class TranslationTakeover {
        public static final double DEFAULT_ENTER_RANGE_IN = 9.0;
        public static final double DEFAULT_EXIT_RANGE_IN = 12.0;
        public static final double DEFAULT_BLEND_SEC = 0.15;

        public final double enterRangeInches;
        public final double exitRangeInches;
        public final double blendSec;

        /**
         * Creates adaptive translation takeover hysteresis and blending parameters.
         *
         * @param enterRangeInches range below which AprilTag translation may take over
         * @param exitRangeInches range above which guidance falls back out of takeover
         * @param blendSec first-order blend time constant in seconds
         */
        public TranslationTakeover(double enterRangeInches, double exitRangeInches, double blendSec) {
            this.enterRangeInches = enterRangeInches;
            this.exitRangeInches = exitRangeInches;
            this.blendSec = blendSec;
        }

        /**
         * Returns the framework defaults for adaptive translation takeover.
         */
        public static TranslationTakeover defaults() {
            return new TranslationTakeover(DEFAULT_ENTER_RANGE_IN, DEFAULT_EXIT_RANGE_IN, DEFAULT_BLEND_SEC);
        }
    }

    /**
     * Explicit solve-lane configuration for guidance.
     */
    public static final class ResolveWith {
        public final SolveMode mode;
        public final AprilTags aprilTags;
        public final Localization localization;
        public final TagLayout fixedAprilTagLayout;
        public final TranslationTakeover translationTakeover;
        public final OmegaPolicy omegaPolicy;
        public final LossPolicy lossPolicy;

        private ResolveWith(SolveMode mode,
                            AprilTags aprilTags,
                            Localization localization,
                            TagLayout fixedAprilTagLayout,
                            TranslationTakeover translationTakeover,
                            OmegaPolicy omegaPolicy,
                            LossPolicy lossPolicy) {
            this.mode = mode;
            this.aprilTags = aprilTags;
            this.localization = localization;
            this.fixedAprilTagLayout = fixedAprilTagLayout;
            this.translationTakeover = translationTakeover;
            this.omegaPolicy = omegaPolicy;
            this.lossPolicy = lossPolicy;
        }

        /**
         * Creates a validated immutable solve-lane configuration.
         *
         * <p>This method centralizes the invariants for each {@link SolveMode}, fills in default
         * takeover / loss / omega-policy values where appropriate, and throws descriptive
         * {@link IllegalArgumentException}s for mismatched combinations.</p>
         */
        public static ResolveWith create(SolveMode mode,
                                         AprilTags aprilTags,
                                         Localization localization,
                                         TagLayout fixedAprilTagLayout,
                                         TranslationTakeover translationTakeover,
                                         OmegaPolicy omegaPolicy,
                                         LossPolicy lossPolicy) {
            Objects.requireNonNull(mode, "mode");
            LossPolicy lp = (lossPolicy != null) ? lossPolicy : LossPolicy.PASS_THROUGH;
            OmegaPolicy op = (omegaPolicy != null) ? omegaPolicy : OmegaPolicy.PREFER_APRIL_TAGS_WHEN_VALID;

            switch (mode) {
                case LOCALIZATION_ONLY:
                    if (localization == null) {
                        throw new IllegalArgumentException("LOCALIZATION_ONLY requires localization(...)");
                    }
                    if (aprilTags != null) {
                        throw new IllegalArgumentException("LOCALIZATION_ONLY does not accept aprilTags(...); use ADAPTIVE when both lanes are intended");
                    }
                    return new ResolveWith(mode, null, localization, fixedAprilTagLayout, null, op, lp);
                case APRIL_TAGS_ONLY:
                    if (aprilTags == null) {
                        throw new IllegalArgumentException("APRIL_TAGS_ONLY requires aprilTags(...)");
                    }
                    if (localization != null) {
                        throw new IllegalArgumentException("APRIL_TAGS_ONLY does not accept localization(...); use ADAPTIVE when both lanes are intended");
                    }
                    return new ResolveWith(mode, aprilTags, null, fixedAprilTagLayout, null, op, lp);
                case ADAPTIVE:
                default:
                    if (aprilTags == null || localization == null) {
                        throw new IllegalArgumentException("ADAPTIVE requires both localization(...) and aprilTags(...)");
                    }
                    return new ResolveWith(mode,
                            aprilTags,
                            localization,
                            fixedAprilTagLayout,
                            translationTakeover != null ? translationTakeover : TranslationTakeover.defaults(),
                            op,
                            lp);
            }
        }

        /**
         * Returns {@code true} when a live AprilTag solve lane is configured.
         */
        public boolean hasAprilTags() {
            return aprilTags != null;
        }

        /**
         * Returns {@code true} when a localization solve lane is configured.
         */
        public boolean hasLocalization() {
            return localization != null;
        }

        /**
         * Returns {@code true} when fixed AprilTag field metadata is available.
         */
        public boolean hasFixedAprilTagLayout() {
            return fixedAprilTagLayout != null;
        }

        /**
         * Returns {@code true} when both solve lanes are configured for explicit adaptive behavior.
         */
        public boolean isAdaptive() {
            return mode == SolveMode.ADAPTIVE;
        }
    }

    public final TranslationTarget translationTarget;
    public final AimTarget aimTarget;
    public final ControlFrames controlFrames;
    public final ResolveWith resolveWith;

    DriveGuidanceSpec(TranslationTarget translationTarget,
                      AimTarget aimTarget,
                      ControlFrames controlFrames,
                      ResolveWith resolveWith) {
        this.translationTarget = translationTarget;
        this.aimTarget = aimTarget;
        this.controlFrames = Objects.requireNonNull(controlFrames, "controlFrames");
        this.resolveWith = Objects.requireNonNull(resolveWith, "resolveWith");
    }

    /**
     * Returns the natural overlay mask implied by the configured targets.
     */
    public DriveOverlayMask requestedMask() {
        boolean t = translationTarget != null;
        boolean o = aimTarget != null;
        if (t && o) {
            return DriveOverlayMask.ALL;
        }
        if (t) {
            return DriveOverlayMask.TRANSLATION_ONLY;
        }
        if (o) {
            return DriveOverlayMask.OMEGA_ONLY;
        }
        return DriveOverlayMask.NONE;
    }

    /**
     * Returns the recommended runtime mask for overlays/tasks/queries.
     *
     * <p>Today this is the same as {@link #requestedMask()}, but the separate method keeps call
     * sites expressive and leaves room for future heuristics.</p>
     */
    public DriveOverlayMask suggestedMask() {
        return requestedMask();
    }
}
