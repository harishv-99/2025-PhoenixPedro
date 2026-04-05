package edu.ftcphoenix.fw.drive.guidance;

import java.util.Objects;

import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.PoseEstimator;
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
 *   <li><b>Which feedback paths are available?</b> field pose, live AprilTags, or both</li>
 * </ol>
 *
 * <p>A spec intentionally contains <strong>no controller tuning</strong>. That keeps “what I want”
 * reusable across TeleOp overlays, autonomous tasks, and readiness queries.</p>
 */
public final class DriveGuidanceSpec {

    /**
     * Marker for translation targets.
     */
    public interface TranslationTarget {
        // Marker.
    }

    /**
     * Marker for aim targets.
     */
    public interface AimTarget {
        // Marker.
    }

    /**
     * Field point target used for translation or point-at aiming.
     */
    public static final class FieldPoint implements TranslationTarget, AimTarget {
        /**
         * Field X coordinate in inches.
         */
        public final double xInches;
        /**
         * Field Y coordinate in inches.
         */
        public final double yInches;

        /**
         * @param xInches field X coordinate in inches
         * @param yInches field Y coordinate in inches
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
        /** Desired heading in field coordinates, radians. */
        public final double fieldHeadingRad;

        /**
         * @param fieldHeadingRad desired field heading in radians
         */
        public FieldHeading(double fieldHeadingRad) {
            this.fieldHeadingRad = fieldHeadingRad;
        }
    }

    /**
     * Robot-relative movement captured from the translation control frame when guidance enables.
     *
     * <p>Use this for “move forward 6 inches from wherever I enabled the plan”. The anchor pose is
     * captured once on enable and exposed in {@link DriveGuidanceStatus#fieldToTranslationFrameAnchor}
     * for debugging.</p>
     */
    public static final class RobotRelativePoint implements TranslationTarget {
        /** Forward offset in the translation control frame, inches. */
        public final double forwardInches;
        /** Left offset in the translation control frame, inches. */
        public final double leftInches;

        /**
         * @param forwardInches forward offset in inches
         * @param leftInches left offset in inches
         */
        public RobotRelativePoint(double forwardInches, double leftInches) {
            this.forwardInches = forwardInches;
            this.leftInches = leftInches;
        }
    }

    /**
     * Semantic reference point target.
     *
     * <p>Guidance resolves the point either directly from live AprilTags or indirectly from field
     * pose, depending on the configured feedback and the kind of reference.</p>
     */
    public static final class ReferencePointTarget implements TranslationTarget, AimTarget {
        /**
         * Semantic point reference to resolve.
         */
        public final ReferencePoint2d reference;

        /**
         * @param reference semantic point reference, never {@code null}
         */
        public ReferencePointTarget(ReferencePoint2d reference) {
            this.reference = Objects.requireNonNull(reference, "reference");
        }
    }

    /**
     * Uses the origin point of a semantic reference frame.
     */
    public static final class ReferenceFrameOriginTarget implements TranslationTarget, AimTarget {
        /**
         * Semantic frame whose origin should be used.
         */
        public final ReferenceFrame2d reference;

        /**
         * @param reference semantic frame reference, never {@code null}
         */
        public ReferenceFrameOriginTarget(ReferenceFrame2d reference) {
            this.reference = Objects.requireNonNull(reference, "reference");
        }
    }

    /**
     * Uses an offset expressed in a semantic reference frame.
     *
     * <p>Example: “drive 6 inches in front of the backdrop frame origin” or “aim at a point 4
     * inches to the left of this frame”.</p>
     */
    public static final class ReferenceFrameOffsetTarget implements TranslationTarget, AimTarget {
        /**
         * Semantic frame whose axes define the offset.
         */
        public final ReferenceFrame2d reference;
        /**
         * Forward offset in the reference frame, inches.
         */
        public final double forwardInches;
        /**
         * Left offset in the reference frame, inches.
         */
        public final double leftInches;

        /**
         * @param reference     semantic frame reference, never {@code null}
         * @param forwardInches forward offset in the frame, inches
         * @param leftInches    left offset in the frame, inches
         */
        public ReferenceFrameOffsetTarget(ReferenceFrame2d reference,
                                          double forwardInches,
                                          double leftInches) {
            this.reference = Objects.requireNonNull(reference, "reference");
            this.forwardInches = forwardInches;
            this.leftInches = leftInches;
        }
    }

    /**
     * Aligns the aim control frame to a semantic reference frame heading.
     */
    public static final class ReferenceFrameHeadingTarget implements AimTarget {
        /**
         * Semantic frame whose heading should be matched.
         */
        public final ReferenceFrame2d reference;
        /**
         * Additional heading offset applied after resolving the frame heading.
         */
        public final double headingOffsetRad;

        /**
         * @param reference        semantic frame reference, never {@code null}
         * @param headingOffsetRad extra offset added to the resolved frame heading, radians
         */
        public ReferenceFrameHeadingTarget(ReferenceFrame2d reference, double headingOffsetRad) {
            this.reference = Objects.requireNonNull(reference, "reference");
            this.headingOffsetRad = headingOffsetRad;
        }
    }

    /**
     * Behavior when guidance cannot produce a valid command from the available feedback.
     */
    public enum LossPolicy {
        /**
         * Leave the base drive command untouched for the channels guidance cannot solve.
         */
        PASS_THROUGH,
        /**
         * Explicitly output zero for the channels guidance cannot solve.
         */
        ZERO_OUTPUT
    }

    /**
     * Feedback configuration for guidance.
     *
     * <p>The three knobs are intentionally separate:</p>
     * <ul>
     *   <li>{@link AprilTags} = live sensing from the camera</li>
     *   <li>{@link FieldPose} = estimated robot pose in field coordinates</li>
     *   <li>{@link TagLayout} = static field metadata for fixed tags</li>
     * </ul>
     *
     * <p>This separation is what lets guidance reason about the same semantic reference using
     * either live visual geometry or the localizer, without requiring duplicate target
     * definitions.</p>
     */
    public static final class Feedback {
        /**
         * Live AprilTag sensing path, or {@code null} when unavailable.
         */
        public final AprilTags aprilTags;
        /**
         * Field-pose feedback path, or {@code null} when unavailable.
         */
        public final FieldPose fieldPose;
        /**
         * Fixed AprilTag field metadata, or {@code null} when unavailable.
         */
        public final TagLayout fixedTagLayout;
        /**
         * Adaptive-takeover hysteresis and blend config, or {@code null} for defaults.
         */
        public final Gates gates;
        /**
         * Whether omega should prefer AprilTags whenever that path is valid.
         */
        public final boolean preferAprilTagsForOmega;
        /** Behavior when no valid command can be produced. */
        public final LossPolicy lossPolicy;

        private Feedback(AprilTags aprilTags,
                         FieldPose fieldPose,
                         TagLayout fixedTagLayout,
                         Gates gates,
                         boolean preferAprilTagsForOmega,
                         LossPolicy lossPolicy) {
            this.aprilTags = aprilTags;
            this.fieldPose = fieldPose;
            this.fixedTagLayout = fixedTagLayout;
            this.gates = gates;
            this.preferAprilTagsForOmega = preferAprilTagsForOmega;
            this.lossPolicy = lossPolicy;
        }

        /**
         * Validates and creates a feedback bundle.
         *
         * <p>At least one active feedback path is required. When both AprilTags and field pose are
         * present, guidance becomes adaptive and will use {@link Gates#defaults()} when no custom
         * gate config is supplied.</p>
         */
        public static Feedback create(AprilTags aprilTags,
                                      FieldPose fieldPose,
                                      TagLayout fixedTagLayout,
                                      Gates gates,
                                      boolean preferAprilTagsForOmega,
                                      LossPolicy lossPolicy) {
            if (aprilTags == null && fieldPose == null) {
                throw new IllegalArgumentException("feedback requires aprilTags and/or fieldPose");
            }

            Gates g = gates;
            if (aprilTags != null && fieldPose != null && g == null) {
                g = Gates.defaults();
            }

            LossPolicy lp = (lossPolicy != null) ? lossPolicy : LossPolicy.PASS_THROUGH;
            return new Feedback(aprilTags, fieldPose, fixedTagLayout, g, preferAprilTagsForOmega, lp);
        }

        /**
         * @return whether a live AprilTag feedback path is configured
         */
        public boolean hasAprilTags() {
            return aprilTags != null;
        }

        /**
         * @return whether a field-pose feedback path is configured
         */
        public boolean hasFieldPose() {
            return fieldPose != null;
        }

        /**
         * @return whether fixed tag field metadata is available
         */
        public boolean hasFixedTagLayout() {
            return fixedTagLayout != null;
        }

        /**
         * @return whether both AprilTags and field pose are available, enabling adaptive selection
         */
        public boolean isAdaptive() {
            return hasAprilTags() && hasFieldPose();
        }
    }

    /**
     * Live AprilTag sensing path used directly by guidance.
     */
    public static final class AprilTags {
        /**
         * Default freshness bound for visual observations.
         */
        public static final double DEFAULT_MAX_AGE_SEC = 0.50;

        /**
         * Sensor that provides the current best AprilTag observations.
         */
        public final AprilTagSensor sensor;
        /**
         * Robot→camera extrinsics used to convert camera measurements into robot geometry.
         */
        public final CameraMountConfig cameraMount;
        /**
         * Maximum allowed observation age in seconds.
         */
        public final double maxAgeSec;

        /**
         * Creates a visual feedback path with default freshness bounds.
         */
        public AprilTags(AprilTagSensor sensor, CameraMountConfig cameraMount) {
            this(sensor, cameraMount, DEFAULT_MAX_AGE_SEC);
        }

        /**
         * @param sensor      live AprilTag sensor
         * @param cameraMount robot→camera extrinsics
         * @param maxAgeSec   maximum accepted observation age in seconds
         */
        public AprilTags(AprilTagSensor sensor, CameraMountConfig cameraMount, double maxAgeSec) {
            this.sensor = Objects.requireNonNull(sensor, "sensor");
            this.cameraMount = Objects.requireNonNull(cameraMount, "cameraMount");
            this.maxAgeSec = maxAgeSec;
        }
    }

    /**
     * Field-pose feedback parameters.
     *
     * <p>This is usually a fused localizer or odometry-backed estimator. Guidance treats it as a
     * source of {@code field -> robot} rather than as a direct AprilTag observation path.</p>
     */
    public static final class FieldPose {
        /** Default maximum allowed estimate age. */
        public static final double DEFAULT_MAX_AGE_SEC = 0.50;
        /** Default minimum quality required to use a pose estimate. */
        public static final double DEFAULT_MIN_QUALITY = 0.10;

        /** Pose estimator that yields {@code field -> robot}. */
        public final PoseEstimator poseEstimator;
        /** Maximum allowed estimate age in seconds. */
        public final double maxAgeSec;
        /**
         * Minimum required estimate quality.
         */
        public final double minQuality;

        /**
         * Creates a field-pose feedback path with Phoenix defaults.
         */
        public FieldPose(PoseEstimator poseEstimator) {
            this(poseEstimator, DEFAULT_MAX_AGE_SEC, DEFAULT_MIN_QUALITY);
        }

        /**
         * @param poseEstimator field-pose estimator
         * @param maxAgeSec maximum allowed estimate age in seconds
         * @param minQuality minimum required estimate quality
         */
        public FieldPose(PoseEstimator poseEstimator, double maxAgeSec, double minQuality) {
            this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator");
            this.maxAgeSec = maxAgeSec;
            this.minQuality = minQuality;
        }
    }

    /**
     * Hysteresis + blending parameters for adaptive source selection.
     *
     * <p>These values only matter when both AprilTags and field pose are configured. Translation
     * takeover uses range hysteresis; omega can either track that same takeover or always prefer
     * AprilTags when valid.</p>
     */
    public static final class Gates {
        /**
         * Default range at which AprilTags take over translation.
         */
        public static final double DEFAULT_ENTER_RANGE_IN = 9.0;
        /** Default range at which translation falls back to field pose. */
        public static final double DEFAULT_EXIT_RANGE_IN = 12.0;
        /** Default blend duration when switching between solve paths. */
        public static final double DEFAULT_TAKEOVER_BLEND_SEC = 0.15;

        /** Translation-takeover entry threshold. */
        public final double enterRangeInches;
        /**
         * Translation-takeover exit threshold.
         */
        public final double exitRangeInches;
        /**
         * Seconds to blend between field-pose and AprilTag commands.
         */
        public final double takeoverBlendSec;

        /**
         * @param enterRangeInches entry threshold for AprilTag takeover
         * @param exitRangeInches exit threshold for AprilTag takeover
         * @param takeoverBlendSec blend duration, seconds
         */
        public Gates(double enterRangeInches, double exitRangeInches, double takeoverBlendSec) {
            this.enterRangeInches = enterRangeInches;
            this.exitRangeInches = exitRangeInches;
            this.takeoverBlendSec = takeoverBlendSec;
        }

        /**
         * @return Phoenix default adaptive-takeover gates
         */
        public static Gates defaults() {
            return new Gates(DEFAULT_ENTER_RANGE_IN, DEFAULT_EXIT_RANGE_IN, DEFAULT_TAKEOVER_BLEND_SEC);
        }
    }

    /**
     * Translation target, or {@code null} if this spec only aims.
     */
    public final TranslationTarget translationTarget;
    /** Aim target, or {@code null} if this spec only translates. */
    public final AimTarget aimTarget;
    /** Control frames defining which point on the robot is being driven/aimed. */
    public final ControlFrames controlFrames;
    /** Feedback configuration used to solve the targets. */
    public final Feedback feedback;

    DriveGuidanceSpec(TranslationTarget translationTarget,
                      AimTarget aimTarget,
                      ControlFrames controlFrames,
                      Feedback feedback) {
        this.translationTarget = translationTarget;
        this.aimTarget = aimTarget;
        this.controlFrames = Objects.requireNonNull(controlFrames, "controlFrames");
        this.feedback = Objects.requireNonNull(feedback, "feedback");
    }

    /**
     * Returns the overlay mask implied by the configured targets.
     *
     * @return {@link DriveOverlayMask#ALL}, translation-only, omega-only, or none
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
     * Suggested overlay mask for callers that want the obvious default behavior.
     *
     * <p>Today this is identical to {@link #requestedMask()}, but it remains a separate method so
     * the public meaning stays clear.</p>
     */
    public DriveOverlayMask suggestedMask() {
        return requestedMask();
    }
}
