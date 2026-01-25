package edu.ftcphoenix.fw.drive.guidance;

import java.util.Objects;

import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.PoseEstimator;
import edu.ftcphoenix.fw.sensing.observation.ObservationSource2d;

/**
 * Controller-neutral configuration for “driver guidance”.
 *
 * <p>A {@link DriveGuidanceSpec} describes:</p>
 * <ul>
 *   <li><b>What you want to do</b>: translate, aim, or both.</li>
 *   <li><b>Where the target is defined</b>: field coordinates, relative to an anchor (e.g. AprilTag),
 *       or relative to the robot at the moment guidance becomes enabled.</li>
 *   <li><b>Which point on the robot is being controlled</b>: robot center or an off-center mechanism.</li>
 *   <li><b>What feedback sources are available</b>: observation, field pose, or an adaptive combination.</li>
 * </ul>
 *
 * <p>Unlike {@link DriveGuidancePlan}, a spec contains <b>no controller tuning</b> and does not produce
 * drive commands by itself. Specs are meant to be reusable across overlays, tasks, and query-only
 * telemetry/gating.</p>
 *
 * <p>To execute a spec, combine it with {@link DriveGuidancePlan.Tuning} to form a plan via
 * {@link DriveGuidance#plan(DriveGuidanceSpec)}.</p>
 */
public final class DriveGuidanceSpec {

    // ------------------------------------------------------------------------
    // Targets
    // ------------------------------------------------------------------------

    /**
     * Marker interface for translation targets.
     */
    public interface TranslationTarget {
        // Marker.
    }

    /**
     * Marker interface for aim targets.
     */
    public interface AimTarget {
        // Marker.
    }

    /**
     * Target point defined in <b>field</b> coordinates.
     */
    public static final class FieldPoint implements TranslationTarget, AimTarget {
        public final double xInches;
        public final double yInches;

        public FieldPoint(double xInches, double yInches) {
            this.xInches = xInches;
            this.yInches = yInches;
        }
    }

    /**
     * A point defined relative to an AprilTag’s reference frame.
     *
     * <p>Coordinates follow Phoenix conventions in the tag frame:</p>
     * <ul>
     *   <li>{@code forwardInches} is +X (forward/out from the tag),</li>
     *   <li>{@code leftInches} is +Y (left when looking in +X).</li>
     * </ul>
     */
    public static final class TagRelativePoint implements TranslationTarget, AimTarget {

        /**
         * Tag ID used to resolve this point in a {@link TagLayout}.
         *
         * <p>Use {@code -1} to mean “whichever tag is currently observed”. This is useful for
         * vision-first specs that start with observation and may optionally add a field pose
         * estimator later.</p>
         */
        public final int tagId;

        public final double forwardInches;
        public final double leftInches;

        public TagRelativePoint(int tagId, double forwardInches, double leftInches) {
            this.tagId = tagId;
            this.forwardInches = forwardInches;
            this.leftInches = leftInches;
        }
    }

    /**
     * Aim target that requests an <b>absolute field heading</b>.
     *
     * <p>This is the “turn to heading” sibling of {@link FieldPoint} / {@link TagRelativePoint} aiming.
     * Instead of aiming at a point, guidance will rotate the robot until the aim control
     * frame's heading matches {@link #fieldHeadingRad}.</p>
     *
     * <p><b>Important:</b> This target requires {@link Feedback#hasFieldPose()} because it needs a
     * current robot heading estimate in the field frame. It cannot be solved from observation-only
     * feedback.</p>
     */
    public static final class FieldHeading implements AimTarget {

        /**
         * Desired heading in the field frame, in radians.
         */
        public final double fieldHeadingRad;

        public FieldHeading(double fieldHeadingRad) {
            this.fieldHeadingRad = fieldHeadingRad;
        }
    }

    /**
     * Aim target that resolves to an <b>absolute field heading</b> derived from a specific AprilTag.
     *
     * <p>This is useful when you want to align with a tag's orientation (or face a tag plane)
     * rather than “look at” a point.</p>
     *
     * <p><b>Requires:</b> field-pose feedback with a {@link TagLayout}.</p>
     */
    public static final class TagHeading implements AimTarget {

        /**
         * Tag ID used to resolve this heading in a {@link TagLayout}.
         */
        public final int tagId;

        /**
         * Heading offset added to the tag's yaw to form the desired field heading.
         */
        public final double headingOffsetRad;

        public TagHeading(int tagId, double headingOffsetRad) {
            this.tagId = tagId;
            this.headingOffsetRad = headingOffsetRad;
        }
    }

    /**
     * Translation target defined <b>relative to the robot</b> when guidance becomes enabled.
     *
     * <p>This is intended for “nudge” style behaviors such as:</p>
     * <ul>
     *   <li>drive forward 6 inches from wherever you are,</li>
     *   <li>strafe left 3 inches,</li>
     *   <li>micro-adjust an intake position without thinking in field coordinates.</li>
     * </ul>
     *
     * <p>Each time guidance is enabled, Phoenix captures the current field pose of the
     * <b>translation control frame</b> (see {@link ControlFrames#robotToTranslationFrame()}).
     * The requested offset is applied in that frame to produce a fixed field target point.</p>
     *
     * <p><b>Requires:</b> field pose feedback (odometry / localization). Observation-only feedback
     * cannot measure robot displacement, so this target cannot be solved from vision alone.</p>
     */
    public static final class RobotRelativePoint implements TranslationTarget {

        /**
         * Forward offset in inches (+X in the robot/translation frame).
         */
        public final double forwardInches;

        /**
         * Left offset in inches (+Y in the robot/translation frame).
         */
        public final double leftInches;

        public RobotRelativePoint(double forwardInches, double leftInches) {
            this.forwardInches = forwardInches;
            this.leftInches = leftInches;
        }
    }

    // ------------------------------------------------------------------------
    // Feedback
    // ------------------------------------------------------------------------

    /**
     * Behavior when guidance cannot produce a valid command.
     */
    public enum LossPolicy {
        /**
         * Do not override any DOF when guidance is invalid.
         *
         * <p>This is the recommended default for TeleOp assist.</p>
         */
        PASS_THROUGH,

        /**
         * Override the requested DOFs with zero commands when guidance is invalid.
         *
         * <p>This “hard stops” the robot in those DOFs and can feel surprising in TeleOp.
         * Use intentionally.</p>
         */
        ZERO_OUTPUT
    }

    /**
     * Feedback configuration for driver guidance.
     */
    public static final class Feedback {

        /**
         * Observation feedback (may be null).
         */
        public final Observation observation;

        /**
         * Field pose feedback (may be null).
         */
        public final FieldPose fieldPose;

        /**
         * Auto-selection gates (used only when both sources are present).
         */
        public final Gates gates;

        /**
         * If true, use observation for omega whenever it is valid, even if translation uses field pose.
         */
        public final boolean preferObservationForOmega;

        /**
         * Loss behavior when no valid command can be produced.
         */
        public final LossPolicy lossPolicy;

        private Feedback(Observation observation,
                         FieldPose fieldPose,
                         Gates gates,
                         boolean preferObservationForOmega,
                         LossPolicy lossPolicy) {
            this.observation = observation;
            this.fieldPose = fieldPose;
            this.gates = gates;
            this.preferObservationForOmega = preferObservationForOmega;
            this.lossPolicy = lossPolicy;
        }

        /**
         * Create a feedback configuration.
         *
         * <p>If both observation and field pose are provided, Phoenix uses adaptive selection.
         * If gates are null, {@link Gates#defaults()} will be used.</p>
         */
        public static Feedback create(Observation observation,
                                      FieldPose fieldPose,
                                      Gates gates,
                                      boolean preferObservationForOmega,
                                      LossPolicy lossPolicy) {
            if (observation == null && fieldPose == null) {
                throw new IllegalArgumentException("feedback requires observation and/or fieldPose");
            }

            Gates g = gates;
            if (observation != null && fieldPose != null && g == null) {
                g = Gates.defaults();
            }

            LossPolicy lp = (lossPolicy != null) ? lossPolicy : LossPolicy.PASS_THROUGH;
            return new Feedback(observation, fieldPose, g, preferObservationForOmega, lp);
        }

        /**
         * @return true if this feedback config includes an observation source.
         */
        public boolean hasObservation() {
            return observation != null;
        }

        /**
         * @return true if this feedback config includes a field pose estimator.
         */
        public boolean hasFieldPose() {
            return fieldPose != null;
        }

        /**
         * @return true if the overlay will adaptively choose between sources (both are present).
         */
        public boolean isAdaptive() {
            return hasObservation() && hasFieldPose();
        }

        /**
         * Convenience: return the configured {@link TagLayout} if (and only if) field pose feedback is present.
         */
        public TagLayout tagLayoutOrNull() {
            return fieldPose != null ? fieldPose.tagLayout : null;
        }
    }

    /**
     * Observation feedback parameters.
     */
    public static final class Observation {

        public static final double DEFAULT_MAX_AGE_SEC = 0.50;
        public static final double DEFAULT_MIN_QUALITY = 0.10;

        public final ObservationSource2d source;

        /**
         * Maximum age (seconds) for an observation to be considered valid.
         */
        public final double maxAgeSec;

        /**
         * Minimum quality (0..1) for an observation to be considered valid.
         */
        public final double minQuality;

        public Observation(ObservationSource2d source) {
            this(source, DEFAULT_MAX_AGE_SEC, DEFAULT_MIN_QUALITY);
        }

        public Observation(ObservationSource2d source, double maxAgeSec, double minQuality) {
            this.source = Objects.requireNonNull(source, "source");
            this.maxAgeSec = maxAgeSec;
            this.minQuality = minQuality;
        }
    }

    /**
     * Field pose feedback parameters.
     */
    public static final class FieldPose {

        public static final double DEFAULT_MAX_AGE_SEC = 0.50;
        public static final double DEFAULT_MIN_QUALITY = 0.10;

        public final PoseEstimator poseEstimator;

        /**
         * Optional tag layout (required for tag-relative targets when using field pose).
         */
        public final TagLayout tagLayout;

        /**
         * Maximum age (seconds) for a pose estimate to be considered valid.
         */
        public final double maxAgeSec;

        /**
         * Minimum quality (0..1) for a pose estimate to be considered valid.
         */
        public final double minQuality;

        public FieldPose(PoseEstimator poseEstimator) {
            this(poseEstimator, null, DEFAULT_MAX_AGE_SEC, DEFAULT_MIN_QUALITY);
        }

        public FieldPose(PoseEstimator poseEstimator, TagLayout tagLayout) {
            this(poseEstimator, tagLayout, DEFAULT_MAX_AGE_SEC, DEFAULT_MIN_QUALITY);
        }

        public FieldPose(PoseEstimator poseEstimator,
                         TagLayout tagLayout,
                         double maxAgeSec,
                         double minQuality) {
            this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator");
            this.tagLayout = tagLayout;
            this.maxAgeSec = maxAgeSec;
            this.minQuality = minQuality;
        }
    }

    /**
     * Hysteresis + blending parameters for adaptive feedback selection.
     */
    public static final class Gates {

        public static final double DEFAULT_ENTER_RANGE_IN = 9.0;
        public static final double DEFAULT_EXIT_RANGE_IN = 12.0;
        public static final double DEFAULT_TAKEOVER_BLEND_SEC = 0.15;

        /**
         * Switch to observation when the observed range (inches) is &lt;= this value.
         */
        public final double enterRangeInches;

        /**
         * Switch back to field pose when the observed range (inches) is &gt;= this value.
         */
        public final double exitRangeInches;

        /**
         * Blend time (seconds) when switching sources. 0 for instantaneous.
         */
        public final double takeoverBlendSec;

        public Gates(double enterRangeInches, double exitRangeInches, double takeoverBlendSec) {
            this.enterRangeInches = enterRangeInches;
            this.exitRangeInches = exitRangeInches;
            this.takeoverBlendSec = takeoverBlendSec;
        }

        /**
         * Reasonable defaults for typical “approach then fine align” driver assist.
         */
        public static Gates defaults() {
            return new Gates(DEFAULT_ENTER_RANGE_IN, DEFAULT_EXIT_RANGE_IN, DEFAULT_TAKEOVER_BLEND_SEC);
        }
    }

    // ------------------------------------------------------------------------
    // Spec data
    // ------------------------------------------------------------------------

    public final TranslationTarget translationTarget; // may be null
    public final AimTarget aimTarget;                 // may be null
    public final ControlFrames controlFrames;
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
     * @return the natural (requested) mask implied by the configured targets.
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
     * Alias for {@link #requestedMask()}.
     */
    public DriveOverlayMask suggestedMask() {
        return requestedMask();
    }
}
