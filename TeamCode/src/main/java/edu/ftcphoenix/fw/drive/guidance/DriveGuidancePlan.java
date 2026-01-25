package edu.ftcphoenix.fw.drive.guidance;

import java.util.Objects;

import edu.ftcphoenix.fw.drive.DriveOverlay;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.PoseEstimator;
import edu.ftcphoenix.fw.sensing.observation.ObservationSource2d;

/**
 * Immutable configuration for a “driver guidance” overlay.
 *
 * <p>A {@link DriveGuidancePlan} describes:</p>
 * <ul>
 *   <li><b>What you want to do</b>: translate, aim, or both.</li>
 *   <li><b>Where the target is defined</b>: field coordinates, relative to an anchor (e.g. AprilTag),
 *       or relative to the robot at the moment the overlay is enabled.</li>
 *   <li><b>Which point on the robot is being controlled</b>: robot center or an off-center mechanism.</li>
 *   <li><b>What feedback source is used</b>: observation, field pose, or an adaptive combination.</li>
 * </ul>
 *
 * <p>The plan does <em>not</em> directly move hardware. Instead you convert it into a
 * {@link DriveOverlay} via {@link #overlay()} and apply it to a base
 * {@link edu.ftcphoenix.fw.drive.DriveSource} using
 * {@link edu.ftcphoenix.fw.drive.DriveSource#overlayWhen}.</p>
 */
public final class DriveGuidancePlan {

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
         * vision-first plans that start with observation and may optionally add a field pose
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
     * Instead of aiming at a point, DriveGuidance will rotate the robot until the aim control
     * frame's heading matches {@link #fieldHeadingRad}.</p>
     *
     * <p><b>Important:</b> This target requires {@link Feedback#hasFieldPose()} because it needs a
     * current robot heading estimate in the field frame. It cannot be solved from observation-only
     * feedback.</p>
     *
     * <p>If you use the default {@link ControlFrames#robotCenter()}, this means “rotate the robot
     * to this field heading”. If you set an aim frame with a heading offset, the aim frame's
     * +X axis is what will be aligned to the field heading.</p>
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
     * Translation target defined <b>relative to the robot</b> when the overlay becomes enabled.
     *
     * <p>This is intended for “nudge” style behaviors such as:</p>
     * <ul>
     *   <li>drive forward 6 inches from wherever you are,</li>
     *   <li>strafe left 3 inches,</li>
     *   <li>micro-adjust an intake position without thinking in field coordinates.</li>
     * </ul>
     *
     * <p>Each time the guidance overlay is enabled, it captures the current field pose of the
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
    // Tuning
    // ------------------------------------------------------------------------

    /**
     * Tuning knobs for DriveGuidance controllers.
     *
     * <p><b>Student-friendly explanation:</b> DriveGuidance measures an error (how far you are from
     * a target point, or how many radians you are turned away) and produces a drive command that
     * reduces that error. These constants decide how “strong” the corrections are.</p>
     *
     * <p>These values are intentionally <b>unitless</b> and operate directly in
     * {@link edu.ftcphoenix.fw.drive.DriveSignal} units (roughly -1..1). That makes them easy to use
     * in TeleOp without any drivetrain characterization.</p>
     *
     * <h3>How to tune (quick checklist)</h3>
     * <ol>
     *   <li>Start with {@link #defaults()}.</li>
     *   <li>If the assist feels <b>too weak</b>: increase the relevant kP.</li>
     *   <li>If it feels <b>twitchy / oscillates</b>: decrease kP, or increase {@link #aimDeadbandRad}.</li>
     *   <li>If it moves/turns <b>too fast</b>: lower {@link #maxTranslateCmd} / {@link #maxOmegaCmd}.</li>
     * </ol>
     *
     * <h3>Typical values</h3>
     * <ul>
     *   <li>{@code maxTranslateCmd}: 0.3–0.8</li>
     *   <li>{@code maxOmegaCmd}: 0.3–1.0</li>
     *   <li>{@code aimDeadbandRad}: 0.5°–2° (convert to radians with {@code Math.toRadians(...)})</li>
     * </ul>
     */
    public static final class Tuning {

        /**
         * P gain for translation: command per inch of error.
         */
        public final double kPTranslate;

        /**
         * Max translation command magnitude (0..1-ish).
         */
        public final double maxTranslateCmd;

        /**
         * P gain for aim: omega command per radian of bearing error.
         */
        public final double kPAim;

        /**
         * Max omega command magnitude (0..1-ish).
         */
        public final double maxOmegaCmd;

        /**
         * Deadband for aim (radians): errors smaller than this output 0 omega.
         */
        public final double aimDeadbandRad;

        public Tuning(double kPTranslate,
                      double maxTranslateCmd,
                      double kPAim,
                      double maxOmegaCmd,
                      double aimDeadbandRad) {
            this.kPTranslate = kPTranslate;
            this.maxTranslateCmd = maxTranslateCmd;
            this.kPAim = kPAim;
            this.maxOmegaCmd = maxOmegaCmd;
            this.aimDeadbandRad = aimDeadbandRad;
        }

        /**
         * Return a copy of this tuning with a different translation kP gain.
         *
         * <p>Higher values make the robot drive toward the translation target more aggressively.</p>
         */
        public Tuning withTranslateKp(double kPTranslate) {
            return new Tuning(kPTranslate, maxTranslateCmd, kPAim, maxOmegaCmd, aimDeadbandRad);
        }

        /**
         * Return a copy of this tuning with a different maximum translation command.
         *
         * <p>This caps how fast the assist can drive the robot in X/Y (typical range: 0..1).</p>
         */
        public Tuning withMaxTranslateCmd(double maxTranslateCmd) {
            return new Tuning(kPTranslate, maxTranslateCmd, kPAim, maxOmegaCmd, aimDeadbandRad);
        }

        /**
         * Return a copy of this tuning with a different aim (turn) kP gain.
         *
         * <p>Higher values make the robot turn toward the aim target more aggressively.</p>
         */
        public Tuning withAimKp(double kPAim) {
            return new Tuning(kPTranslate, maxTranslateCmd, kPAim, maxOmegaCmd, aimDeadbandRad);
        }

        /**
         * Return a copy of this tuning with a different maximum omega (turn) command.
         *
         * <p>This caps how fast the assist can turn the robot (typical range: 0..1).</p>
         */
        public Tuning withMaxOmegaCmd(double maxOmegaCmd) {
            return new Tuning(kPTranslate, maxTranslateCmd, kPAim, maxOmegaCmd, aimDeadbandRad);
        }

        /**
         * Return a copy of this tuning with a different aim deadband in radians.
         *
         * <p>If the aim error magnitude is smaller than this, DriveGuidance outputs 0 turn command.</p>
         */
        public Tuning withAimDeadbandRad(double aimDeadbandRad) {
            return new Tuning(kPTranslate, maxTranslateCmd, kPAim, maxOmegaCmd, aimDeadbandRad);
        }

        /**
         * Convenience overload: set the aim deadband in degrees.
         */
        public Tuning withAimDeadbandDeg(double aimDeadbandDeg) {
            return withAimDeadbandRad(Math.toRadians(aimDeadbandDeg));
        }

        /**
         * Reasonable defaults for typical TeleOp assist.
         */
        public static Tuning defaults() {
            return new Tuning(
                    0.05,                 // kPTranslate
                    0.60,                 // maxTranslateCmd
                    2.50,                 // kPAim
                    0.80,                 // maxOmegaCmd
                    Math.toRadians(1.0)   // aimDeadbandRad
            );
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
    // Plan data
    // ------------------------------------------------------------------------

    public final TranslationTarget translationTarget; // may be null
    public final AimTarget aimTarget;                 // may be null
    public final ControlFrames controlFrames;
    public final Tuning tuning;
    public final Feedback feedback;

    DriveGuidancePlan(TranslationTarget translationTarget,
                      AimTarget aimTarget,
                      ControlFrames controlFrames,
                      Tuning tuning,
                      Feedback feedback) {
        this.translationTarget = translationTarget;
        this.aimTarget = aimTarget;
        this.controlFrames = Objects.requireNonNull(controlFrames, "controlFrames");
        this.tuning = Objects.requireNonNull(tuning, "tuning");
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
     *
     * <p>"Suggested" exists because the overlay may choose to return {@link DriveOverlayMask#NONE}
     * at runtime if it cannot compute a safe command. The plan's suggestion is still based on what
     * the user configured.</p>
     */
    public DriveOverlayMask suggestedMask() {
        return requestedMask();
    }

    /**
     * Create a {@link DriveOverlay} implementing this plan.
     */
    public DriveOverlay overlay() {
        return new DriveGuidanceOverlay(this);
    }

    /**
     * Build a {@link DriveGuidanceTask} that executes this plan as an autonomous-style task.
     */
    public DriveGuidanceTask task(edu.ftcphoenix.fw.drive.MecanumDrivebase drivebase, DriveGuidanceTask.Config cfg) {
        return new DriveGuidanceTask(drivebase, this, cfg);
    }

    /**
     * Build a {@link DriveGuidanceQuery} that can sample this plan's errors (and predicted drive command)
     * without enabling an overlay.
     *
     * <p>This is useful for driver telemetry and safety gating:</p>
     * <ul>
     *   <li>"Only shoot when the aim error is small"</li>
     *   <li>"Show the current omega error on telemetry"</li>
     * </ul>
     */
    public DriveGuidanceQuery query() {
        return new DriveGuidanceQuery(this);
    }
}
