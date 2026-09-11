package edu.ftcsushi.fw.drive.guidance;

import java.util.Objects;

import edu.ftcsushi.fw.drive.DriveOverlayMask;
import edu.ftcsushi.fw.field.TagLayout;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcsushi.fw.spatial.FacingTarget2d;
import edu.ftcsushi.fw.spatial.SpatialControlFrames;
import edu.ftcsushi.fw.spatial.SpatialQuerySpec;
import edu.ftcsushi.fw.spatial.TranslationTarget2d;

/**
 * Immutable controller-neutral target, control-frame, and evidence choice for guidance.
 *
 * <p>Construct through {@link DriveGuidance#spec()} or {@link DriveGuidance#plan()}. One configured
 * evidence mode owns every requested channel. Guidance borrows its sources; it neither corrects
 * localization nor switches to another pose source when evidence is lost.</p>
 */
public final class DriveGuidanceSpec {

    /** Explicit evidence authority, independent of whether this loop solved a requested channel. */
    public enum SolveMode {
        /** Field geometry from one already-updated absolute pose estimator. */
        ABSOLUTE_POSE,
        /** Actual tag-relative geometry from fresh camera observations, without a field solve. */
        RELATIVE_APRIL_TAGS,
        /** Delayed robot-at-capture points; no invented field pose or motion compensation. */
        OBSERVED_POINTS
    }

    /** Behavior for each requested channel whose geometry is unavailable. */
    public enum LossPolicy {
        /** Leave unsolved requested channels with the underlying drive source. */
        PASS_THROUGH,
        /** Override unsolved requested channels with zero; solved channels keep their correction. */
        ZERO_OUTPUT
    }

    /**
     * Robot-relative movement captured from the translation control frame at the first valid solve
     * after guidance enables.
     *
     * <p>This remains a drive-guidance-specific target because it is not just a geometric point.
     * It is a <em>latched target semantic</em>: capture the current translation frame in field space
     * at the first valid solve after enable, then offset from that captured anchor in the frame's
     * local forward/left axes. Unavailable evidence cannot establish the anchor.</p>
     */
    public static final class RobotRelativePoint implements TranslationTarget2d {
        public final double forwardInches;
        public final double leftInches;

        /**
         * Creates a robot-relative translation target captured from the translation control frame
         * when guidance enables. Both offsets must be finite; signed offsets are valid.
         */
        RobotRelativePoint(double forwardInches, double leftInches) {
            this.forwardInches = requireFinite("forwardInches", forwardInches);
            this.leftInches = requireFinite("leftInches", leftInches);
        }

        private static double requireFinite(String fieldName, double value) {
            if (!Double.isFinite(value)) {
                throw new IllegalArgumentException(
                        "DriveGuidanceSpec.RobotRelativePoint." + fieldName
                                + " must be finite, got " + value);
            }
            return value;
        }
    }

    /** Fixed camera and frame-age policy for direct tag-relative guidance. */
    public static final class RelativeAprilTags {
        public static final double DEFAULT_MAX_AGE_SEC = 0.50;
        public final AprilTagSensor sensor;
        public final CameraMountConfig cameraMount;
        public final double maxAgeSec;

        RelativeAprilTags(AprilTagSensor sensor, CameraMountConfig cameraMount, double maxAgeSec) {
            this.sensor = Objects.requireNonNull(sensor, "sensor");
            this.cameraMount = Objects.requireNonNull(cameraMount, "cameraMount");
            this.maxAgeSec = maxAgeSec;
        }
    }

    /** Borrowed absolute-pose estimator and action-specific evidence admission. */
    public static final class AbsolutePose {
        public static final double DEFAULT_MAX_AGE_SEC = 0.50;
        public static final double DEFAULT_MIN_QUALITY = 0.10;
        public final AbsolutePoseEstimator poseEstimator;
        public final double maxAgeSec;
        public final double minQuality;

        AbsolutePose(AbsolutePoseEstimator poseEstimator, double maxAgeSec, double minQuality) {
            this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator");
            this.maxAgeSec = maxAgeSec;
            this.minQuality = minQuality;
        }
    }

    /** Readable immutable result of the selected evidence branch, not another construction API. */
    public static final class ResolveWith {
        public final SolveMode mode;
        public final RelativeAprilTags relativeAprilTags;
        public final AbsolutePose absolutePose;
        public final TagLayout fixedAprilTagLayout;
        public final LossPolicy lossPolicy;

        private ResolveWith(SolveMode mode, RelativeAprilTags relativeAprilTags,
                            AbsolutePose absolutePose, TagLayout fixedAprilTagLayout,
                            LossPolicy lossPolicy) {
            this.mode = mode;
            this.relativeAprilTags = relativeAprilTags;
            this.absolutePose = absolutePose;
            this.fixedAprilTagLayout = fixedAprilTagLayout;
            this.lossPolicy = lossPolicy;
        }

        /** Validates the sole configured evidence authority at the completed-owner boundary. */
        static ResolveWith create(SolveMode mode, RelativeAprilTags relativeAprilTags,
                                  AbsolutePose absolutePose, TagLayout fixedAprilTagLayout,
                                  LossPolicy lossPolicy) {
            Objects.requireNonNull(mode, "mode");
            Objects.requireNonNull(lossPolicy, "lossPolicy");
            if ((mode == SolveMode.ABSOLUTE_POSE) != (absolutePose != null)
                    || (mode == SolveMode.RELATIVE_APRIL_TAGS) != (relativeAprilTags != null)
                    || (mode != SolveMode.ABSOLUTE_POSE && fixedAprilTagLayout != null)) {
                throw new IllegalArgumentException("Solve mode must have exactly its own evidence source; "
                        + "only absolutePose accepts fixedAprilTagLayout");
            }
            return new ResolveWith(mode, relativeAprilTags, absolutePose,
                    fixedAprilTagLayout, lossPolicy);
        }
    }

    public final TranslationTarget2d translationTarget;
    public final FacingTarget2d facingTarget;
    public final SpatialControlFrames controlFrames;
    public final ResolveWith resolveWith;
    public final SpatialQuerySpec spatialQuerySpec;

    DriveGuidanceSpec(TranslationTarget2d translationTarget, FacingTarget2d facingTarget,
                      SpatialControlFrames controlFrames, ResolveWith resolveWith,
                      SpatialQuerySpec spatialQuerySpec) {
        this.translationTarget = translationTarget;
        this.facingTarget = facingTarget;
        this.controlFrames = Objects.requireNonNull(controlFrames, "controlFrames");
        this.resolveWith = Objects.requireNonNull(resolveWith, "resolveWith");
        this.spatialQuerySpec = spatialQuerySpec;
    }

    /** Returns the channels this description actually requests. */
    public DriveOverlayMask requestedMask() {
        boolean t = translationTarget != null;
        boolean o = facingTarget != null;
        if (t && o) return DriveOverlayMask.ALL;
        if (t) return DriveOverlayMask.TRANSLATION_ONLY;
        if (o) return DriveOverlayMask.OMEGA_ONLY;
        return DriveOverlayMask.NONE;
    }
}
