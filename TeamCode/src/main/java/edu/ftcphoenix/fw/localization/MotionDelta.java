package edu.ftcphoenix.fw.localization;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.math.MathUtil;

/**
 * Immutable timestamped motion increment produced by a {@link MotionPredictor}.
 *
 * <p>A {@code MotionDelta} is the predictor-side counterpart to {@link PoseEstimate}:</p>
 * <ul>
 *   <li>{@link PoseEstimate} says <em>where the robot is</em> in field coordinates.</li>
 *   <li>{@code MotionDelta} says <em>how the robot moved</em> between two timestamps.</li>
 * </ul>
 *
 * <p>The delta pose is expressed as a rigid transform from the predictor's previous pose to its
 * current pose. In practice, drivetrain localizers typically use only the planar components
 * ({@code x}, {@code y}, {@code yaw}), but the value object stays 6DOF so the geometry remains
 * explicit and future predictors are not forced into a planar-only API.</p>
 */
public final class MotionDelta {

    /**
     * Relative motion transform from the earlier predictor sample to the later predictor sample.
     */
    public final Pose3d deltaPose;

    /**
     * True when this instance represents a usable motion increment.
     */
    public final boolean hasDelta;

    /**
     * Simple quality score in the range [0, 1].
     */
    public final double quality;

    /**
     * Timestamp of the earlier predictor sample used to form this delta.
     */
    public final double startTimestampSec;

    /**
     * Timestamp of the later predictor sample used to form this delta.
     */
    public final double endTimestampSec;

    /**
     * Creates a new motion increment.
     *
     * @param deltaPose         relative transform from the previous predictor pose to the current pose
     * @param hasDelta          whether this instance represents a usable motion increment
     * @param quality           simple trust score in [0, 1]
     * @param startTimestampSec timestamp of the earlier sample
     * @param endTimestampSec   timestamp of the later sample
     */
    public MotionDelta(Pose3d deltaPose,
                       boolean hasDelta,
                       double quality,
                       double startTimestampSec,
                       double endTimestampSec) {
        if (deltaPose == null) {
            throw new IllegalArgumentException("deltaPose is required");
        }
        this.deltaPose = deltaPose;
        this.hasDelta = hasDelta;
        this.quality = quality;
        this.startTimestampSec = startTimestampSec;
        this.endTimestampSec = endTimestampSec;
    }

    /**
     * @return {@code endTimestampSec - startTimestampSec}.
     */
    public double durationSec() {
        return endTimestampSec - startTimestampSec;
    }

    /**
     * Returns the planar translation magnitude of the delta pose in inches.
     */
    public double planarTranslationInches() {
        return Math.hypot(deltaPose.xInches, deltaPose.yInches);
    }

    /**
     * Returns the planar yaw change in radians, wrapped to (-pi, pi].
     */
    public double planarYawDeltaRad() {
        return MathUtil.wrapToPi(deltaPose.yawRad);
    }

    /**
     * Creates a value representing "no usable delta" at a single timestamp.
     */
    public static MotionDelta none(double timestampSec) {
        return new MotionDelta(Pose3d.zero(), false, 0.0, timestampSec, timestampSec);
    }

    @Override
    public String toString() {
        if (!hasDelta) {
            return "MotionDelta{no delta, timestampSec=" + endTimestampSec + "}";
        }
        return "MotionDelta{" +
                "deltaPose=" + deltaPose +
                ", quality=" + quality +
                ", startTimestampSec=" + startTimestampSec +
                ", endTimestampSec=" + endTimestampSec +
                '}';
    }
}
