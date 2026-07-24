package edu.ftcphoenix.fw.localization;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;

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
    public final LoopTimestamp startTimestamp;

    /**
     * Timestamp of the later predictor sample used to form this delta.
     */
    public final LoopTimestamp endTimestamp;

    /**
     * Creates a new motion increment.
     *
     * @param deltaPose         relative transform from the previous predictor pose to the current pose
     * @param hasDelta          whether this instance represents a usable motion increment
     * @param quality           simple trust score in [0, 1]
     * @param startTimestamp timestamp of the earlier sample
     * @param endTimestamp   timestamp of the later sample
     */
    public MotionDelta(Pose3d deltaPose,
                       boolean hasDelta,
                       double quality,
                       LoopTimestamp startTimestamp,
                       LoopTimestamp endTimestamp) {
        if (deltaPose == null) {
            throw new IllegalArgumentException("deltaPose is required");
        }
        if (startTimestamp == null || endTimestamp == null) {
            throw new IllegalArgumentException(
                    "startTimestamp and endTimestamp are required; use LoopTimestamp.unavailable() when needed");
        }
        this.deltaPose = deltaPose;
        this.hasDelta = hasDelta;
        this.quality = quality;
        this.startTimestamp = startTimestamp;
        this.endTimestamp = endTimestamp;
    }

    /**
     * @return signed seconds from {@link #startTimestamp} to {@link #endTimestamp}, or
     *         {@code NaN} when either timestamp is unavailable or no longer in the current epoch
     */
    public double durationSec() {
        return endTimestamp.secondsSince(startTimestamp);
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
    public static MotionDelta none(LoopTimestamp timestamp) {
        return new MotionDelta(Pose3d.zero(), false, 0.0, timestamp, timestamp);
    }

    @Override
    public String toString() {
        if (!hasDelta) {
            return "MotionDelta{no delta, timestamp=" + endTimestamp + "}";
        }
        return "MotionDelta{" +
                "deltaPose=" + deltaPose +
                ", quality=" + quality +
                ", startTimestamp=" + startTimestamp +
                ", endTimestamp=" + endTimestamp +
                '}';
    }
}
