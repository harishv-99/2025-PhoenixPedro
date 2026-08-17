package edu.ftcphoenix.fw.localization;

import edu.ftcphoenix.fw.core.geometry.Pose2d;

/**
 * Optional capability interface for {@link AbsolutePoseEstimator}s that can be "snapped" to a known pose.
 *
 * <p>In practice, some estimators (e.g., odometry hardware like Pinpoint) can accept an absolute
 * pose reset. Fusion estimators can use this to keep the underlying odometry aligned to the fused
 * estimate after vision corrections.</p>
 */
public interface PoseResetter {

    /**
     * Overwrites the estimator's internal pose to the provided value.
     *
     * <p>Implementations should treat this as an immediate pose set (not a "start pose" shift).
     * Units: inches for x/y and radians for heading, consistent with {@link Pose2d}. A null pose or
     * any non-finite x/y/heading component must fail before changing estimator state, history,
     * another predictor, or hardware. A concrete boundary may reject additional values that cannot
     * be represented by its downstream device.</p>
     *
     * @throws NullPointerException if {@code pose} is null
     * @throws IllegalArgumentException if any authored pose component is non-finite or cannot be
     *                                  represented by the implementation's downstream boundary
     */
    void setPose(Pose2d pose);
}
