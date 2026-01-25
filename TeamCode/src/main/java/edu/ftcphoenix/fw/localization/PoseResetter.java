package edu.ftcphoenix.fw.localization;

import edu.ftcphoenix.fw.core.geometry.Pose2d;

/**
 * Optional capability interface for {@link PoseEstimator}s that can be "snapped" to a known pose.
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
     * Units: inches for x/y and radians for heading, consistent with {@link Pose2d}.</p>
     */
    void setPose(Pose2d pose);
}
