package edu.ftcphoenix.fw.localization.fusion;

import edu.ftcphoenix.fw.localization.PoseEstimator;
import edu.ftcphoenix.fw.localization.PoseResetter;

/**
 * Shared contract for global localizers that combine a smooth odometry source with an optional
 * absolute-correction vision source.
 *
 * <p>Phoenix currently ships two implementations with this contract:</p>
 * <ul>
 *   <li>{@link OdometryTagFusionPoseEstimator}: a lightweight, gain-based complementary localizer</li>
 *   <li>{@link OdometryTagEkfPoseEstimator}: an optional covariance-aware EKF-style localizer</li>
 * </ul>
 *
 * <p>The goal is not to force every robot to use the more advanced estimator. Instead, robot code
 * and tooling can depend on this narrow interface and swap implementations intentionally.</p>
 */
public interface VisionCorrectionPoseEstimator extends PoseEstimator, PoseResetter {

    /**
     * Enables or disables vision corrections while keeping odometry prediction alive.
     */
    void setVisionEnabled(boolean enabled);

    /**
     * Returns whether the vision lane is currently enabled.
     */
    boolean isVisionEnabled();

    /**
     * Returns a compact immutable snapshot of common vision-correction telemetry.
     */
    VisionCorrectionStats getVisionCorrectionStats();
}
