package edu.ftcphoenix.fw.localization.fusion;

import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.PoseResetter;

/**
 * Shared contract for global localizers that combine a smooth motion predictor with an optional
 * absolute correction source.
 *
 * <p>Phoenix currently ships two implementations with this contract:</p>
 * <ul>
 *   <li>{@link OdometryCorrectionFusionEstimator}: a lightweight, gain-based complementary localizer</li>
 *   <li>{@link OdometryCorrectionEkfEstimator}: an optional covariance-aware EKF-style localizer</li>
 * </ul>
 *
 * <p>The goal is not to force every robot to use the more advanced estimator. Robot code and tools
 * can depend on this narrow interface and swap implementations intentionally.</p>
 */
public interface CorrectedPoseEstimator extends AbsolutePoseEstimator, PoseResetter {

    /**
     * Enables or disables absolute corrections while keeping motion prediction alive.
     */
    void setCorrectionEnabled(boolean enabled);

    /**
     * @return whether the correction path is currently enabled.
     */
    boolean isCorrectionEnabled();

    /**
     * @return compact immutable snapshot of common correction telemetry and gating state.
     */
    CorrectionStats getCorrectionStats();
}
