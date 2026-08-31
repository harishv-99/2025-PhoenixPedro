package edu.ftcsushi.fw.localization.fusion;

import edu.ftcsushi.fw.localization.PoseResetter;
import edu.ftcsushi.fw.localization.PoseTrajectoryEstimator;

/**
 * Shared contract for global localizers that combine a smooth motion predictor with an optional
 * absolute correction source.
 *
 * <p>Sushi currently ships two implementations with this contract:</p>
 * <ul>
 *   <li>{@link OdometryCorrectionFusionEstimator}: a lightweight, gain-based complementary localizer</li>
 *   <li>{@link OdometryCorrectionEkfEstimator}: an optional covariance-aware EKF-style localizer</li>
 * </ul>
 *
 * <p>The goal is not to force every robot to use the more advanced estimator. Robot code and tools
 * can depend on this narrow interface and swap implementations intentionally.</p>
 *
 * <p>All implementations share one final-trajectory continuity rule. An accepted ordinary
 * correction stays in the current {@link #trajectorySegmentId()} even when the estimator performs
 * an expected rebase of its private predictor. A successful manual {@link #setPose} anchor starts
 * a different final segment. Observing an unexpected predictor-segment change also starts a
 * different final segment and must clear any replay state that could cross that boundary. If a
 * nontransactional predictor rebase changes its segment and then throws, the corrected estimator
 * must retain the new final segment and fail closed rather than republish its old available pose.</p>
 */
public interface CorrectedPoseEstimator extends PoseTrajectoryEstimator, PoseResetter {

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
