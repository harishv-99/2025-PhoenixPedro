package edu.ftcphoenix.fw.localization;

/**
 * Absolute pose estimator that also exposes the most recent incremental motion update.
 *
 * <p>This interface formalizes the predictor side of Phoenix localization:</p>
 * <ul>
 *   <li>the predictor still exposes a current absolute pose estimate via {@link #getEstimate()},</li>
 *   <li>but fusion-style estimators can also ask for the latest timestamped motion increment via
 *       {@link #getLatestMotionDelta()} instead of reverse-engineering deltas from successive
 *       absolute poses.</li>
 * </ul>
 *
 * <p>Typical implementations include dead-wheel odometry computers, wheel+IMU dead-reckoners, or
 * other high-rate motion trackers that are good at short-term propagation but can drift without an
 * occasional absolute correction.</p>
 */
public interface MotionPredictor extends AbsolutePoseEstimator {

    /**
     * Returns the most recent motion increment produced by the predictor.
     *
     * <p>The returned delta must correspond to the predictor update most recently applied by
     * {@link #update(edu.ftcphoenix.fw.core.time.LoopClock)}. Like {@link #getEstimate()}, this
     * method must be safe to call multiple times between update calls.</p>
     *
     * <p>Common usage:</p>
     * <pre>
     * predictor.update(clock);
     * MotionDelta delta = predictor.getLatestMotionDelta();
     * PoseEstimate pose = predictor.getEstimate();
     * </pre>
     *
     * @return latest timestamped motion increment; callers should check {@link MotionDelta#hasDelta}
     */
    MotionDelta getLatestMotionDelta();
}
