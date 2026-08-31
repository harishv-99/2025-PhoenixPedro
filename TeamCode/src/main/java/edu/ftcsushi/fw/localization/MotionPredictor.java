package edu.ftcsushi.fw.localization;

/**
 * Absolute pose estimator that also exposes the most recent incremental motion update.
 *
 * <p>This interface formalizes the predictor side of Sushi localization:</p>
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
 *
 * <p>One successful {@link #update(edu.ftcsushi.fw.core.time.LoopClock)} publishes one coherent
 * pair: {@link #getEstimate()} is the latest absolute sample and
 * {@link #getLatestMotionDelta()} is the interval ending at that same sample. A usable delta has a
 * strictly positive duration in the current clock epoch. If a new-cycle sample arrives without
 * positive elapsed time, the predictor publishes no usable delta and retains its accepted motion
 * baseline so that movement is included when a strictly later sample arrives.</p>
 */
public interface MotionPredictor extends PoseTrajectoryEstimator {

    /**
     * Returns the most recent motion increment produced by the predictor.
     *
     * <p>The returned delta must correspond to the predictor update most recently applied by
     * {@link #update(edu.ftcsushi.fw.core.time.LoopClock)}. When both values carry available
     * timestamps, its {@link MotionDelta#endTimestamp} identifies the same latest sample as
     * {@link #getEstimate()}. Like {@link #getEstimate()}, this method must be safe to call multiple
     * times between update calls.</p>
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
