package edu.ftcphoenix.fw.localization;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Common interface for components that estimate the robot's pose on the field.
 *
 * <p>A {@code PoseEstimator} typically consumes one or more sensing sources
 * (e.g., AprilTags, wheel encoders, IMU) and produces a {@link PoseEstimate}
 * that can be used by drive controllers and tasks. Different implementations
 * may have very different internal logic, but all share the same basic
 * contract:</p>
 *
 * <ul>
 *   <li>Call {@link #update(LoopClock)} once per control loop to advance the
 *       estimator's internal state.</li>
 *   <li>Call {@link #getEstimate()} to retrieve the most recent pose estimate.</li>
 * </ul>
 *
 * <p>This keeps the estimator usage simple and consistent with the rest of
 * the Phoenix framework's loop-based design.</p>
 */
public interface PoseEstimator {

    /**
     * Advance the estimator's internal state using the current time.
     *
     * <p>Implementations are responsible for:
     * <ul>
     *   <li>Reading any sensors they depend on (directly or via adapters),</li>
     *   <li>Updating their internal filters / state machines, and</li>
     *   <li>Preparing the next {@link PoseEstimate} that
     *       {@link #getEstimate()} will return.</li>
     * </ul>
     *
     * <p>The framework expects this method to be called once per main
     * control loop, typically from a robot's OpMode loop body or a
     * higher-level framework loop.</p>
     *
     * @param clock loop clock providing the current time in seconds
     */
    void update(LoopClock clock);

    /**
     * Returns the most recent pose estimate.
     *
     * <p>This method must be safe to call multiple times between {@link #update(LoopClock)}
     * calls; it should simply return the last estimate computed by the most
     * recent {@code update()}.</p>
     *
     * @return the latest {@link PoseEstimate}; callers should check
     * {@link PoseEstimate#hasPose} before using the pose for control
     */
    PoseEstimate getEstimate();


    /**
     * Debug helper: emit a compact summary of the most recent estimate.
     *
     * <p>Implementations with meaningful internal state should override this method
     * to expose richer telemetry (sources, residuals, gating decisions, etc.).</p>
     *
     * @param dbg    debug sink (may be {@code null}; if null, no output is produced)
     * @param prefix base key prefix, e.g. "localizer"
     */
    default void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "poseEstimator" : prefix;

        PoseEstimate est = getEstimate();
        dbg.addData(p + ".class", getClass().getSimpleName());

        if (est == null) {
            dbg.addData(p + ".hasPose", false);
            return;
        }

        dbg.addData(p + ".hasPose", est.hasPose)
                .addData(p + ".quality", est.quality)
                .addData(p + ".ageSec", est.ageSec)
                .addData(p + ".timestampSec", est.timestampSec);

        if (est.hasPose) {
            dbg.addData(p + ".fieldToRobotPose", est.fieldToRobotPose);
        }
    }

}
