package edu.ftcphoenix.fw.localization;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Common interface for components that output an absolute robot pose in field coordinates.
 *
 * <p>Phoenix distinguishes between two localization-side concepts:</p>
 * <ul>
 *   <li>{@link AbsolutePoseEstimator}: something that answers <em>"where is the robot on the field?"</em></li>
 *   <li>{@link MotionPredictor}: something that answers both <em>"where is the robot now?"</em> and
 *       <em>"how did it move since the last loop?"</em></li>
 * </ul>
 *
 * <p>Most consumers such as drive guidance, go-to-pose tasks, targeting, and telemetry only need an
 * absolute pose, so they should depend on this interface. Fusion-style localizers that replay or
 * blend incremental motion should depend on {@link MotionPredictor} for the predictor side and on
 * {@code AbsolutePoseEstimator} for the correction side.</p>
 *
 * <p>Common examples:</p>
 * <ul>
 *   <li>{@code AprilTagPoseEstimator}: absolute pose from raw AprilTag observations</li>
 *   <li>{@code LimelightFieldPoseEstimator}: absolute pose from Limelight botpose / MegaTag</li>
 *   <li>{@code PinpointOdometryPredictor}: implements {@link MotionPredictor} because it provides
 *       both an absolute odometry pose and incremental motion deltas</li>
 * </ul>
 */
public interface AbsolutePoseEstimator {

    /**
     * Advance the estimator's internal state using the current loop time.
     *
     * <p>Implementations are responsible for reading the sources they depend on, updating any
     * internal filters or caches, and preparing the next {@link PoseEstimate} returned by
     * {@link #getEstimate()}.</p>
     *
     * @param clock shared loop clock for the current OpMode cycle
     */
    void update(LoopClock clock);

    /**
     * Returns the most recent absolute field pose estimate.
     *
     * <p>This method must be safe to call multiple times between {@link #update(LoopClock)} calls.
     * Callers should always check {@link PoseEstimate#hasPose} before using the returned pose for
     * control.</p>
     *
     * <p>Common usage:</p>
     * <pre>
     * estimator.update(clock);
     * PoseEstimate est = estimator.getEstimate();
     * if (est.hasPose) {
     *     // use est.toPose2d() for guidance / targeting / telemetry
     * }
     * </pre>
     *
     * @return latest pose estimate snapshot
     */
    PoseEstimate getEstimate();

    /**
     * Emits a compact telemetry/debug summary of the current estimate.
     *
     * <p>Implementations with meaningful internal state should override this method to expose richer
     * diagnostics such as source freshness, gating decisions, residuals, or uncertainty terms.</p>
     *
     * @param dbg    debug sink (may be {@code null})
     * @param prefix base key prefix, for example {@code "localizer"}
     */
    default void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "absolutePoseEstimator" : prefix;

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
