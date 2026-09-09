package edu.ftcsushi.fw.localization;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Common interface for components that output an absolute robot pose in field coordinates.
 *
 * <p>Sushi distinguishes three nested localization-side concepts:</p>
 * <ul>
 *   <li>{@link AbsolutePoseEstimator}: something that answers <em>"where is the robot on the field?"</em></li>
 *   <li>{@link PoseTrajectoryEstimator}: a high-rate absolute estimator that also identifies
 *       interpolation-safe trajectory segments</li>
 *   <li>{@link MotionPredictor}: a trajectory estimator that answers both
 *       <em>"where is the robot now?"</em> and
 *       <em>"how did it move since the last accepted motion baseline?"</em></li>
 * </ul>
 *
 * <p>Most consumers such as drive guidance, go-to-pose tasks, targeting, and telemetry only need an
 * absolute pose, so they should depend on this interface. Historical trajectory consumers depend
 * on {@link PoseTrajectoryEstimator}; sparse delayed measurements do not claim that capability.
 * Fusion-style localizers that replay or blend incremental motion should depend on
 * {@link MotionPredictor} for the predictor side and on {@code AbsolutePoseEstimator} for the
 * correction side.</p>
 *
 * <p><b>Absolute is a coordinate description, not an independence guarantee.</b> Two field-pose
 * estimates can share the same heading input, camera observations, calibration, or prior
 * corrections, and therefore share an error. This interface and {@link PoseEstimate} do not
 * describe those dependencies. An assembler choosing a correction source must check that its
 * evidence fits the consuming estimator's model; freshness or agreement alone cannot establish
 * that it provides independent confirmation.</p>
 *
 * <p>Common examples:</p>
 * <ul>
 *   <li>{@code AprilTagPoseEstimator}: absolute pose from raw AprilTag observations</li>
 *   <li>{@code LimelightFieldPoseEstimator}: absolute pose from standard Limelight botpose</li>
 *   <li>{@code PinpointOdometryPredictor}: implements {@link MotionPredictor} because it provides
 *       both an absolute odometry pose and incremental motion deltas</li>
 * </ul>
 */
public interface AbsolutePoseEstimator extends HeadingEstimator {

    /**
     * Advance the estimator's internal state using the current loop time.
     *
     * <p>Implementations are responsible for reading the sources they depend on, updating any
     * internal filters or caches, and preparing the next {@link PoseEstimate} returned by
     * {@link #getEstimate()}.</p>
     *
     * <p>This is an <b>attempt-idempotent</b> per-cycle operation. The first call claims the
     * supplied {@link LoopClock#cycle()} before invoking owned sources, filters, hardware, or
     * vendor side effects. After a successful attempt, repeated calls in that cycle leave the
     * exact published snapshot and all dependencies unchanged. Reentrant updates are lifecycle
     * errors. If the first attempt throws a {@link RuntimeException}, a same-cycle repeat must
     * rethrow that retained failure rather than silently appearing successful or retrying effects
     * that may already have occurred; the next cycle may make a new attempt.</p>
     *
     * <p>Pass the one non-null shared OpMode clock. Complete pipeline, processor, and other
     * dependency-lifecycle transitions before this estimator's first update in a cycle; changes
     * made afterward are reflected by its next-cycle snapshot.</p>
     *
     * @param clock shared loop clock for the current OpMode cycle
     * @throws NullPointerException if {@code clock} is {@code null}
     * @throws IllegalStateException for a reentrant update
     */
    void update(LoopClock clock);

    /**
     * Returns the most recent absolute field pose estimate.
     *
     * <p>This method must be safe to call multiple times between {@link #update(LoopClock)} calls.
     * Availability is not sufficient control evidence: callers also apply their action's
     * timestamp-age and quality requirements. A retained available pose can be old even when
     * this cycle's update succeeds.</p>
     *
     * <p>Common usage after choosing {@code maxAgeSec} and {@code minQuality} for this action;
     * neither value is a universal robot-safety threshold:</p>
     * <pre>
     * // The lifecycle owner updates before downstream consumers.
     * estimator.update(clock);
     * PoseEstimate est = estimator.getEstimate();
     * if (est.hasPose
     *         &amp;&amp; est.timestamp.isFresh(clock, maxAgeSec)
     *         &amp;&amp; Double.isFinite(est.quality)
     *         &amp;&amp; est.quality &gt;= minQuality) {
     *     // Use est.toPose2d() subject to the action's remaining control checks.
     * }
     * </pre>
     *
     * @return latest pose estimate snapshot
     */
    PoseEstimate getEstimate();

    /**
     * Project the latest cached pose into equivalent heading evidence without another update.
     */
    @Override
    default HeadingEstimate getHeadingEstimate() {
        PoseEstimate estimate = getEstimate();
        if (estimate == null || !estimate.hasPose
                || !Double.isFinite(estimate.fieldToRobotPose.yawRad)) {
            return HeadingEstimate.noHeading(
                    estimate == null ? edu.ftcsushi.fw.core.time.LoopTimestamp.unavailable()
                            : estimate.timestamp
            );
        }
        return new HeadingEstimate(
                estimate.fieldToRobotPose.yawRad,
                true,
                estimate.quality,
                estimate.timestamp
        );
    }

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
                .addData(p + ".timestampAvailable", est.timestamp.isAvailable());

        if (est.hasPose) {
            dbg.addData(p + ".fieldToRobotPose", est.fieldToRobotPose);
        }
    }
}
