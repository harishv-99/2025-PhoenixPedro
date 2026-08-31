package edu.ftcsushi.fw.localization;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Managed producer of cached robot-heading evidence in a fixed field frame.
 *
 * <p>The lifecycle contract matches {@link AbsolutePoseEstimator}: the composition root updates
 * the estimator once in its service phase, while downstream consumers only read the cached
 * snapshot. Implementations must make updates attempt-idempotent within one clock cycle.</p>
 */
public interface HeadingEstimator {

    /** Advance this estimator once for the current shared-clock cycle. */
    void update(LoopClock clock);

    /** Return the latest cached heading snapshot without polling hardware. */
    HeadingEstimate getHeadingEstimate();

    /** Emit compact cached-heading diagnostics. */
    default void debugDumpHeading(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = prefix == null || prefix.isEmpty() ? "headingEstimator" : prefix;
        HeadingEstimate estimate = getHeadingEstimate();
        dbg.addData(p + ".class", getClass().getSimpleName());
        if (estimate == null) {
            dbg.addData(p + ".hasHeading", false);
            return;
        }
        dbg.addData(p + ".hasHeading", estimate.hasHeading)
                .addData(p + ".quality", estimate.quality)
                .addData(p + ".timestampAvailable", estimate.timestamp.isAvailable());
        if (estimate.hasHeading) {
            dbg.addData(p + ".fieldHeadingRad", estimate.fieldHeadingRad);
        }
    }
}
