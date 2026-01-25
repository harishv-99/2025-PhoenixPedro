package edu.ftcphoenix.fw.sensing.observation;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Source of {@link TargetObservation2d} samples.
 *
 * <p>This mirrors the Phoenix “polled” architecture: each OpMode loop, you call
 * {@link #sample(LoopClock)} to update any internal state and return the current observation.
 * If no target is detected, return {@link TargetObservation2d#none()}.</p>
 */
public interface ObservationSource2d {

    /**
     * Sample the current observation.
     *
     * @param clock loop timing helper (non-null)
     */
    TargetObservation2d sample(LoopClock clock);

    /**
     * Optional debug hook.
     */
    default void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "obs2d" : prefix;
        dbg.addData(p + ".class", getClass().getSimpleName());
    }
}
