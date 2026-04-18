package edu.ftcphoenix.fw.spatial;

import edu.ftcphoenix.fw.core.debug.DebugSink;

/**
 * One backend/strategy that can solve some or all of a spatial query.
 */
public interface SpatialSolveLane {

    /**
     * Solves this loop's request. Return {@link SpatialLaneResult#none()} when the lane cannot
     * solve either channel for the current request.
     */
    SpatialLaneResult solve(SpatialSolveRequest request);

    /**
     * Optional lifecycle hook to clear lane-local state.
     */
    default void reset() {
        // no-op
    }

    /**
     * Optional debug hook.
     */
    default void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "spatialSolveLane" : prefix;
        dbg.addData(p + ".class", getClass().getSimpleName());
    }
}
