package edu.ftcphoenix.fw.actuation;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Source that resolves the requested target for a {@link Plant}.
 *
 * <p>This is the plant-aware sibling of a scalar source. A simple implementation may ignore the
 * {@link PlantTargetContext} and return an exact number. A smarter implementation can use the
 * context's measurement, legal range, and periodic topology to choose among equivalent targets.</p>
 *
 * <p>Student-facing rule: for anything intended to become a Plant target, build a
 * {@code PlantTargetSource} through {@link PlantTargets}. The final source bound to a Plant should
 * be total: it should provide a target every loop through an exact value, overlay base, or explicit
 * unavailable policy.</p>
 */
public interface PlantTargetSource {

    /**
     * Resolve this source for the current plant update.
     *
     * <p>Conditional composition may skip this method while another target producer has higher
     * priority. In particular, {@link PlantTargets#overlay(PlantTargetSource)} samples every layer's
     * activation gate but resolves only the selected target path and any explicit
     * {@code addIfAvailable(...)} fall-through attempts. Implementations must not rely on being
     * resolved while shadowed; independently recurring work needs its own explicit loop owner.</p>
     *
     * @param context plant facts for this loop
     * @param clock   non-null current loop clock
     * @return target-selection plan; final sources should normally return {@code hasTarget() == true}
     */
    PlantTargetPlan resolve(PlantTargetContext context, LoopClock clock);

    /**
     * Reset source-local state such as held fallback targets or child sources.
     */
    default void reset() {
    }

    /**
     * Emit target-source debug state.
     */
    default void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) return;
        String p = (prefix == null || prefix.isEmpty()) ? "plantTargetSource" : prefix;
        dbg.addData(p + ".class", getClass().getSimpleName());
    }
}
