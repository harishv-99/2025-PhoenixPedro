package edu.ftcphoenix.fw.actuation;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Plant-aware logic that resolves one requested target for each {@link Plant} update.
 *
 * <p>This is not a Phoenix {@code Source<T>}: the consuming Plant must supply the
 * {@link PlantTargetContext} containing its measurement, legal range, periodicity, and prior
 * targets. A simple resolver may ignore that context and return an exact number. A smarter resolver
 * can select among equivalent or requested alternatives. Use
 * {@link PlantTargets#equivalentPositionsOf(edu.ftcphoenix.fw.core.source.ScalarTarget)} for the
 * common case where one logical command may use any legal periodic representative; reserve
 * {@link PlantTargets#plan(PlantTargetRequest)} for fixed advanced requests and
 * {@link PlantTargets#plan(edu.ftcphoenix.fw.core.source.Source)} for live request sources.</p>
 *
 * <p>In ordinary FTC robot code, a mechanism/subsystem constructor receives {@code HardwareMap}
 * and its data-only config, snapshots that config, and constructs its final resolver and Plant
 * graph internally. When an ordinary exact Plant needs a writable command with no independent
 * owner, that Plant construction uses {@code targetFromNewCommand(initialValue)}. Name the target
 * and pass {@code PlantTargets.exact(target)} to {@code targetFromResolver(...)} when it is shared,
 * a target-only policy owns it, or it usefully identifies the stable base of an overlaid or
 * periodic-equivalent final graph. Read-only and planned graphs need not carry a command.</p>
 *
 * <p>Outside that ordinary ownership boundary, a completed Plant may cross a constructor only at a
 * clearly labeled hardware-neutral test, custom-adapter, portable-host, or advanced-assembly seam.
 * Pass the Plant alone at that seam. If that owner writes the persistent command, retrieve it
 * through {@link Plant#commandTarget()} rather than injecting the same target separately; a
 * read-only/planned owner derives no nonexistent command. Any final resolver bound to a Plant
 * should be total: it should provide a target every loop through an exact value, overlay base, or
 * explicit unavailable policy.</p>
 */
public interface PlantTargetResolver {

    /**
     * Resolve a target for the current Plant update.
     *
     * <p>Conditional composition may skip this method while another target producer has higher
     * priority. In particular, {@link PlantTargets#overlay(PlantTargetResolver)} samples every
     * layer's activation gate but resolves only the selected target path and any explicit
     * {@code addIfAvailable(...)} fall-through attempts. Implementations must not rely on being
     * resolved while shadowed; independently recurring work needs its own explicit loop owner.</p>
     *
     * @param context Plant facts for this loop
     * @param clock non-null current loop clock
     * @return target-selection resolution; final resolvers should normally return
     *         {@code hasTarget() == true}
     */
    PlantTargetResolution resolve(PlantTargetContext context, LoopClock clock);

    /**
     * Reset resolver-local state such as held fallback targets or child resolvers.
     */
    default void reset() {
    }

    /**
     * Emit target-resolver debug state.
     */
    default void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) return;
        String p = (prefix == null || prefix.isEmpty()) ? "plantTargetResolver" : prefix;
        dbg.addData(p + ".class", getClass().getSimpleName());
    }
}
