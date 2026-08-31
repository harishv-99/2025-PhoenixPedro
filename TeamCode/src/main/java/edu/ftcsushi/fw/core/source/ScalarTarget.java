package edu.ftcsushi.fw.core.source;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Writable scalar request that can also be read as a {@link ScalarSource}.
 *
 * <p>Use {@code ScalarTarget} for persistent robot requests such as “arm goal”,
 * “flywheel velocity request”, or “manual feeder power”. Bind a named target to a source-driven
 * Plant with {@code targetFromResolver(PlantTargets.exact(target))}, or use it as the base layer of a richer
 * source graph. Retain the target itself when it is standalone, shared, owned by target-only
 * policy, or useful while assembling a composed graph. In ordinary FTC robot code, the
 * mechanism/subsystem constructor receives {@code HardwareMap} and its data-only config, snapshots
 * that config, and builds its privately owned Plant graph. An ordinary exact mechanism with no
 * separate target role uses {@code targetFromNewCommand(initialValue)} and retrieves the generated stable
 * request later through {@code plant.commandTarget()}.</p>
 *
 * <h2>Exact command inside a mechanism constructor</h2>
 * <pre>{@code
 * LiftConfig cfg = config.copy();
 * this.lift = FtcActuators.plant(hardwareMap)
 *     .motor(cfg.motorName, cfg.direction)
 *     .position()
 *     .deviceManaged()
 *     .nonPeriodic()
 *         .bounded(0.0, 4200.0)
 *         .nativeUnits()
 *         .alreadyReferenced()
 *     .positionTolerance(20.0)
 *     .targetFromNewCommand(0.0)
 *     .build();
 *
 * // Later, in a semantic intent method:
 * lift.commandTarget().set(1200.0); // changes the graph-owned request
 *
 * // Once in the mechanism owner's per-loop update phase:
 * lift.update(clock); // plant samples the request this loop
 * }</pre>
 */
public interface ScalarTarget extends ScalarSource {

    /**
     * Replace the current scalar request.
     *
     * @param value new value returned by this target until changed again
     */
    void set(double value);

    /**
     * Add {@code delta} to the current scalar request.
     */
    default void adjust(double delta) {
        set(get() + delta);
    }

    /**
     * Return the most recently requested value without needing a loop clock.
     */
    double get();

    @Override
    default double getAsDouble(LoopClock clock) {
        return get();
    }

    /**
     * Create a stored scalar target initialized to {@code initialValue}.
     *
     * <p>The target retains its most recently set value until changed and restores
     * {@code initialValue} when reset. This is the framework's one standard construction path;
     * specialized integrations may still implement {@code ScalarTarget} directly.</p>
     */
    static ScalarTarget create(double initialValue) {
        return new DefaultScalarTarget(initialValue);
    }
}

/** Package-private default implementation so construction has one public name: ScalarTarget.create. */
final class DefaultScalarTarget implements ScalarTarget {
    private final double initialValue;
    private double value;

    DefaultScalarTarget(double initialValue) {
        this.initialValue = initialValue;
        this.value = initialValue;
    }

    @Override
    public void set(double value) {
        this.value = value;
    }

    @Override
    public double get() {
        return value;
    }

    @Override
    public void reset() {
        value = initialValue;
    }

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) return;
        String p = (prefix == null || prefix.isEmpty()) ? "scalarTarget" : prefix;
        dbg.addData(p + ".class", "ScalarTarget")
                .addData(p + ".initial", initialValue)
                .addData(p + ".value", value);
    }
}
