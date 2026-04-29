package edu.ftcphoenix.fw.core.source;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Writable scalar request that can also be read as a {@link ScalarSource}.
 *
 * <p>Use {@code ScalarTarget} for persistent robot requests such as “arm goal”,
 * “flywheel velocity request”, or “manual feeder power”. A source-driven plant may be built
 * directly from a {@code ScalarTarget}, or a target can be used as the base layer of a richer
 * source graph.</p>
 *
 * <h2>Typical usage</h2>
 * <pre>{@code
 * ScalarTarget liftGoal = ScalarTarget.held(0.0);
 *
 * PositionPlant lift = FtcActuators.plant(hardwareMap)
 *     .motor("lift", Direction.FORWARD)
 *     .position()
 *     ...
 *     .targetedBy(liftGoal)
 *     .build();
 *
 * liftGoal.set(1200.0);   // behavior changes the request
 * lift.update(clock);     // plant samples the request this loop
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
     * Create a simple held scalar target initialized to {@code initialValue}.
     */
    static ScalarTarget held(double initialValue) {
        return new HeldScalarTarget(initialValue);
    }

    /**
     * Create a held scalar target. Alias for {@link #held(double)}.
     */
    static ScalarTarget of(double initialValue) {
        return held(initialValue);
    }

    /**
     * Simple in-memory implementation.
     */
    final class HeldScalarTarget implements ScalarTarget {
        private final double initialValue;
        private double value;

        private HeldScalarTarget(double initialValue) {
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
            dbg.addData(p + ".class", "HeldScalarTarget")
                    .addData(p + ".initial", initialValue)
                    .addData(p + ".value", value);
        }
    }
}
