package edu.ftcphoenix.fw.core.source;

import java.util.Objects;
import java.util.function.Function;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A {@code Source<T>} produces a value once per loop.
 *
 * <p>Phoenix is built around a single loop heartbeat ({@link LoopClock}). Sources make that
 * heartbeat explicit: instead of reading raw values from random places, you build a small graph
 * of sources (gamepad intent, sensors, target generation) and sample them using the current
 * {@link LoopClock}.</p>
 *
 * <p>Sources may be stateless (pure functions of current inputs) or stateful (filters with memory).
 * Stateful sources should be <b>idempotent by</b> {@link LoopClock#cycle()} so that calling
 * {@link #get(LoopClock)} multiple times in the same cycle does not advance internal state twice.</p>
 *
 * <p>This abstraction is the generalized form of the old {@code Axis} concept: "something that
 * returns a value each loop". For common primitives, see {@link ScalarSource} and
 * {@link BooleanSource}.</p>
 */
public interface Source<T> {

    /**
     * Produce the value for the current loop.
     *
     * @param clock current loop clock (required)
     * @return value (never null)
     */
    T get(LoopClock clock);

    /**
     * Optional: reset any internal state.
     *
     * <p>Most stateless sources can ignore this. Stateful sources should implement it to return
     * to a known state.</p>
     */
    default void reset() {
        // no-op
    }

    /**
     * Optional debug hook.
     *
     * <p>Follow Phoenix conventions: if {@code dbg} is null, do nothing. Use stable keys.
     * Prefer delegating to children with a dotted prefix.</p>
     */
    default void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "source" : prefix;
        dbg.addData(p + ".class", getClass().getSimpleName());
    }

    /**
     * Map this source's value through a pure function.
     */
    default <U> Source<U> map(Function<? super T, ? extends U> fn) {
        Objects.requireNonNull(fn, "fn");
        Source<T> self = this;
        return new Source<U>() {
            @Override
            public U get(LoopClock clock) {
                return Objects.requireNonNull(fn.apply(self.get(clock)), "mapped source returned null");
            }

            @Override
            public void reset() {
                self.reset();
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) {
                    return;
                }
                String p = (prefix == null || prefix.isEmpty()) ? "map" : prefix;
                dbg.addData(p + ".class", "MappedSource");
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    /**
     * Memoize this source for the current {@link LoopClock#cycle()}.
     *
     * <p>The returned source samples the upstream source at most once per cycle and returns the cached
     * value for any additional reads in that same cycle. This is the simplest way to enforce Phoenix's
     * "one loop, one heartbeat" rule for values that may be expensive to compute or may change if read
     * multiple times.</p>
     *
     * <p>Use this at <b>boundaries</b> (raw hardware reads, shared sensor signals, derived values used in
     * multiple places) to guarantee consistent results within a loop.</p>
     */
    default Source<T> memoized() {
        Source<T> self = this;
        return new Source<T>() {
            private long lastCycle = Long.MIN_VALUE;
            private T last = null;

            @Override
            public T get(LoopClock clock) {
                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return last;
                }
                lastCycle = cyc;
                last = Objects.requireNonNull(self.get(clock), "memoized source returned null");
                return last;
            }

            @Override
            public void reset() {
                self.reset();
                lastCycle = Long.MIN_VALUE;
                last = null;
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) {
                    return;
                }
                String p = (prefix == null || prefix.isEmpty()) ? "memo" : prefix;
                dbg.addData(p + ".class", "MemoizedSource");
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    /**
     * Create a constant source.
     */
    static <T> Source<T> constant(final T value) {
        Objects.requireNonNull(value, "value");
        return new Source<T>() {
            @Override
            public T get(LoopClock clock) {
                return value;
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "const" : prefix;
                dbg.addData(p + ".class", "ConstantSource")
                        .addData(p + ".value", value);
            }
        };
    }
}
