package edu.ftcphoenix.fw.core.source;

import java.util.Objects;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.function.ToDoubleFunction;

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
     * Optional lifecycle hook to clear internal memory.
     *
     * <p>Most stateless sources can ignore this. Stateful sources should implement it to return
     * to a known state, and should usually propagate the reset to any owned child sources.</p>
     *
     * <p>This hook is for lifecycle / ownership boundaries: OpMode restarts, tester re-init,
     * macro restarts, or any other case where code that owns the source graph wants a clean
     * slate immediately.</p>
     *
     * <p>Normal runtime behavior should usually prefer explicit signals inside the source graph.
     * If the reset boundary is itself a loop-time condition, model it with a
     * {@link BooleanSource} and helpers such as
     * {@link #accumulateUntil(BooleanSource, BiFunction, Object)} instead of inventing an
     * out-of-band imperative call.</p>
     *
     * <p>Implementations should make {@code reset()} safe, immediate, and idempotent.</p>
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
            /**
             * {@inheritDoc}
             */
            @Override
            public U get(LoopClock clock) {
                return Objects.requireNonNull(fn.apply(self.get(clock)), "mapped source returned null");
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                self.reset();
            }

            /**
             * {@inheritDoc}
             */
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
     * Accumulate source samples into a state value, advancing at most once per loop cycle.
     *
     * <p>This is Phoenix's generic "remember across samples" primitive for value-object sources.
     * It is useful when the code that owns the source decides when that memory window starts and
     * ends, and can explicitly call {@link #reset()} on the returned source.</p>
     *
     * <p>Common uses include window-local classification, agreement / conflict reduction across
     * noisy samples, and small supervisor state machines where the accumulated state is still best
     * modeled as a source. When the window boundary is itself a signal in the source graph,
     * prefer {@link #accumulateUntil(BooleanSource, BiFunction, Object)}.</p>
     *
     * <p>Contract:
     * <ul>
     *   <li>The upstream source must never return {@code null}.</li>
     *   <li>{@code initial} must be non-null.</li>
     *   <li>{@code step} must return a non-null state.</li>
     * </ul>
     * </p>
     *
     * <p>The accumulator state should usually behave like a small immutable value object or enum.
     * Mutable state objects can work, but they make it easier to accidentally mutate the returned
     * state from outside the source graph.</p>
     *
     * @param step    reducer called as {@code step(previousState, currentSample)}
     * @param initial initial state, restored on {@link Source#reset()}
     * @param <U>     accumulated state type
     */
    default <U> Source<U> accumulate(BiFunction<? super U, ? super T, ? extends U> step, U initial) {
        Objects.requireNonNull(step, "step");
        Objects.requireNonNull(initial, "initial");

        Source<T> self = this;
        return new Source<U>() {
            private long lastCycle = Long.MIN_VALUE;
            private U state = initial;

            /**
             * {@inheritDoc}
             */
            @Override
            public U get(LoopClock clock) {
                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return state;
                }
                lastCycle = cyc;

                T cur = Objects.requireNonNull(self.get(clock), "source returned null");
                state = Objects.requireNonNull(step.apply(state, cur), "step returned null");
                return state;
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                self.reset();
                lastCycle = Long.MIN_VALUE;
                state = initial;
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) {
                    return;
                }
                String p = (prefix == null || prefix.isEmpty()) ? "accumulate" : prefix;
                dbg.addData(p + ".class", "AccumulatedSource")
                        .addData(p + ".state", state);
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    /**
     * Accumulate source samples into a state value and reset that state whenever {@code reset}
     * is true for the current loop.
     *
     * <p>This is the reset-by-event sibling of {@link #accumulate(BiFunction, Object)}. It is a
     * good fit for "remember within a window" logic such as slot-local classification, one-piece
     * observation windows, and other cases where the memory should be cleared by an explicit
     * boundary signal instead of a timer.</p>
     *
     * <p>This complements rather than replaces {@link #reset()}. Use this helper when the reset
     * boundary is part of the loop graph. Keep {@code reset()} for lifecycle / owner-driven clears
     * that need to happen immediately, even outside a normal sampling pass.</p>
     *
     * <p>Reset semantics: when {@code reset} is true, the internal state is first set back to
     * {@code initial}, then the current sample is folded into that fresh state during the same
     * loop. This lets the first sample of a new window participate immediately.</p>
     *
     * <p>When both {@code reset} and the upstream source are derived from the same sensor graph,
     * prefer memoizing the shared boundary reads so both paths observe the same per-loop sample.</p>
     *
     * @param reset   one-loop or level-style reset signal; when true the accumulator is cleared
     * @param step    reducer called as {@code step(previousState, currentSample)}
     * @param initial initial state used after reset and on construction
     * @param <U>     accumulated state type
     */
    default <U> Source<U> accumulateUntil(BooleanSource reset,
                                          BiFunction<? super U, ? super T, ? extends U> step,
                                          U initial) {
        Objects.requireNonNull(reset, "reset");
        Objects.requireNonNull(step, "step");
        Objects.requireNonNull(initial, "initial");

        Source<T> self = this;
        return new Source<U>() {
            private long lastCycle = Long.MIN_VALUE;
            private U state = initial;

            /**
             * {@inheritDoc}
             */
            @Override
            public U get(LoopClock clock) {
                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return state;
                }
                lastCycle = cyc;

                if (reset.getAsBoolean(clock)) {
                    state = initial;
                }

                T cur = Objects.requireNonNull(self.get(clock), "source returned null");
                state = Objects.requireNonNull(step.apply(state, cur), "step returned null");
                return state;
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                self.reset();
                reset.reset();
                lastCycle = Long.MIN_VALUE;
                state = initial;
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) {
                    return;
                }
                String p = (prefix == null || prefix.isEmpty()) ? "accumulateUntil" : prefix;
                dbg.addData(p + ".class", "AccumulatedUntilSource")
                        .addData(p + ".state", state);
                reset.debugDump(dbg, p + ".reset");
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    /**
     * Hold the last valid value for up to {@code maxHoldSec} seconds.
     *
     * <p>This is a generic "anti-flicker" helper for value-object sources. A common pattern is a
     * noisy classifier that sometimes outputs an "unknown" value; holding the last non-unknown
     * value for a short time makes downstream logic much more stable.</p>
     *
     * <p>Unlike some time-based filters, this implementation uses {@link LoopClock#nowSec()} so it
     * still behaves correctly even if the source is not sampled every loop.</p>
     *
     * <p>When the input becomes invalid:
     * <ul>
     *   <li>If the last valid value is newer than {@code maxHoldSec}, the last valid value is returned.</li>
     *   <li>Otherwise, {@code fallback} is returned.</li>
     * </ul>
     * </p>
     *
     * <p>Contract:
     * <ul>
     *   <li>The upstream source must never return {@code null}.</li>
     *   <li>{@code fallback} must be non-null.</li>
     * </ul>
     * </p>
     *
     * @param isValid    predicate that defines which values are considered valid
     * @param maxHoldSec maximum age of the held value in seconds; must be {@code >= 0}
     * @param fallback   value returned when no valid value is available (or the hold has expired)
     */
    default Source<T> holdLastValid(Predicate<? super T> isValid, double maxHoldSec, T fallback) {
        Objects.requireNonNull(isValid, "isValid");
        Objects.requireNonNull(fallback, "fallback");
        if (maxHoldSec < 0.0) {
            throw new IllegalArgumentException("maxHoldSec must be >= 0, got " + maxHoldSec);
        }

        Source<T> self = this;
        return new Source<T>() {
            private long lastCycle = Long.MIN_VALUE;
            private T lastOut = fallback;

            private T lastValid = fallback;
            private boolean hasValid = false;
            private double lastValidSec = Double.NEGATIVE_INFINITY;
            /**
             * The last time {@link #get(LoopClock)} was sampled (for debug only).
             */
            private double lastSampleSec = Double.NEGATIVE_INFINITY;

            /**
             * {@inheritDoc}
             */
            @Override
            public T get(LoopClock clock) {
                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return lastOut;
                }
                lastCycle = cyc;

                T cur = Objects.requireNonNull(self.get(clock), "source returned null");
                double now = clock.nowSec();
                lastSampleSec = now;

                if (isValid.test(cur)) {
                    hasValid = true;
                    lastValid = cur;
                    lastValidSec = now;
                    lastOut = cur;
                    return lastOut;
                }

                if (hasValid && (now - lastValidSec) <= maxHoldSec) {
                    lastOut = lastValid;
                    return lastOut;
                }

                lastOut = fallback;
                return lastOut;
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                self.reset();
                lastCycle = Long.MIN_VALUE;
                lastOut = fallback;
                hasValid = false;
                lastValid = fallback;
                lastValidSec = Double.NEGATIVE_INFINITY;
                lastSampleSec = Double.NEGATIVE_INFINITY;
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) {
                    return;
                }
                String p = (prefix == null || prefix.isEmpty()) ? "holdLastValid" : prefix;
                double ageSec = hasValid ? Math.max(0.0, lastSampleSec - lastValidSec) : Double.POSITIVE_INFINITY;
                dbg.addData(p + ".class", "HoldLastValid")
                        .addData(p + ".maxHoldSec", maxHoldSec)
                        .addData(p + ".hasValid", hasValid)
                        .addData(p + ".lastValidAgeSec", ageSec);
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    /**
     * Map this source into a {@link BooleanSource} using a predicate.
     *
     * <p>This is a convenience for the common pattern "I have a value object source and I want
     * a boolean gate derived from it" (example: a color reading -> isGreen?).</p>
     */
    default BooleanSource mapToBoolean(Predicate<? super T> predicate) {
        Objects.requireNonNull(predicate, "predicate");
        Source<T> self = this;

        return new BooleanSource() {
            /**
             * {@inheritDoc}
             */
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                return predicate.test(self.get(clock));
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                self.reset();
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) {
                    return;
                }
                String p = (prefix == null || prefix.isEmpty()) ? "mapBool" : prefix;
                dbg.addData(p + ".class", "MappedBoolean");
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    /**
     * Map this source into a {@link ScalarSource} using a {@link ToDoubleFunction}.
     */
    default ScalarSource mapToDouble(ToDoubleFunction<? super T> fn) {
        Objects.requireNonNull(fn, "fn");
        Source<T> self = this;

        return new ScalarSource() {
            /**
             * {@inheritDoc}
             */
            @Override
            public double getAsDouble(LoopClock clock) {
                return fn.applyAsDouble(self.get(clock));
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                self.reset();
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) {
                    return;
                }
                String p = (prefix == null || prefix.isEmpty()) ? "mapScalar" : prefix;
                dbg.addData(p + ".class", "MappedScalar");
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

            /**
             * {@inheritDoc}
             */
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

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                self.reset();
                lastCycle = Long.MIN_VALUE;
                last = null;
            }

            /**
             * {@inheritDoc}
             */
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
     * Create a source from a function of the current {@link LoopClock}.
     *
     * <p>This is the generic sibling of {@link ScalarSource#of(java.util.function.DoubleSupplier)}
     * and {@link BooleanSource#of(java.util.function.BooleanSupplier)}. It is useful when you want
     * to create a small derived source inline, especially one that depends on other sources sampled
     * with the same clock.</p>
     */
    static <T> Source<T> of(Function<? super LoopClock, ? extends T> fn) {
        Objects.requireNonNull(fn, "fn");
        return new Source<T>() {
            /**
             * {@inheritDoc}
             */
            @Override
            public T get(LoopClock clock) {
                return Objects.requireNonNull(fn.apply(clock), "raw source returned null");
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "raw" : prefix;
                dbg.addData(p + ".class", "RawSource");
            }
        };
    }

    /**
     * Create a constant source.
     */
    static <T> Source<T> constant(final T value) {
        Objects.requireNonNull(value, "value");
        return new Source<T>() {
            /**
             * {@inheritDoc}
             */
            @Override
            public T get(LoopClock clock) {
                return value;
            }

            /**
             * {@inheritDoc}
             */
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
