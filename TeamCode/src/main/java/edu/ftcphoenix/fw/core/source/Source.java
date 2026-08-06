package edu.ftcphoenix.fw.core.source;

import java.util.Objects;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.function.ToDoubleFunction;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;

/**
 * A {@code Source<T>} produces values on the shared loop heartbeat.
 *
 * <p>Phoenix is built around a single loop heartbeat ({@link LoopClock}). Sources make that
 * heartbeat explicit: instead of reading raw values from random places, you build a small graph
 * of sources (gamepad intent, sensors, target generation) and sample them using the current
 * {@link LoopClock}.</p>
 *
 * <p>Sources may be stateless (pure functions of current inputs) or stateful (filters with memory).
 * Stateful sources should publish at most one <em>successful</em> observation for each
 * {@link LoopClock#cycle()} so that repeated reads do not advance internal state twice. A failed
 * value observation must not be published as a cached result; a later nonrecursive read in the
 * same cycle may retry.</p>
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
     * <p>Implementations should make {@code reset()} safe, immediate, and idempotent. Framework
     * stateful decorators reject sampling/reset overlap and recursive reset, reset owned children
     * first, and clear their own local state only after every child reset succeeds. Child resets
     * cannot be rolled back: if a later child reset fails, an earlier child may already be reset
     * while the decorator's previously published local state remains intact.</p>
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
     *   <li>{@code initial} and every prior accumulated state are read-only values.</li>
     *   <li>{@code step} must have no external effects or in-place mutations.</li>
     *   <li>{@code step} must return a non-null immutable or otherwise independently stable
     *       value. Returning the same value-semantic instance for an unchanged state is valid.</li>
     * </ul>
     * </p>
     *
     * <p>A reducer failure leaves the previously published state intact and may be retried in the
     * same cycle. Generic code cannot truthfully roll back a mutable object that the reducer changed
     * before throwing, which is why mutable accumulator state is outside this contract.</p>
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
            private final SourceOperationGuard guard =
                    new SourceOperationGuard("accumulated source");
            private long lastCycle = Long.MIN_VALUE;
            private U state = initial;

            /**
             * {@inheritDoc}
             */
            @Override
            public U get(LoopClock clock) {
                guard.beginSample();
                try {
                    Objects.requireNonNull(clock, "clock");
                    long cyc = clock.cycle();
                    if (cyc == lastCycle) {
                        return state;
                    }

                    T cur = Objects.requireNonNull(self.get(clock), "source returned null");
                    U nextState = Objects.requireNonNull(
                            step.apply(state, cur),
                            "step returned null"
                    );

                    state = nextState;
                    lastCycle = cyc;
                    return state;
                } finally {
                    guard.endSample();
                }
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                guard.beginReset();
                try {
                    self.reset();
                    state = initial;
                    lastCycle = Long.MIN_VALUE;
                } finally {
                    guard.endReset();
                }
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
     * <p>The reducer has the same value contract as
     * {@link #accumulate(BiFunction, Object)}: prior state is read-only, reduction has no external
     * effects or in-place mutation, and the returned state is non-null and independently stable.
     * The reset signal, current sample, and candidate reduced value are all observed before the
     * new state is published.</p>
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
            private final SourceOperationGuard guard =
                    new SourceOperationGuard("resettable accumulated source");
            private long lastCycle = Long.MIN_VALUE;
            private U state = initial;

            /**
             * {@inheritDoc}
             */
            @Override
            public U get(LoopClock clock) {
                guard.beginSample();
                try {
                    Objects.requireNonNull(clock, "clock");
                    long cyc = clock.cycle();
                    if (cyc == lastCycle) {
                        return state;
                    }

                    boolean resetNow = reset.getAsBoolean(clock);
                    U previousState = resetNow ? initial : state;
                    T cur = Objects.requireNonNull(self.get(clock), "source returned null");
                    U nextState = Objects.requireNonNull(
                            step.apply(previousState, cur),
                            "step returned null"
                    );

                    state = nextState;
                    lastCycle = cyc;
                    return state;
                } finally {
                    guard.endSample();
                }
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                guard.beginReset();
                try {
                    self.reset();
                    reset.reset();
                    state = initial;
                    lastCycle = Long.MIN_VALUE;
                } finally {
                    guard.endReset();
                }
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
     * <p>The retained value carries a clock-owned timestamp, so the hold duration remains correct
     * when the source is not sampled every loop and a deliberate clock reset cannot make an old
     * value current again.</p>
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
     *   <li>{@code isValid} is a side-effect-free value decision and may be retried after a
     *       failure.</li>
     * </ul>
     * </p>
     *
     * @param isValid    predicate that defines which values are considered valid
     * @param maxHoldSec finite maximum age of the held value in seconds; must be {@code >= 0}
     * @param fallback   value returned when no valid value is available (or the hold has expired)
     */
    default Source<T> holdLastValid(Predicate<? super T> isValid, double maxHoldSec, T fallback) {
        Objects.requireNonNull(isValid, "isValid");
        Objects.requireNonNull(fallback, "fallback");
        if (!Double.isFinite(maxHoldSec) || maxHoldSec < 0.0) {
            throw new IllegalArgumentException(
                    "maxHoldSec must be finite and >= 0, got " + maxHoldSec);
        }

        Source<T> self = this;
        return new Source<T>() {
            private final SourceOperationGuard guard =
                    new SourceOperationGuard("held generic source");
            private long lastCycle = Long.MIN_VALUE;
            private T lastOut = fallback;

            private T lastValid = fallback;
            private boolean hasValid = false;
            private LoopTimestamp lastValidTimestamp = LoopTimestamp.unavailable();
            /** Derived diagnostic age at the most recent sample. */
            private double lastValidAgeSec = Double.POSITIVE_INFINITY;

            /**
             * {@inheritDoc}
             */
            @Override
            public T get(LoopClock clock) {
                guard.beginSample();
                try {
                    Objects.requireNonNull(clock, "clock");
                    long cyc = clock.cycle();
                    if (cyc == lastCycle) {
                        return lastOut;
                    }

                    T cur = Objects.requireNonNull(self.get(clock), "source returned null");
                    boolean valid = isValid.test(cur);

                    boolean nextHasValid = hasValid;
                    T nextLastValid = lastValid;
                    LoopTimestamp nextLastValidTimestamp = lastValidTimestamp;
                    double nextLastValidAgeSec;
                    T nextOut;

                    if (valid) {
                        nextHasValid = true;
                        nextLastValid = cur;
                        nextLastValidTimestamp = clock.nowTimestamp();
                        nextLastValidAgeSec = 0.0;
                        nextOut = cur;
                    } else {
                        nextLastValidAgeSec = hasValid
                                ? lastValidTimestamp.ageSec(clock)
                                : Double.POSITIVE_INFINITY;
                        nextOut = Double.isFinite(nextLastValidAgeSec)
                                && nextLastValidAgeSec <= maxHoldSec
                                ? lastValid
                                : fallback;
                    }

                    hasValid = nextHasValid;
                    lastValid = nextLastValid;
                    lastValidTimestamp = nextLastValidTimestamp;
                    lastValidAgeSec = nextLastValidAgeSec;
                    lastOut = nextOut;
                    lastCycle = cyc;
                    return lastOut;
                } finally {
                    guard.endSample();
                }
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                guard.beginReset();
                try {
                    self.reset();
                    lastOut = fallback;
                    hasValid = false;
                    lastValid = fallback;
                    lastValidTimestamp = LoopTimestamp.unavailable();
                    lastValidAgeSec = Double.POSITIVE_INFINITY;
                    lastCycle = Long.MIN_VALUE;
                } finally {
                    guard.endReset();
                }
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
                dbg.addData(p + ".class", "HoldLastValid")
                        .addData(p + ".maxHoldSec", maxHoldSec)
                        .addData(p + ".hasValid", hasValid)
                        .addData(p + ".lastValidAgeSec", lastValidAgeSec);
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
     * <p>The returned source publishes one successful upstream observation per cycle and returns
     * that exact cached value for additional reads in the same cycle. A failed or null observation
     * is not cached, so a later nonrecursive read in that cycle may retry. Recursive sampling and
     * sampling/reset overlap fail fast with an actionable lifecycle error.</p>
     *
     * <p>Use this at <b>boundaries</b> (raw hardware reads, shared sensor signals, derived values used in
     * multiple places) to guarantee consistent results within a loop.</p>
     */
    default Source<T> memoized() {
        Source<T> self = this;
        return new Source<T>() {
            private final SourceOperationGuard guard =
                    new SourceOperationGuard("memoized generic source");
            private long lastCycle = Long.MIN_VALUE;
            private T last = null;

            /**
             * {@inheritDoc}
             */
            @Override
            public T get(LoopClock clock) {
                guard.beginSample();
                try {
                    Objects.requireNonNull(clock, "clock");
                    long cyc = clock.cycle();
                    if (cyc == lastCycle) {
                        return last;
                    }

                    T next = Objects.requireNonNull(
                            self.get(clock),
                            "memoized source returned null"
                    );
                    last = next;
                    lastCycle = cyc;
                    return last;
                } finally {
                    guard.endSample();
                }
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                guard.beginReset();
                try {
                    self.reset();
                    last = null;
                    lastCycle = Long.MIN_VALUE;
                } finally {
                    guard.endReset();
                }
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
     * <p>This is the ordinary generic adaptation grammar for robot code and the generic sibling of
     * {@link ScalarSource#of(java.util.function.DoubleSupplier)} and
     * {@link BooleanSource#of(java.util.function.BooleanSupplier)}. Use it for a small derived source
     * inline, especially one that depends on other sources sampled with the same clock, then add
     * stateful behavior with decorators such as {@link #memoized()}.</p>
     *
     * <p>Direct implementations remain the framework/integration extension seam for named reusable
     * source abstractions with additional domain behavior; they are not a parallel ordinary
     * robot-code recipe.</p>
     *
     * <p>The returned adapter is stateless: each call delegates to {@code fn}, and
     * {@link #reset()} is a no-op. Add an owning decorator when the graph needs state or a
     * lifecycle reset.</p>
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

/** Package-private lifecycle guard shared by the primitive and generic source decorators. */
final class SourceOperationGuard {
    private final String owner;
    private boolean sampling;
    private boolean resetting;

    SourceOperationGuard(String owner) {
        this.owner = Objects.requireNonNull(owner, "owner");
    }

    void beginSample() {
        if (sampling) {
            throw new IllegalStateException(
                    "Cannot sample " + owner + " recursively; the source graph contains "
                            + "a sampling cycle"
            );
        }
        if (resetting) {
            throw new IllegalStateException(
                    "Cannot sample " + owner + " while its reset is in progress"
            );
        }
        sampling = true;
    }

    void endSample() {
        sampling = false;
    }

    void beginReset() {
        if (sampling) {
            throw new IllegalStateException(
                    "Cannot reset " + owner + " while sampling is in progress"
            );
        }
        if (resetting) {
            throw new IllegalStateException(
                    "Cannot reset " + owner + " recursively"
            );
        }
        resetting = true;
    }

    void endReset() {
        resetting = false;
    }
}
