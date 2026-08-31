package edu.ftcsushi.fw.core.source;

import java.util.Objects;
import java.util.function.BooleanSupplier;

import edu.ftcsushi.fw.core.control.DebounceBoolean;
import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * A {@link Source} that produces a {@code boolean} each loop.
 *
 * <p>This is the boolean sibling of {@link ScalarSource}. Use it for sensor gates,
 * mode enables, "ready" signals, and any other discrete state that you want to sample
 * in a clock-consistent way.</p>
 */
public interface BooleanSource extends Source<Boolean> {

    /**
     * Sample the current value.
     */
    boolean getAsBoolean(LoopClock clock);

    @Override
    default Boolean get(LoopClock clock) {
        return getAsBoolean(clock);
    }

    // ---------------------------------------------------------------------------------------------
    // Memoization
    // ---------------------------------------------------------------------------------------------

    /**
     * Memoize this boolean for the current {@link LoopClock#cycle()}.
     *
     * <p>The returned source publishes one successful upstream observation per cycle and returns
     * that exact value for additional reads in the same cycle. A failed observation is not cached,
     * so a later nonrecursive read in that cycle may retry.</p>
     */
    default BooleanSource memoized() {
        BooleanSource self = this;
        return new BooleanSource() {
            private final SourceOperationGuard guard =
                    new SourceOperationGuard("memoized Boolean source");
            private long lastCycle = Long.MIN_VALUE;
            private boolean last = false;

            /**
             * {@inheritDoc}
             */
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                guard.beginSample();
                try {
                    Objects.requireNonNull(clock, "clock");
                    long cyc = clock.cycle();
                    if (cyc == lastCycle) {
                        return last;
                    }

                    boolean next = self.getAsBoolean(clock);
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
                    last = false;
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
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "memo" : prefix;
                dbg.addData(p + ".class", "MemoizedBoolean");
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    // ---------------------------------------------------------------------------------------------
    // Common transforms
    // ---------------------------------------------------------------------------------------------

    /**
     * Logical NOT.
     */
    default BooleanSource not() {
        BooleanSource self = this;
        return new BooleanSource() {
            /**
             * {@inheritDoc}
             */
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                return !self.getAsBoolean(clock);
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
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "not" : prefix;
                dbg.addData(p + ".class", "NotBoolean");
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    /**
     * Logical AND.
     *
     * <p>Each observation samples this source first. If that succeeds, it samples {@code other}
     * exactly once regardless of the left Boolean value, then applies the AND truth table. Unlike
     * Java's {@code &&} operator, a {@code false} left value does not suppress the right
     * observation. This keeps stateful operands such as debouncers and edge detectors current;
     * each stateful operand remains responsible for its own same-cycle idempotency.</p>
     *
     * <p>If this source throws, the exception propagates and {@code other} is not sampled. If
     * {@code other} throws, that exception propagates even when this source returned
     * {@code false}.</p>
     *
     * @param other right-hand source to observe
     * @return source containing the logical AND of both observed values
     */
    default BooleanSource and(BooleanSource other) {
        Objects.requireNonNull(other, "other");
        BooleanSource self = this;
        return new BooleanSource() {
            /**
             * {@inheritDoc}
             */
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                boolean leftValue = self.getAsBoolean(clock);
                boolean rightValue = other.getAsBoolean(clock);
                return leftValue && rightValue;
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                self.reset();
                other.reset();
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "and" : prefix;
                dbg.addData(p + ".class", "AndBoolean");
                self.debugDump(dbg, p + ".a");
                other.debugDump(dbg, p + ".b");
            }
        };
    }

    /**
     * Logical OR.
     *
     * <p>Each observation samples this source first. If that succeeds, it samples {@code other}
     * exactly once regardless of the left Boolean value, then applies the OR truth table. Unlike
     * Java's {@code ||} operator, a {@code true} left value does not suppress the right observation.
     * This keeps stateful operands such as debouncers and edge detectors current; each stateful
     * operand remains responsible for its own same-cycle idempotency.</p>
     *
     * <p>If this source throws, the exception propagates and {@code other} is not sampled. If
     * {@code other} throws, that exception propagates even when this source returned
     * {@code true}.</p>
     *
     * @param other right-hand source to observe
     * @return source containing the logical OR of both observed values
     */
    default BooleanSource or(BooleanSource other) {
        Objects.requireNonNull(other, "other");
        BooleanSource self = this;
        return new BooleanSource() {
            /**
             * {@inheritDoc}
             */
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                boolean leftValue = self.getAsBoolean(clock);
                boolean rightValue = other.getAsBoolean(clock);
                return leftValue || rightValue;
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                self.reset();
                other.reset();
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "or" : prefix;
                dbg.addData(p + ".class", "OrBoolean");
                self.debugDump(dbg, p + ".a");
                other.debugDump(dbg, p + ".b");
            }
        };
    }

    // ---------------------------------------------------------------------------------------------
    // State-aware filters
    // ---------------------------------------------------------------------------------------------

    /**
     * Debounce this boolean: it turns ON only after being continuously true for {@code onDelaySec}
     * seconds, and turns OFF immediately when it becomes false.
     */
    default BooleanSource debouncedOn(double onDelaySec) {
        return debounced(DebounceBoolean.onAfterOffImmediately(onDelaySec));
    }

    /**
     * Debounce this boolean: it turns ON only after being continuously true for {@code onDelaySec}
     * seconds, and turns OFF only after being continuously false for {@code offDelaySec} seconds.
     */
    default BooleanSource debouncedOnOff(double onDelaySec, double offDelaySec) {
        return debounced(DebounceBoolean.onAfterOffAfter(onDelaySec, offDelaySec));
    }

    /**
     * Debounce this boolean using a shared {@link DebounceBoolean} state machine.
     *
     * <p>This is useful when you want to retain and inspect the debouncer explicitly. If one
     * debouncer is deliberately shared by multiple wrapper graphs, it remains a single
     * first-updater-wins state owner for each cycle.</p>
     *
     * <p>A failed upstream observation does not advance or cache the debouncer. Recursive sampling
     * and sampling/reset overlap fail fast.</p>
     */
    default BooleanSource debounced(DebounceBoolean debouncer) {
        Objects.requireNonNull(debouncer, "debouncer");
        BooleanSource self = this;

        return new BooleanSource() {
            private final SourceOperationGuard guard =
                    new SourceOperationGuard("debounced Boolean source");
            private long lastCycle = Long.MIN_VALUE;
            private boolean last = false;

            /**
             * {@inheritDoc}
             */
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                guard.beginSample();
                try {
                    Objects.requireNonNull(clock, "clock");
                    long cyc = clock.cycle();
                    if (cyc == lastCycle) {
                        return last;
                    }

                    boolean raw = self.getAsBoolean(clock);
                    boolean next = debouncer.update(clock, raw);
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
                    debouncer.reset(false);
                    last = false;
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
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "debounce" : prefix;
                dbg.addData(p + ".class", "DebouncedBoolean");
                debouncer.debugDump(dbg, p + ".debouncer");
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    // ---------------------------------------------------------------------------------------------
    // Edge helpers
    // ---------------------------------------------------------------------------------------------

    /**
     * A one-loop pulse that is true only when this source transitions false -> true.
     *
     * <p>On the first sample after reset/construction, this returns false (no spurious edge).
     * Sample every loop for best results. A failed upstream observation does not advance the
     * remembered edge state and may be retried in the same cycle.</p>
     */
    default BooleanSource risingEdge() {
        BooleanSource self = this;
        return new BooleanSource() {
            private final SourceOperationGuard guard =
                    new SourceOperationGuard("rising-edge Boolean source");
            private long lastCycle = Long.MIN_VALUE;
            private boolean initialized = false;
            private boolean prev = false;
            private boolean last = false;

            /**
             * {@inheritDoc}
             */
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                guard.beginSample();
                try {
                    Objects.requireNonNull(clock, "clock");
                    long cyc = clock.cycle();
                    if (cyc == lastCycle) {
                        return last;
                    }

                    boolean cur = self.getAsBoolean(clock);
                    boolean nextInitialized = true;
                    boolean nextPrev = cur;
                    boolean nextLast = initialized && cur && !prev;

                    initialized = nextInitialized;
                    prev = nextPrev;
                    last = nextLast;
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
                    initialized = false;
                    prev = false;
                    last = false;
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
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "rise" : prefix;
                dbg.addData(p + ".class", "RisingEdge");
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    /**
     * A one-loop pulse that is true only when this source transitions true -> false.
     *
     * <p>On the first sample after reset/construction, this returns false (no spurious edge).
     * Sample every loop for best results. A failed upstream observation does not advance the
     * remembered edge state and may be retried in the same cycle.</p>
     */
    default BooleanSource fallingEdge() {
        BooleanSource self = this;
        return new BooleanSource() {
            private final SourceOperationGuard guard =
                    new SourceOperationGuard("falling-edge Boolean source");
            private long lastCycle = Long.MIN_VALUE;
            private boolean initialized = false;
            private boolean prev = false;
            private boolean last = false;

            /**
             * {@inheritDoc}
             */
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                guard.beginSample();
                try {
                    Objects.requireNonNull(clock, "clock");
                    long cyc = clock.cycle();
                    if (cyc == lastCycle) {
                        return last;
                    }

                    boolean cur = self.getAsBoolean(clock);
                    boolean nextInitialized = true;
                    boolean nextPrev = cur;
                    boolean nextLast = initialized && !cur && prev;

                    initialized = nextInitialized;
                    prev = nextPrev;
                    last = nextLast;
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
                    initialized = false;
                    prev = false;
                    last = false;
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
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "fall" : prefix;
                dbg.addData(p + ".class", "FallingEdge");
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    // ---------------------------------------------------------------------------------------------
    // Selection helpers
    // ---------------------------------------------------------------------------------------------

    /**
     * Select between two sources based on this boolean.
     *
     * <p>If this boolean is true, {@code whenTrue} is sampled; otherwise {@code whenFalse} is
     * sampled. This is a small but very useful building block for "manual vs auto" selection and
     * priority rules. The unselected branch is intentionally not sampled.</p>
     */
    default <T> Source<T> choose(Source<T> whenTrue, Source<T> whenFalse) {
        Objects.requireNonNull(whenTrue, "whenTrue");
        Objects.requireNonNull(whenFalse, "whenFalse");
        BooleanSource cond = this;

        return new Source<T>() {
            /**
             * {@inheritDoc}
             */
            @Override
            public T get(LoopClock clock) {
                return cond.getAsBoolean(clock) ? whenTrue.get(clock) : whenFalse.get(clock);
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                cond.reset();
                whenTrue.reset();
                whenFalse.reset();
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "choose" : prefix;
                dbg.addData(p + ".class", "ChooseSource");
                cond.debugDump(dbg, p + ".cond");
                whenTrue.debugDump(dbg, p + ".true");
                whenFalse.debugDump(dbg, p + ".false");
            }
        };
    }

    /**
     * Select between two scalars based on this boolean.
     *
     * <p>The condition is sampled first, then only the selected branch is sampled. The unselected
     * branch is intentionally not sampled.</p>
     */
    default ScalarSource choose(ScalarSource whenTrue, ScalarSource whenFalse) {
        Objects.requireNonNull(whenTrue, "whenTrue");
        Objects.requireNonNull(whenFalse, "whenFalse");
        BooleanSource cond = this;

        return new ScalarSource() {
            /**
             * {@inheritDoc}
             */
            @Override
            public double getAsDouble(LoopClock clock) {
                return cond.getAsBoolean(clock) ? whenTrue.getAsDouble(clock) : whenFalse.getAsDouble(clock);
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                cond.reset();
                whenTrue.reset();
                whenFalse.reset();
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "chooseScalar" : prefix;
                dbg.addData(p + ".class", "ChooseScalar");
                cond.debugDump(dbg, p + ".cond");
                whenTrue.debugDump(dbg, p + ".true");
                whenFalse.debugDump(dbg, p + ".false");
            }
        };
    }

    /**
     * Select between two booleans based on this boolean.
     *
     * <p>The condition is sampled first, then only the selected branch is sampled. The unselected
     * branch is intentionally not sampled.</p>
     */
    default BooleanSource choose(BooleanSource whenTrue, BooleanSource whenFalse) {
        Objects.requireNonNull(whenTrue, "whenTrue");
        Objects.requireNonNull(whenFalse, "whenFalse");
        BooleanSource cond = this;

        return new BooleanSource() {
            /**
             * {@inheritDoc}
             */
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                return cond.getAsBoolean(clock) ? whenTrue.getAsBoolean(clock) : whenFalse.getAsBoolean(clock);
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                cond.reset();
                whenTrue.reset();
                whenFalse.reset();
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "chooseBool" : prefix;
                dbg.addData(p + ".class", "ChooseBoolean");
                cond.debugDump(dbg, p + ".cond");
                whenTrue.debugDump(dbg, p + ".true");
                whenFalse.debugDump(dbg, p + ".false");
            }
        };
    }

    /**
     * Toggle state each time this source has a rising edge.
     */
    default BooleanSource toggled() {
        return toggled(false);
    }

    /**
     * Toggle state each time this source has a rising edge.
     *
     * <p>A failed upstream observation leaves the toggle state unchanged and may be retried in the
     * same cycle.</p>
     *
     * @param initialState initial output state
     */
    default BooleanSource toggled(boolean initialState) {
        BooleanSource self = this;
        return new BooleanSource() {
            private final SourceOperationGuard guard =
                    new SourceOperationGuard("toggled Boolean source");
            private long lastCycle = Long.MIN_VALUE;
            private boolean initialized = false;
            private boolean prev = false;
            private boolean state = initialState;

            /**
             * {@inheritDoc}
             */
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                guard.beginSample();
                try {
                    Objects.requireNonNull(clock, "clock");
                    long cyc = clock.cycle();
                    if (cyc == lastCycle) {
                        return state;
                    }

                    boolean cur = self.getAsBoolean(clock);
                    boolean nextInitialized = true;
                    boolean nextPrev = cur;
                    boolean nextState = state;
                    if (initialized && cur && !prev) {
                        nextState = !state;
                    }

                    initialized = nextInitialized;
                    prev = nextPrev;
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
                    initialized = false;
                    prev = false;
                    state = initialState;
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
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "toggle" : prefix;
                dbg.addData(p + ".class", "ToggledBoolean")
                        .addData(p + ".state", state);
                self.debugDump(dbg, p + ".src");
            }
        };
    }

    // ---------------------------------------------------------------------------------------------
    // Factories
    // ---------------------------------------------------------------------------------------------

    /**
     * Create a boolean source from a raw supplier.
     *
     * <p>The supplier is sampled each time {@link #getAsBoolean(LoopClock)} is called.
     * The {@code clock} parameter is ignored.</p>
     */
    static BooleanSource of(BooleanSupplier raw) {
        Objects.requireNonNull(raw, "raw");
        return new BooleanSource() {
            /**
             * {@inheritDoc}
             */
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                return raw.getAsBoolean();
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "raw" : prefix;
                dbg.addData(p + ".class", "RawBoolean");
            }
        };
    }

    /**
     * Constant boolean source.
     */
    static BooleanSource constant(boolean value) {
        return new BooleanSource() {
            /**
             * {@inheritDoc}
             */
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                return value;
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "const" : prefix;
                dbg.addData(p + ".class", "ConstantBoolean")
                        .addData(p + ".value", value);
            }
        };
    }
}
