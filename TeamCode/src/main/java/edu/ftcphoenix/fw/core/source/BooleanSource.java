package edu.ftcphoenix.fw.core.source;

import java.util.Objects;
import java.util.function.BooleanSupplier;

import edu.ftcphoenix.fw.core.control.DebounceBoolean;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

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
     * <p>The returned source samples the upstream source at most once per cycle and returns the cached
     * value for any additional reads in that same cycle.</p>
     */
    default BooleanSource memoized() {
        BooleanSource self = this;
        return new BooleanSource() {
            private long lastCycle = Long.MIN_VALUE;
            private boolean last = false;

            @Override
            public boolean getAsBoolean(LoopClock clock) {
                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return last;
                }
                lastCycle = cyc;
                last = self.getAsBoolean(clock);
                return last;
            }

            @Override
            public void reset() {
                self.reset();
                lastCycle = Long.MIN_VALUE;
                last = false;
            }

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
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                return !self.getAsBoolean(clock);
            }

            @Override
            public void reset() {
                self.reset();
            }

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
     */
    default BooleanSource and(BooleanSource other) {
        Objects.requireNonNull(other, "other");
        BooleanSource self = this;
        return new BooleanSource() {
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                return self.getAsBoolean(clock) && other.getAsBoolean(clock);
            }

            @Override
            public void reset() {
                self.reset();
                other.reset();
            }

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
     */
    default BooleanSource or(BooleanSource other) {
        Objects.requireNonNull(other, "other");
        BooleanSource self = this;
        return new BooleanSource() {
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                return self.getAsBoolean(clock) || other.getAsBoolean(clock);
            }

            @Override
            public void reset() {
                self.reset();
                other.reset();
            }

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
     * <p>This is useful when you want to reset the debouncer explicitly, or share the same
     * debouncer between multiple consumers.</p>
     */
    default BooleanSource debounced(DebounceBoolean debouncer) {
        Objects.requireNonNull(debouncer, "debouncer");
        BooleanSource self = this;

        return new BooleanSource() {
            private long lastCycle = Long.MIN_VALUE;
            private boolean last = false;

            @Override
            public boolean getAsBoolean(LoopClock clock) {
                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return last;
                }
                lastCycle = cyc;
                last = debouncer.update(clock, self.getAsBoolean(clock));
                return last;
            }

            @Override
            public void reset() {
                self.reset();
                debouncer.reset(false);
                lastCycle = Long.MIN_VALUE;
                last = false;
            }

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
     * Sample every loop for best results.</p>
     */
    default BooleanSource risingEdge() {
        BooleanSource self = this;
        return new BooleanSource() {
            private long lastCycle = Long.MIN_VALUE;
            private boolean initialized = false;
            private boolean prev = false;
            private boolean last = false;

            @Override
            public boolean getAsBoolean(LoopClock clock) {
                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return last;
                }
                lastCycle = cyc;

                boolean cur = self.getAsBoolean(clock);
                if (!initialized) {
                    initialized = true;
                    prev = cur;
                    last = false;
                    return false;
                }

                last = cur && !prev;
                prev = cur;
                return last;
            }

            @Override
            public void reset() {
                self.reset();
                lastCycle = Long.MIN_VALUE;
                initialized = false;
                prev = false;
                last = false;
            }

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
     * Sample every loop for best results.</p>
     */
    default BooleanSource fallingEdge() {
        BooleanSource self = this;
        return new BooleanSource() {
            private long lastCycle = Long.MIN_VALUE;
            private boolean initialized = false;
            private boolean prev = false;
            private boolean last = false;

            @Override
            public boolean getAsBoolean(LoopClock clock) {
                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return last;
                }
                lastCycle = cyc;

                boolean cur = self.getAsBoolean(clock);
                if (!initialized) {
                    initialized = true;
                    prev = cur;
                    last = false;
                    return false;
                }

                last = !cur && prev;
                prev = cur;
                return last;
            }

            @Override
            public void reset() {
                self.reset();
                lastCycle = Long.MIN_VALUE;
                initialized = false;
                prev = false;
                last = false;
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "fall" : prefix;
                dbg.addData(p + ".class", "FallingEdge");
                self.debugDump(dbg, p + ".src");
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
     * @param initialState initial output state
     */
    default BooleanSource toggled(boolean initialState) {
        BooleanSource self = this;
        return new BooleanSource() {
            private long lastCycle = Long.MIN_VALUE;
            private boolean initialized = false;
            private boolean prev = false;
            private boolean state = initialState;

            @Override
            public boolean getAsBoolean(LoopClock clock) {
                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return state;
                }
                lastCycle = cyc;

                boolean cur = self.getAsBoolean(clock);
                if (!initialized) {
                    initialized = true;
                    prev = cur;
                    return state;
                }

                if (cur && !prev) {
                    state = !state;
                }
                prev = cur;
                return state;
            }

            @Override
            public void reset() {
                self.reset();
                lastCycle = Long.MIN_VALUE;
                initialized = false;
                prev = false;
                state = initialState;
            }

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
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                return raw.getAsBoolean();
            }

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
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                return value;
            }

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
