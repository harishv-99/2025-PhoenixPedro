package edu.ftcphoenix.fw.supervisor;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Minimal enum-backed state machine with entry timing.
 *
 * <p>Phoenix supervisors are encouraged to keep state machines <b>small</b> and
 * delegate signal conditioning to {@code Source} combinators and time-based control to
 * {@code Task} / {@code OutputTask} where possible.</p>
 *
 * <p>This class exists for the 20% of cases where you really do need an explicit
 * state variable and want convenient "time in state" measurement for transitions.</p>
 */
public final class EnumStateMachine<S extends Enum<S>> {

    private final S initial;
    private S state;
    private double enteredSec;

    /**
     * Create a state machine with an initial state.
     */
    public EnumStateMachine(S initial) {
        this.initial = Objects.requireNonNull(initial, "initial");
        this.state = initial;
        // Best-effort: most robots create supervisors at runtime=0, so this is fine.
        this.enteredSec = 0.0;
    }

    /**
     * @return current state
     */
    public S state() {
        return state;
    }

    /**
     * @return {@code true} if the current state is {@code s}.
     */
    public boolean is(S s) {
        return state == s;
    }

    /**
     * Transition to {@code next}.
     *
     * <p>If {@code next == state}, this is a no-op and the entry timer is not reset.</p>
     */
    public void set(LoopClock clock, S next) {
        Objects.requireNonNull(clock, "clock");
        Objects.requireNonNull(next, "next");
        if (next == state) {
            return;
        }
        state = next;
        enteredSec = clock.nowSec();
    }

    /**
     * Reset to the initial state and restart the entry timer.
     */
    public void reset(LoopClock clock) {
        reset(clock, initial);
    }

    /**
     * Reset to an explicit state and restart the entry timer.
     */
    public void reset(LoopClock clock, S state) {
        Objects.requireNonNull(clock, "clock");
        Objects.requireNonNull(state, "state");
        this.state = state;
        this.enteredSec = clock.nowSec();
    }

    /**
     * @return seconds elapsed since entering the current state.
     */
    public double timeInStateSec(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        return Math.max(0.0, clock.nowSec() - enteredSec);
    }

    /**
     * A {@link BooleanSource} view of {@link #is(Enum)}.
     */
    public BooleanSource isSource(S s) {
        Objects.requireNonNull(s, "s");
        EnumStateMachine<S> self = this;
        return new BooleanSource() {
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                return self.is(s);
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) {
                    return;
                }
                String p = (prefix == null || prefix.isEmpty()) ? "state" : prefix;
                dbg.addData(p + ".class", "EnumStateMachineIs")
                        .addData(p + ".state", self.state)
                        .addData(p + ".equals", self.is(s));
            }
        };
    }

    /**
     * Debug helper.
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "sm" : prefix;
        dbg.addData(p + ".class", "EnumStateMachine")
                .addData(p + ".state", state)
                .addData(p + ".enteredSec", enteredSec);
    }

    @Override
    public String toString() {
        return "EnumStateMachine{" + "state=" + state + ", enteredSec=" + enteredSec + "}";
    }
}
