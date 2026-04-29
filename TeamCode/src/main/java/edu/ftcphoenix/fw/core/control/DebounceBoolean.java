package edu.ftcphoenix.fw.core.control;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A small, reusable time-based latch ("debouncer") that turns a potentially chattery boolean
 * signal into a stable boolean.
 *
 * <p>This is the <b>time-domain</b> analogue of {@link HysteresisBoolean}:</p>
 *
 * <ul>
 *   <li>{@link HysteresisBoolean} uses two <i>thresholds</i> (enter/exit) to avoid chatter around
 *       a numeric boundary.</li>
 *   <li>{@code DebounceBoolean} uses two <i>durations</i> (on/off delays) to avoid chatter across
 *       a boolean boundary.</li>
 * </ul>
 *
 * <h2>Common uses</h2>
 * <ul>
 *   <li><b>Shooter ready</b>: require {@code atTarget()} to be true continuously for
 *       0.10–0.20s before feeding a ring.</li>
 *   <li><b>Sensor validity</b>: require a vision target to be present for N frames/time before
 *       trusting it.</li>
 *   <li><b>Driver intent</b>: require a button/condition to be held for N seconds before it
 *       "counts".</li>
 * </ul>
 *
 * <h2>Semantics</h2>
 *
 * <p>Given a raw boolean {@code value}:</p>
 *
 * <ul>
 *   <li>The latch transitions <b>OFF → ON</b> only after {@code value == true} continuously for
 *       {@code onDelaySec} seconds.</li>
 *   <li>The latch transitions <b>ON → OFF</b> only after {@code value == false} continuously for
 *       {@code offDelaySec} seconds.</li>
 * </ul>
 *
 * <p>Setting either delay to {@code 0} makes that edge immediate.</p>
 *
 * <p>{@link #update(LoopClock, boolean)} is <b>idempotent by</b> {@link LoopClock#cycle()}.
 * If called twice in the same loop cycle, the second call is a no-op (it will not double-count
 * time).</p>
 */
public final class DebounceBoolean {

    private final double onDelaySec;
    private final double offDelaySec;

    private boolean state;

    // How long the raw value has been continuously in the opposite state.
    private double pendingSec;

    // Debug: last raw value observed.
    private boolean lastRaw;

    // Idempotence by clock.cycle().
    private long lastCycle = Long.MIN_VALUE;

    private DebounceBoolean(double onDelaySec, double offDelaySec, boolean initialState) {
        if (!Double.isFinite(onDelaySec) || !Double.isFinite(offDelaySec)) {
            throw new IllegalArgumentException("delays must be finite");
        }
        if (onDelaySec < 0.0 || offDelaySec < 0.0) {
            throw new IllegalArgumentException("delays must be >= 0");
        }

        this.onDelaySec = onDelaySec;
        this.offDelaySec = offDelaySec;
        this.state = initialState;
        this.pendingSec = 0.0;
        this.lastRaw = initialState;
    }

    /**
     * Create a latch that turns ON only after {@code onDelaySec} seconds of continuous true,
     * and turns OFF immediately when the raw input becomes false.
     */
    public static DebounceBoolean onAfterOffImmediately(double onDelaySec) {
        return new DebounceBoolean(onDelaySec, 0.0, false);
    }

    /**
     * Create a latch with explicit ON and OFF delays.
     */
    public static DebounceBoolean onAfterOffAfter(double onDelaySec, double offDelaySec) {
        return new DebounceBoolean(onDelaySec, offDelaySec, false);
    }

    /**
     * Update the latch using the latest raw boolean value.
     *
     * <p>This method is idempotent by {@link LoopClock#cycle()}.</p>
     *
     * @param clock loop clock (required)
     * @param value latest raw boolean
     * @return the updated latched state
     */
    public boolean update(LoopClock clock, boolean value) {
        if (clock == null) {
            throw new IllegalArgumentException("clock is required");
        }

        long c = clock.cycle();
        if (c == lastCycle) {
            // Do not double-count dt if nested code calls update twice in one loop.
            lastRaw = value;
            return state;
        }
        lastCycle = c;

        lastRaw = value;

        // If raw matches the current state, we're stable; clear pending time.
        if (value == state) {
            pendingSec = 0.0;
            return state;
        }

        // Raw differs from current state: accumulate time toward a state change.
        double delay = value ? onDelaySec : offDelaySec;
        if (delay <= 0.0) {
            // Immediate edge.
            state = value;
            pendingSec = 0.0;
            return state;
        }

        double dt = clock.dtSec();
        if (dt < 0.0) {
            dt = 0.0;
        }
        pendingSec += dt;

        if (pendingSec >= delay) {
            state = value;
            pendingSec = 0.0;
        }

        return state;
    }

    /**
     * @return current latched state.
     */
    public boolean get() {
        return state;
    }

    /**
     * Force the latch to a known state and clear any pending transition.
     */
    public void reset(boolean state) {
        this.state = state;
        this.pendingSec = 0.0;
        this.lastRaw = state;
        this.lastCycle = Long.MIN_VALUE;
    }

    /**
     * @return configured ON delay in seconds.
     */
    public double onDelaySec() {
        return onDelaySec;
    }

    /**
     * @return configured OFF delay in seconds.
     */
    public double offDelaySec() {
        return offDelaySec;
    }

    /**
     * @return seconds accumulated toward the next transition.
     */
    public double pendingSec() {
        return pendingSec;
    }

    /**
     * @return last raw value sampled by {@link #update(LoopClock, boolean)}.
     */
    public boolean lastRaw() {
        return lastRaw;
    }

    /**
     * Optional debug helper.
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "debounce" : prefix;
        dbg.addLine(p + ": DebounceBoolean")
                .addData(p + ".onDelaySec", onDelaySec)
                .addData(p + ".offDelaySec", offDelaySec)
                .addData(p + ".state", state)
                .addData(p + ".pendingSec", pendingSec)
                .addData(p + ".lastRaw", lastRaw);
    }
}
