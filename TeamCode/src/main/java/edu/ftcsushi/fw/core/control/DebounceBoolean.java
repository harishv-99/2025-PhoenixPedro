package edu.ftcsushi.fw.core.control;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * A small, reusable time-based latch ("debouncer") that filters brief observed changes in a
 * boolean signal. For example, a physical switch can flicker while its contacts settle.
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
 *   <li><b>Shooter ready</b>: delay accepting sampled {@code atTarget()} observations before
 *       feeding a ring.</li>
 *   <li><b>Sensor validity</b>: delay accepting a sampled validity change; this does not establish
 *       freshness or physical visibility between samples.</li>
 *   <li><b>Driver intent</b>: delay accepting a sampled button/condition change.</li>
 * </ul>
 *
 * <h2>Semantics</h2>
 *
 * <p>Given a raw boolean {@code value}:</p>
 *
 * <ul>
 *   <li>On each new sampled cycle, a value different from the accepted state adds the nonnegative
 *       {@link LoopClock#dtSec()} interval to the pending change.</li>
 *   <li>The latch accepts that change when the accumulated intervals reach {@code onDelaySec}
 *       for <b>OFF → ON</b>, or {@code offDelaySec} for <b>ON → OFF</b>.</li>
 *   <li>A sample matching the accepted state clears the pending change.</li>
 * </ul>
 *
 * <p>The interval belongs to the current sample and may begin before the raw change was observed.
 * This is sampled conditioning, not a timer beginning at the first changed sample or proof of
 * continuous physical stability. Unobserved transitions between samples remain unknown.</p>
 *
 * <p>Setting either delay to {@code 0} makes that edge immediate.</p>
 *
 * <p>{@link #update(LoopClock, boolean)} is <b>idempotent by</b> {@link LoopClock#cycle()}.
 * If called twice in the same loop cycle, the accepted state and pending time do not advance
 * again. The last-raw diagnostic may reflect the repeated call's argument.</p>
 */
public final class DebounceBoolean {

    private final double onDelaySec;
    private final double offDelaySec;

    private boolean state;

    // Sum of loop intervals contributed by observations opposite the accepted state.
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
     * Create a latch that accepts ON after differing true samples accumulate {@code onDelaySec}
     * seconds of loop intervals, and accepts OFF immediately on a false sample.
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
