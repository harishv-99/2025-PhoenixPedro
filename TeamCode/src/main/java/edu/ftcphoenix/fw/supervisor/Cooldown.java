package edu.ftcphoenix.fw.supervisor;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A simple time-based cooldown gate.
 *
 * <p>{@code Cooldown} is a small supervisor-layer helper: it answers the question
 * "am I allowed to do this again yet?" and provides a {@link BooleanSource}
 * view so it composes naturally with other signals.</p>
 *
 * <h2>Semantics</h2>
 * <ul>
 *   <li>{@link #getAsBoolean(LoopClock)} returns {@code true} when the cooldown is ready.</li>
 *   <li>Calling {@link #trigger(LoopClock)} starts/restarts the cooldown window.</li>
 *   <li>{@link #remainingSec(LoopClock)} reports how long until ready (clamped to 0).</li>
 * </ul>
 *
 * <p>This class is intentionally tiny. Most robots only need "ready" + "trigger".
 * If you find yourself adding a lot of policy here, that policy probably belongs
 * in a supervisor (not in this helper).</p>
 */
public final class Cooldown implements BooleanSource {

    private final double cooldownSec;
    private double readyAtSec = Double.NEGATIVE_INFINITY;

    /**
     * Create a cooldown gate.
     *
     * @param cooldownSec cooldown duration in seconds (must be {@code >= 0})
     */
    public Cooldown(double cooldownSec) {
        if (cooldownSec < 0.0) {
            throw new IllegalArgumentException("cooldownSec must be >= 0, got " + cooldownSec);
        }
        this.cooldownSec = cooldownSec;
    }

    /**
     * Start (or restart) the cooldown window.
     */
    public void trigger(LoopClock clock) {
        if (clock == null) {
            throw new IllegalArgumentException("clock is required");
        }
        readyAtSec = clock.nowSec() + cooldownSec;
    }

    /**
     * @return {@code true} if the cooldown is ready.
     */
    public boolean isReady(LoopClock clock) {
        return getAsBoolean(clock);
    }

    /**
     * @return seconds remaining until ready (clamped to 0).
     */
    public double remainingSec(LoopClock clock) {
        if (clock == null) {
            throw new IllegalArgumentException("clock is required");
        }
        double remaining = readyAtSec - clock.nowSec();
        return Math.max(0.0, remaining);
    }

    /**
     * {@inheritDoc}
     *
     * <p>Returns {@code true} when ready.</p>
     */
    @Override
    public boolean getAsBoolean(LoopClock clock) {
        if (clock == null) {
            throw new IllegalArgumentException("clock is required");
        }
        return clock.nowSec() >= readyAtSec;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void reset() {
        readyAtSec = Double.NEGATIVE_INFINITY;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "cooldown" : prefix;
        dbg.addData(p + ".class", "Cooldown")
                .addData(p + ".cooldownSec", cooldownSec)
                .addData(p + ".readyAtSec", readyAtSec);

        // Remaining time is only meaningful when we have a clock, but debugDump does not.
        // Consumers that want remaining time should call remainingSec(clock) explicitly.
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "Cooldown{" + "cooldownSec=" + cooldownSec + ", readyAtSec=" + readyAtSec + "}";
    }
}
