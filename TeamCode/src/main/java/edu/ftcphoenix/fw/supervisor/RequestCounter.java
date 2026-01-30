package edu.ftcphoenix.fw.supervisor;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A tiny saturating counter for "requests".
 *
 * <p>This is useful for human input patterns like:</p>
 * <ul>
 *   <li>Tap a button to request "shoot one"; multiple taps queue multiple requests.</li>
 *   <li>Autonomous code can request N actions up front (burst / preload).</li>
 *   <li>A supervisor can consume requests when a real-world event occurs (ball left sensor).</li>
 * </ul>
 *
 * <p>{@code RequestCounter} implements {@link BooleanSource} where {@code true}
 * means "at least one request is pending". This makes it easy to combine with
 * other boolean gates (cooldowns, readiness, sensors).</p>
 *
 * <p>This helper is intentionally dumb: it does not schedule tasks. Supervisors decide
 * what a request <i>means</i> and when it is consumed.</p>
 */
public final class RequestCounter implements BooleanSource {

    private final int maxCount;
    private int count = 0;

    /**
     * Create an unbounded request counter.
     */
    public RequestCounter() {
        this(Integer.MAX_VALUE);
    }

    /**
     * Create a request counter with a maximum pending count.
     *
     * <p>When the counter is full, additional {@link #request()} calls are ignored.
     * This is a common "driver safety" choice so accidental spam cannot queue
     * dozens of actions.</p>
     *
     * @param maxCount maximum pending requests (must be {@code >= 1})
     */
    public RequestCounter(int maxCount) {
        if (maxCount < 1) {
            throw new IllegalArgumentException("maxCount must be >= 1, got " + maxCount);
        }
        this.maxCount = maxCount;
    }

    /**
     * Add one pending request (saturating at {@link #maxCount()}).
     */
    public void request() {
        if (count < maxCount) {
            count++;
        }
    }

    /**
     * Consume one pending request.
     *
     * @return {@code true} if a request was consumed
     */
    public boolean consume() {
        if (count <= 0) {
            return false;
        }
        count--;
        return true;
    }

    /**
     * Remove all pending requests.
     */
    public void clear() {
        count = 0;
    }

    /**
     * @return the number of pending requests.
     */
    public int count() {
        return count;
    }

    /**
     * @return maximum pending requests (saturation limit).
     */
    public int maxCount() {
        return maxCount;
    }

    @Override
    public boolean getAsBoolean(LoopClock clock) {
        return count > 0;
    }

    @Override
    public void reset() {
        clear();
    }

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "requests" : prefix;
        dbg.addData(p + ".class", "RequestCounter")
                .addData(p + ".count", count)
                .addData(p + ".maxCount", maxCount)
                .addData(p + ".hasRequest", count > 0);
    }

    @Override
    public String toString() {
        return "RequestCounter{" + "count=" + count + ", maxCount=" + maxCount + "}";
    }
}
