package edu.ftcphoenix.fw.supervisor;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A tiny saturating counter for pending requests or other small counted tokens.
 *
 * <p>This helper is commonly used as the "pending" layer-1 input shape inside robot-owned
 * capabilities and supervisors:</p>
 * <ul>
 *   <li>Tap a button to request "shoot one"; multiple taps queue multiple requests.</li>
 *   <li>Autonomous code can request N actions up front (burst / preload).</li>
 *   <li>A supervisor can consume requests when a real-world event occurs.</li>
 * </ul>
 *
 * <p>{@code RequestCounter} also has uses beyond layer-1 input memory: any small capped token count
 * that wants a convenient {@link BooleanSource} view can reuse it.</p>
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
        request(1);
    }

    /**
     * Add {@code count} pending requests (saturating at {@link #maxCount()}).
     *
     * <p>Non-positive counts are ignored.</p>
     *
     * @param count number of requests to add
     */
    public void request(int count) {
        if (count <= 0) {
            return;
        }
        long capped = Math.min((long) maxCount, (long) this.count + count);
        this.count = (int) capped;
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
     * Consume up to {@code maxToConsume} pending requests.
     *
     * <p>Non-positive limits consume nothing.</p>
     *
     * @param maxToConsume maximum number of requests to consume
     * @return number of requests actually consumed
     */
    public int consumeUpTo(int maxToConsume) {
        if (maxToConsume <= 0 || count <= 0) {
            return 0;
        }
        int consumed = Math.min(maxToConsume, count);
        count -= consumed;
        return consumed;
    }

    /**
     * Consume every pending request.
     *
     * @return number of requests that were pending before the clear
     */
    public int consumeAll() {
        return consumeUpTo(Integer.MAX_VALUE);
    }

    /**
     * Remove all pending requests.
     */
    public void clear() {
        count = 0;
    }

    /**
     * @return true when at least one request is pending.
     */
    public boolean hasRequest() {
        return count > 0;
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

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean getAsBoolean(LoopClock clock) {
        return hasRequest();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void reset() {
        clear();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "requests" : prefix;
        dbg.addData(p + ".class", "RequestCounter")
                .addData(p + ".count", count)
                .addData(p + ".maxCount", maxCount)
                .addData(p + ".hasRequest", hasRequest());
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "RequestCounter{" + "count=" + count + ", maxCount=" + maxCount + "}";
    }
}
