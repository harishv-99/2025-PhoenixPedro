package edu.ftcphoenix.fw.core.source;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;

/**
 * A {@link Source} that can answer either for the current loop or for a previous timestamp.
 *
 * <p>This is useful whenever a value is used to interpret a delayed sensor measurement. For
 * example, a turret-mounted camera frame may be 80 ms old; the AprilTag solve should use the
 * turret/camera frame from the frame timestamp, not the turret's current frame after it has already
 * moved.</p>
 *
 * <p>Implementations that do not keep history may return the current value from
 * {@link #getAt(LoopClock, LoopTimestamp)}. Use {@link TimeAwareSources#currentOnly(Source)} for that
 * explicit fallback, and prefer a real history-backed source for fast-moving mechanisms.</p>
 *
 * @param <T> sampled value type
 */
public interface TimeAwareSource<T> extends Source<T> {

    /**
     * Returns the value associated with {@code timestamp} when available.
     *
     * @param clock     current loop clock, used for timebase and for current-only fallbacks
     * @param timestamp epoch-safe requested time created by that same clock
     * @return value at or near the requested timestamp; never {@code null}
     */
    T getAt(LoopClock clock, LoopTimestamp timestamp);

    /**
     * Samples the current-loop value. By default this delegates to
     * {@link #getAt(LoopClock, LoopTimestamp)}
     * using {@link LoopClock#nowTimestamp()}.
     */
    @Override
    default T get(LoopClock clock) {
        return getAt(clock, clock.nowTimestamp());
    }
}
