package edu.ftcphoenix.fw.core.source;

import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A {@link Source} that can answer either for the current loop or for a previous timestamp.
 *
 * <p>This is useful whenever a value is used to interpret a delayed sensor measurement. For
 * example, a turret-mounted camera frame may be 80 ms old; the AprilTag solve should use the
 * turret/camera frame from the frame timestamp, not the turret's current frame after it has already
 * moved.</p>
 *
 * <p>Implementations that do not keep history may return the current value from
 * {@link #getAt(LoopClock, double)}. Use {@link TimeAwareSources#currentOnly(Source)} for that
 * explicit fallback, and prefer a real history-backed source for fast-moving mechanisms.</p>
 *
 * @param <T> sampled value type
 */
public interface TimeAwareSource<T> extends Source<T> {

    /**
     * Returns the value associated with {@code timestampSec} when available.
     *
     * @param clock        current loop clock, used for timebase and for current-only fallbacks
     * @param timestampSec timestamp in the same timebase as {@link LoopClock#nowSec()}
     * @return value at or near the requested timestamp; never {@code null}
     */
    T getAt(LoopClock clock, double timestampSec);

    /**
     * Samples the current-loop value. By default this delegates to {@link #getAt(LoopClock, double)}
     * using {@link LoopClock#nowSec()}.
     */
    @Override
    default T get(LoopClock clock) {
        return getAt(clock, clock.nowSec());
    }
}
