package edu.ftcphoenix.fw.core.source;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Factory helpers for {@link TimeAwareSource} values.
 */
public final class TimeAwareSources {

    private TimeAwareSources() {
        // utility
    }

    /**
     * Returns a time-aware source whose value never changes.
     *
     * @param value constant value to return for current and historical samples
     * @param <T>   value type
     * @return fixed time-aware source
     */
    public static <T> TimeAwareSource<T> fixed(T value) {
        Objects.requireNonNull(value, "value");
        return new TimeAwareSource<T>() {
            @Override
            public T getAt(LoopClock clock, double timestampSec) {
                return value;
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "fixedTimeAwareSource" : prefix;
                dbg.addData(p + ".value", value);
            }

            @Override
            public String toString() {
                return "TimeAwareSources.fixed{" + value + "}";
            }
        };
    }

    /**
     * Wraps a normal source as a current-only time-aware source.
     *
     * <p>This is appropriate for fixed/slow signals or as an explicit fallback. For fast-moving
     * mechanisms that are used to interpret delayed camera frames, prefer a history-backed source.</p>
     *
     * @param source current-loop source
     * @param <T>    value type
     * @return time-aware source that ignores historical timestamps
     */
    public static <T> TimeAwareSource<T> currentOnly(Source<T> source) {
        Objects.requireNonNull(source, "source");
        return new TimeAwareSource<T>() {
            @Override
            public T getAt(LoopClock clock, double timestampSec) {
                return Objects.requireNonNull(source.get(clock), "source returned null");
            }

            @Override
            public void reset() {
                source.reset();
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) return;
                String p = (prefix == null || prefix.isEmpty()) ? "currentOnlyTimeAwareSource" : prefix;
                source.debugDump(dbg, p + ".source");
            }

            @Override
            public String toString() {
                return "TimeAwareSources.currentOnly{" + source + "}";
            }
        };
    }
}
