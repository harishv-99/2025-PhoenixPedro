package edu.ftcphoenix.fw.supervisor;

import java.util.Objects;
import java.util.function.LongSupplier;

import edu.ftcphoenix.fw.core.debug.DebugSink;

/**
 * Small supervisor-side value slot that is fresh only for the current loop cycle.
 *
 * <p>This is the "frame" layer-1 input shape used for continuous manual commands that should be
 * refreshed every loop. A caller writes the current value with {@link #set(Object)}. Reads through
 * {@link #get()} return that value only during the same cycle reported by the shared cycle source.
 * On later cycles, the slot falls back automatically to the configured default value unless the
 * caller writes again.</p>
 *
 * <p>Typical uses include manual power, jog commands, and other non-drive mechanism inputs wired
 * through {@code Bindings.copyEachCycle(...)} or direct per-loop capability calls.</p>
 *
 * @param <T> stored value type; values must be non-null
 */
public final class FrameValue<T> {

    private final LongSupplier cycleSource;
    private final T defaultValue;

    private long writtenCycle = Long.MIN_VALUE;
    private T value;

    /**
     * Create a frame value tied to a shared loop-cycle source.
     *
     * @param cycleSource  supplier returning the current loop-cycle id (required)
     * @param defaultValue value returned when no write has occurred in the current cycle
     */
    public FrameValue(LongSupplier cycleSource, T defaultValue) {
        this.cycleSource = Objects.requireNonNull(cycleSource, "cycleSource is required");
        this.defaultValue = Objects.requireNonNull(defaultValue, "defaultValue is required");
        this.value = defaultValue;
    }

    /**
     * Write the frame value for the current cycle.
     *
     * <p>If called multiple times in the same cycle, the last write wins.</p>
     *
     * @param value fresh value for the current cycle; must be non-null
     */
    public void set(T value) {
        this.value = Objects.requireNonNull(value, "value is required");
        this.writtenCycle = cycleSource.getAsLong();
    }

    /**
     * @return the fresh value when written in the current cycle; otherwise the default value.
     */
    public T get() {
        return isFresh() ? value : defaultValue;
    }

    /**
     * @return true when this slot was written during the current loop cycle.
     */
    public boolean isFresh() {
        return writtenCycle == cycleSource.getAsLong();
    }

    /**
     * @return the fallback/default value returned when the slot is not fresh.
     */
    public T defaultValue() {
        return defaultValue;
    }

    /**
     * Clear freshness and restore the stored value to the default.
     */
    public void reset() {
        value = defaultValue;
        writtenCycle = Long.MIN_VALUE;
    }

    /**
     * Emit a compact debug summary.
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "frame" : prefix;
        dbg.addData(p + ".class", "FrameValue")
                .addData(p + ".fresh", isFresh())
                .addData(p + ".value", get())
                .addData(p + ".defaultValue", defaultValue)
                .addData(p + ".writtenCycle", writtenCycle);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "FrameValue{" + "fresh=" + isFresh() + ", value=" + get()
                + ", defaultValue=" + defaultValue + ", writtenCycle=" + writtenCycle + "}";
    }
}
