package edu.ftcphoenix.fw.supervisor;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;

/**
 * Small supervisor-side value slot that remembers the last explicitly-set value until changed.
 *
 * <p>This is the "held" layer-1 input shape used by robot-owned capabilities and supervisors:
 * a caller writes a value once, and that value keeps applying on later loops until another call
 * changes it. Typical examples include selected setpoints, enable flags, and remembered operating
 * modes.</p>
 *
 * <p>{@code HeldValue} is intentionally tiny. It does not know anything about game semantics,
 * execution policy, or plants. It simply remembers the current non-null value and restores the
 * initial value on {@link #reset()}.</p>
 *
 * @param <T> stored value type; values must be non-null
 */
public final class HeldValue<T> {

    private final T initialValue;
    private T value;

    /**
     * Create a held value initialized to {@code initialValue}.
     *
     * @param initialValue initial/current value; must be non-null
     */
    public HeldValue(T initialValue) {
        this.initialValue = Objects.requireNonNull(initialValue, "initialValue is required");
        this.value = initialValue;
    }

    /**
     * @return the current held value.
     */
    public T get() {
        return value;
    }

    /**
     * Replace the current held value.
     *
     * @param value new current value; must be non-null
     */
    public void set(T value) {
        this.value = Objects.requireNonNull(value, "value is required");
    }

    /**
     * @return the initial value restored by {@link #reset()}.
     */
    public T initialValue() {
        return initialValue;
    }

    /**
     * Restore the initial value.
     */
    public void reset() {
        value = initialValue;
    }

    /**
     * Emit a compact debug summary.
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "held" : prefix;
        dbg.addData(p + ".class", "HeldValue")
                .addData(p + ".value", value)
                .addData(p + ".initialValue", initialValue);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return "HeldValue{" + "value=" + value + ", initialValue=" + initialValue + "}";
    }
}
