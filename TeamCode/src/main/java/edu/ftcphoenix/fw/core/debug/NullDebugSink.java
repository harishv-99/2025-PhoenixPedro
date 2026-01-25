package edu.ftcphoenix.fw.core.debug;

/**
 * {@link DebugSink} implementation that silently discards all debug output.
 *
 * <h2>Purpose</h2>
 * <p>
 * {@code NullDebugSink} is a classic "null object" implementation of
 * {@link DebugSink}. It allows framework and robot code to call
 * {@code addData(...)} and {@code addLine(...)} without having to check for
 * {@code null} on every call when debugging is disabled.
 * </p>
 *
 * <p>Instead of writing:</p>
 *
 * <pre>{@code
 * public void debugDump(DebugSink sink, String prefix) {
 *     if (sink == null) return;
 *     sink.addData(prefix + ".axial",   lastCmd.axial);
 *     sink.addData(prefix + ".lateral", lastCmd.lateral);
 *     sink.addData(prefix + ".omega",   lastCmd.omega);
 * }
 * }</pre>
 *
 * <p>you can create a {@code NullDebugSink} and always pass a non-null sink:</p>
 *
 * <pre>{@code
 * DebugSink dbg = debugEnabled
 *         ? new FtcTelemetryDebugSink(telemetry)
 *         : NullDebugSink.INSTANCE;
 *
 * robot.debugDump(dbg);  // no null checks needed inside
 * }</pre>
 *
 * <p>
 * Because {@code NullDebugSink} is stateless and side-effect-free, a single
 * shared instance ({@link #INSTANCE}) is sufficient for the entire app.
 * </p>
 */
public final class NullDebugSink implements DebugSink {

    /**
     * Shared singleton instance.
     *
     * <p>
     * Since {@code NullDebugSink} performs no work and holds no state, there
     * is no need to allocate more than one instance. Use this constant instead
     * of creating new objects.
     * </p>
     */
    public static final NullDebugSink INSTANCE = new NullDebugSink();

    /**
     * Private constructor to enforce the singleton pattern.
     */
    private NullDebugSink() {
        // no-op
    }

    /**
     * Discard a key/value debug entry.
     *
     * <p>
     * This implementation ignores the provided key and value and simply
     * returns {@code this} to support fluent usage.
     * </p>
     *
     * @param key   debug key/name (ignored)
     * @param value value to record (ignored)
     * @return this sink, for fluent chaining
     */
    @Override
    public DebugSink addData(String key, Object value) {
        // no-op
        return this;
    }

    /**
     * Discard a numeric debug entry.
     *
     * <p>
     * This overrides {@link DebugSink#addData(String, double)} to avoid any
     * unnecessary boxing or formatting work; it simply ignores the value and
     * returns {@code this}.
     * </p>
     *
     * @param key   debug key/name (ignored)
     * @param value numeric value to record (ignored)
     * @return this sink, for fluent chaining
     */
    @Override
    public DebugSink addData(String key, double value) {
        // no-op
        return this;
    }

    /**
     * Discard a numeric debug entry with an explicit format string.
     *
     * <p>
     * This overrides {@link DebugSink#addData(String, String, double)} and
     * ignores all arguments, performing no work and returning {@code this}.
     * </p>
     *
     * @param key   debug key/name (ignored)
     * @param fmt   format string (ignored)
     * @param value numeric value to record (ignored)
     * @return this sink, for fluent chaining
     */
    @Override
    public DebugSink addData(String key, String fmt, double value) {
        // no-op
        return this;
    }

    /**
     * Discard a free-form debug line.
     *
     * <p>
     * This implementation ignores the provided text and simply returns
     * {@code this} to support fluent usage.
     * </p>
     *
     * @param text debug line text (ignored)
     * @return this sink, for fluent chaining
     */
    @Override
    public DebugSink addLine(String text) {
        // no-op
        return this;
    }
}
