package edu.ftcphoenix.fw.core.debug;

/**
 * {@link DebugSink} implementation that silently discards all debug output.
 *
 * <h2>Purpose</h2>
 * <p>
 * {@code NullDebugSink} is a classic "null object" implementation of
 * {@link DebugSink}. Debug producers must still accept a nullable sink and
 * return immediately, but a composition root may retain this singleton as a
 * stable non-null placeholder until or unless it installs a real output
 * adapter.
 * </p>
 *
 * <p>For example, retained root wiring can stay non-null while topic selection
 * remains an explicit call-site decision:</p>
 *
 * <pre>{@code
 * private DebugSink debugSink = NullDebugSink.INSTANCE;
 *
 * public void init() {
 *     if (telemetryDebugAvailable) {
 *         debugSink = new FtcTelemetryDebugSink(telemetry);
 *     }
 * }
 *
 * public void loop() {
 *     if (debugDrive) {
 *         drive.debugDump(debugSink, "drive");
 *     }
 * }
 * }</pre>
 *
 * <p>
 * This sink discards only the writes that reach it. It cannot avoid work a
 * producer already performed to traverse an object graph or compute arguments,
 * so optional topics should still be gated at their composition-root call site.
 * Because {@code NullDebugSink} is stateless and side-effect-free, a single
 * shared instance ({@link #INSTANCE}) is sufficient for the entire app.
 * </p>
 */
public final class NullDebugSink implements DebugSink {

    /**
     * Shared singleton instance.
     *
     * <p>
     * Since {@code NullDebugSink} performs no output work and holds no state,
     * there is no need to allocate more than one instance. Use this constant
     * instead of creating new objects, while still gating disabled diagnostic
     * producers at their call sites.
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
