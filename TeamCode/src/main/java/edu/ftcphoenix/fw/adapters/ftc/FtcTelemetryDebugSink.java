package edu.ftcphoenix.fw.adapters.ftc;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.ftcphoenix.fw.debug.DebugSink;

/**
 * {@link DebugSink} implementation backed by FTC {@link Telemetry}.
 *
 * <h2>Purpose</h2>
 * <p>
 * This class adapts the framework-level {@link DebugSink} interface to the FTC
 * SDK {@link Telemetry} API so that core framework classes (drive, sensing,
 * tasks, plants) can emit debug information without depending directly on the
 * FTC SDK.
 * </p>
 *
 * <p>
 * Typical usage in a TeleOp shell or {@code PhoenixRobot}:
 * </p>
 *
 * <pre>{@code
 * public void loop() {
 *     DebugSink dbg = debugEnabled
 *             ? new FtcTelemetryDebugSink(telemetry)
 *             : NullDebugSink.INSTANCE;
 *
 *     robot.debugDump(dbg);
 *
 *     telemetry.update();
 * }
 * }</pre>
 *
 * <p>
 * The intent is that:
 * </p>
 * <ul>
 *   <li>Core code uses {@link DebugSink} only.</li>
 *   <li>FTC adapter code wraps {@link Telemetry} as a {@link DebugSink} via
 *       this class.</li>
 *   <li>Tests can provide their own {@link DebugSink} implementations that
 *       write to logs or capture entries for assertions.</li>
 * </ul>
 */
public final class FtcTelemetryDebugSink implements DebugSink {

    private final Telemetry telemetry;

    /**
     * Create a new telemetry-backed {@link DebugSink}.
     *
     * @param telemetry FTC telemetry instance (typically from the OpMode);
     *                  must not be {@code null}
     */
    public FtcTelemetryDebugSink(Telemetry telemetry) {
        if (telemetry == null) {
            throw new IllegalArgumentException("telemetry must not be null");
        }
        this.telemetry = telemetry;
    }

    /**
     * Record a single key/value debug entry by delegating to
     * {@link Telemetry#addData(String, Object)}.
     *
     * <p>
     * Keys are typically short, dot-separated names such as:
     * {@code "drive.axial"}, {@code "shooter.targetRps"}, etc.
     * </p>
     *
     * <p>
     * Returns {@code this} to allow fluent chaining:
     * </p>
     *
     * <pre>{@code
     * dbg.addData("drive.axial",   cmd.axial)
     *    .addData("drive.lateral", cmd.lateral)
     *    .addData("drive.omega",   cmd.omega);
     * }</pre>
     *
     * <p>
     * Note: for primitive doubles, {@link #addData(String, double)} and
     * {@link #addData(String, String, double)} are also available and may
     * provide more convenient or context-specific formatting.
     * </p>
     *
     * @param key   debug key/name
     * @param value value to record
     * @return this sink, for fluent chaining
     */
    @Override
    public DebugSink addData(String key, Object value) {
        telemetry.addData(key, value);
        return this;
    }

    /**
     * Default numeric overload with fixed 3-decimal formatting.
     *
     * <p>
     * This overrides {@link DebugSink#addData(String, double)} to use
     * {@code "%.3f"} as the format string. This is a good default for most
     * normalized values (powers, velocities, etc.) on Driver Station telemetry.
     * </p>
     *
     * <p>
     * For context-specific formatting (for example, distances in inches with
     * one decimal place), use
     * {@link #addData(String, String, double)} instead:
     * </p>
     *
     * <pre>{@code
     * dbg.addData("range.in", "%.1f", distanceInches);
     * }</pre>
     *
     * @param key   debug key/name
     * @param value numeric value to record
     * @return this sink, for fluent chaining
     */
    @Override
    public DebugSink addData(String key, double value) {
        telemetry.addData(key, "%.3f", value);
        return this;
    }

    /**
     * Context-specific numeric formatting.
     *
     * <p>
     * This overrides the default {@link DebugSink#addData(String, String, double)}
     * implementation and delegates directly to
     * {@link Telemetry#addData(String, String, Object...)} so callers can
     * specify a format string appropriate for the quantity being displayed.
     * </p>
     *
     * <p>Examples:</p>
     * <pre>{@code
     * // Distance in inches with one decimal place:
     * dbg.addData("range.in", "%.1f", distanceInches);
     *
     * // Angle in degrees with one decimal place:
     * dbg.addData("arm.angleDeg", "%.1f", angleDeg);
     * }</pre>
     *
     * @param key   debug key/name
     * @param fmt   {@link String#format(String, Object...)}-style format string
     *              (e.g. {@code "%.1f"})
     * @param value numeric value to record
     * @return this sink, for fluent chaining
     */
    @Override
    public DebugSink addData(String key, String fmt, double value) {
        telemetry.addData(key, fmt, value);
        return this;
    }

    /**
     * Record a free-form debug line by delegating to
     * {@link Telemetry#addLine(String)}.
     *
     * <p>
     * This is useful for headings or short summaries; individual values
     * should generally use {@link #addData(String, Object)} or
     * {@link #addData(String, double)} / {@link #addData(String, String, double)}
     * so they remain easy to group and filter.
     * </p>
     *
     * @param text debug line text
     * @return this sink, for fluent chaining
     */
    @Override
    public DebugSink addLine(String text) {
        telemetry.addLine(text);
        return this;
    }
}
