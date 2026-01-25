package edu.ftcphoenix.fw.core.debug;

import java.util.Locale;

/**
 * Simple, framework-level debug sink for recording key/value diagnostics.
 *
 * <h2>Purpose</h2>
 * <p>
 * {@code DebugSink} provides a tiny abstraction for emitting debug information
 * from core framework classes (drive, sensing, tasks, plants) without taking
 * a direct dependency on any particular output mechanism such as:
 * </p>
 *
 * <ul>
 *   <li>FTC {@code Telemetry}</li>
 *   <li>Log files</li>
 *   <li>{@code System.out} (unit tests)</li>
 * </ul>
 *
 * <p>
 * The core idea:
 * </p>
 *
 * <ul>
 *   <li>Core packages (e.g., {@code fw.drive}, {@code fw.sensing}) only know
 *       about {@code DebugSink}.</li>
 *   <li>Adapter code (e.g., {@code fw.adapters.ftc}) wraps FTC
 *       {@code Telemetry} or other backends in a {@code DebugSink}
 *       implementation.</li>
 *   <li>Subsystems and robots decide when to call {@code debugDump(...)}
 *       methods that accept a {@code DebugSink}.</li>
 * </ul>
 *
 * <p>
 * This keeps the framework layering clean while still allowing a systematic
 * way to inspect internal state during development.
 * </p>
 *
 * <h2>Typical usage</h2>
 *
 * <h3>1. In a core class (e.g., GamepadDriveSource)</h3>
 *
 * <pre>{@code
 * public void debugDump(DebugSink sink, String prefix) {
 *     if (sink == null) return;
 *     sink.addData(prefix + ".deadband", params.deadband)
 *         .addData(prefix + ".translateExpo", params.translateExpo)
 *         .addData(prefix + ".rotateExpo", params.rotateExpo);
 * }
 * }</pre>
 *
 * <h3>2. In an FTC adapter (Telemetry-backed DebugSink)</h3>
 *
 * <pre>{@code
 * public final class FtcTelemetryDebugSink implements DebugSink {
 *     private final Telemetry telemetry;
 *
 *     public FtcTelemetryDebugSink(Telemetry telemetry) {
 *         this.telemetry = telemetry;
 *     }
 *
 *     @Override
 *     public DebugSink kv(String key, Object value) {
 *         telemetry.addData(key, value);
 *         return this;
 *     }
 *
 *     @Override
 *     public DebugSink line(String text) {
 *         telemetry.addLine(text);
 *         return this;
 *     }
 * }
 * }</pre>
 *
 * <h3>3. In a TeleOp shell or PhoenixRobot</h3>
 *
 * <pre>{@code
 * DebugSink dbg = new FtcTelemetryDebugSink(telemetry);
 *
 * stickDrive.debugDump(dbg, "drive.sticks");
 * drivebase.debugDump(dbg, "drive.base");
 *
 * telemetry.update();
 * }</pre>
 *
 * <p>
 * In unit tests, you can provide a different implementation that writes to
 * logs or captures entries in memory for assertions.
 * </p>
 */
public interface DebugSink {

    /**
     * Record a single key/value debug entry.
     *
     * <p>
     * Implementations may choose to display these entries immediately (e.g.,
     * via FTC telemetry) or accumulate them for later flushing.
     * </p>
     *
     * <p>
     * The {@code key} is typically a short, dot-separated name that can be
     * grouped by subsystem, for example:
     * </p>
     *
     * <ul>
     *   <li>{@code "drive.axial"}</li>
     *   <li>{@code "shooter.targetRps"}</li>
     *   <li>{@code "arm.angleRad"}</li>
     * </ul>
     *
     * <p>
     * Returning {@code this} allows for simple fluent usage:
     * </p>
     *
     * <pre>{@code
     * sink.addData("drive.axial", cmd.axial)
     *     .addData("drive.lateral", cmd.lateral)
     *     .addData("drive.omega", cmd.omega);
     * }</pre>
     *
     * @param key   debug key/name
     * @param value value to record (implementations may call {@code toString()})
     * @return this sink, for fluent chaining
     */
    DebugSink addData(String key, Object value);

    /**
     * Convenience overload for numeric values.
     *
     * <p>By default this just boxes the double and delegates to
     * {@link #addData(String, Object)}, but it also provides a single choke
     * point if you later want consistent formatting for all doubles
     * (e.g., rounding to a fixed number of decimals).</p>
     */
    default DebugSink addData(String key, double value) {
        return addData(key, Double.valueOf(value));
    }

    /**
     * Context-specific numeric formatting.
     *
     * <p>By default this uses {@link String#format(Locale, String, Object...)}
     * with {@link Locale#US} to produce a String and delegates to
     * {@link #addData(String, Object)}. Implementations that have their own
     * formatting API (like FTC Telemetry) can override this method to use it
     * directly.</p>
     */
    default DebugSink addData(String key, String fmt, double value) {
        String formatted = String.format(Locale.US, fmt, value);
        return addData(key, formatted);
    }

    /**
     * Record a free-form debug line.
     *
     * <p>
     * This is useful for short summaries or headings when grouping related
     * key/value entries.
     * </p>
     *
     * <pre>{@code
     * sink.addLine("Shooter")
     *     .addData("shooter.targetRps", targetRps)
     *     .addData("shooter.measuredRps", measuredRps);
     * }</pre>
     *
     * @param text debug line text
     * @return this sink, for fluent chaining
     */
    DebugSink addLine(String text);
}
