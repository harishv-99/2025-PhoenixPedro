package edu.ftcphoenix.fw.core.hal;

/**
 * Generic command-only power channel for an actuator.
 *
 * <p>{@code PowerOutput} is the lowest-level scalar command seam used by Phoenix plants when the
 * actuation lever is a normalized power-like signal. Typical examples include FTC DC motors driven
 * with {@code setPower(-1..+1)} and continuous-rotation servos treated as motors. Direct Phoenix
 * power Plants enforce that normalized range before calling this boundary. Lower-level control
 * paths may rely on the output adapter's defensive saturation.</p>
 *
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * PowerOutput intake = FtcHardware.motorPower(hardwareMap, "intake", Direction.FORWARD);
 * intake.setPower(0.75);
 * }</pre>
 */
public interface PowerOutput {

    /**
     * Command the actuator with a normalized power value.
     *
     * <p>The exact physical interpretation depends on the implementation, but {@code -1.0} is full
     * reverse, {@code 0.0} is neutral, and {@code +1.0} is full forward in the logical command
     * domain. Implementations are expected to clamp or otherwise sanitize invalid inputs before
     * talking to hardware.</p>
     *
     * @param power normalized power request, usually in {@code [-1.0, +1.0]}
     */
    void setPower(double power);

    /**
     * Return the most recent command sent through this output.
     *
     * <p>This is a cached command value, not a physical measurement. It answers "what did Phoenix
     * ask the actuator to do?" rather than "what did the mechanism actually do?".</p>
     *
     * @return most recently commanded power value
     */
    double getCommandedPower();

    /**
     * Convenience helper that stops the output using the implementation's natural zero command.
     *
     * <p>The default behavior is equivalent to {@code setPower(0.0)}. Implementations may override
     * this when zero has a more specific meaning, such as brake versus coast.</p>
     */
    default void stop() {
        setPower(0.0);
    }
}
