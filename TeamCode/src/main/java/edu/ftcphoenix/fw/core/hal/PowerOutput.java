package edu.ftcphoenix.fw.core.hal;

/**
 * Generic command-only power channel for an actuator.
 *
 * <p>{@code PowerOutput} is the lowest-level scalar command seam used by Phoenix plants when the
 * actuation lever is a normalized power-like signal. Typical examples include FTC DC motors driven
 * with {@code setPower(-1..+1)} and continuous-rotation servos treated as motors.</p>
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
     * <p>The exact interpretation of {@code power} depends on the implementation, but callers should
     * generally treat {@code -1.0} as full reverse, {@code 0.0} as neutral, and {@code +1.0} as
     * full forward. Implementations are expected to clamp or otherwise sanitize invalid inputs
     * before talking to hardware.</p>
     *
     * @param power normalized power command, usually in {@code [-1.0, +1.0]}
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
