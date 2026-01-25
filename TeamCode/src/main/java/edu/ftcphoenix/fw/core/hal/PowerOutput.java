package edu.ftcphoenix.fw.core.hal;

/**
 * Generic "power" control channel for an actuator.
 *
 * <p>This is the lowest-level interface that plants see for
 * anything driven by a scalar power command:
 *
 * <ul>
 *   <li>DC motors driven with {@code setPower(-1..+1)}</li>
 *   <li>Continuous rotation servos treated as "motors"</li>
 *   <li>Other platforms' actuators that conceptually accept a
 *       dimensionless power signal</li>
 * </ul>
 *
 * <h2>Design goals</h2>
 * ...
 */
public interface PowerOutput {

    /**
     * Command the actuator with a normalized power value.
     *
     * <p>The exact interpretation of "power" depends on the
     * implementation, but the general expectation is:</p>
     *
     * <ul>
     *   <li>{@code -1.0} = full reverse</li>
     *   <li>{@code 0.0} = stop / neutral</li>
     *   <li>{@code +1.0} = full forward</li>
     * </ul>
     *
     * <p>Implementations should clamp or otherwise sanitize the input
     * to a valid range before applying it to hardware.</p>
     *
     * @param power normalized power in the range {@code [-1.0, +1.0]}
     */
    void setPower(double power);

    /**
     * Returns the most recently <b>commanded</b> power value.
     *
     * <p>This is a cached command value, not a sensor reading. It
     * reflects "what we asked the actuator to do", which may differ
     * from the actual physical behavior if, for example, the mechanism
     * is stalled or saturated.</p>
     *
     * @return last commanded power value
     */
    double getCommandedPower();

    /**
     * Convenience method to set power to zero.
     *
     * <p>Implementations may override this if zero has a special
     * meaning (e.g., brake vs. coast), but the default behavior is
     * equivalent to {@code setPower(0.0)}.</p>
     */
    default void stop() {
        setPower(0.0);
    }
}
