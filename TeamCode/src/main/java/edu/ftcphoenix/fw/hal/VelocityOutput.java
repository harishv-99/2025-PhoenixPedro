package edu.ftcphoenix.fw.hal;

/**
 * Generic "velocity" control channel for an actuator.
 *
 * <p>This interface represents a single degree of freedom driven by a
 * scalar velocity in the actuator's <b>native units</b>. The exact
 * meaning of the velocity value depends on the implementation:</p>
 *
 * <ul>
 *   <li>Motor with encoder:
 *       <ul>
 *         <li>Domain: encoder ticks per second (or similar native units).</li>
 *         <li>Typically backed by {@code DcMotorEx#setVelocity(double)}.</li>
 *       </ul>
 *   </li>
 * </ul>
 *
 * <p>Higher-level code should use this interface rather than accessing
 * SDK-specific objects directly, to keep mechanism code portable and
 * testable.</p>
 */
public interface VelocityOutput {

    /**
     * Command the actuator to run at the given target velocity in its
     * native units.
     *
     * <p>For a DC motor with an encoder, this is typically in ticks per
     * second or another platform-defined velocity unit.</p>
     *
     * <p>Implementations may clamp the input to a valid range before
     * applying it to the underlying hardware.</p>
     *
     * @param velocity desired target velocity (native units)
     */
    void setVelocity(double velocity);

    /**
     * Returns the most recently <b>commanded</b> target velocity that was
     * passed to {@link #setVelocity(double)}, in native units.
     *
     * <p>This is a cached command value, not necessarily a sensor reading.
     * It reflects "what we asked the actuator to do", which may differ from
     * the actual measured velocity if the mechanism is saturating, stalled,
     * or still accelerating.</p>
     *
     * @return last commanded target velocity (native units)
     */
    double getCommandedVelocity();

    /**
     * Returns the most recently <b>measured</b> velocity of the actuator, in
     * its native units, if available.
     *
     * <p>Implementations with access to a real sensor (for example,
     * a motor with an encoder) should override this to return the
     * current measured velocity. Implementations without a sensor
     * may choose to approximate this or, in extreme cases, simply
     * return {@link #getCommandedVelocity()} if no better information
     * is available.</p>
     *
     * @return measured velocity in native units, if available; otherwise
     *         an implementation-defined approximation
     */
    double getMeasuredVelocity();

    /**
     * Convenience method to "stop" motion in the most reasonable way for
     * this actuator.
     *
     * <p>The default behavior is to command a velocity of zero:</p>
     *
     * <pre>{@code
     * stop()  ==  setVelocity(0.0);
     * }</pre>
     *
     * <p>Implementations may override this if zero velocity has a special
     * meaning or requires additional state changes, but the default
     * preserves the intent of stopping the actuator.</p>
     */
    default void stop() {
        setVelocity(0.0);
    }
}
