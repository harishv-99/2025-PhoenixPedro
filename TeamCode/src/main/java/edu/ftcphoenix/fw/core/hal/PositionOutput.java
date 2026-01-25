package edu.ftcphoenix.fw.core.hal;

/**
 * Generic "position" control channel for an actuator.
 *
 * <p>This interface represents a single degree of freedom with a
 * scalar position in the actuator's <b>native units</b>. The exact
 * meaning of the position value depends on the implementation:</p>
 *
 * <ul>
 *   <li>Standard FTC servo:
 *       <ul>
 *         <li>Domain: {@code 0.0 .. 1.0}</li>
 *         <li>Matches {@link com.qualcomm.robotcore.hardware.Servo#setPosition(double)}.</li>
 *       </ul>
 *   </li>
 *   <li>Motor with encoder in position mode:
 *       <ul>
 *         <li>Domain: encoder ticks (or a similar native unit).</li>
 *         <li>May be backed by a {@code DcMotorEx} in
 *             {@code RUN_TO_POSITION} mode.</li>
 *       </ul>
 *   </li>
 * </ul>
 *
 * <p>Higher-level code should use this interface rather than accessing
 * SDK-specific objects directly, to keep mechanism code portable and
 * testable.</p>
 */
public interface PositionOutput {

    /**
     * Command the actuator to move to the given target position in its
     * native units.
     *
     * <p>For a standard FTC servo this is typically in the range
     * {@code 0.0 .. 1.0}. For a motor in position-control mode, this
     * may be encoder ticks or another platform-defined unit.</p>
     *
     * <p>Implementations may clamp the input to a valid range before
     * applying it to the underlying hardware.</p>
     *
     * @param position desired target position (native units)
     */
    void setPosition(double position);

    /**
     * Returns the most recently <b>commanded</b> target position that was
     * passed to {@link #setPosition(double)}, in native units.
     *
     * <p>This is a cached command value, not necessarily a sensor reading.
     * It reflects "what we asked the actuator to do", which may differ from
     * the actual measured position if the mechanism is still moving or has
     * been blocked.</p>
     *
     * @return last commanded target position (native units)
     */
    double getCommandedPosition();

    /**
     * Returns the most recently <b>measured</b> position of the actuator, in
     * its native units, if available.
     *
     * <p>Implementations with access to a real sensor (for example,
     * a motor with an encoder) should override this to return the
     * current measured position. Implementations without a sensor
     * (for example, a standard servo where only the last commanded
     * position is known) may rely on the default implementation,
     * which simply returns {@link #getCommandedPosition()}.</p>
     *
     * @return measured position in native units, if available; otherwise
     *         the last commanded position
     */
    default double getMeasuredPosition() {
        return getCommandedPosition();
    }

    /**
     * Convenience method to "stop" motion in the most reasonable way for
     * this actuator.
     *
     * <p>The default behavior is to command the <b>current measured
     * position</b> as the new target:</p>
     *
     * <pre>{@code
     * stop()  ==  setPosition(getMeasuredPosition());
     * }</pre>
     *
     * <p>For a standard servo, where measured position is effectively the
     * last commanded position, this is usually a no-op and simply keeps
     * holding the current target. For a motor in {@code RUN_TO_POSITION}
     * backed by an encoder, implementations may override this method to
     * perform a more explicit stop (for example, switching modes and
     * cutting power), but the default preserves the intent of "stop
     * chasing the old target" by retargeting to "where we are now".</p>
     */
    default void stop() {
        setPosition(getMeasuredPosition());
    }
}
