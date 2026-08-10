package edu.ftcphoenix.fw.core.hal;

/**
 * Generic command-only position channel for an actuator.
 *
 * <p>{@code PositionOutput} answers one question only: <em>what position command should be sent to
 * this actuator?</em> It does not model measurement or feedback. Feedback is handled separately
 * through {@code ScalarSource} so the same readback abstraction can be used for internal encoders,
 * external encoders, potentiometers, and subsystem-defined mechanism units.</p>
 *
 * <p>The native position domain belongs to the concrete output. Hardware-neutral mapped Plants
 * require their realized Plant-to-native command to be finite before calling this seam, but they
 * do not assume an FTC Servo range or an encoder-integer representation. Raw callers remain
 * responsible for the concrete output's documented native domain.</p>
 *
 * <h2>Typical examples</h2>
 *
 * <ul>
 *   <li>FTC standard servo: position domain {@code 0.0 .. 1.0}</li>
 *   <li>FTC motor native position command: encoder ticks</li>
 * </ul>
 *
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * PositionOutput wrist = FtcHardware.servoPosition(hardwareMap, "wrist", Direction.FORWARD);
 * wrist.setPosition(0.72);
 *
 * // Measurement, if needed, comes from a source rather than from PositionOutput itself.
 * ScalarSource armTicks = FtcSensors.motorPositionTicks(hardwareMap, "armEncoder");
 * }</pre>
 */
public interface PositionOutput {

    /**
     * Command the actuator to the requested position in the output's native position units.
     *
     * @param position desired finite position command in the output's native units and domain
     */
    void setPosition(double position);

    /**
     * Return the most recent position command sent through this output.
     *
     * <p>This is a cached command value, not a measurement. For physical readback, use an
     * appropriate {@code ScalarSource}.</p>
     *
     * @return most recently commanded position
     */
    double getCommandedPosition();

    /**
     * Convenience stop behavior for position-commanded outputs.
     *
     * <p>The default implementation simply re-commands the most recent target, which is appropriate
     * for many set-and-hold actuators such as standard servos. Implementations may override this if
     * the underlying device exposes a more explicit stop behavior. An adapter that can be stopped
     * before its first {@link #setPosition(double)} must either return a safe, truthful initial
     * command from {@link #getCommandedPosition()} or override this method with a safe pre-command
     * action such as doing nothing; it must not invent and submit an arbitrary position.</p>
     */
    default void stop() {
        setPosition(getCommandedPosition());
    }
}
