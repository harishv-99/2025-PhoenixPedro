package edu.ftcsushi.fw.core.hal;

/**
 * Generic command-only velocity channel for an actuator.
 *
 * <p>{@code VelocityOutput} models actuator-side velocity commands only. Feedback is handled via
 * {@code ScalarSource} so device-managed velocity control and framework-regulated velocity control
 * can share the same measurement abstraction.</p>
 *
 * <p>Velocity unit mappings are zero-preserving: {@code 0.0} means zero velocity in every unit
 * system. Hardware-neutral mapped Plants reject a non-finite realized command before calling this
 * seam. A concrete output may impose additional native-unit limits, but this generic interface
 * does not invent a universal hardware-independent maximum velocity.</p>
 *
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * VelocityOutput flywheel = FtcHardware.motorVelocity(hardwareMap, "flywheel", Direction.FORWARD);
 * flywheel.setVelocity(1750.0); // ticks/sec for FTC motors
 * }</pre>
 */
public interface VelocityOutput {

    /**
     * Command the actuator to the requested velocity in the output's native velocity units.
     *
     * @param velocity desired finite velocity command in the output's native units; {@code 0.0}
     *                 requests zero velocity
     */
    void setVelocity(double velocity);

    /**
     * Return the most recent velocity command sent through this output.
     *
     * <p>This is a cached command value rather than a measured velocity. Use an appropriate
     * {@code ScalarSource} for readback.</p>
     *
     * @return most recently commanded velocity
     */
    double getCommandedVelocity();

    /**
     * Convenience stop behavior for velocity-commanded outputs.
     *
     * <p>The default implementation commands zero velocity.</p>
     */
    default void stop() {
        setVelocity(0.0);
    }
}
