package edu.ftcphoenix.fw.core.control;

/**
 * Generic PID controller interface for real-time robot control.
 *
 * <p>This interface is intentionally small and opinionated:
 * <ul>
 *   <li>It is <em>error-centric</em>: callers pass in the current error
 *       (setpoint - measurement) and a time step.</li>
 *   <li>It returns a single control output (for example, motor power,
 *       angular velocity command, or some other effort).</li>
 *   <li>Implementations may maintain internal state (integral term,
 *       previous error for derivative term, filters, etc.).</li>
 * </ul>
 *
 * <p>The goal is to provide a single, reusable contract for feedback control
 * across the framework:</p>
 *
 * <ul>
 *   <li>Tag aiming (using {@code AprilTagSensor} bearing as error).</li>
 *   <li>Flywheel/shooter RPM controllers.</li>
 *   <li>Heading controllers for drive.</li>
 *   <li>Any other "drive error towards zero over time" behaviors.</li>
 * </ul>
 *
 * <h2>Typical usage</h2>
 *
 * <p>In a loop that runs every cycle (for example, in TeleOp or autonomous):</p>
 *
 * <pre>{@code
 * // Example: aiming at a target angle (in radians).
 * double targetAngle = 0.0;              // radians, "straight ahead"
 * double currentAngle = imu.getHeading(); // radians
 * double error = targetAngle - currentAngle;
 *
 * double output = pid.update(error, dtSec);
 * drive.turn(output); // interpret output as turn command
 * }</pre>
 *
 * <p>For advanced controllers, callers may choose to:</p>
 * <ul>
 *   <li>Call {@link #reset()} when changing modes or after large setpoint
 *       jumps, to clear integral and derivative state.</li>
 *   <li>Wrap this interface into higher-level classes that manage setpoints
 *       and sensors (for example, a drive guidance / auto-aim helper).</li>
 * </ul>
 */
public interface PidController {

    /**
     * Compute the next control output given the current error and loop
     * time step.
     *
     * <p>Implementations may internally apply proportional, integral,
     * derivative, and filtering logic. Callers are responsible for:
     * <ul>
     *   <li>Supplying the error with the correct sign and units.</li>
     *   <li>Passing a reasonable estimate of the loop time step in seconds.</li>
     *   <li>Clamping or interpreting the returned output appropriately for
     *       their actuators (for example, clamping to [-1, +1] for power).</li>
     * </ul>
     *
     * @param error current control error (for example, setpoint - measurement)
     * @param dtSec time step in seconds since the previous {@code update} call
     * @return control output (units are caller-defined)
     */
    double update(double error, double dtSec);

    /**
     * Optional lifecycle hook to clear any internal state kept by the PID
     * implementation (for example, integral accumulator or previous-error
     * history).
     *
     * <p>Callers should invoke this when:
     * <ul>
     *   <li>Starting a new control task.</li>
     *   <li>Changing setpoints abruptly.</li>
     *   <li>Recovering from a condition where the controller was saturated
     *       for a long time.</li>
     * </ul>
     *
     * <p>Implementations that do not maintain state may safely ignore this.</p>
     */
    default void reset() {
        // default: no-op
    }
}
