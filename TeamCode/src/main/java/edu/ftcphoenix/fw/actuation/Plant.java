package edu.ftcphoenix.fw.actuation;

import edu.ftcphoenix.fw.debug.DebugSink;

/**
 * A generic setpoint-driven mechanism.
 *
 * <p>A {@code Plant} is the low-level "sink" that accepts a scalar target
 * and drives one or more hardware outputs (motors, servos, etc.) toward
 * that target using whatever control logic it chooses (open-loop, PID,
 * vendor velocity control, feedforward, etc.).</p>
 *
 * <h2>Semantic categories</h2>
 *
 * <p>Each concrete plant should document which of these categories it uses:</p>
 *
 * <ul>
 *   <li><b>Power</b>: target is a normalized power command (e.g. {@code -1..+1})
 *       sent directly to a motor/CR-servo output.</li>
 *   <li><b>Position</b>: target is a desired position in some native units
 *       (servo position, encoder ticks, etc.).</li>
 *   <li><b>Velocity</b>: target is a desired velocity/speed in native units
 *       (ticks per second, RPM, etc.).</li>
 * </ul>
 *
 * <p>Plants are intentionally simple: they do not know about tasks or macros,
 * but they may implement local control logic, clamping, rate limiting, or
 * other behavior that makes sense for a given mechanism.</p>
 */
public interface Plant {

    /**
     * Set the current target for this plant.
     *
     * <p>The interpretation of {@code target} depends on the concrete plant
     * type (power, position, velocity, etc.) and should be documented by the
     * implementation.</p>
     *
     * <p>This method should be cheap to call; it is expected that high-level
     * code may update targets frequently (e.g., each loop for joystick-driven
     * drivebases).</p>
     *
     * @param target new target value in the plant's native units
     */
    void setTarget(double target);

    /**
     * @return the most recently commanded target value.
     */
    double getTarget();

    /**
     * Update the plant's internal state for the current loop.
     *
     * <p>Typical responsibilities include:</p>
     *
     * <ul>
     *   <li>Running closed-loop control (PID, velocity control, etc.)</li>
     *   <li>Applying rate limits or filters to the commanded output</li>
     *   <li>Forwarding the resulting command to one or more hardware outputs</li>
     * </ul>
     *
     * <p>This method should be called once per loop with the elapsed time
     * since the previous call.</p>
     *
     * @param dtSec time since the last update call, in seconds (non-negative)
     */
    void update(double dtSec);

    /**
     * Reset any internal state used by the plant (integrators, filters, etc.).
     *
     * <p>This is typically called at the beginning of a mode (e.g. TeleOp,
     * autonomous) or when a mechanism is reinitialized.</p>
     *
     * <p>Default implementation does nothing.</p>
     */
    default void reset() {
        // Default: no internal state to clear.
    }

    /**
     * Immediately stop driving this plant in the most reasonable way for
     * its underlying hardware.
     *
     * <p>Concrete plants <b>must</b> implement this. A typical implementation
     * will forward to one or more HAL outputs, for example:</p>
     *
     * <ul>
     *   <li>Power plants: call {@code powerOutput.stop()} (equivalent to power 0)</li>
     *   <li>Velocity plants: call {@code velocityOutput.stop()} (velocity 0)</li>
     *   <li>Position plants:
     *     <ul>
     *       <li>Servos: usually a no-op or re-command the current position</li>
     *       <li>Motors: call {@code positionOutput.stop()}, which may switch
     *           modes and cut power to stop chasing the old target</li>
     *     </ul>
     *   </li>
     *   <li>Decorator plants (rate limiters, interlocks): forward to the
     *       wrapped plant's {@link #stop()}.</li>
     * </ul>
     *
     * <p>Purely virtual or simulated plants <em>may</em> choose to implement
     * this as a no-op, but they must still provide an implementation.</p>
     */
    void stop();

    /**
     * @return {@code true} if this plant considers itself "at" its current
     * target setpoint. Implementations that do not track this can
     * simply return {@code false} or {@code true} unconditionally.
     */
    default boolean atSetpoint() {
        return false;
    }

    /**
     * Indicates whether this plant has meaningful feedback for determining
     * when it has reached its current target setpoint.
     *
     * <p>Examples of feedback-capable plants include velocity or position
     * controllers that compare a measured value against the commanded target
     * with some tolerance. For simple open-loop plants (e.g., plain power
     * outputs or "fire-and-forget" servos), this may return {@code false},
     * and callers should prefer time-based completion instead of relying on
     * {@link #atSetpoint()}.</p>
     *
     * <p>The default implementation returns {@code false}. Implementations
     * that override {@link #atSetpoint()} with a sensor-based definition
     * should also override this to return {@code true}.</p>
     *
     * @return {@code true} if this plant exposes a meaningful
     *         {@link #atSetpoint()} value; {@code false} otherwise
     */
    default boolean hasFeedback() {
        return false;
    }

    /**
     * Optional debug hook: emit a compact summary of this plant's state.
     *
     * <p>The default implementation writes only the target and atSetpoint
     * flag. Implementations are encouraged to override this to include
     * additional mechanism-specific details (sensor feedback, errors,
     * internal controller state, etc.).</p>
     *
     * @param dbg    debug sink (may be {@code null}; if null, no output is produced)
     * @param prefix base key prefix, e.g. "intake", "shooter", or "arm"
     */
    default void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "plant" : prefix;
        dbg.addData(p + ".target", getTarget())
                .addData(p + ".atSetpoint", atSetpoint());
    }
}
