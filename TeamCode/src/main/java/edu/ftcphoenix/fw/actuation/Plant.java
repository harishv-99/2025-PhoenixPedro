package edu.ftcphoenix.fw.actuation;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A generic single-degree-of-freedom mechanism sink.
 *
 * <p>A {@code Plant} is the low-level runtime object that owns a scalar target and drives one or
 * more outputs toward that target using whatever control strategy it chooses: direct power, direct
 * position command, device-managed velocity/position control, or a framework-owned regulator over a
 * raw actuator command.</p>
 *
 * <h2>Target semantics</h2>
 *
 * <p>Every concrete plant defines exactly one caller-facing target domain. Typical examples:</p>
 *
 * <ul>
 *   <li><b>Power plants</b>: target is normalized power, usually in {@code [-1.0, +1.0]}.</li>
 *   <li><b>Position plants</b>: target is a position in the plant's own scalar units
 *       (for example servo {@code 0..1}, encoder ticks, or a subsystem-defined mechanism unit).</li>
 *   <li><b>Velocity plants</b>: target is a velocity in the plant's own scalar units
 *       (for example encoder ticks/sec).</li>
 * </ul>
 *
 * <p>The framework intentionally keeps {@code Plant} simple: tasks set targets on plants, robots
 * call {@link #update(LoopClock)} once per loop, and each concrete implementation decides how to
 * drive the mechanism.</p>
 *
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * Plant flywheel = FtcActuators.plant(hardwareMap)
 *     .motor("flywheel", Direction.FORWARD)
 *     .velocity()
 *     .deviceManagedWithDefaults()
 *     .bounded(0.0, 2600.0)
 *     .nativeUnits()
 *     .velocityTolerance(50.0)
 *     .build();
 *
 * LoopClock clock = new LoopClock();
 *
 * // In init():
 * clock.reset(getRuntime());
 *
 * // In loop():
 * clock.update(getRuntime());
 * flywheel.setTarget(1800.0);   // ticks/sec for a motor velocity plant
 * flywheel.update(clock);
 *
 * telemetry.addData("flywheel.target", flywheel.getTarget());
 * telemetry.addData("flywheel.measured", flywheel.getMeasurement());
 * telemetry.addData("flywheel.atSetpoint", flywheel.atSetpoint());
 * }</pre>
 */
public interface Plant {

    /**
     * Set the current target for this plant.
     *
     * <p>The meaning of {@code target} depends on the concrete plant implementation and should be
     * documented by that implementation. Callers should treat the value as being in the plant's own
     * scalar units.</p>
     *
     * @param target new target value in the plant's units
     */
    void setTarget(double target);

    /**
     * Return the most recently commanded target value.
     *
     * @return current plant target in the plant's target units
     */
    double getTarget();

    /**
     * Update this plant once for the current loop.
     *
     * <p>Concrete plants typically use this to sample feedback, run any internal control logic,
     * cache status such as measurement/error/at-setpoint, and forward the resulting command to the
     * underlying actuator output.</p>
     *
     * <p>Robot code should call this once per loop after updating the shared {@link LoopClock}.</p>
     *
     * @param clock loop clock for the current cycle
     */
    void update(LoopClock clock);

    /**
     * Reset any transient internal state used by the plant.
     *
     * <p>This is intended for controller/filter lifecycle state (integrators, memoized values,
     * cached debounce windows, etc.). It should <b>not</b> be used to redefine the physical
     * coordinate frame of the mechanism.</p>
     */
    default void reset() {
        // default: no transient state to clear
    }

    /**
     * Immediately stop driving this plant in the most reasonable way for its implementation.
     */
    void stop();

    /**
     * Whether the plant considers itself at its current target based on the most recent
     * {@link #update(LoopClock)}.
     *
     * <p>Open-loop plants typically return {@code false} or {@code true} unconditionally. Feedback
     * plants should cache and report a meaningful answer based on their authoritative measurement.</p>
     *
     * @return {@code true} when the plant considers its last measured state close enough to its
     *         current target; otherwise {@code false}
     */
    default boolean atSetpoint() {
        return false;
    }

    /**
     * Whether this plant has meaningful feedback.
     *
     * <p>If {@code false}, callers should generally prefer time-based completion instead of relying
     * on {@link #atSetpoint()} or {@link #getMeasurement()}.</p>
     *
     * @return {@code true} if the plant exposes an authoritative measurement; otherwise {@code false}
     */
    default boolean hasFeedback() {
        return false;
    }

    /**
     * Return the plant's authoritative measurement from the most recent {@link #update(LoopClock)}.
     *
     * <p>This value is in the same units and reference frame as {@link #getTarget()}. Plants
     * without meaningful feedback should return {@link Double#NaN}.</p>
     *
     * @return last authoritative measurement in plant units, or {@code NaN} if unavailable
     */
    default double getMeasurement() {
        return Double.NaN;
    }

    /**
     * Convenience helper returning {@code getTarget() - getMeasurement()} using the most recent
     * plant update.
     *
     * @return last plant error in plant units, or {@code NaN} if measurement is unavailable
     */
    default double getError() {
        double measurement = getMeasurement();
        return Double.isFinite(measurement) ? (getTarget() - measurement) : Double.NaN;
    }

    /**
     * Optional debug hook: emit a compact summary of this plant's state.
     *
     * <p>Implementations are encouraged to extend this with mechanism-specific details, but the
     * default implementation already reports the core target/feedback summary.</p>
     *
     * @param dbg    debug sink (may be {@code null})
     * @param prefix base key prefix, for example {@code "arm"} or {@code "shooter.flywheel"}
     */
    default void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "plant" : prefix;
        dbg.addData(p + ".target", getTarget())
                .addData(p + ".hasFeedback", hasFeedback())
                .addData(p + ".atSetpoint", atSetpoint())
                .addData(p + ".measurement", getMeasurement())
                .addData(p + ".error", getError());
    }
}
