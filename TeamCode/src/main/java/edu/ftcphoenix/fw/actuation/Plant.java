package edu.ftcphoenix.fw.actuation;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Source-driven single-degree-of-freedom mechanism.
 *
 * <p>A {@code Plant} owns the hardware/control details for one scalar mechanism output: motor
 * power, servo position, motor velocity, lift position, flywheel velocity, and similar targets.
 * Robot behavior does <b>not</b> imperatively set a plant every loop. Instead, each plant is built
 * with one target source. During {@link #update(LoopClock)}, the plant samples that source once,
 * applies plant-level hardware guards, sends one safe target to hardware/control, and refreshes
 * feedback/status.</p>
 *
 * <h2>Target vocabulary</h2>
 * <ul>
 *   <li><b>Requested target</b>: raw value sampled from the behavior target source this loop.</li>
 *   <li><b>Applied target</b>: value actually sent to hardware/control after static bounds,
 *       reference checks, target guards, and rate limits.</li>
 *   <li><b>Writable target</b>: optional {@link ScalarTarget} registered with the plant so
 *       {@link PlantTasks} can write requests without being passed a separate target variable.</li>
 * </ul>
 *
 * <h2>Typical usage</h2>
 * <pre>{@code
 * ScalarTarget flywheelTarget = ScalarTarget.held(0.0);
 *
 * Plant flywheel = FtcActuators.plant(hardwareMap)
 *     .motor("flywheel", Direction.FORWARD)
 *     .velocity()
 *     .deviceManagedWithDefaults()
 *     .bounded(0.0, 2600.0)
 *     .nativeUnits()
 *     .velocityTolerance(50.0)
 *     .targetedBy(flywheelTarget)
 *     .build();
 *
 * flywheelTarget.set(1800.0);  // behavior changes the request
 * flywheel.update(clock);      // plant samples the request and applies a safe target
 * }</pre>
 */
public interface Plant {

    /**
     * Update this plant once for the current loop.
     *
     * <p>Implementations should sample their configured target source, compute their applied target,
     * command hardware/control, and refresh measurement/status caches. Robot code should call this
     * once per loop after updating the shared {@link LoopClock}.</p>
     */
    void update(LoopClock clock);

    /**
     * Raw target sampled from this plant's source on the most recent update.
     */
    double getRequestedTarget();

    /**
     * Target actually applied to hardware/control on the most recent update.
     */
    double getAppliedTarget();

    /**
     * Diagnostic explanation of how requested target became applied target on the last update.
     */
    PlantTargetStatus getTargetStatus();

    /**
     * Whether this plant has meaningful authoritative feedback.
     */
    default boolean hasFeedback() {
        return false;
    }

    /**
     * Authoritative measurement from the most recent update, in plant units, or {@code NaN}.
     */
    default double getMeasurement() {
        return Double.NaN;
    }

    /**
     * Requested-target error: {@code getRequestedTarget() - getMeasurement()}.
     */
    default double getTargetError() {
        double measurement = getMeasurement();
        return Double.isFinite(measurement) ? getRequestedTarget() - measurement : Double.NaN;
    }

    /**
     * Applied-target error: {@code getAppliedTarget() - getMeasurement()}, mostly diagnostic.
     */
    default double getAppliedTargetError() {
        double measurement = getMeasurement();
        return Double.isFinite(measurement) ? getAppliedTarget() - measurement : Double.NaN;
    }

    /**
     * Whether the plant is at its current requested target.
     *
     * <p>Open-loop plants default to {@code false} because Phoenix cannot prove physical arrival.</p>
     */
    default boolean atTarget() {
        return false;
    }

    /**
     * Whether the plant is truly at a specific target value.
     *
     * <p>Feedback tasks use this overload so a behavior overlay, clamp, fallback, or rate limiter
     * cannot make a task complete early while the plant is following a different target.</p>
     */
    default boolean atTarget(double target) {
        return false;
    }

    /**
     * Whether this plant has a registered writable command target.
     */
    default boolean hasWritableTarget() {
        return false;
    }

    /**
     * Return this plant's registered writable command target.
     *
     * @throws IllegalStateException if the plant was built from a read-only source without a
     *                               registered command target
     */
    default ScalarTarget writableTarget() {
        throw new IllegalStateException("This plant has no writable target. Build it with targetedBy(ScalarTarget), targetedByDefaultWritable(...), or targetedBy(readOnlySource).writableTarget(commandTarget).");
    }

    /**
     * Reset transient state such as controllers, target guards, and cached measurements.
     */
    default void reset() {
    }

    /**
     * Immediately stop driving this plant in the most reasonable way for its implementation.
     */
    void stop();

    /**
     * Emit a compact plant state summary for debug telemetry.
     */
    default void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) return;
        String p = (prefix == null || prefix.isEmpty()) ? "plant" : prefix;
        dbg.addData(p + ".requestedTarget", getRequestedTarget())
                .addData(p + ".appliedTarget", getAppliedTarget())
                .addData(p + ".targetStatus", getTargetStatus().toString())
                .addData(p + ".hasWritableTarget", hasWritableTarget())
                .addData(p + ".hasFeedback", hasFeedback())
                .addData(p + ".atTarget", atTarget())
                .addData(p + ".measurement", getMeasurement())
                .addData(p + ".targetError", getTargetError())
                .addData(p + ".appliedTargetError", getAppliedTargetError());
    }
}
