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
 * with one {@link PlantTargetResolver}. During {@link #update(LoopClock)}, the plant asks that
 * resolver for one target,
 * applies plant-level hardware guards, applies one final mechanism target through its selected
 * control path, and refreshes feedback/status.</p>
 *
 * <h2>Target vocabulary</h2>
 * <ul>
 *   <li><b>Requested target</b>: raw value selected by the behavior
 *       {@link PlantTargetResolver} this loop.</li>
 *   <li><b>Applied target</b>: final mechanism target selected after static bounds, reference
 *       checks, target guards, and rate limits. For a framework-regulated Plant, the regulator then
 *       derives a separate normalized actuator command from this target; the applied target is not
 *       that command or hardware readback.</li>
 *   <li><b>Command target</b>: optional stable {@link ScalarTarget} carried by the exact resolver or
 *       overlay base. Robot policy and {@link ScalarTasks} change that same persistent request.</li>
 * </ul>
 *
 * <h2>Ordinary FTC mechanism ownership</h2>
 * <p>Build and retain the Plant inside the mechanism/subsystem constructor that receives
 * {@code HardwareMap} and a data-only mechanism config. That owner snapshots its config and owns
 * the Plant's update and stop lifecycle; the composition root constructs the mechanism rather than
 * constructing this Plant itself. The following is the Plant-building part of that constructor.</p>
 * <pre>{@code
 * FlywheelConfig cfg = config.copy();
 * this.flywheel = FtcActuators.plant(hardwareMap)
 *     .motor(cfg.motorName, cfg.direction)
 *     .velocity()
 *     .deviceManagedWithDefaults()
 *     .bounded(cfg.minVelocity, cfg.maxVelocity)
 *     .nativeUnits()
 *     .velocityTolerance(cfg.velocityTolerance)
 *     .targetedBy(ScalarTarget.create(0.0))
 *     .build();
 *
 * // Later, in a semantic intent method:
 * flywheel.commandTarget().set(1800.0); // changes the graph-owned request
 *
 * // Once in the mechanism owner's per-loop update phase:
 * flywheel.update(clock); // plant resolves the request and applies a safe target
 * }</pre>
 */
public interface Plant {

    /**
     * Update this plant once for the current loop.
     *
     * <p>Implementations should invoke their configured {@link PlantTargetResolver}, compute their applied
     * mechanism target, command hardware/control, and refresh measurement/status caches. Robot code
     * should call this once per loop after updating the shared {@link LoopClock}.</p>
     */
    void update(LoopClock clock);

    /**
     * Raw target selected by this plant's resolver on the most recent update.
     */
    double getRequestedTarget();

    /**
     * Return this Plant's cached final mechanism target.
     *
     * <p>For a framework-regulated Plant this remains a target in plant units, not the regulator's
     * raw result, the normalized power command, or physical hardware readback. If an output stop
     * fails, an implementation may retain its prior target fact rather than falsely report a new
     * stopped target.</p>
     */
    double getAppliedTarget();

    /**
     * Diagnostic explanation of how this plant's target resolver selected the requested target.
     *
     * <p>The resolution is about target selection only: exact target, equivalent position, planned
     * candidate, fallback, hold, or unavailable. Physical arrival still belongs to
     * {@link #atTarget()} and
     * {@link #atTarget(double)}.</p>
     */
    default PlantTargetResolution getTargetResolution() {
        return PlantTargetResolution.unavailable("plant has not reported a target resolution");
    }

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
    default double getRequestedTargetError() {
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
     * <p>Open-loop plants default to {@code false} because Phoenix cannot prove physical arrival.
     * Framework-regulated feedback Plants additionally require the latest regulated actuation to
     * have completed normally; reset, stop, or a failed actuation invalidates completion evidence
     * until a later successful update.</p>
     */
    default boolean atTarget() {
        return false;
    }

    /**
     * Whether the plant is truly at a specific target value.
     *
     * <p>The supplied value is one literal physical target in Plant units. This method does not
     * compare modulo a periodic Plant's period: exact unwrapped movement remains distinct from an
     * equivalent-position request. Framework feedback implementations also require the requested
     * and applied targets, guards/status, measurement, and latest actuation evidence to agree.
     * {@link ScalarTasks} combines this physical query with target-resolution command evidence when a
     * logical command resolves to a different equivalent physical value.</p>
     */
    default boolean atTarget(double target) {
        return false;
    }

    /**
     * Whether this plant's final target graph carries a stable command target.
     *
     * <p>When this returns {@code true}, {@link #commandTarget()} must return the same non-null
     * object for this Plant's lifetime. An ordinary FTC mechanism normally constructs this Plant
     * internally and may retrieve that command where it is used. If a clearly labeled
     * hardware-neutral test, custom-adapter, or advanced-assembly seam instead injects a completed
     * Plant, it passes this Plant alone rather than accepting a second, independently supplied
     * target.</p>
     */
    default boolean hasCommandTarget() {
        return false;
    }

    /**
     * Return this plant's stable command target.
     *
     * <p>This target is the persistent behavior request that tasks may change. An overlay can mask
     * that request without changing command ownership. Calling this method is side-effect-free and
     * returns the same object for this Plant's lifetime; it does not sample the graph or bypass the
     * Plant's target graph, guards, or update lifecycle.</p>
     *
     * <p>An ordinary exact mechanism builds and retains only the Plant, then retrieves this stable
     * target at a command or Task-construction point. A separate retained target is useful only
     * when it has an independent shared, composed-graph, or target-only policy role. Passing a
     * completed Plant across a constructor is an explicit hardware-neutral test, custom-adapter,
     * portable-host, or advanced-assembly seam; that seam receives only the Plant. A read-only or
     * planned realization does not require a command target. A policy object that does not own the
     * Plant may instead receive only the {@link ScalarTarget} it writes.</p>
     *
     * @throws IllegalStateException if the plant was built from a read-only source without a
     *                               stable command target
     */
    default ScalarTarget commandTarget() {
        throw new IllegalStateException("This plant has no command target. Build it with "
                + "targetedBy(command), or use a ScalarTarget as the stable base of its final "
                + "target graph.");
    }

    /**
     * Reset transient state such as controllers, target guards, and cached measurements.
     *
     * <p>For a framework-regulated Plant, reset invalidates completion and resets the regulator but
     * does not write or imply a stopped actuator command.</p>
     */
    default void reset() {
    }

    /**
     * Immediately stop driving this plant in the most reasonable way for its implementation.
     *
     * <p>Framework-regulated implementations attempt to stop owned outputs before resetting the
     * regulator, invalidate completion evidence, and propagate runtime cleanup failures. A normally
     * returning top-level stop establishes seam-level zero submission, not physical proof.</p>
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
                .addData(p + ".targetResolution", getTargetResolution().toString())
                .addData(p + ".targetStatus", getTargetStatus().toString())
                .addData(p + ".hasCommandTarget", hasCommandTarget())
                .addData(p + ".hasFeedback", hasFeedback())
                .addData(p + ".atTarget", atTarget())
                .addData(p + ".measurement", getMeasurement())
                .addData(p + ".requestedTargetError", getRequestedTargetError())
                .addData(p + ".appliedTargetError", getAppliedTargetError());
    }
}
