package edu.ftcphoenix.fw.core.control;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Small helpers for the common Phoenix scalar-regulation pattern:
 *
 * <pre>
 * setpoint source + measurement source + scalar controller -> command source
 * </pre>
 *
 * <p>This is intentionally lightweight. It does not introduce a new scheduler or subsystem model;
 * it simply packages the most common scalar feedback loop into a reusable {@link ScalarSource} that
 * you can feed into a {@code Plant}.</p>
 *
 * <p>Typical usage:</p>
 * <pre>{@code
 * ScalarSource desiredHeightIn = ScalarSource.constant(12.0);
 * ScalarSource measuredHeightIn =
 *         ScalarSource.of(() -> liftDistanceSensor.getDistance(DistanceUnit.INCH));
 * Pid pid = Pid.withGains(0.12, 0.0, 0.0).setOutputLimits(-0.55, 0.55);
 *
 * ScalarSource liftPower = ScalarControllers.pid(desiredHeightIn, measuredHeightIn, pid);
 *
 * // In the loop:
 * liftTarget.set(liftPower.getAsDouble(clock));
 * liftPlant.update(clock);
 * liftPlant.update(clock);
 * }</pre>
 *
 * <p>This is Phoenix's lane-2 scalar-regulation pattern: one measured scalar is regulated
 * toward a setpoint. It is intentionally separate from event-driven supervision and from spatial
 * guidance.</p>
 *
 * <p>Framework-regulated Plants construct their standard PID and typed feedforward law through
 * the Plant builder's control stages. They accept a {@link ScalarRegulator} only through the
 * explicitly custom-regulator branch; this source helper deliberately does not duplicate either
 * construction API.</p>
 */
public final class ScalarControllers {

    private ScalarControllers() {
        // utility class
    }

    /**
     * Build a command source from a scalar error controller.
     *
     * <p>The returned source publishes at most one successful controller output per loop cycle. It
     * samples the setpoint and measurement, computes {@code error = setpoint - measurement}, and
     * passes that error into {@code controller}; each child source governs its own same-cycle
     * observation semantics. A setpoint or measurement failure happens before the controller
     * attempt and may therefore resample either input on a same-cycle retry. Once the controller
     * has been invoked, a {@link RuntimeException} is retained and rethrown by identity for the
     * rest of that cycle because arbitrary controller state cannot be rolled back or safely
     * advanced twice.</p>
     *
     * <p>{@link ScalarSource#reset()} resets the owned setpoint, measurement, and controller in
     * that order. The returned source clears its committed diagnostics and retained attempt only
     * after all three resets succeed. Sampling/reset overlap and recursive sampling fail fast with
     * an actionable {@link IllegalStateException}.</p>
     *
     * <p>Use this when you already have a {@link PidController} (or compatible implementation) and
     * want a reusable command source you can clamp, combine, or feed into a {@code Plant}.</p>
     */
    public static ScalarSource pid(ScalarSource setpoint,
                                   ScalarSource measurement,
                                   PidController controller) {
        Objects.requireNonNull(setpoint, "setpoint");
        Objects.requireNonNull(measurement, "measurement");
        Objects.requireNonNull(controller, "controller");

        return new ScalarSource() {
            private long lastCycle = Long.MIN_VALUE;
            private long controllerAttemptCycle = Long.MIN_VALUE;
            private RuntimeException controllerFailure;
            private double lastSetpoint = 0.0;
            private double lastMeasurement = 0.0;
            private double lastError = 0.0;
            private double lastOutput = 0.0;
            private boolean sampling;
            private boolean resetting;

            /**
             * {@inheritDoc}
             */
            @Override
            public double getAsDouble(LoopClock clock) {
                Objects.requireNonNull(clock, "clock");
                if (sampling) {
                    throw new IllegalStateException(
                            "Scalar controller source sampling reentered; check the setpoint, "
                                    + "measurement, and controller graph for a cycle");
                }
                if (resetting) {
                    throw new IllegalStateException(
                            "Scalar controller source cannot be sampled while reset is in progress");
                }

                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return lastOutput;
                }
                if (cyc == controllerAttemptCycle && controllerFailure != null) {
                    throw controllerFailure;
                }

                sampling = true;
                try {
                    double candidateSetpoint = setpoint.getAsDouble(clock);
                    double candidateMeasurement = measurement.getAsDouble(clock);
                    double candidateError = candidateSetpoint - candidateMeasurement;

                    controllerAttemptCycle = cyc;
                    controllerFailure = null;
                    double candidateOutput;
                    try {
                        candidateOutput = controller.update(candidateError, clock.dtSec());
                    } catch (RuntimeException failure) {
                        controllerFailure = failure;
                        throw failure;
                    }

                    lastSetpoint = candidateSetpoint;
                    lastMeasurement = candidateMeasurement;
                    lastError = candidateError;
                    lastOutput = candidateOutput;
                    controllerFailure = null;
                    lastCycle = cyc;
                    return candidateOutput;
                } finally {
                    sampling = false;
                }
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                if (sampling) {
                    throw new IllegalStateException(
                            "Scalar controller source cannot reset while sampling is in progress");
                }
                if (resetting) {
                    throw new IllegalStateException(
                            "Scalar controller source reset reentered");
                }

                resetting = true;
                try {
                    setpoint.reset();
                    measurement.reset();
                    controller.reset();

                    lastCycle = Long.MIN_VALUE;
                    controllerAttemptCycle = Long.MIN_VALUE;
                    controllerFailure = null;
                    lastSetpoint = 0.0;
                    lastMeasurement = 0.0;
                    lastError = 0.0;
                    lastOutput = 0.0;
                } finally {
                    resetting = false;
                }
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) {
                    return;
                }
                String p = (prefix == null || prefix.isEmpty()) ? "scalarPid" : prefix;
                dbg.addData(p + ".class", "ScalarPidController")
                        .addData(p + ".setpoint", lastSetpoint)
                        .addData(p + ".measurement", lastMeasurement)
                        .addData(p + ".error", lastError)
                        .addData(p + ".output", lastOutput);
                setpoint.debugDump(dbg, p + ".setpointSrc");
                measurement.debugDump(dbg, p + ".measurementSrc");
            }
        };
    }

    /**
     * Convenience overload for a constant setpoint.
     *
     * <p>This is the most common form for fixed targets such as "hold 12 inches" or "hold 90°".</p>
     */
    public static ScalarSource pid(double setpoint,
                                   ScalarSource measurement,
                                   PidController controller) {
        return pid(ScalarSource.constant(setpoint), measurement, controller);
    }

    /**
     * Simple proportional-only scalar controller.
     *
     * <p>This is a compact way to express small lane-2 helpers when full PID tuning is not needed.</p>
     *
     * @param setpoint desired-value source
     * @param measurement measured-value source in the same units
     * @param kP finite proportional gain
     * @return cycle-memoized command source
     * @throws IllegalArgumentException if {@code kP} is not finite
     */
    public static ScalarSource proportional(ScalarSource setpoint,
                                            ScalarSource measurement,
                                            double kP) {
        return pid(setpoint, measurement, Pid.withGains(kP, 0.0, 0.0));
    }

    /**
     * Convenience overload for a constant setpoint and proportional-only control.
     *
     * @param setpoint constant desired value
     * @param measurement measured-value source in the same units
     * @param kP finite proportional gain
     * @return cycle-memoized command source
     * @throws IllegalArgumentException if {@code kP} is not finite
     */
    public static ScalarSource proportional(double setpoint,
                                            ScalarSource measurement,
                                            double kP) {
        return proportional(ScalarSource.constant(setpoint), measurement, kP);
    }
}
