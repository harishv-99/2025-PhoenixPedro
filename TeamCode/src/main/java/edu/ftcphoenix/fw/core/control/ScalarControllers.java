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
 * ScalarSource measuredHeightIn = clock -> liftDistanceSensor.getDistance(DistanceUnit.INCH);
 * Pid pid = Pid.withGains(0.12, 0.0, 0.0).setOutputLimits(-0.55, 0.55);
 *
 * ScalarSource liftPower = ScalarControllers.pid(desiredHeightIn, measuredHeightIn, pid);
 *
 * // In the loop:
 * liftPlant.setTarget(liftPower.getAsDouble(clock));
 * liftPlant.update(clock.dtSec());
 * }</pre>
 *
 * <p>This is Phoenix's lane-2 scalar-regulation pattern: one measured scalar is regulated
 * toward a setpoint. It is intentionally separate from event-driven supervision and from spatial
 * guidance.</p>
 */
public final class ScalarControllers {

    private ScalarControllers() {
        // utility class
    }

    /**
     * Build a command source from a scalar error controller.
     *
     * <p>The returned source samples the setpoint and measurement once per loop cycle, computes
     * {@code error = setpoint - measurement}, and passes that error into {@code controller}. It
     * memoizes the result by {@link LoopClock#cycle()} so multiple reads in the same loop do not
     * advance controller state twice.</p>
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
            private double lastSetpoint = 0.0;
            private double lastMeasurement = 0.0;
            private double lastError = 0.0;
            private double lastOutput = 0.0;

            @Override
            public double getAsDouble(LoopClock clock) {
                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return lastOutput;
                }
                lastCycle = cyc;

                lastSetpoint = setpoint.getAsDouble(clock);
                lastMeasurement = measurement.getAsDouble(clock);
                lastError = lastSetpoint - lastMeasurement;
                lastOutput = controller.update(lastError, clock.dtSec());
                return lastOutput;
            }

            @Override
            public void reset() {
                setpoint.reset();
                measurement.reset();
                controller.reset();
                lastCycle = Long.MIN_VALUE;
                lastSetpoint = 0.0;
                lastMeasurement = 0.0;
                lastError = 0.0;
                lastOutput = 0.0;
            }

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
     */
    public static ScalarSource proportional(ScalarSource setpoint,
                                            ScalarSource measurement,
                                            double kP) {
        return pid(setpoint, measurement, new Pid(kP, 0.0, 0.0));
    }

    /**
     * Convenience overload for a constant setpoint and proportional-only control.
     */
    public static ScalarSource proportional(double setpoint,
                                            ScalarSource measurement,
                                            double kP) {
        return proportional(ScalarSource.constant(setpoint), measurement, kP);
    }
}
