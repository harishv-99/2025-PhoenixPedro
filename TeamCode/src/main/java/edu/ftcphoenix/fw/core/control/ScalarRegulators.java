package edu.ftcphoenix.fw.core.control;

import java.util.Objects;
import java.util.function.DoubleUnaryOperator;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Helper factories and decorators for common {@link ScalarRegulator} implementations.
 *
 * <p>Regulators own the control law for Phoenix-regulated plants. They receive plant-unit setpoints
 * and measurements and return the command that the enclosing plant sends to its output channel
 * (commonly normalized motor power).</p>
 */
public final class ScalarRegulators {

    private ScalarRegulators() {
        // utility class
    }

    /**
     * Adapt an error-centric {@link PidController} into a setpoint/measurement-based regulator.
     *
     * <pre>{@code
     * ScalarRegulator regulator = ScalarRegulators.pid(
     *     Pid.withGains(0.01, 0.0, 0.0005).setOutputLimits(-1.0, 1.0)
     * );
     * }</pre>
     */
    public static ScalarRegulator pid(PidController controller) {
        return new PidScalarRegulator(controller);
    }

    /**
     * Adapt an error-centric {@link PidController} with a conventional setpoint feedforward term.
     *
     * <p>The feedforward function receives the requested setpoint in plant units. This matches common
     * velocity-control PIDF usage such as a shooter flywheel where the {@code F} term is proportional
     * to the requested RPM/ticks-per-second rather than to the instantaneous error.</p>
     *
     * <pre>{@code
     * ScalarRegulator flywheel = ScalarRegulators.pidf(
     *     Pid.withGains(kP, kI, kD).setIntegralLimits(-0.15, 0.15),
     *     rpm -> kV * rpm
     * );
     * }</pre>
     */
    public static ScalarRegulator pidf(PidController controller,
                                       DoubleUnaryOperator feedforwardFromSetpoint) {
        return setpointFeedforward(pid(controller), feedforwardFromSetpoint);
    }

    /**
     * Adapt an error-centric {@link PidController} with a linear setpoint feedforward term.
     *
     * <p>This is a convenience overload for the common case where feedforward is simply
     * {@code kF * setpoint}.</p>
     */
    public static ScalarRegulator pidf(PidController controller, double kF) {
        return pidf(controller, setpoint -> kF * setpoint);
    }

    /**
     * Add a plant-setpoint feedforward term to an existing regulator.
     *
     * <p>The returned regulator computes {@code inner.update(setpoint, measurement, clock)} and then
     * adds {@code feedforwardFromSetpoint.applyAsDouble(setpoint)}. This keeps feedforward composition
     * independent of whether the feedback controller is PID, bang-bang, asymmetric, or custom.</p>
     */
    public static ScalarRegulator setpointFeedforward(ScalarRegulator inner,
                                                      DoubleUnaryOperator feedforwardFromSetpoint) {
        return new SetpointFeedforwardScalarRegulator(inner, feedforwardFromSetpoint);
    }

    /**
     * Decorate a regulator with supply-voltage compensation.
     *
     * <p>The returned regulator first asks {@code inner} for its normal command, then scales that
     * command by {@code referenceVoltage / measuredVoltage}. This captures the common FTC flywheel
     * pattern of tuning a power-based velocity loop at a known voltage, then increasing the requested
     * power as the battery sags.</p>
     *
     * <p>{@code minimumVoltage} is used as a denominator floor so a very low or noisy voltage reading
     * cannot create an extreme scale. {@code maximumScale} is an additional upper cap on the multiplier.
     * If the voltage source returns {@code NaN}, infinity, or a non-positive value, compensation is
     * disabled for that sample and the scale is {@code 1.0}. The wrapper does not clamp the final
     * command; use controller output limits or the enclosing output adapter when the command is
     * normalized power.</p>
     *
     * <pre>{@code
     * ScalarRegulator compensated = ScalarRegulators.voltageCompensated(
     *     baseRegulator,
     *     FtcSensors.batteryVoltage(hardwareMap),
     *     13.0,  // reference voltage used while tuning
     *     9.0,   // denominator floor
     *     1.4    // maximum multiplier
     * );
     * }</pre>
     *
     * @param inner            regulator that computes the nominal command
     * @param supplyVoltage    source of current supply voltage in volts
     * @param referenceVoltage voltage the nominal regulator was tuned for; must be finite and > 0
     * @param minimumVoltage   lowest denominator used for scaling; must be finite, > 0, and <= referenceVoltage
     * @param maximumScale     maximum compensation multiplier; must be finite and >= 1
     * @return regulator decorator that applies voltage compensation to {@code inner}
     */
    public static ScalarRegulator voltageCompensated(ScalarRegulator inner,
                                                     ScalarSource supplyVoltage,
                                                     double referenceVoltage,
                                                     double minimumVoltage,
                                                     double maximumScale) {
        return new VoltageCompensatedScalarRegulator(inner, supplyVoltage,
                referenceVoltage, minimumVoltage, maximumScale);
    }

    private static final class PidScalarRegulator implements ScalarRegulator {
        private final PidController controller;
        private double lastSetpoint = Double.NaN;
        private double lastMeasurement = Double.NaN;
        private double lastError = Double.NaN;
        private double lastOutput = Double.NaN;

        private PidScalarRegulator(PidController controller) {
            this.controller = Objects.requireNonNull(controller, "controller");
        }

        @Override
        public double update(double setpoint, double measurement, LoopClock clock) {
            lastSetpoint = setpoint;
            lastMeasurement = measurement;
            lastError = setpoint - measurement;
            lastOutput = controller.update(lastError, clock.dtSec());
            return lastOutput;
        }

        @Override
        public void reset() {
            controller.reset();
            lastSetpoint = Double.NaN;
            lastMeasurement = Double.NaN;
            lastError = Double.NaN;
            lastOutput = Double.NaN;
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "pidRegulator" : prefix;
            dbg.addData(p + ".class", "PidScalarRegulator")
                    .addData(p + ".lastSetpoint", lastSetpoint)
                    .addData(p + ".lastMeasurement", lastMeasurement)
                    .addData(p + ".lastError", lastError)
                    .addData(p + ".lastOutput", lastOutput);
            if (controller instanceof Pid) {
                // Preserve the long-standing PID debug keys at the regulator prefix while adding
                // regulator-level live state alongside them.
                ((Pid) controller).debugDump(dbg, p);
            }
        }
    }

    private static final class SetpointFeedforwardScalarRegulator implements ScalarRegulator {
        private final ScalarRegulator inner;
        private final DoubleUnaryOperator feedforwardFromSetpoint;
        private double lastSetpoint = Double.NaN;
        private double lastMeasurement = Double.NaN;
        private double lastInnerOutput = Double.NaN;
        private double lastFeedforward = Double.NaN;
        private double lastOutput = Double.NaN;

        private SetpointFeedforwardScalarRegulator(ScalarRegulator inner,
                                                   DoubleUnaryOperator feedforwardFromSetpoint) {
            this.inner = Objects.requireNonNull(inner, "inner");
            this.feedforwardFromSetpoint = Objects.requireNonNull(feedforwardFromSetpoint,
                    "feedforwardFromSetpoint");
        }

        @Override
        public double update(double setpoint, double measurement, LoopClock clock) {
            lastSetpoint = setpoint;
            lastMeasurement = measurement;
            lastInnerOutput = inner.update(setpoint, measurement, clock);
            lastFeedforward = feedforwardFromSetpoint.applyAsDouble(setpoint);
            lastOutput = lastInnerOutput + lastFeedforward;
            return lastOutput;
        }

        @Override
        public void reset() {
            inner.reset();
            lastSetpoint = Double.NaN;
            lastMeasurement = Double.NaN;
            lastInnerOutput = Double.NaN;
            lastFeedforward = Double.NaN;
            lastOutput = Double.NaN;
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "setpointFeedforwardRegulator" : prefix;
            dbg.addData(p + ".class", "SetpointFeedforwardScalarRegulator")
                    .addData(p + ".lastSetpoint", lastSetpoint)
                    .addData(p + ".lastMeasurement", lastMeasurement)
                    .addData(p + ".lastInnerOutput", lastInnerOutput)
                    .addData(p + ".lastFeedforward", lastFeedforward)
                    .addData(p + ".lastOutput", lastOutput);
            inner.debugDump(dbg, p + ".inner");
        }
    }

    private static final class VoltageCompensatedScalarRegulator implements ScalarRegulator {
        private final ScalarRegulator inner;
        private final ScalarSource supplyVoltage;
        private final double referenceVoltage;
        private final double minimumVoltage;
        private final double maximumScale;

        private double lastSetpoint = Double.NaN;
        private double lastMeasurement = Double.NaN;
        private double lastMeasuredVoltage = Double.NaN;
        private boolean lastVoltageValid;
        private boolean lastVoltageFloored;
        private double lastRawScale = Double.NaN;
        private boolean lastScaleCapped;
        private double lastScale = Double.NaN;
        private double lastInnerOutput = Double.NaN;
        private double lastOutput = Double.NaN;

        private VoltageCompensatedScalarRegulator(ScalarRegulator inner,
                                                  ScalarSource supplyVoltage,
                                                  double referenceVoltage,
                                                  double minimumVoltage,
                                                  double maximumScale) {
            this.inner = Objects.requireNonNull(inner, "inner");
            this.supplyVoltage = Objects.requireNonNull(supplyVoltage, "supplyVoltage");
            if (!Double.isFinite(referenceVoltage) || referenceVoltage <= 0.0) {
                throw new IllegalArgumentException("referenceVoltage must be finite and > 0");
            }
            if (!Double.isFinite(minimumVoltage) || minimumVoltage <= 0.0 || minimumVoltage > referenceVoltage) {
                throw new IllegalArgumentException("minimumVoltage must be finite, > 0, and <= referenceVoltage");
            }
            if (!Double.isFinite(maximumScale) || maximumScale < 1.0) {
                throw new IllegalArgumentException("maximumScale must be finite and >= 1.0");
            }
            this.referenceVoltage = referenceVoltage;
            this.minimumVoltage = minimumVoltage;
            this.maximumScale = maximumScale;
        }

        @Override
        public double update(double setpoint, double measurement, LoopClock clock) {
            lastSetpoint = setpoint;
            lastMeasurement = measurement;
            lastInnerOutput = inner.update(setpoint, measurement, clock);

            lastMeasuredVoltage = supplyVoltage.getAsDouble(clock);
            lastVoltageValid = Double.isFinite(lastMeasuredVoltage) && lastMeasuredVoltage > 0.0;
            if (lastVoltageValid) {
                double denominator = Math.max(lastMeasuredVoltage, minimumVoltage);
                lastVoltageFloored = denominator > lastMeasuredVoltage;
                lastRawScale = referenceVoltage / denominator;
                lastScaleCapped = lastRawScale > maximumScale;
                lastScale = MathUtil.clamp(lastRawScale, 0.0, maximumScale);
            } else {
                lastVoltageFloored = false;
                lastRawScale = 1.0;
                lastScaleCapped = false;
                lastScale = 1.0;
            }

            lastOutput = lastInnerOutput * lastScale;
            return lastOutput;
        }

        @Override
        public void reset() {
            inner.reset();
            supplyVoltage.reset();
            lastSetpoint = Double.NaN;
            lastMeasurement = Double.NaN;
            lastMeasuredVoltage = Double.NaN;
            lastVoltageValid = false;
            lastVoltageFloored = false;
            lastRawScale = Double.NaN;
            lastScaleCapped = false;
            lastScale = Double.NaN;
            lastInnerOutput = Double.NaN;
            lastOutput = Double.NaN;
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "voltageCompensatedRegulator" : prefix;
            dbg.addData(p + ".class", "VoltageCompensatedScalarRegulator")
                    .addData(p + ".referenceVoltage", referenceVoltage)
                    .addData(p + ".minimumVoltage", minimumVoltage)
                    .addData(p + ".maximumScale", maximumScale)
                    .addData(p + ".lastSetpoint", lastSetpoint)
                    .addData(p + ".lastMeasurement", lastMeasurement)
                    .addData(p + ".lastMeasuredVoltage", lastMeasuredVoltage)
                    .addData(p + ".lastVoltageValid", lastVoltageValid)
                    .addData(p + ".lastVoltageFloored", lastVoltageFloored)
                    .addData(p + ".lastRawScale", lastRawScale)
                    .addData(p + ".lastScaleCapped", lastScaleCapped)
                    .addData(p + ".lastScale", lastScale)
                    .addData(p + ".lastInnerOutput", lastInnerOutput)
                    .addData(p + ".lastOutput", lastOutput);
            supplyVoltage.debugDump(dbg, p + ".supplyVoltage");
            inner.debugDump(dbg, p + ".inner");
        }
    }
}
