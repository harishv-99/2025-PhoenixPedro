package edu.ftcphoenix.fw.core.control;

import java.util.Objects;
import java.util.function.DoubleUnaryOperator;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.math.MathUtil;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Sole public factories and decorators for common {@link ScalarRegulator} implementations.
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
     * Create a standard Phoenix software PIDF regulator with four finite gains.
     *
     * <p>The returned regulator computes
     * {@code PID(setpoint - measurement, clock.dtSec()) + kF * setpoint}. The feedforward gain is
     * expressed in command units per plant-setpoint unit. Signed gains are allowed. This software
     * control law is distinct from an FTC SDK motor controller's device-managed PIDF settings.</p>
     *
     * <p>Retain the concrete result when live tuning or applied-gain reporting is needed. Static
     * configurations may store it as a {@link ScalarRegulator}:</p>
     *
     * <pre>{@code
     * PidfRegulator pidf = ScalarRegulators.pidf(kP, kI, kD, kF)
     *     .setIntegralLimits(-0.15, 0.15)
     *     .setPidOutputLimits(-1.0, 1.0);
     * }</pre>
     *
     * <p>For a nonlinear or dynamic feedforward law, compose it explicitly with
     * {@link #setpointFeedforward(ScalarRegulator, DoubleUnaryOperator)}.</p>
     *
     * @param kP finite proportional gain
     * @param kI finite integral gain
     * @param kD finite derivative gain
     * @param kF finite linear setpoint-feedforward gain
     * @return retained standard PIDF regulator
     * @throws IllegalArgumentException if any gain is not finite
     */
    public static PidfRegulator pidf(double kP, double kI, double kD, double kF) {
        return new PidfRegulator(kP, kI, kD, kF);
    }

    /**
     * Add a plant-setpoint feedforward term to an existing regulator.
     *
     * <p>The returned regulator computes {@code inner.update(setpoint, measurement, clock)} and then
     * adds {@code feedforwardFromSetpoint.applyAsDouble(setpoint)}. This keeps feedforward composition
     * independent of whether the feedback controller is PID, bang-bang, asymmetric, or custom.
     * Use {@link #pidf(double, double, double, double)} for ordinary linear PIDF so all four gains
     * share one validated retained owner.</p>
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
     * disabled for that sample and the scale is {@code 1.0}. The wrapper does not constrain its
     * output. Wrap the complete composition with {@link #outputLimited(ScalarRegulator, double,
     * double)} when it needs an intentional policy range; the enclosing Plant/output boundary still
     * owns its separate defensive command invariant.</p>
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

    /**
     * Constrain an existing regulator's complete output to an inclusive range.
     *
     * <p>Place this decorator outside every output-changing decorator that the range must cover.
     * For example, the following range applies after both feedforward and voltage compensation:</p>
     *
     * <pre>{@code
     * ScalarRegulator flywheel = ScalarRegulators.outputLimited(
     *     ScalarRegulators.voltageCompensated(
     *         ScalarRegulators.pidf(kP, kI, kD, kF)
     *             .setPidOutputLimits(-1.0, 1.0),
     *         supplyVoltage,
     *         13.0,
     *         9.0,
     *         1.4
     *     ),
     *     0.0,
     *     maximumOutputPower
     * );
     * }</pre>
     *
     * <p>{@link Pid#setOutputLimits(double, double)} and
     * {@link PidfRegulator#setPidOutputLimits(double, double)} instead limit only the PID
     * contribution before feedforward or later regulator decorators run. This decorator is an
     * intentional control-law policy; it does not replace Plant target bounds, final
     * normalized-output defense, controller-specific anti-windup, or robot-owned
     * enable/coast/reset policy.</p>
     *
     * <p>The returned regulator rejects a non-finite inner result rather than turning invalid
     * control math into a bounded but potentially dangerous command.</p>
     *
     * @param inner     regulator whose complete result should be constrained
     * @param minOutput inclusive finite lower output bound, in the inner regulator's output units
     * @param maxOutput inclusive finite upper output bound, in the inner regulator's output units
     * @return regulator decorator that constrains the output of {@code inner}
     * @throws NullPointerException     if {@code inner} is null
     * @throws IllegalArgumentException if either bound is non-finite or
     *                                  {@code minOutput > maxOutput}
     */
    public static ScalarRegulator outputLimited(ScalarRegulator inner,
                                                double minOutput,
                                                double maxOutput) {
        return new OutputLimitedScalarRegulator(inner, minOutput, maxOutput);
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

    /**
     * Applies one explicit policy range after an inner regulator has computed its complete output.
     */
    private static final class OutputLimitedScalarRegulator implements ScalarRegulator {
        private final ScalarRegulator inner;
        private final double minOutput;
        private final double maxOutput;

        private double lastUnconstrainedOutput = Double.NaN;
        private double lastOutput = Double.NaN;
        private boolean lastOutputLimited;

        private OutputLimitedScalarRegulator(ScalarRegulator inner,
                                             double minOutput,
                                             double maxOutput) {
            this.inner = Objects.requireNonNull(inner, "inner");
            if (!Double.isFinite(minOutput)) {
                throw new IllegalArgumentException(
                        "minOutput must be finite; got minOutput=" + minOutput
                                + ", maxOutput=" + maxOutput);
            }
            if (!Double.isFinite(maxOutput)) {
                throw new IllegalArgumentException(
                        "maxOutput must be finite; got minOutput=" + minOutput
                                + ", maxOutput=" + maxOutput);
            }
            if (minOutput > maxOutput) {
                throw new IllegalArgumentException(
                        "minOutput must be <= maxOutput; got minOutput=" + minOutput
                                + ", maxOutput=" + maxOutput);
            }
            this.minOutput = minOutput;
            this.maxOutput = maxOutput;
        }

        @Override
        public double update(double setpoint, double measurement, LoopClock clock) {
            lastUnconstrainedOutput = inner.update(setpoint, measurement, clock);
            lastOutput = Double.NaN;
            lastOutputLimited = false;

            if (!Double.isFinite(lastUnconstrainedOutput)) {
                throw new IllegalStateException(
                        "outputLimited(...) received a non-finite output from the inner regulator; got "
                                + lastUnconstrainedOutput
                                + ". Check the setpoint, measurement, feedforward, and compensation inputs.");
            }

            lastOutputLimited = lastUnconstrainedOutput < minOutput
                    || lastUnconstrainedOutput > maxOutput;
            lastOutput = MathUtil.clamp(lastUnconstrainedOutput, minOutput, maxOutput);
            return lastOutput;
        }

        @Override
        public void reset() {
            inner.reset();
            lastUnconstrainedOutput = Double.NaN;
            lastOutput = Double.NaN;
            lastOutputLimited = false;
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = (prefix == null || prefix.isEmpty()) ? "outputLimitedRegulator" : prefix;
            dbg.addData(p + ".class", "OutputLimitedScalarRegulator")
                    .addData(p + ".minOutput", minOutput)
                    .addData(p + ".maxOutput", maxOutput)
                    .addData(p + ".lastUnconstrainedOutput", lastUnconstrainedOutput)
                    .addData(p + ".lastOutput", lastOutput)
                    .addData(p + ".lastOutputLimited", lastOutputLimited);
            inner.debugDump(dbg, p + ".inner");
        }
    }
}
