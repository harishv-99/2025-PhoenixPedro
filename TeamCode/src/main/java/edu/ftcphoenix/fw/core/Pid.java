package edu.ftcphoenix.fw.core;

import edu.ftcphoenix.fw.debug.DebugSink;
import edu.ftcphoenix.fw.util.MathUtil;

/**
 * General-purpose PID controller with optional integral and output clamping.
 *
 * <p>This implementation is intended to be simple enough for students to
 * reason about, while still covering the common control needs in FTC:
 * flywheel RPM control, heading control, and aiming (for example, using
 * AprilTag bearing as the error signal).</p>
 *
 * <h2>Design</h2>
 *
 * <ul>
 *   <li>Error-centric API: callers pass {@code error} and {@code dtSec} into
 *       {@link #update(double, double)}, and receive a control output.</li>
 *   <li>Supports proportional, integral, and derivative terms:
 *     {@code output = kP * error + integral + derivative}.</li>
 *   <li>Optional clamping of the integral accumulator and output to avoid
 *       runaway behavior ("integral windup").</li>
 *   <li>Derivative term is computed from the difference in error between
 *       successive calls.</li>
 * </ul>
 *
 * <h2>Typical usage</h2>
 *
 * <p>Create a controller with gains, then call {@link #update(double, double)}
 * once per loop:</p>
 *
 * <pre>{@code
 * // Create a basic PID with only P and D gains.
 * PidController pid = Pid.withGains(1.0, 0.0, 0.1);
 *
 * // In your loop:
 * double error = targetValue - measuredValue;
 * double output = pid.update(error, dtSec);
 * // Apply or clamp output as appropriate for your actuator.
 * }</pre>
 *
 * <p>For more advanced use-cases, you can set integral and output limits:</p>
 *
 * <pre>{@code
 * Pid pid = Pid.withGains(0.8, 0.2, 0.05);
 * pid.setIntegralLimits(-0.5, 0.5); // clamp integral contribution
 * pid.setOutputLimits(-1.0, 1.0);   // clamp final output
 * }</pre>
 *
 * <p>When switching modes or making large setpoint jumps, call
 * {@link #reset()} to clear internal state.</p>
 */
public final class Pid implements PidController {

    // Gains
    private double kP;
    private double kI;
    private double kD;

    // Integral and derivative state
    private double integral;
    private double prevError;
    private boolean firstUpdate = true;

    // Integral clamp (applied to the integral term contribution)
    private double integralMin = Double.NEGATIVE_INFINITY;
    private double integralMax = Double.POSITIVE_INFINITY;

    // Output clamp (applied to the final output)
    private double outputMin = Double.NEGATIVE_INFINITY;
    private double outputMax = Double.POSITIVE_INFINITY;

    /**
     * Create a PID controller with the given gains and no clamping.
     *
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     */
    public Pid(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Convenience factory for a PID controller with the given gains and no
     * clamping.
     *
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     * @return a new {@link Pid} instance
     */
    public static Pid withGains(double kP, double kI, double kD) {
        return new Pid(kP, kI, kD);
    }

    /**
     * Set the PID gains.
     *
     * <p>This can be used to adjust gains at runtime (for example, from
     * dashboard tuning) without creating a new controller instance.</p>
     *
     * @param kP new proportional gain
     * @param kI new integral gain
     * @param kD new derivative gain
     * @return this controller, for chaining
     */
    public Pid setGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        return this;
    }

    /**
     * Set limits for the integral accumulator term.
     *
     * <p>This is a simple anti-windup mechanism: after integrating, the
     * integral term is clamped to the provided range before contributing to
     * the output.</p>
     *
     * @param min minimum integral value
     * @param max maximum integral value
     * @return this controller, for chaining
     */
    public Pid setIntegralLimits(double min, double max) {
        this.integralMin = min;
        this.integralMax = max;
        return this;
    }

    /**
     * Set limits for the final controller output.
     *
     * <p>This is useful when the PID output is applied directly as a power
     * command or velocity setpoint that must stay within a certain range.</p>
     *
     * @param min minimum output value
     * @param max maximum output value
     * @return this controller, for chaining
     */
    public Pid setOutputLimits(double min, double max) {
        this.outputMin = min;
        this.outputMax = max;
        return this;
    }

    @Override
    public double update(double error, double dtSec) {
        if (dtSec <= 0.0) {
            // Guard against bad dt values; skip integration/derivative.
            dtSec = 0.0;
        }

        // Proportional term
        double pTerm = kP * error;

        // Integral term (simple rectangular integration)
        if (kI != 0.0 && dtSec > 0.0) {
            integral += kI * error * dtSec;

            // Clamp integral to avoid windup
            integral = MathUtil.clamp(integral, integralMin, integralMax);
        }

        // Derivative term (based on change in error)
        double dTerm = 0.0;
        if (!firstUpdate && kD != 0.0 && dtSec > 0.0) {
            double errorDelta = error - prevError;
            dTerm = kD * (errorDelta / dtSec);
        }

        prevError = error;
        firstUpdate = false;

        // Combine terms
        double output = pTerm + integral + dTerm;

        // Clamp output if requested
        output = MathUtil.clamp(output, outputMin, outputMax);

        return output;
    }

    @Override
    public void reset() {
        integral = 0.0;
        prevError = 0.0;
        firstUpdate = true;
    }

    // ---------------------------------------------------------------------
    // Optional getters for debugging or dashboards
    // ---------------------------------------------------------------------

    /**
     * @return current proportional gain
     */
    public double getkP() {
        return kP;
    }

    /**
     * @return current integral gain
     */
    public double getkI() {
        return kI;
    }

    /**
     * @return current derivative gain
     */
    public double getkD() {
        return kD;
    }

    /**
     * @return current integral accumulator value
     */
    public double getIntegral() {
        return integral;
    }

    /**
     * Debug helper: emit current gains & state.
     *
     * @param dbg    debug sink (may be {@code null}; if null, no output is produced)
     * @param prefix base key prefix, e.g. "shooter.pid" or "heading.pid"
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "pid" : prefix;
        dbg.addLine(p)
                .addData(p + ".kP", kP)
                .addData(p + ".kI", kI)
                .addData(p + ".kD", kD)
                .addData(p + ".integral", integral)
                .addData(p + ".prevError", prevError)
                .addData(p + ".firstUpdate", firstUpdate)
                .addData(p + ".integralMin", integralMin)
                .addData(p + ".integralMax", integralMax)
                .addData(p + ".outputMin", outputMin)
                .addData(p + ".outputMax", outputMax);
    }
}
