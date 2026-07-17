package edu.ftcphoenix.fw.core.control;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.math.MathUtil;

/**
 * General-purpose error-centric PID controller with optional integral and output clamping.
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
 *   <li>Independent optional limits for the integral contribution and this PID controller's
 *       output.</li>
 *   <li>Derivative term is computed from the difference in error between
 *       successive calls.</li>
 *   <li>Configuration is validated when it is supplied so an invalid gain or limit is reported
 *       before it can enter the control loop.</li>
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
 * pid.setOutputLimits(-1.0, 1.0);   // clamp this PID controller's output
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
     * Construct one PID after the sole public factory has validated all gains.
     */
    private Pid(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Create a PID controller with the given finite gains and no clamping.
     *
     * <p>Signed gains are allowed. Use this factory rather than a constructor so Phoenix has one
     * public PID construction path and can report invalid configuration consistently.</p>
     *
     * @param kP finite proportional gain
     * @param kI finite integral gain
     * @param kD finite derivative gain
     * @return a new {@link Pid} instance
     * @throws IllegalArgumentException if any gain is not finite
     */
    public static Pid withGains(double kP, double kI, double kD) {
        validateFiniteGains("Pid.withGains(...)", kP, kI, kD);
        return new Pid(kP, kI, kD);
    }

    /**
     * Set all PID gains as one validated update.
     *
     * <p>This can be used to adjust gains at runtime (for example, from
     * dashboard tuning) without creating a new controller instance. All arguments are validated
     * before any gain changes; a rejected update leaves all three applied gains unchanged.
     * Changing gains does not reset integral or derivative history.</p>
     *
     * @param kP new finite proportional gain
     * @param kI new finite integral gain
     * @param kD new finite derivative gain
     * @return this controller, for chaining
     * @throws IllegalArgumentException if any gain is not finite
     */
    public Pid setGains(double kP, double kI, double kD) {
        validateFiniteGains("Pid.setGains(...)", kP, kI, kD);
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        return this;
    }

    /**
     * Set finite limits for the integral contribution.
     *
     * <p>This is a simple anti-windup mechanism: after integrating, the
     * integral term is clamped to the provided range before contributing to
     * the output. The range must include zero so {@link #reset()} can truthfully clear the
     * integral contribution while preserving configuration. A valid new range immediately clamps
     * the existing integral contribution when it is finite. A non-finite value produced by
     * invalid dynamic input or arithmetic overflow is not turned into a plausible bounded
     * contribution; it remains non-finite for the caller's output boundary to reject.</p>
     *
     * @param min finite minimum integral contribution; must be {@code <= 0}
     * @param max finite maximum integral contribution; must be {@code >= 0}
     * @return this controller, for chaining
     * @throws IllegalArgumentException if either bound is non-finite, {@code min > max}, or the
     *                                  range does not include zero
     */
    public Pid setIntegralLimits(double min, double max) {
        validateIntegralLimits("Pid.setIntegralLimits(...)", min, max);
        this.integralMin = min;
        this.integralMax = max;
        if (Double.isFinite(integral)) {
            integral = MathUtil.clamp(integral, integralMin, integralMax);
        }
        return this;
    }

    /**
     * Set finite limits for the final PID-controller output.
     *
     * <p>This is useful when the PID output is applied directly as a power
     * command or velocity setpoint that must stay within a certain range.</p>
     *
     * <p>The limits apply to this controller's combined {@code P + I + D} result. A later
     * {@link ScalarRegulator} decorator may still add or scale output. When a complete regulator
     * composition needs a policy range, wrap it outermost with
     * {@link ScalarRegulators#outputLimited(ScalarRegulator, double, double)}. Output limiting alone
     * is not saturation-aware anti-windup; configure integral limits and reset policy separately.
     * The limit saturates finite results only. Non-finite controller math remains non-finite so an
     * enclosing Plant or direct caller can fail closed instead of receiving a disguised boundary
     * value.</p>
     *
     * @param min finite minimum output value
     * @param max finite maximum output value
     * @return this controller, for chaining
     * @throws IllegalArgumentException if either bound is non-finite or {@code min > max}
     */
    public Pid setOutputLimits(double min, double max) {
        validateOrderedFiniteBounds("Pid.setOutputLimits(...)", min, max);
        this.outputMin = min;
        this.outputMax = max;
        return this;
    }

    /**
     * Compute the next PID output from one error sample.
     *
     * <p>A non-positive {@code dtSec} is treated as zero. Integral and derivative terms advance
     * only when the resulting time step is positive; the proportional term and previous-error
     * history still observe the supplied error. Non-finite control math is returned unchanged for
     * the caller's output boundary to reject.</p>
     *
     * @param error current control error in caller-selected units
     * @param dtSec elapsed time in seconds
     * @return raw PID result after finite-only configured limits
     */
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

            // Saturate only valid finite controller state. A non-finite dynamic result must
            // remain visible to the final output boundary instead of becoming a plausible command.
            if (Double.isFinite(integral)) {
                integral = MathUtil.clamp(integral, integralMin, integralMax);
            }
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

        // Saturate finite control output only; preserve invalid math for fail-stop handling.
        if (Double.isFinite(output)) {
            output = MathUtil.clamp(output, outputMin, outputMax);
        }

        return output;
    }

    /** {@inheritDoc} */
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
    public double getKP() {
        return kP;
    }

    /**
     * @return current integral gain
     */
    public double getKI() {
        return kI;
    }

    /**
     * @return current derivative gain
     */
    public double getKD() {
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

    /**
     * Reject every non-finite PID gain while reporting the complete candidate.
     */
    private static void validateFiniteGains(String api, double kP, double kI, double kD) {
        String invalid = nonFiniteGainList(kP, kI, kD);
        if (!invalid.isEmpty()) {
            throw new IllegalArgumentException(
                    api + " requires finite gains; got kP=" + kP + ", kI=" + kI
                            + ", kD=" + kD + "; non-finite setting(s): " + invalid);
        }
    }

    /**
     * Return a readable list of every non-finite gain in one candidate.
     */
    private static String nonFiniteGainList(double kP, double kI, double kD) {
        String invalid = "";
        if (!Double.isFinite(kP)) {
            invalid = appendSetting(invalid, "kP", kP);
        }
        if (!Double.isFinite(kI)) {
            invalid = appendSetting(invalid, "kI", kI);
        }
        if (!Double.isFinite(kD)) {
            invalid = appendSetting(invalid, "kD", kD);
        }
        return invalid;
    }

    /**
     * Append one named invalid setting to an error-message fragment.
     */
    private static String appendSetting(String settings, String name, double value) {
        String entry = name + "=" + value;
        return settings.isEmpty() ? entry : settings + ", " + entry;
    }

    /**
     * Validate finite ordered integral limits that include zero.
     *
     * <p>Package-private so {@link PidfRegulator} can reuse the exact internal-PID invariant while
     * retaining its own caller-facing API name in diagnostics.</p>
     */
    static void validateIntegralLimits(String api, double min, double max) {
        validateOrderedFiniteBounds(api, min, max);
        if (min > 0.0 || max < 0.0) {
            throw new IllegalArgumentException(
                    api + " requires bounds that include zero (min <= 0 <= max); got min="
                            + min + ", max=" + max);
        }
    }

    /**
     * Validate finite inclusive bounds without silently reordering them.
     *
     * <p>Package-private for the built-in PIDF regulator's internal PID-limit configuration.</p>
     */
    static void validateOrderedFiniteBounds(String api, double min, double max) {
        String invalid = "";
        if (!Double.isFinite(min)) {
            invalid = appendSetting(invalid, "min", min);
        }
        if (!Double.isFinite(max)) {
            invalid = appendSetting(invalid, "max", max);
        }
        if (!invalid.isEmpty()) {
            throw new IllegalArgumentException(
                    api + " requires finite bounds; got min=" + min + ", max=" + max
                            + "; non-finite setting(s): " + invalid);
        }
        if (min > max) {
            throw new IllegalArgumentException(
                    api + " requires min <= max; got min=" + min + ", max=" + max);
        }
    }
}
