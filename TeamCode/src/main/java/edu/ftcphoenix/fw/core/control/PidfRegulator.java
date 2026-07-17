package edu.ftcphoenix.fw.core.control;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Standard Phoenix software PIDF regulator with one validated, retained four-gain configuration.
 *
 * <p>The regulator computes:</p>
 *
 * <pre>
 * error = setpoint - measurement
 * output = PID(error, dtSec) + kF * setpoint
 * </pre>
 *
 * <p>{@code kF} is a linear setpoint-feedforward gain in command units per plant-setpoint unit.
 * It is not a static-friction ({@code kS}), acceleration ({@code kA}), gravity, or FTC SDK
 * device-controller coefficient. Use
 * {@link ScalarRegulators#setpointFeedforward(ScalarRegulator, java.util.function.DoubleUnaryOperator)}
 * when the feedforward law is nonlinear, dynamic, or wraps a custom feedback regulator.</p>
 *
 * <p>Create this retained capability through
 * {@link ScalarRegulators#pidf(double, double, double, double)}. Keeping the returned type is useful
 * when a robot applies live tuning, resets the surrounding regulator composition, or reports the
 * gains that are actually in use:</p>
 *
 * <pre>{@code
 * PidfRegulator pidf = ScalarRegulators.pidf(kP, kI, kD, kF)
 *     .setIntegralLimits(-0.15, 0.15)
 *     .setPidOutputLimits(-1.0, 1.0);
 *
 * ScalarRegulator regulator = ScalarRegulators.outputLimited(pidf, 0.0, maximumPower);
 *
 * // In one OpMode-loop apply phase, after obtaining one coherent candidate:
 * pidf.setGains(candidateKP, candidateKI, candidateKD, candidateKF);
 * regulator.reset(); // outermost retained composition; no actuator command
 * }</pre>
 *
 * <p>Live configuration is development input, not production configuration authority. Production
 * TeleOp and Auto should construct from checked-in profile snapshots. A dedicated tuning mode
 * should accept one coherently published candidate at an explicit apply boundary, report the
 * accepted values, and require those values to be copied into the checked-in profile before
 * starting a fresh production mode. The robot realization retains the outer composition because
 * this inner PIDF object cannot discover wrappers that also need reset.</p>
 *
 * <p>The optional PID-output limits apply to the combined {@code P + I + D} contribution before
 * feedforward is added. Use
 * {@link ScalarRegulators#outputLimited(ScalarRegulator, double, double)} outside the complete
 * composition when the final command needs a policy range.</p>
 *
 * <p>This stateful regulator intentionally does not memoize by {@link LoopClock#cycle()}. Each
 * {@link #update(double, double, LoopClock)} invocation advances the internal PID state, matching
 * the {@link ScalarRegulator} invocation contract.</p>
 */
public final class PidfRegulator implements ScalarRegulator {

    private final Pid pid;
    private double kF;

    private double lastSetpoint = Double.NaN;
    private double lastMeasurement = Double.NaN;
    private double lastError = Double.NaN;
    private double lastPidOutput = Double.NaN;
    private double lastFeedforwardOutput = Double.NaN;
    private double lastOutput = Double.NaN;

    /**
     * Package-private so {@link ScalarRegulators} remains the sole public construction layer.
     */
    PidfRegulator(double kP, double kI, double kD, double kF) {
        validateFiniteGains("ScalarRegulators.pidf(...)", kP, kI, kD, kF);
        this.pid = Pid.withGains(kP, kI, kD);
        this.kF = kF;
    }

    /**
     * Set all four gains from one caller-supplied candidate.
     *
     * <p>Every argument is validated before any applied gain changes. If any gain is non-finite,
     * this method throws and leaves all four applied gains unchanged. A valid update preserves
     * integral and derivative history.</p>
     *
     * <p>For live tuning, call this method from the same OpMode-loop thread that calls
     * {@link #update(double, double, LoopClock)}, at one explicit apply boundary. Obtain one
     * coherently published candidate, copy its four values into loop-owned local variables once,
     * and invoke this method once. This method neither reads mutable UI fields nor provides
     * cross-thread synchronization or a cross-field atomic snapshot. Reading four independently
     * mutable fields once is not, by itself, an atomic tuple snapshot.</p>
     *
     * <p>For the documented live-tuning workflow, reset the robot-owned outermost retained
     * regulator composition in the same loop phase, before its next ordinary control update.
     * Resetting only this inner PIDF handle cannot clear state owned by enclosing decorators. Reset
     * does not itself command or stop an actuator; the mechanism owner still owns safe arming,
     * stop, known-good reapply, and the decision to apply a candidate.</p>
     *
     * @param kP new finite proportional gain
     * @param kI new finite integral gain
     * @param kD new finite derivative gain
     * @param kF new finite linear setpoint-feedforward gain
     * @return this regulator, for chaining or retained live-tuning use
     * @throws IllegalArgumentException if any gain is not finite
     */
    public PidfRegulator setGains(double kP, double kI, double kD, double kF) {
        validateFiniteGains("PidfRegulator.setGains(...)", kP, kI, kD, kF);
        pid.setGains(kP, kI, kD);
        this.kF = kF;
        return this;
    }

    /**
     * Set finite limits for the internal PID integral contribution.
     *
     * <p>The range must include zero so {@link #reset()} can clear the integral contribution
     * without changing configuration. A valid new range immediately clamps an existing finite
     * integral contribution. Non-finite dynamic controller state is deliberately not converted
     * into a finite limit value.</p>
     *
     * @param min finite minimum integral contribution; must be {@code <= 0}
     * @param max finite maximum integral contribution; must be {@code >= 0}
     * @return this regulator, for chaining
     * @throws IllegalArgumentException if either bound is non-finite, {@code min > max}, or the
     *                                  range does not include zero
     */
    public PidfRegulator setIntegralLimits(double min, double max) {
        Pid.validateIntegralLimits("PidfRegulator.setIntegralLimits(...)", min, max);
        pid.setIntegralLimits(min, max);
        return this;
    }

    /**
     * Set finite limits for the internal PID contribution before feedforward is added.
     *
     * <p>This constrains only {@code P + I + D}. It is deliberately distinct from
     * {@link ScalarRegulators#outputLimited(ScalarRegulator, double, double)}, which constrains the
     * complete wrapped result after feedforward and any other inner decorators. Both limits
     * saturate finite values only; non-finite PID math remains visible to the enclosing fail-stop
     * boundary.</p>
     *
     * @param min finite minimum PID contribution
     * @param max finite maximum PID contribution
     * @return this regulator, for chaining
     * @throws IllegalArgumentException if either bound is non-finite or {@code min > max}
     */
    public PidfRegulator setPidOutputLimits(double min, double max) {
        Pid.validateOrderedFiniteBounds("PidfRegulator.setPidOutputLimits(...)", min, max);
        pid.setOutputLimits(min, max);
        return this;
    }

    /**
     * Return the currently applied proportional gain.
     */
    public double getKP() {
        return pid.getKP();
    }

    /**
     * Return the currently applied integral gain.
     */
    public double getKI() {
        return pid.getKI();
    }

    /**
     * Return the currently applied derivative gain.
     */
    public double getKD() {
        return pid.getKD();
    }

    /**
     * Return the currently applied linear setpoint-feedforward gain.
     */
    public double getKF() {
        return kF;
    }

    /**
     * Compute {@code PID(setpoint - measurement, clock.dtSec()) + kF * setpoint}.
     *
     * <p>This layer adds no cycle memoization or setpoint/measurement substitution. It passes
     * {@link LoopClock#dtSec()} to the internal {@link Pid}, whose documented timing rule treats a
     * non-positive step as zero and advances integral/derivative terms only for a positive step.
     * Non-finite controller math is not converted into a finite limit value. Framework-regulated
     * Plants retain their final fail-stop command defense; direct callers own their output
     * boundary.</p>
     *
     * @param setpoint desired value in plant units
     * @param measurement measured value in the same plant units
     * @param clock current loop clock; its {@link LoopClock#dtSec()} drives the PID update
     * @return raw combined PIDF command
     */
    @Override
    public double update(double setpoint, double measurement, LoopClock clock) {
        lastSetpoint = setpoint;
        lastMeasurement = measurement;
        lastError = setpoint - measurement;
        lastPidOutput = Double.NaN;
        lastFeedforwardOutput = Double.NaN;
        lastOutput = Double.NaN;

        lastPidOutput = pid.update(lastError, clock.dtSec());
        lastFeedforwardOutput = kF * setpoint;
        lastOutput = lastPidOutput + lastFeedforwardOutput;
        return lastOutput;
    }

    /**
     * Clear transient PID history and last-sample diagnostics while preserving gains and limits.
     *
     * <p>Reset does not itself command or stop an actuator. When this object is wrapped by stateful
     * decorators, reset the outermost retained composition so every layer clears together.</p>
     */
    @Override
    public void reset() {
        pid.reset();
        lastSetpoint = Double.NaN;
        lastMeasurement = Double.NaN;
        lastError = Double.NaN;
        lastPidOutput = Double.NaN;
        lastFeedforwardOutput = Double.NaN;
        lastOutput = Double.NaN;
    }

    /**
     * Emit stable current-gain, last-sample, and nested PID diagnostic fields.
     *
     * @param dbg debug sink; may be {@code null}, in which case this method does nothing
     * @param prefix base key prefix; blank values use {@code pidfRegulator}
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "pidfRegulator" : prefix;
        dbg.addData(p + ".class", "PidfRegulator")
                .addData(p + ".kP", getKP())
                .addData(p + ".kI", getKI())
                .addData(p + ".kD", getKD())
                .addData(p + ".kF", kF)
                .addData(p + ".lastSetpoint", lastSetpoint)
                .addData(p + ".lastMeasurement", lastMeasurement)
                .addData(p + ".lastError", lastError)
                .addData(p + ".lastPidOutput", lastPidOutput)
                .addData(p + ".lastFeedforwardOutput", lastFeedforwardOutput)
                .addData(p + ".lastOutput", lastOutput);
        pid.debugDump(dbg, p + ".pid");
    }

    /**
     * Reject every non-finite standard PIDF gain while reporting the complete candidate.
     */
    private static void validateFiniteGains(String api,
                                            double kP,
                                            double kI,
                                            double kD,
                                            double kF) {
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
        if (!Double.isFinite(kF)) {
            invalid = appendSetting(invalid, "kF", kF);
        }
        if (!invalid.isEmpty()) {
            throw new IllegalArgumentException(
                    api + " requires finite gains; got kP=" + kP + ", kI=" + kI
                            + ", kD=" + kD + ", kF=" + kF
                            + "; non-finite setting(s): " + invalid);
        }
    }

    /**
     * Append one named invalid setting to an error-message fragment.
     */
    private static String appendSetting(String settings, String name, double value) {
        String entry = name + "=" + value;
        return settings.isEmpty() ? entry : settings + ", " + entry;
    }
}
