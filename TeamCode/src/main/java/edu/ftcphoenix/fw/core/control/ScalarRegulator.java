package edu.ftcphoenix.fw.core.control;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Generic framework-owned scalar regulator.
 *
 * <p>A {@code ScalarRegulator} converts a setpoint and measurement into an actuator command. It is
 * the control-law seam used by framework-regulated plants such as position-from-power or
 * velocity-from-power plants.</p>
 *
 * <p>This interface is intentionally broader than plain PID. A PID controller or the standard
 * {@link PidfRegulator} is one possible implementation, but so are nonlinear
 * feedforward-plus-feedback blends, asymmetric up/down regulators, voltage compensation
 * decorators, or other custom scalar control laws.</p>
 *
 * <p>A {@link Pid#setOutputLimits(double, double) PID output limit} or
 * {@link PidfRegulator#setPidOutputLimits(double, double) PIDF's PID-contribution limit} bounds only
 * a finite {@code P + I + D} result; neither turns non-finite controller math into a boundary
 * value. If feedforward or later decorators add or scale output, put
 * {@link ScalarRegulators#outputLimited(ScalarRegulator, double, double)} outermost when the complete
 * composed result needs an intentional narrower range. That policy limit is distinct from Plant
 * target bounds and from the enclosing output boundary's universal command safety.</p>
 *
 * <p>A regulator is allowed to express a generic raw command; it does not need a redundant
 * {@code [-1,+1]} policy decorator solely for actuator safety. An enclosing framework-regulated
 * Plant requires a finite result and normalizes the final command to the {@code PowerOutput}
 * domain. A non-finite result or runtime failure causes that Plant to perform fail-stop cleanup and
 * propagate an actionable failure. Direct callers of this interface own their own output-boundary
 * validation and cleanup.</p>
 *
 * <p>An outer output limiter cannot provide generic saturation-aware anti-windup for an arbitrary
 * inner controller. Controller-specific integral limits remain explicit, and the robot mechanism
 * owner still decides enable, coast/hold, and reset policy.</p>
 *
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * ScalarRegulator regulator = ScalarRegulators.outputLimited(
 *     ScalarRegulators.pidf(0.006, 0.0, 0.0002, 0.0004)
 *         .setIntegralLimits(-0.15, 0.15)
 *         .setPidOutputLimits(-1.0, 1.0),
 *     0.0,
 *     0.8
 * );
 *
 * double command = regulator.update(setpointTicks, measuredTicks, clock);
 * }</pre>
 */
public interface ScalarRegulator {

    /**
     * Compute the next actuator command from the current setpoint and measurement.
     *
     * <p>Unless an implementation explicitly documents cycle memoization, every invocation may
     * advance controller state, including repeated calls with the same
     * {@link LoopClock#cycle()}. Normal owners should invoke a stateful regulator once per intended
     * control update.</p>
     *
     * @param setpoint    desired value in plant units
     * @param measurement measured value in the same units as {@code setpoint}
     * @param clock       current loop clock for the cycle being processed
     * @return raw actuator command; units are chosen by the enclosing plant and usually correspond
     * to its actuator command channel. Framework-regulated power Plants require a finite result and
     * normalize it to {@code [-1.0, +1.0]} before submitting it to hardware.
     */
    double update(double setpoint, double measurement, LoopClock clock);

    /**
     * Reset any transient internal controller state such as integrators or derivative history.
     *
     * <p>Resetting a regulator does not itself command or stop an actuator. The enclosing Plant owns
     * output lifecycle and failure cleanup.</p>
     */
    default void reset() {
        // default: no transient state
    }

    /**
     * Optional debug hook for regulator-specific state.
     *
     * @param dbg    debug sink that should receive any emitted fields; may be {@code null}
     * @param prefix base key prefix to use for emitted debug fields
     */
    default void debugDump(DebugSink dbg, String prefix) {
        // default: no-op
    }
}
