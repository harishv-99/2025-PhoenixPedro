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
 * <p>This interface is intentionally broader than plain PID. A PID controller is one possible
 * implementation, but so are feedforward-plus-PID blends, asymmetric up/down regulators, or other
 * custom scalar control laws.</p>
 *
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * ScalarRegulator regulator = ScalarRegulators.pid(
 *     Pid.withGains(0.006, 0.0, 0.0002).setOutputLimits(-0.8, 0.8)
 * );
 *
 * double command = regulator.update(setpointTicks, measuredTicks, clock);
 * }</pre>
 */
public interface ScalarRegulator {

    /**
     * Compute the next actuator command from the current setpoint and measurement.
     *
     * @param setpoint    desired value in plant units
     * @param measurement measured value in the same units as {@code setpoint}
     * @param clock       current loop clock for the cycle being processed
     * @return actuator command; units are chosen by the enclosing plant and usually correspond to the
     * raw actuator command channel (for example normalized power)
     */
    double update(double setpoint, double measurement, LoopClock clock);

    /**
     * Reset any transient internal controller state such as integrators or derivative history.
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
