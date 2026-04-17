package edu.ftcphoenix.fw.core.control;

import java.util.Objects;
import java.util.function.DoubleUnaryOperator;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Helper factories for common {@link ScalarRegulator} implementations.
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
        Objects.requireNonNull(controller, "controller");
        return new ScalarRegulator() {
            @Override
            public double update(double setpoint, double measurement, LoopClock clock) {
                return controller.update(setpoint - measurement, clock.dtSec());
            }

            @Override
            public void reset() {
                controller.reset();
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (controller instanceof Pid) {
                    ((Pid) controller).debugDump(dbg, prefix);
                }
            }
        };
    }

    /**
     * Adapt an error-centric {@link PidController} with an additional feedforward term derived from
     * the current error.
     *
     * <p>This is intentionally lightweight: it covers common FTC cases such as adding a small static
     * term for stiction compensation while still keeping the public abstraction generic.</p>
     */
    public static ScalarRegulator pidf(PidController controller,
                                       DoubleUnaryOperator feedforwardFromError) {
        Objects.requireNonNull(controller, "controller");
        Objects.requireNonNull(feedforwardFromError, "feedforwardFromError");
        return new ScalarRegulator() {
            @Override
            public double update(double setpoint, double measurement, LoopClock clock) {
                double error = setpoint - measurement;
                return controller.update(error, clock.dtSec())
                        + feedforwardFromError.applyAsDouble(error);
            }

            @Override
            public void reset() {
                controller.reset();
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (controller instanceof Pid) {
                    ((Pid) controller).debugDump(dbg, prefix);
                }
            }
        };
    }
}
