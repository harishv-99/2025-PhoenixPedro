package edu.ftcphoenix.fw.task;

import java.util.Objects;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;

/**
 * Convenience factory helpers for {@link OutputTask}.
 *
 * <p>This mirrors {@link Tasks}, but for tasks that produce a scalar output. The typical pattern is:
 *
 * <ol>
 *   <li>Use {@link OutputTasks} to build small output-producing tasks (feed pulses, gated feeds,
 *       spit-outs, etc.).</li>
 *   <li>Enqueue them into an {@link OutputTaskRunner}.</li>
 *   <li>Apply the runner's output to a Plant in your subsystem loop.</li>
 * </ol>
 */
public final class OutputTasks {

    private OutputTasks() {
        // utility class
    }

    // ------------------------------------------------------------------------------------------------
    // Simple primitives
    // ------------------------------------------------------------------------------------------------

    /**
     * Output a constant value for a fixed duration.
     */
    public static OutputTask outputForSeconds(String name, double output, double durationSec) {
        return new OutputForSecondsTask(name, output, durationSec);
    }

    /**
     * Alias for {@link #outputForSeconds(String, double, double)}.
     */
    public static OutputTask pulse(String name, double output, double durationSec) {
        return outputForSeconds(name, output, durationSec);
    }

    // ------------------------------------------------------------------------------------------------
    // Gated primitives
    // ------------------------------------------------------------------------------------------------

    /**
     * Wait for {@code startWhen}, then output {@code runOutput} until {@code doneWhen} is satisfied
     * (and {@code minRunSec} has elapsed), or until {@code maxRunSec} elapses.
     */
    public static OutputTask gatedUntil(String name,
                                        BooleanSource startWhen,
                                        BooleanSource doneWhen,
                                        ScalarSource runOutput,
                                        double idleOutput,
                                        double minRunSec,
                                        double maxRunSec,
                                        double cooldownSec) {

        Objects.requireNonNull(startWhen, "startWhen");
        Objects.requireNonNull(doneWhen, "doneWhen");
        Objects.requireNonNull(runOutput, "runOutput");

        return new GatedOutputUntilTask(name, startWhen, doneWhen, runOutput, idleOutput, minRunSec, maxRunSec, cooldownSec);
    }

    /**
     * Convenience overload for constant run output and common defaults.
     */
    public static OutputTask gatedUntil(String name,
                                        BooleanSource startWhen,
                                        BooleanSource doneWhen,
                                        double runOutput,
                                        double minRunSec,
                                        double maxRunSec) {

        return gatedUntil(name,
                startWhen,
                doneWhen,
                ScalarSource.constant(runOutput),
                0.0,
                minRunSec,
                maxRunSec,
                0.0);
    }

    /**
     * Convenience overload: output while a condition is true, up to a timeout.
     *
     * <p>This is useful when you want to "run until the sensor says stop" but still have a safety cap.
     */
    public static OutputTask runUntil(String name,
                                      BooleanSource doneWhen,
                                      double runOutput,
                                      double maxRunSec) {

        return gatedUntil(name,
                BooleanSource.constant(true),
                doneWhen,
                ScalarSource.constant(runOutput),
                0.0,
                0.0,
                maxRunSec,
                0.0);
    }
}
