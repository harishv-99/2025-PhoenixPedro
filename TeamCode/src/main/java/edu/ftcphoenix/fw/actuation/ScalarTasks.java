package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.RunForSecondsTask;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;

/**
 * Task helpers for writing {@link ScalarTarget} values.
 *
 * <p>These tasks do not know about hardware feedback. They are useful for behavior variables,
 * command targets, and open-loop pulses. For tasks that also wait on plant feedback, use
 * {@link PlantTasks}.</p>
 */
public final class ScalarTasks {
    private ScalarTasks() {
    }

    /**
     * Set a scalar target once and complete immediately.
     */
    public static Task set(final ScalarTarget target, final double value) {
        Objects.requireNonNull(target, "target");
        return new Task() {
            private boolean done;

            @Override
            public void start(LoopClock clock) {
                target.set(value);
                done = true;
            }

            @Override
            public void update(LoopClock clock) {
            }

            @Override
            public boolean isComplete() {
                return done;
            }

            @Override
            public TaskOutcome getOutcome() {
                return done ? TaskOutcome.SUCCESS : TaskOutcome.NOT_DONE;
            }

            @Override
            public String getDebugName() {
                return "ScalarTasks.set(" + value + ")";
            }
        };
    }

    /**
     * Hold a scalar target at {@code value} for a duration, then leave it there.
     */
    public static Task holdFor(final ScalarTarget target, final double value, final double seconds) {
        Objects.requireNonNull(target, "target");
        return new RunForSecondsTask(seconds, () -> target.set(value), clock -> target.set(value), null);
    }

    /**
     * Hold a scalar target at {@code value} for a duration, then set {@code finalValue}.
     */
    public static Task holdForThen(final ScalarTarget target,
                                   final double value,
                                   final double seconds,
                                   final double finalValue) {
        Objects.requireNonNull(target, "target");
        return new RunForSecondsTask(seconds, () -> target.set(value), clock -> target.set(value), () -> target.set(finalValue));
    }
}
