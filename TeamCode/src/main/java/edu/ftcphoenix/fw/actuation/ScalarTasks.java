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
 * command targets, and open-loop output requests. For tasks that also wait on plant feedback, use
 * {@link PlantTasks}.</p>
 *
 * <h2>Guided builder usage</h2>
 * <pre>{@code
 * ScalarTarget feederPower = ScalarTarget.held(0.0);
 *
 * Task feedPulse = ScalarTasks.write(feederPower)
 *         .to(0.8)
 *         .forSeconds(0.20)
 *         .then(0.0)
 *         .build();
 *
 * Task stop = ScalarTasks.write(feederPower)
 *         .to(0.0)
 *         .build();
 * }</pre>
 */
public final class ScalarTasks {
    private ScalarTasks() {
    }

    /**
     * Start a guided task builder for writing one {@link ScalarTarget}.
     *
     * <p>The builder is useful when students are choosing between “set once”, “hold for time”, and
     * “hold then return somewhere else”. The existing static helpers remain for compact code.</p>
     *
     * @param target writable scalar target to command
     * @return first builder step asking which value should be written
     */
    public static WriteValueStep write(final ScalarTarget target) {
        return new WriteBuilder(Objects.requireNonNull(target, "target"));
    }

    /**
     * First scalar-write builder step: choose the requested value.
     */
    public interface WriteValueStep {
        /**
         * Choose the value that this task should write.
         */
        WriteReadyStep to(double value);
    }

    /**
     * Final scalar-write step for an immediate write, or branch into a timed hold.
     */
    public interface WriteReadyStep {
        /**
         * Build a task that writes the value once and completes immediately.
         */
        Task build();

        /**
         * Hold the value for {@code seconds} before completing.
         */
        WriteTimedStep forSeconds(double seconds);
    }

    /**
     * Timed scalar-write step.
     */
    public interface WriteTimedStep {
        /**
         * After the duration, leave the scalar target at the held value.
         */
        WriteCompleteStep leaveThere();

        /**
         * After the duration, write {@code finalValue}.
         */
        WriteCompleteStep then(double finalValue);

        /**
         * Build a hold task that leaves the scalar target at the held value.
         */
        Task build();
    }

    /**
     * Final build step after the timed hold's ending behavior has been chosen.
     */
    public interface WriteCompleteStep {
        /**
         * Build the configured scalar-write task.
         */
        Task build();
    }

    private static final class WriteBuilder implements WriteValueStep, WriteReadyStep, WriteTimedStep, WriteCompleteStep {
        private final ScalarTarget target;
        private double value;
        private double seconds = Double.NaN;
        private boolean hasFinalValue;
        private double finalValue;

        private WriteBuilder(ScalarTarget target) {
            this.target = target;
        }

        @Override
        public WriteReadyStep to(double value) {
            requireFinite(value, "value");
            this.value = value;
            return this;
        }

        @Override
        public WriteTimedStep forSeconds(double seconds) {
            requireNonNegativeFinite(seconds, "seconds");
            this.seconds = seconds;
            return this;
        }

        @Override
        public WriteCompleteStep leaveThere() {
            this.hasFinalValue = false;
            return this;
        }

        @Override
        public WriteCompleteStep then(double finalValue) {
            requireFinite(finalValue, "finalValue");
            this.hasFinalValue = true;
            this.finalValue = finalValue;
            return this;
        }

        @Override
        public Task build() {
            if (Double.isNaN(seconds)) {
                return ScalarTasks.set(target, value);
            }
            return hasFinalValue
                    ? ScalarTasks.holdForThen(target, value, seconds, finalValue)
                    : ScalarTasks.holdFor(target, value, seconds);
        }
    }

    /**
     * Set a scalar target once and complete immediately.
     */
    public static Task set(final ScalarTarget target, final double value) {
        Objects.requireNonNull(target, "target");
        requireFinite(value, "value");
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
        requireFinite(value, "value");
        requireNonNegativeFinite(seconds, "seconds");
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
        requireFinite(value, "value");
        requireNonNegativeFinite(seconds, "seconds");
        requireFinite(finalValue, "finalValue");
        return new RunForSecondsTask(seconds, () -> target.set(value), clock -> target.set(value), () -> target.set(finalValue));
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
    }

    private static void requireNonNegativeFinite(double value, String name) {
        if (value < 0.0 || !Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite and >= 0, got " + value);
        }
    }
}
