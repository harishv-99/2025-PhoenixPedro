package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.control.DebounceBoolean;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;

/**
 * Task helpers for source-driven {@link Plant}s.
 *
 * <p>A plant task writes the plant's registered {@link ScalarTarget}; it never calls a plant setter.
 * Feedback-aware helpers such as {@link #moveTo(Plant, double)} then wait for
 * {@link Plant#atTarget(double)}, which prevents false success when a behavior overlay, static
 * bounds, target guard, or rate limiter is making the plant follow a different value.</p>
 *
 * <p>Build a task-driven plant with {@code targetedBy(ScalarTarget)},
 * {@code targetedByDefaultWritable(...)}, or {@code targetedBy(readOnlySource).writableTarget(...)}.
 * If a plant has only a read-only composed source, these helpers throw a clear error because there
 * is no single command variable for the task to write.</p>
 */
public final class PlantTasks {
    private PlantTasks() {
    }

    /**
     * Set the plant's writable target once and complete immediately.
     */
    public static Task setTarget(final Plant plant, final double target) {
        return ScalarTasks.set(writableTargetOf(plant, "setTarget"), target);
    }

    /**
     * Hold the plant's writable target at {@code target} for {@code seconds}, then leave it there.
     */
    public static Task holdTargetFor(final Plant plant, final double target, final double seconds) {
        return ScalarTasks.holdFor(writableTargetOf(plant, "holdTargetFor"), target, seconds);
    }

    /**
     * Hold the plant's writable target at {@code target} for {@code seconds}, then set {@code finalTarget}.
     */
    public static Task holdTargetForThen(final Plant plant, final double target, final double seconds, final double finalTarget) {
        return ScalarTasks.holdForThen(writableTargetOf(plant, "holdTargetForThen"), target, seconds, finalTarget);
    }

    /**
     * Move a feedback plant to {@code target} and finish when it is truly at that target.
     */
    public static Task moveTo(final Plant plant, final double target) {
        ensureFeedbackPlant(plant, "moveTo");
        return new MoveTask(plant, target, 0.0, -1.0, Double.NaN);
    }

    /**
     * Move a feedback plant to {@code target}, finishing on success or timeout.
     */
    public static Task moveTo(final Plant plant, final double target, final double timeoutSec) {
        ensureFeedbackPlant(plant, "moveTo");
        requirePositive(timeoutSec, "timeoutSec");
        return new MoveTask(plant, target, 0.0, timeoutSec, Double.NaN);
    }

    /**
     * Move to {@code target} and require the condition to remain true for {@code stableSec}.
     */
    public static Task moveToStable(final Plant plant, final double target, final double stableSec) {
        ensureFeedbackPlant(plant, "moveToStable");
        requireNonNegative(stableSec, "stableSec");
        return new MoveTask(plant, target, stableSec, -1.0, Double.NaN);
    }

    /**
     * Move to {@code target}, require stability, and give up after {@code timeoutSec}.
     */
    public static Task moveToStable(final Plant plant, final double target, final double stableSec, final double timeoutSec) {
        ensureFeedbackPlant(plant, "moveToStable");
        requireNonNegative(stableSec, "stableSec");
        requirePositive(timeoutSec, "timeoutSec");
        return new MoveTask(plant, target, stableSec, timeoutSec, Double.NaN);
    }

    /**
     * Move to {@code target}, then set {@code finalTarget} once after success or timeout.
     */
    public static Task moveToThen(final Plant plant, final double target, final double timeoutSec, final double finalTarget) {
        ensureFeedbackPlant(plant, "moveToThen");
        requirePositive(timeoutSec, "timeoutSec");
        return new MoveTask(plant, target, 0.0, timeoutSec, finalTarget);
    }

    /**
     * Move to {@code target}, require stability, then set {@code finalTarget} once.
     */
    public static Task moveToThen(final Plant plant, final double target, final double stableSec, final double timeoutSec, final double finalTarget) {
        ensureFeedbackPlant(plant, "moveToThen");
        requireNonNegative(stableSec, "stableSec");
        requirePositive(timeoutSec, "timeoutSec");
        return new MoveTask(plant, target, stableSec, timeoutSec, finalTarget);
    }

    private static ScalarTarget writableTargetOf(Plant plant, String method) {
        Objects.requireNonNull(plant, "plant");
        if (!plant.hasWritableTarget()) {
            throw new IllegalStateException("PlantTasks." + method + " requires a plant with a registered writable target. Build it with targetedBy(ScalarTarget), targetedByDefaultWritable(...), or targetedBy(readOnlySource).writableTarget(commandTarget).");
        }
        return plant.writableTarget();
    }

    private static void ensureFeedbackPlant(Plant plant, String method) {
        writableTargetOf(plant, method);
        if (!plant.hasFeedback()) {
            throw new IllegalStateException("PlantTasks." + method + " requires feedback so plant.atTarget(value) is meaningful.");
        }
    }

    private static void requirePositive(double value, String name) {
        if (!(value > 0.0) || !Double.isFinite(value))
            throw new IllegalArgumentException(name + " must be finite and > 0, got " + value);
    }

    private static void requireNonNegative(double value, String name) {
        if (value < 0.0 || !Double.isFinite(value))
            throw new IllegalArgumentException(name + " must be finite and >= 0, got " + value);
    }

    private static final class MoveTask implements Task {
        private final Plant plant;
        private final ScalarTarget writableTarget;
        private final double target;
        private final double stableSec;
        private final double timeoutSec;
        private final double finalTarget;
        private final boolean hasFinalTarget;
        private DebounceBoolean stableLatch;
        private boolean started;
        private boolean complete;
        private double elapsedSec;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;

        MoveTask(Plant plant, double target, double stableSec, double timeoutSec, double finalTarget) {
            this.plant = Objects.requireNonNull(plant, "plant");
            this.writableTarget = writableTargetOf(plant, "move");
            this.target = target;
            this.stableSec = stableSec;
            this.timeoutSec = timeoutSec;
            this.finalTarget = finalTarget;
            this.hasFinalTarget = Double.isFinite(finalTarget);
        }

        @Override
        public void start(LoopClock clock) {
            started = true;
            complete = false;
            elapsedSec = 0.0;
            outcome = TaskOutcome.NOT_DONE;
            writableTarget.set(target);
            stableLatch = stableSec > 0.0 ? DebounceBoolean.onAfterOffImmediately(stableSec) : null;
        }

        @Override
        public void update(LoopClock clock) {
            if (!started || complete) return;
            elapsedSec += clock != null ? clock.dtSec() : 0.0;
            boolean reached = plant.atTarget(target);
            boolean done = stableLatch != null ? stableLatch.update(clock, reached) : reached;
            if (done) finish(TaskOutcome.SUCCESS);
            else if (timeoutSec > 0.0 && elapsedSec >= timeoutSec) finish(TaskOutcome.TIMEOUT);
        }

        private void finish(TaskOutcome result) {
            if (hasFinalTarget) writableTarget.set(finalTarget);
            outcome = result;
            complete = true;
        }

        @Override
        public void cancel() {
            if (!complete) {
                outcome = TaskOutcome.CANCELLED;
                complete = true;
            }
        }

        @Override
        public boolean isComplete() {
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            return complete ? outcome : TaskOutcome.NOT_DONE;
        }

        @Override
        public String getDebugName() {
            return "PlantTasks.moveTo(" + target + ")";
        }
    }
}
