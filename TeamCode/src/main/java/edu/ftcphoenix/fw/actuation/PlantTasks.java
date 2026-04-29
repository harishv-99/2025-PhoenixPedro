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
 *
 * <h2>Guided builder usage</h2>
 * <pre>{@code
 * Task spinUp = PlantTasks.move(shooter)
 *         .to(3200.0)
 *         .stableFor(0.15)
 *         .timeout(1.5)
 *         .build();
 *
 * Task feed = PlantTasks.write(transfer)
 *         .to(1.0)
 *         .forSeconds(0.20)
 *         .then(0.0)
 *         .build();
 * }</pre>
 */
public final class PlantTasks {
    private PlantTasks() {
    }

    /**
     * Start a guided builder for writing a plant's registered target.
     *
     * <p>This is the plant-aware sibling of {@link ScalarTasks#write(ScalarTarget)}. It retrieves
     * the writable target from the plant, so callers cannot accidentally pass a different target
     * variable than the one the plant actually follows.</p>
     */
    public static ScalarTasks.WriteValueStep write(final Plant plant) {
        return ScalarTasks.write(writableTargetOf(plant, "write"));
    }

    /**
     * Start a guided builder for a feedback-aware move.
     *
     * <p>The resulting task writes the plant's registered target, then waits for
     * {@link Plant#atTarget(double)}. Optional builder steps add a stability requirement, timeout,
     * or final target to write after completion.</p>
     *
     * @param plant feedback-capable plant with a registered writable target
     * @return first builder step asking which target to request
     */
    public static MoveTargetStep move(final Plant plant) {
        ensureFeedbackPlant(plant, "move");
        return new MoveBuilder(plant);
    }

    /**
     * First feedback-move builder step: choose the target to request.
     */
    public interface MoveTargetStep {
        /**
         * Set the target that the task should request and wait for.
         */
        MoveReadyStep to(double target);
    }

    /**
     * Final feedback-move step with optional completion modifiers.
     */
    public interface MoveReadyStep {
        /**
         * Require {@link Plant#atTarget(double)} to stay true for {@code stableSec}.
         */
        MoveReadyStep stableFor(double stableSec);

        /**
         * Finish with {@link TaskOutcome#TIMEOUT} if the plant has not reached the target in time.
         */
        MoveReadyStep timeout(double timeoutSec);

        /**
         * Write {@code finalTarget} once after success or timeout.
         */
        MoveReadyStep thenTarget(double finalTarget);

        /**
         * Build the configured feedback-aware move task.
         */
        Task build();
    }

    private static final class MoveBuilder implements MoveTargetStep, MoveReadyStep {
        private final Plant plant;
        private double target = Double.NaN;
        private double stableSec = 0.0;
        private double timeoutSec = -1.0;
        private double finalTarget = Double.NaN;

        private MoveBuilder(Plant plant) {
            this.plant = Objects.requireNonNull(plant, "plant");
        }

        @Override
        public MoveReadyStep to(double target) {
            requireFinite(target, "target");
            this.target = target;
            return this;
        }

        @Override
        public MoveReadyStep stableFor(double stableSec) {
            requireNonNegative(stableSec, "stableSec");
            this.stableSec = stableSec;
            return this;
        }

        @Override
        public MoveReadyStep timeout(double timeoutSec) {
            requirePositive(timeoutSec, "timeoutSec");
            this.timeoutSec = timeoutSec;
            return this;
        }

        @Override
        public MoveReadyStep thenTarget(double finalTarget) {
            requireFinite(finalTarget, "finalTarget");
            this.finalTarget = finalTarget;
            return this;
        }

        @Override
        public Task build() {
            if (!Double.isFinite(target)) {
                throw new IllegalStateException("PlantTasks.move(...).to(target) is required before build()");
            }
            return new MoveTask(plant, target, stableSec, timeoutSec, finalTarget);
        }
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
        return move(plant).to(target).build();
    }

    /**
     * Move a feedback plant to {@code target}, finishing on success or timeout.
     */
    public static Task moveTo(final Plant plant, final double target, final double timeoutSec) {
        return move(plant).to(target).timeout(timeoutSec).build();
    }

    /**
     * Move to {@code target} and require the condition to remain true for {@code stableSec}.
     */
    public static Task moveToStable(final Plant plant, final double target, final double stableSec) {
        return move(plant).to(target).stableFor(stableSec).build();
    }

    /**
     * Move to {@code target}, require stability, and give up after {@code timeoutSec}.
     */
    public static Task moveToStable(final Plant plant, final double target, final double stableSec, final double timeoutSec) {
        return move(plant).to(target).stableFor(stableSec).timeout(timeoutSec).build();
    }

    /**
     * Move to {@code target}, then set {@code finalTarget} once after success or timeout.
     */
    public static Task moveToThen(final Plant plant, final double target, final double timeoutSec, final double finalTarget) {
        return move(plant).to(target).timeout(timeoutSec).thenTarget(finalTarget).build();
    }

    /**
     * Move to {@code target}, require stability, then set {@code finalTarget} once.
     */
    public static Task moveToThen(final Plant plant, final double target, final double stableSec, final double timeoutSec, final double finalTarget) {
        return move(plant).to(target).stableFor(stableSec).timeout(timeoutSec).thenTarget(finalTarget).build();
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

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value))
            throw new IllegalArgumentException(name + " must be finite, got " + value);
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
            return "PlantTasks.move(" + target + ")";
        }
    }
}
