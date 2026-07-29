package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;

/**
 * Task helpers for source-driven {@link Plant}s.
 *
 * <p>A plant task writes the plant's graph-owned command {@link ScalarTarget}; it never calls a
 * plant setter. For framework {@link PlantTargets} graphs, feedback-aware moves require that
 * logical command to be the winning target-graph path and then wait for physical arrival at the
 * selected requested target. This supports periodic equivalent positions while preventing false
 * success from a same-valued overlay, fallback, planner clamp, static bound, target guard, or rate
 * limiter. A custom Plant that does not publish framework-private plan evidence retains its own
 * {@link Plant#atTarget(double)} completion contract.</p>
 *
 * <p>Build a task-driven plant from a {@code ScalarTarget}, from an overlay whose stable base is a
 * {@code ScalarTarget}, or with the FTC builder's {@code targetedByCommand(...)} shortcut. If a
 * plant has only a read-only source, these helpers throw a clear error because there is no stable
 * command variable for the task to write.</p>
 *
 * <h2>Guided builder usage</h2>
 * <pre>{@code
 * Task spinUp = PlantTasks.move(shooter)
 *         .to(3200.0)
 *         .cancelTo(0.0)
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
     * Start a guided builder for writing a plant's command target.
     *
     * <p>This is the plant-aware sibling of {@link ScalarTasks#write(ScalarTarget)}. It retrieves
     * the command target from the plant, so callers cannot accidentally pass a different target
     * variable than the one the plant actually follows.</p>
     */
    public static ScalarTasks.WriteValueStep write(final Plant plant) {
        return ScalarTasks.write(commandTargetOf(plant, "write"));
    }

    /**
     * Start a guided builder for a feedback-aware move.
     *
     * <p>The resulting task writes the plant's logical command target. A framework target graph may
     * resolve that value to a different physical equivalent; completion then requires both
     * winning-path command evidence and {@link Plant#atTarget(double)} for that resolved physical
     * target. Custom Plants without that framework evidence use their existing
     * {@code atTarget(requestedLogicalValue)} contract. After choosing the move target, callers must
     * explicitly choose whether active cancellation writes a different Plant-unit request or
     * deliberately leaves the move request in place. Optional builder steps then add a stability
     * requirement, timeout, or final target.</p>
     *
     * @param plant feedback-capable plant with a graph-owned command target
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
         *
         * @param target finite requested target expressed in Plant units
         * @throws IllegalArgumentException if {@code target} is not finite
         */
        MoveCancellationStep to(double target);
    }

    /**
     * Required feedback-move step: choose what happens if the active task is cancelled.
     *
     * <p>Cancellation changes the Plant's command request; the Plant still owns final
     * target resolution, bounds, guards, and hardware application on its next update.</p>
     */
    public interface MoveCancellationStep {
        /**
         * On active cancellation, write {@code target} once to the Plant's command target.
         *
         * @param target finite cancellation request expressed in Plant units
         * @return optional completion-modifier step
         * @throws IllegalArgumentException if {@code target} is not finite
         */
        MoveReadyStep cancelTo(double target);

        /**
         * Deliberately leave the requested move target unchanged on cancellation.
         *
         * <p>The mechanism may continue moving unless a robot-owned coordinator changes the final
         * target source or otherwise disables the behavior.</p>
         *
         * @return optional completion-modifier step
         */
        MoveReadyStep leaveTargetOnCancel();
    }

    /**
     * Final feedback-move step with optional completion modifiers.
     */
    public interface MoveReadyStep {
        /**
         * Require the move's matching logical-command path and physical arrival to remain true for
         * {@code stableSec}, measured from the first loop that observes both conditions.
         */
        MoveReadyStep stableFor(double stableSec);

        /**
         * Finish with {@link TaskOutcome#TIMEOUT} if the plant has not reached the target within
         * {@code timeoutSec} after this task starts. Time before the start call is not counted.
         */
        MoveReadyStep timeout(double timeoutSec);

        /**
         * Write {@code finalTarget} once after success or timeout, never because of cancellation.
         *
         * @param finalTarget finite completion request expressed in Plant units
         * @throws IllegalArgumentException if {@code finalTarget} is not finite
         */
        MoveReadyStep thenTarget(double finalTarget);

        /**
         * Build a new single-use feedback-aware move task.
         *
         * <p>Build a fresh task or macro for each repeated move request.</p>
         */
        Task build();
    }

    private static final class MoveBuilder implements MoveTargetStep, MoveCancellationStep, MoveReadyStep {
        private final Plant plant;
        private double target = Double.NaN;
        private boolean hasCancellationTarget;
        private double cancellationTarget = Double.NaN;
        private double stableSec = 0.0;
        private double timeoutSec = -1.0;
        private double finalTarget = Double.NaN;

        private MoveBuilder(Plant plant) {
            this.plant = Objects.requireNonNull(plant, "plant");
        }

        @Override
        public MoveCancellationStep to(double target) {
            requireFinite(target, "target");
            this.target = target;
            return this;
        }

        @Override
        public MoveReadyStep cancelTo(double target) {
            requireFinite(target, "cancellation target");
            this.hasCancellationTarget = true;
            this.cancellationTarget = target;
            return this;
        }

        @Override
        public MoveReadyStep leaveTargetOnCancel() {
            this.hasCancellationTarget = false;
            this.cancellationTarget = Double.NaN;
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
            return new MoveTask(plant, target, hasCancellationTarget, cancellationTarget,
                    stableSec, timeoutSec, finalTarget);
        }
    }

    /**
     * Set the plant's command target once and complete immediately.
     */
    public static Task setTarget(final Plant plant, final double target) {
        return ScalarTasks.set(commandTargetOf(plant, "setTarget"), target);
    }

    /**
     * Hold the plant's command target at {@code target} for {@code seconds}, then leave it there.
     * Active cancellation also leaves the command request at {@code target}.
     */
    public static Task holdTargetFor(final Plant plant, final double target, final double seconds) {
        return ScalarTasks.holdFor(commandTargetOf(plant, "holdTargetFor"), target, seconds);
    }

    /**
     * Hold the plant's command target at {@code target} for {@code seconds}, then set
     * {@code finalTarget}. Active cancellation also applies {@code finalTarget}.
     */
    public static Task holdTargetForThen(final Plant plant, final double target, final double seconds, final double finalTarget) {
        return ScalarTasks.holdForThen(commandTargetOf(plant, "holdTargetForThen"), target, seconds, finalTarget);
    }

    private static ScalarTarget commandTargetOf(Plant plant, String method) {
        Objects.requireNonNull(plant, "plant");
        if (!plant.hasCommandTarget()) {
            throw new IllegalStateException("PlantTasks." + method + " requires a plant with a command target. Build it from a ScalarTarget, use an overlay whose stable base is a ScalarTarget, or use the FTC builder's targetedByCommand(...).");
        }
        return plant.commandTarget();
    }

    private static void ensureFeedbackPlant(Plant plant, String method) {
        commandTargetOf(plant, method);
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
        private final ScalarTarget commandTarget;
        private final double target;
        private final boolean hasCancellationTarget;
        private final double cancellationTarget;
        private final double stableSec;
        private final double timeoutSec;
        private final double finalTarget;
        private final boolean hasFinalTarget;
        private boolean startAttempted;
        private boolean started;
        private boolean complete;
        private double startSec;
        private double stableSinceSec;
        private double elapsedSec;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;

        MoveTask(Plant plant,
                 double target,
                 boolean hasCancellationTarget,
                 double cancellationTarget,
                 double stableSec,
                 double timeoutSec,
                 double finalTarget) {
            this.plant = Objects.requireNonNull(plant, "plant");
            this.commandTarget = commandTargetOf(plant, "move");
            this.target = target;
            this.hasCancellationTarget = hasCancellationTarget;
            this.cancellationTarget = cancellationTarget;
            this.stableSec = stableSec;
            this.timeoutSec = timeoutSec;
            this.finalTarget = finalTarget;
            this.hasFinalTarget = Double.isFinite(finalTarget);
        }

        @Override
        public void start(LoopClock clock) {
            if (startAttempted) {
                throw new IllegalStateException("PlantTasks.move(" + target + ") is single-use and "
                        + "cannot be started more than once. Create a fresh Task with "
                        + "the guided PlantTasks.move(...) builder or rebuild the macro; use a "
                        + "Supplier<Task> for repeated scheduling.");
            }
            startAttempted = true;
            started = true;
            complete = false;
            startSec = nowSec(clock, 0.0);
            stableSinceSec = Double.NaN;
            elapsedSec = 0.0;
            outcome = TaskOutcome.NOT_DONE;
            commandTarget.set(target);
        }

        @Override
        public void update(LoopClock clock) {
            if (!started) {
                throw new IllegalStateException("PlantTasks.move(" + target + ") cannot be updated "
                        + "before start(clock). Start it first, normally by enqueueing it in a "
                        + "TaskRunner.");
            }
            if (complete) return;
            double nowSec = nowSec(clock, startSec);
            elapsedSec = elapsedSince(startSec, nowSec);
            PlantTargetPlan plan = plant.getTargetPlan();
            boolean reached;
            if (plan != null && plan.reportsCommandResolutionFor(commandTarget)) {
                reached = plan.satisfiesCommand(commandTarget, target)
                        && commandTarget.get() == target
                        && plant.atTarget(plan.target());
            } else {
                // Custom Plants that do not publish framework command provenance keep the
                // original exact-target completion contract.
                reached = plant.atTarget(target);
            }
            if (complete) return;
            boolean done;
            if (stableSec <= 0.0) {
                done = reached;
            } else if (!reached) {
                stableSinceSec = Double.NaN;
                done = false;
            } else {
                if (!Double.isFinite(stableSinceSec)) {
                    stableSinceSec = nowSec;
                }
                done = elapsedSince(stableSinceSec, nowSec) >= stableSec;
            }
            if (done) finish(TaskOutcome.SUCCESS);
            else if (timeoutSec > 0.0 && elapsedSec >= timeoutSec) finish(TaskOutcome.TIMEOUT);
        }

        private static double nowSec(LoopClock clock, double fallbackSec) {
            return clock != null ? clock.nowSec() : fallbackSec;
        }

        private static double elapsedSince(double intervalStartSec, double nowSec) {
            return Math.max(0.0, nowSec - intervalStartSec);
        }

        private void finish(TaskOutcome result) {
            if (hasFinalTarget) {
                commandTarget.set(finalTarget);
                if (complete) return;
            }
            outcome = result;
            complete = true;
        }

        @Override
        public void cancel() {
            if (!started || complete) return;

            // Become terminal before the optional external write so a throwing target is not
            // written again by repeated cancellation or runner failure cleanup.
            outcome = TaskOutcome.CANCELLED;
            complete = true;
            if (hasCancellationTarget) commandTarget.set(cancellationTarget);
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
