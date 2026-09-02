package edu.ftcsushi.fw.actuation;

import java.util.Objects;

import edu.ftcsushi.fw.core.source.ScalarTarget;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.task.RunForSecondsTask;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;

/**
 * Task builders for changing one {@link ScalarTarget}.
 *
 * <p>Use this factory when the scalar value is the complete capability request, such as normalized
 * roller power, flywheel RPM, an explicitly numeric position, or a servo position. Robot methods
 * use {@link ScalarTarget#set(double)} for an immediate persistent scalar request; Tasks use this
 * staged entry point when that same request should be made at Task start, retained for a duration,
 * or observed through a feedback-capable {@link Plant}. Builder calls are side-effect free: the
 * target is not written until the built Task starts.</p>
 *
 * <p>If a mechanism exposes richer named intent such as {@code Height}, {@code Mode}, or a semantic
 * pose, its owner must map and publish that semantic/numeric request through one authoritative
 * setter. Compose a Task from that setter and the owner's status; do not use a raw numeric
 * {@code ScalarTasks} write to bypass the semantic owner.</p>
 *
 * <h2>Examples</h2>
 * <pre>{@code
 * Task setOnce = ScalarTasks.set(release.commandTarget(), RELEASED_POSITION).build();
 *
 * Task pulse = ScalarTasks.set(roller.commandTarget(), COLLECT_POWER)
 *         .forSeconds(0.75)
 *         .then(STOPPED_POWER)
 *         .build();
 *
 * Task move = ScalarTasks.set(flywheel.commandTarget(), SHOT_RPM)
 *         .untilReachedBy(flywheel)
 *         .cancelTo(STOPPED_RPM)
 *         .stableFor(0.10)
 *         .timeout(1.5)
 *         .build();
 * }</pre>
 */
public final class ScalarTasks {
    private ScalarTasks() {
    }

    /**
     * Start a side-effect-free Task builder for setting {@code target} to {@code value}.
     *
     * <p>Call {@link SetReadyStep#build()} for a write-once Task, or select one of the timed or
     * feedback-aware lifetime branches. Every call to {@code build()} returns a fresh single-use
     * Task.</p>
     *
     * @param target writable request changed when the Task starts
     * @param value finite requested value
     * @return lifetime-selection stage
     * @throws NullPointerException if {@code target} is null
     * @throws IllegalArgumentException if {@code value} is not finite
     */
    public static SetReadyStep set(ScalarTarget target, double value) {
        requireFinite(value, "value");
        return new SetBuilder(Objects.requireNonNull(target, "target"), value);
    }

    /** Choose whether this set is write-once, timed, or feedback-aware. */
    public interface SetReadyStep {
        /**
         * Build a Task that writes the requested value once at start and completes successfully.
         */
        Task build();

        /**
         * Reassert the requested value for {@code seconds}, then require an explicit ending policy.
         * A positive duration remains observable by the downstream Plant phase at least once.
         */
        TimedEndStep forSeconds(double seconds);

        /**
         * Write once, then wait for this Plant to reach the command-correlated resolved target.
         *
         * <p>The Plant must expose this exact target by object identity and have authoritative
         * feedback. Selecting this branch does not sample or write the target.</p>
         */
        ReachedCancellationStep untilReachedBy(Plant plant);
    }

    /** Required ending policy for a timed set. */
    public interface TimedEndStep {
        /** Leave the requested value in place after natural completion or active cancellation. */
        TimedBuildStep leaveThere();

        /** Write {@code finalValue} after natural completion or active cancellation. */
        TimedBuildStep then(double finalValue);
    }

    /** Final build stage for a timed set. */
    public interface TimedBuildStep {
        /** Build a fresh single-use timed Task. */
        Task build();
    }

    /** Required active-cancellation policy for a feedback-aware set. */
    public interface ReachedCancellationStep {
        /**
         * Write {@code target} once if the active Task is cancelled.
         * The Plant still resolves and applies that request on its normal update path.
         */
        ReachedReadyStep cancelTo(double target);

        /**
         * Deliberately leave the requested value unchanged if the active Task is cancelled.
         * The mechanism may continue moving until another owner changes its request.
         */
        ReachedReadyStep leaveTargetOnCancel();
    }

    /** Optional completion policy for a feedback-aware set. */
    public interface ReachedReadyStep {
        /** Require command-correlated arrival to remain true for {@code seconds}. */
        ReachedReadyStep stableFor(double seconds);

        /** Complete with {@link TaskOutcome#TIMEOUT} after {@code seconds} if not reached. */
        ReachedReadyStep timeout(double seconds);

        /**
         * Write {@code target} after success or timeout, never because of cancellation.
         * Cancellation follows the required cancellation policy instead.
         */
        ReachedReadyStep thenTarget(double target);

        /** Build a fresh single-use feedback-aware Task. */
        Task build();
    }

    private static final class SetBuilder implements SetReadyStep {
        private final ScalarTarget target;
        private final double value;

        private SetBuilder(ScalarTarget target, double value) {
            this.target = target;
            this.value = value;
        }

        @Override
        public Task build() {
            return new SetOnceTask(target, value);
        }

        @Override
        public TimedEndStep forSeconds(double seconds) {
            requireNonNegativeFinite(seconds, "seconds");
            return new TimedEndBuilder(target, value, seconds);
        }

        @Override
        public ReachedCancellationStep untilReachedBy(Plant plant) {
            validateFeedbackObserver(target, plant);
            return new ReachedCancellationBuilder(target, value, plant);
        }
    }

    private static final class TimedEndBuilder implements TimedEndStep {
        private final ScalarTarget target;
        private final double value;
        private final double seconds;

        private TimedEndBuilder(ScalarTarget target, double value, double seconds) {
            this.target = target;
            this.value = value;
            this.seconds = seconds;
        }

        @Override
        public TimedBuildStep leaveThere() {
            return new TimedBuilder(target, value, seconds, false, Double.NaN);
        }

        @Override
        public TimedBuildStep then(double finalValue) {
            requireFinite(finalValue, "finalValue");
            return new TimedBuilder(target, value, seconds, true, finalValue);
        }
    }

    private static final class TimedBuilder implements TimedBuildStep {
        private final ScalarTarget target;
        private final double value;
        private final double seconds;
        private final boolean hasFinalValue;
        private final double finalValue;

        private TimedBuilder(ScalarTarget target,
                             double value,
                             double seconds,
                             boolean hasFinalValue,
                             double finalValue) {
            this.target = target;
            this.value = value;
            this.seconds = seconds;
            this.hasFinalValue = hasFinalValue;
            this.finalValue = finalValue;
        }

        @Override
        public Task build() {
            return new RunForSecondsTask(
                    seconds,
                    () -> target.set(value),
                    clock -> target.set(value),
                    hasFinalValue ? () -> target.set(finalValue) : null);
        }
    }

    private static final class ReachedCancellationBuilder implements ReachedCancellationStep {
        private final ScalarTarget target;
        private final double value;
        private final Plant plant;

        private ReachedCancellationBuilder(ScalarTarget target, double value, Plant plant) {
            this.target = target;
            this.value = value;
            this.plant = plant;
        }

        @Override
        public ReachedReadyStep cancelTo(double cancellationTarget) {
            requireFinite(cancellationTarget, "cancellation target");
            return ReachedBuilder.initial(target, value, plant, true, cancellationTarget);
        }

        @Override
        public ReachedReadyStep leaveTargetOnCancel() {
            return ReachedBuilder.initial(target, value, plant, false, Double.NaN);
        }
    }

    private static final class ReachedBuilder implements ReachedReadyStep {
        private final ScalarTarget target;
        private final double value;
        private final Plant plant;
        private final boolean hasCancellationTarget;
        private final double cancellationTarget;
        private final boolean stableAnswered;
        private final double stableSec;
        private final boolean timeoutAnswered;
        private final double timeoutSec;
        private final boolean finalAnswered;
        private final double finalTarget;

        private ReachedBuilder(ScalarTarget target,
                               double value,
                               Plant plant,
                               boolean hasCancellationTarget,
                               double cancellationTarget,
                               boolean stableAnswered,
                               double stableSec,
                               boolean timeoutAnswered,
                               double timeoutSec,
                               boolean finalAnswered,
                               double finalTarget) {
            this.target = target;
            this.value = value;
            this.plant = plant;
            this.hasCancellationTarget = hasCancellationTarget;
            this.cancellationTarget = cancellationTarget;
            this.stableAnswered = stableAnswered;
            this.stableSec = stableSec;
            this.timeoutAnswered = timeoutAnswered;
            this.timeoutSec = timeoutSec;
            this.finalAnswered = finalAnswered;
            this.finalTarget = finalTarget;
        }

        static ReachedBuilder initial(ScalarTarget target,
                                      double value,
                                      Plant plant,
                                      boolean hasCancellationTarget,
                                      double cancellationTarget) {
            return new ReachedBuilder(target, value, plant, hasCancellationTarget,
                    cancellationTarget, false, 0.0, false, -1.0, false, Double.NaN);
        }

        @Override
        public ReachedReadyStep stableFor(double seconds) {
            if (stableAnswered) {
                throw new IllegalStateException(
                        "stableFor(...) has already been answered for this ScalarTasks.set(...) branch");
            }
            requireNonNegativeFinite(seconds, "seconds");
            return new ReachedBuilder(target, value, plant, hasCancellationTarget,
                    cancellationTarget, true, seconds, timeoutAnswered, timeoutSec,
                    finalAnswered, finalTarget);
        }

        @Override
        public ReachedReadyStep timeout(double seconds) {
            if (timeoutAnswered) {
                throw new IllegalStateException(
                        "timeout(...) has already been answered for this ScalarTasks.set(...) branch");
            }
            requirePositiveFinite(seconds, "seconds");
            return new ReachedBuilder(target, value, plant, hasCancellationTarget,
                    cancellationTarget, stableAnswered, stableSec, true, seconds,
                    finalAnswered, finalTarget);
        }

        @Override
        public ReachedReadyStep thenTarget(double targetValue) {
            if (finalAnswered) {
                throw new IllegalStateException(
                        "thenTarget(...) has already been answered for this ScalarTasks.set(...) branch");
            }
            requireFinite(targetValue, "target");
            return new ReachedBuilder(target, value, plant, hasCancellationTarget,
                    cancellationTarget, stableAnswered, stableSec, timeoutAnswered, timeoutSec,
                    true, targetValue);
        }

        @Override
        public Task build() {
            return new ReachedTask(plant, target, value, hasCancellationTarget,
                    cancellationTarget, stableSec, timeoutSec, finalAnswered, finalTarget);
        }
    }

    private static final class SetOnceTask implements Task {
        private final ScalarTarget target;
        private final double value;
        private boolean startAttempted;
        private boolean complete;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;

        private SetOnceTask(ScalarTarget target, double value) {
            this.target = target;
            this.value = value;
        }

        @Override
        public void start(LoopClock clock) {
            if (startAttempted) {
                throw new IllegalStateException("ScalarTasks.set(" + value + ") is single-use and "
                        + "cannot be started more than once. Create a fresh Task by calling "
                        + "ScalarTasks.set(...).build() again or by rebuilding the macro; use a "
                        + "Supplier<Task> for repeated scheduling.");
            }
            startAttempted = true;
            complete = false;
            outcome = TaskOutcome.NOT_DONE;
            target.set(value);
            // Preserve terminal cancellation if the target callback re-entered its runner.
            if (!complete) {
                outcome = TaskOutcome.SUCCESS;
                complete = true;
            }
        }

        @Override
        public void update(LoopClock clock) {
            if (!startAttempted) {
                throw new IllegalStateException("ScalarTasks.set(" + value + ") cannot be "
                        + "updated before start(clock). Start it first, normally by enqueueing "
                        + "it in a TaskRunner.");
            }
        }

        @Override
        public void cancel() {
            if (!startAttempted || complete) return;
            outcome = TaskOutcome.CANCELLED;
            complete = true;
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
            return "ScalarTasks.set(" + value + ")";
        }
    }

    private static final class ReachedTask implements Task {
        private final Plant plant;
        private final ScalarTarget target;
        private final double requestedValue;
        private final boolean hasCancellationTarget;
        private final double cancellationTarget;
        private final double stableSec;
        private final double timeoutSec;
        private final boolean hasFinalTarget;
        private final double finalTarget;
        private boolean startAttempted;
        private boolean started;
        private boolean complete;
        private double startSec;
        private double stableSinceSec;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;

        private ReachedTask(Plant plant,
                            ScalarTarget target,
                            double requestedValue,
                            boolean hasCancellationTarget,
                            double cancellationTarget,
                            double stableSec,
                            double timeoutSec,
                            boolean hasFinalTarget,
                            double finalTarget) {
            this.plant = plant;
            this.target = target;
            this.requestedValue = requestedValue;
            this.hasCancellationTarget = hasCancellationTarget;
            this.cancellationTarget = cancellationTarget;
            this.stableSec = stableSec;
            this.timeoutSec = timeoutSec;
            this.hasFinalTarget = hasFinalTarget;
            this.finalTarget = finalTarget;
        }

        @Override
        public void start(LoopClock clock) {
            if (startAttempted) {
                throw new IllegalStateException("ScalarTasks.set(" + requestedValue
                        + ").untilReachedBy(...) is single-use and cannot be started more than once. "
                        + "Create a fresh Task by rebuilding ScalarTasks.set(...); use a "
                        + "Supplier<Task> for repeated scheduling.");
            }
            startAttempted = true;
            started = true;
            complete = false;
            startSec = nowSec(clock, 0.0);
            stableSinceSec = Double.NaN;
            outcome = TaskOutcome.NOT_DONE;
            target.set(requestedValue);
        }

        @Override
        public void update(LoopClock clock) {
            if (!started) {
                throw new IllegalStateException("ScalarTasks.set(" + requestedValue
                        + ").untilReachedBy(...) cannot be updated before start(clock). Start it "
                        + "first, normally by enqueueing it in a TaskRunner.");
            }
            if (complete) return;

            double nowSec = nowSec(clock, startSec);
            double elapsedSec = elapsedSince(startSec, nowSec);
            PlantTargetResolution resolution = plant.getTargetResolution();
            boolean reached;
            if (resolution != null && resolution.reportsCommandResolutionFor(target)) {
                reached = resolution.satisfiesCommand(target, requestedValue)
                        && target.get() == requestedValue
                        && plant.atTarget(resolution.target());
            } else {
                // Custom Plants that do not publish framework command provenance retain their
                // exact requested-value completion contract. The live command must still be this
                // Task's request; a shared target may have been changed by another owner.
                reached = target.get() == requestedValue
                        && plant.atTarget(requestedValue);
            }
            if (complete) return;

            boolean stable;
            if (stableSec <= 0.0) {
                stable = reached;
            } else if (!reached) {
                stableSinceSec = Double.NaN;
                stable = false;
            } else {
                if (!Double.isFinite(stableSinceSec)) stableSinceSec = nowSec;
                stable = elapsedSince(stableSinceSec, nowSec) >= stableSec;
            }

            // Success deliberately wins an exact tie with timeout.
            if (stable) finish(TaskOutcome.SUCCESS);
            else if (timeoutSec > 0.0 && elapsedSec >= timeoutSec) finish(TaskOutcome.TIMEOUT);
        }

        private void finish(TaskOutcome result) {
            if (hasFinalTarget) {
                target.set(finalTarget);
                // Preserve cancellation if the target callback re-entered its runner.
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
            if (hasCancellationTarget) target.set(cancellationTarget);
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
            return "ScalarTasks.set(" + requestedValue + ").untilReachedBy("
                    + plant.getClass().getSimpleName() + ")";
        }
    }

    private static void validateFeedbackObserver(ScalarTarget target, Plant plant) {
        Objects.requireNonNull(plant, "plant");
        if (!plant.hasCommandTarget()) {
            throw new IllegalStateException("ScalarTasks.set(...).untilReachedBy(plant) requires "
                    + "a Plant whose final target graph carries this ScalarTarget command. Build "
                    + "the Plant with targetFromResolver(PlantTargets.exact(command)), or use the command "
                    + "as the stable base of its final target graph.");
        }
        ScalarTarget plantTarget = Objects.requireNonNull(
                plant.commandTarget(), "plant.commandTarget() returned null");
        if (plantTarget != target) {
            throw new IllegalArgumentException("ScalarTasks.set(...).untilReachedBy(plant) requires "
                    + "the exact ScalarTarget used by that Plant. The supplied target and "
                    + "plant.commandTarget() are different objects.");
        }
        if (!plant.hasFeedback()) {
            throw new IllegalStateException("ScalarTasks.set(...).untilReachedBy(plant) requires "
                    + "authoritative Plant feedback. Use build() for a write-once request or "
                    + "forSeconds(...) for timed open-loop behavior.");
        }
    }

    private static double nowSec(LoopClock clock, double fallbackSec) {
        return clock != null ? clock.nowSec() : fallbackSec;
    }

    private static double elapsedSince(double intervalStartSec, double nowSec) {
        return Math.max(0.0, nowSec - intervalStartSec);
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

    private static void requirePositiveFinite(double value, String name) {
        if (!(value > 0.0) || !Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite and > 0, got " + value);
        }
    }
}
