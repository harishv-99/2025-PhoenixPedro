package edu.ftcsushi.fw.actuation;

import java.util.Objects;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.task.RunForSecondsTask;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;

/**
 * Task builders for one named {@link SemanticScalarCommand} request.
 *
 * <p>Use this factory inside a mechanism whose public capability names intent such as a lift
 * {@code Height}, intake {@code Mode}, or launcher {@code Speed}. The same command remains the
 * single owner that maps both direct and Task requests into finite Plant targets. Builder calls
 * map and validate their semantic answers but do not publish them; a built Task first changes the
 * request when it starts.</p>
 *
 * <p>The staged grammar deliberately parallels {@link ScalarTasks}: build immediately, retain
 * active ownership for a duration, or publish once and wait for command-correlated Plant
 * feedback. A timed Task preserves one request identity while uncontested and publishes a fresh
 * occurrence only when reclaiming its request after another writer supersedes it. A feedback Task
 * never reasserts and therefore cannot fight a later request.</p>
 *
 * <h2>Examples</h2>
 * <pre>{@code
 * Task select = SemanticScalarTasks.set(heightCommand, Height.HIGH).build();
 *
 * Task collect = SemanticScalarTasks.set(modeCommand, Mode.COLLECT)
 *         .forSeconds(0.75)
 *         .then(Mode.STOPPED)
 *         .build();
 *
 * Task move = SemanticScalarTasks.set(heightCommand, Height.HIGH)
 *         .untilReachedBy(lift)
 *         .leaveRequestOnCancel()
 *         .stableFor(0.10)
 *         .timeout(1.5)
 *         .build();
 * }</pre>
 */
public final class SemanticScalarTasks {
    private SemanticScalarTasks() {
    }

    /**
     * Start a side-effect-free Task builder for publishing one named request.
     *
     * <p>The command maps and validates {@code request} now, but does not publish it. Every call
     * to {@code build()} returns a fresh single-use Task, and each Task publishes its own fresh
     * request occurrence when started.</p>
     *
     * @param command semantic/numeric request owner
     * @param request non-null named request
     * @param <S> semantic request type
     * @return lifetime-selection stage
     * @throws NullPointerException if {@code command} or {@code request} is null
     * @throws IllegalArgumentException if the command maps {@code request} to a non-finite target
     */
    public static <S> SetReadyStep<S> set(SemanticScalarCommand<S> command, S request) {
        SemanticScalarCommand<S> actualCommand = Objects.requireNonNull(command, "command");
        return new SetBuilder<>(actualCommand, actualCommand.prepare(request));
    }

    /** Choose whether this semantic set is write-once, timed, or feedback-aware. */
    public interface SetReadyStep<S> {
        /** Build a Task that publishes the requested semantic/numeric pair once at start. */
        Task build();

        /**
         * Reassert this semantic request for {@code seconds}, then require an ending policy.
         * A positive duration remains observable by the downstream Plant phase at least once.
         */
        TimedEndStep<S> forSeconds(double seconds);

        /**
         * Publish once, then wait for this Plant to reach the exact request occurrence.
         *
         * <p>The Plant must carry this exact command as its semantic base and expose
         * authoritative feedback. Selecting this branch does not publish the prepared request.</p>
         */
        ReachedCancellationStep<S> untilReachedBy(Plant plant);
    }

    /** Required ending policy for a timed semantic set. */
    public interface TimedEndStep<S> {
        /** Leave whichever persistent request is current when the timed Task ends. */
        TimedBuildStep leaveThere();

        /** Publish {@code finalRequest} after natural completion or active cancellation. */
        TimedBuildStep then(S finalRequest);
    }

    /** Final build stage for a timed semantic set. */
    public interface TimedBuildStep {
        /** Build a fresh single-use timed Task. */
        Task build();
    }

    /** Required active-cancellation policy for a feedback-aware semantic set. */
    public interface ReachedCancellationStep<S> {
        /**
         * Publish {@code request} once if the active Task is cancelled.
         * The Plant still resolves and applies it during its normal update phase.
         */
        ReachedReadyStep<S> cancelTo(S request);

        /**
         * Deliberately leave the persistent request unchanged if the active Task is cancelled.
         * The mechanism may continue moving until another owner changes its request.
         */
        ReachedReadyStep<S> leaveRequestOnCancel();
    }

    /** Optional completion policy for a feedback-aware semantic set. */
    public interface ReachedReadyStep<S> {
        /** Require command-correlated arrival to remain true for {@code seconds}. */
        ReachedReadyStep<S> stableFor(double seconds);

        /** Complete with {@link TaskOutcome#TIMEOUT} after {@code seconds} if not reached. */
        ReachedReadyStep<S> timeout(double seconds);

        /**
         * Build a fresh single-use feedback Task. Success and timeout leave the latest persistent
         * request unchanged; compose any outcome-specific continuation explicitly.
         */
        Task build();
    }

    private static final class SetBuilder<S> implements SetReadyStep<S> {
        private final SemanticScalarCommand<S> command;
        private final SemanticScalarCommand.PreparedRequest<S> request;

        private SetBuilder(SemanticScalarCommand<S> command,
                           SemanticScalarCommand.PreparedRequest<S> request) {
            this.command = command;
            this.request = request;
        }

        @Override
        public Task build() {
            return new SetOnceTask<>(command, request);
        }

        @Override
        public TimedEndStep<S> forSeconds(double seconds) {
            requireNonNegativeFinite(seconds, "seconds");
            return new TimedEndBuilder<>(command, request, seconds);
        }

        @Override
        public ReachedCancellationStep<S> untilReachedBy(Plant plant) {
            validateFeedbackObserver(command, plant);
            return new ReachedCancellationBuilder<>(command, request, plant);
        }
    }

    private static final class TimedEndBuilder<S> implements TimedEndStep<S> {
        private final SemanticScalarCommand<S> command;
        private final SemanticScalarCommand.PreparedRequest<S> request;
        private final double seconds;

        private TimedEndBuilder(SemanticScalarCommand<S> command,
                                SemanticScalarCommand.PreparedRequest<S> request,
                                double seconds) {
            this.command = command;
            this.request = request;
            this.seconds = seconds;
        }

        @Override
        public TimedBuildStep leaveThere() {
            return new TimedBuilder<>(command, request, seconds, null);
        }

        @Override
        public TimedBuildStep then(S finalRequest) {
            return new TimedBuilder<>(command, request, seconds, command.prepare(finalRequest));
        }
    }

    private static final class TimedBuilder<S> implements TimedBuildStep {
        private final SemanticScalarCommand<S> command;
        private final SemanticScalarCommand.PreparedRequest<S> request;
        private final double seconds;
        private final SemanticScalarCommand.PreparedRequest<S> finalRequest;

        private TimedBuilder(SemanticScalarCommand<S> command,
                             SemanticScalarCommand.PreparedRequest<S> request,
                             double seconds,
                             SemanticScalarCommand.PreparedRequest<S> finalRequest) {
            this.command = command;
            this.request = request;
            this.seconds = seconds;
            this.finalRequest = finalRequest;
        }

        @Override
        public Task build() {
            TimedCommand<S> timed = new TimedCommand<>(command, request, finalRequest);
            return new RunForSecondsTask(
                    seconds,
                    timed::start,
                    timed::update,
                    finalRequest != null ? timed::finish : null);
        }
    }

    /** Retains one occurrence while uncontested and creates a new one only to reclaim ownership. */
    private static final class TimedCommand<S> {
        private final SemanticScalarCommand<S> command;
        private final SemanticScalarCommand.PreparedRequest<S> request;
        private final SemanticScalarCommand.PreparedRequest<S> finalRequest;
        private SemanticScalarCommand.Request<S> activeRequest;

        private TimedCommand(SemanticScalarCommand<S> command,
                             SemanticScalarCommand.PreparedRequest<S> request,
                             SemanticScalarCommand.PreparedRequest<S> finalRequest) {
            this.command = command;
            this.request = request;
            this.finalRequest = finalRequest;
        }

        private void start() {
            activeRequest = command.publish(request);
        }

        private void update(LoopClock ignoredClock) {
            if (command.request() != activeRequest) {
                activeRequest = command.publish(request);
            }
        }

        private void finish() {
            command.publish(finalRequest);
        }
    }

    private static final class ReachedCancellationBuilder<S>
            implements ReachedCancellationStep<S> {
        private final SemanticScalarCommand<S> command;
        private final SemanticScalarCommand.PreparedRequest<S> request;
        private final Plant plant;

        private ReachedCancellationBuilder(SemanticScalarCommand<S> command,
                                           SemanticScalarCommand.PreparedRequest<S> request,
                                           Plant plant) {
            this.command = command;
            this.request = request;
            this.plant = plant;
        }

        @Override
        public ReachedReadyStep<S> cancelTo(S cancellationRequest) {
            return ReachedBuilder.initial(
                    command, request, plant, command.prepare(cancellationRequest));
        }

        @Override
        public ReachedReadyStep<S> leaveRequestOnCancel() {
            return ReachedBuilder.initial(command, request, plant, null);
        }
    }

    private static final class ReachedBuilder<S> implements ReachedReadyStep<S> {
        private final SemanticScalarCommand<S> command;
        private final SemanticScalarCommand.PreparedRequest<S> request;
        private final Plant plant;
        private final SemanticScalarCommand.PreparedRequest<S> cancellationRequest;
        private final boolean stableAnswered;
        private final double stableSec;
        private final boolean timeoutAnswered;
        private final double timeoutSec;

        private ReachedBuilder(SemanticScalarCommand<S> command,
                               SemanticScalarCommand.PreparedRequest<S> request,
                               Plant plant,
                               SemanticScalarCommand.PreparedRequest<S> cancellationRequest,
                               boolean stableAnswered,
                               double stableSec,
                               boolean timeoutAnswered,
                               double timeoutSec) {
            this.command = command;
            this.request = request;
            this.plant = plant;
            this.cancellationRequest = cancellationRequest;
            this.stableAnswered = stableAnswered;
            this.stableSec = stableSec;
            this.timeoutAnswered = timeoutAnswered;
            this.timeoutSec = timeoutSec;
        }

        private static <S> ReachedBuilder<S> initial(
                SemanticScalarCommand<S> command,
                SemanticScalarCommand.PreparedRequest<S> request,
                Plant plant,
                SemanticScalarCommand.PreparedRequest<S> cancellationRequest) {
            return new ReachedBuilder<>(command, request, plant, cancellationRequest,
                    false, 0.0, false, -1.0);
        }

        @Override
        public ReachedReadyStep<S> stableFor(double seconds) {
            if (stableAnswered) {
                throw new IllegalStateException("stableFor(...) has already been answered for "
                        + "this SemanticScalarTasks.set(...) branch");
            }
            requireNonNegativeFinite(seconds, "seconds");
            return new ReachedBuilder<>(command, request, plant, cancellationRequest,
                    true, seconds, timeoutAnswered, timeoutSec);
        }

        @Override
        public ReachedReadyStep<S> timeout(double seconds) {
            if (timeoutAnswered) {
                throw new IllegalStateException("timeout(...) has already been answered for "
                        + "this SemanticScalarTasks.set(...) branch");
            }
            requirePositiveFinite(seconds, "seconds");
            return new ReachedBuilder<>(command, request, plant, cancellationRequest,
                    stableAnswered, stableSec, true, seconds);
        }

        @Override
        public Task build() {
            return new ReachedTask<>(command, request, plant, cancellationRequest,
                    stableSec, timeoutSec);
        }
    }

    private static final class SetOnceTask<S> implements Task {
        private final SemanticScalarCommand<S> command;
        private final SemanticScalarCommand.PreparedRequest<S> request;
        private boolean startAttempted;
        private boolean complete;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;

        private SetOnceTask(SemanticScalarCommand<S> command,
                            SemanticScalarCommand.PreparedRequest<S> request) {
            this.command = command;
            this.request = request;
        }

        @Override
        public void start(LoopClock clock) {
            if (startAttempted) {
                throw new IllegalStateException("SemanticScalarTasks.set(" + request.semantic()
                        + ") is single-use and cannot be started more than once. Create a fresh "
                        + "Task by calling SemanticScalarTasks.set(...).build() again or by "
                        + "rebuilding the macro; use a Supplier<Task> for repeated scheduling.");
            }
            startAttempted = true;
            complete = false;
            outcome = TaskOutcome.NOT_DONE;
            command.publish(request);
            outcome = TaskOutcome.SUCCESS;
            complete = true;
        }

        @Override
        public void update(LoopClock clock) {
            if (!startAttempted) {
                throw new IllegalStateException("SemanticScalarTasks.set(" + request.semantic()
                        + ") cannot be updated before start(clock). Start it first, normally by "
                        + "enqueueing it in a TaskRunner.");
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
            return "SemanticScalarTasks.set(" + request.semantic() + ")";
        }
    }

    private static final class ReachedTask<S> implements Task {
        private final SemanticScalarCommand<S> command;
        private final SemanticScalarCommand.PreparedRequest<S> request;
        private final Plant plant;
        private final SemanticScalarCommand.PreparedRequest<S> cancellationRequest;
        private final double stableSec;
        private final double timeoutSec;
        private SemanticScalarCommand.Request<S> startedRequest;
        private boolean startAttempted;
        private boolean started;
        private boolean complete;
        private double startSec;
        private double stableSinceSec;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;

        private ReachedTask(SemanticScalarCommand<S> command,
                            SemanticScalarCommand.PreparedRequest<S> request,
                            Plant plant,
                            SemanticScalarCommand.PreparedRequest<S> cancellationRequest,
                            double stableSec,
                            double timeoutSec) {
            this.command = command;
            this.request = request;
            this.plant = plant;
            this.cancellationRequest = cancellationRequest;
            this.stableSec = stableSec;
            this.timeoutSec = timeoutSec;
        }

        @Override
        public void start(LoopClock clock) {
            if (startAttempted) {
                throw new IllegalStateException("SemanticScalarTasks.set(" + request.semantic()
                        + ").untilReachedBy(...) is single-use and cannot be started more than "
                        + "once. Create a fresh Task by rebuilding SemanticScalarTasks.set(...); "
                        + "use a Supplier<Task> for repeated scheduling.");
            }
            startAttempted = true;
            started = true;
            complete = false;
            startSec = nowSec(clock, 0.0);
            stableSinceSec = Double.NaN;
            outcome = TaskOutcome.NOT_DONE;
            startedRequest = command.publish(request);
        }

        @Override
        public void update(LoopClock clock) {
            if (!started) {
                throw new IllegalStateException("SemanticScalarTasks.set(" + request.semantic()
                        + ").untilReachedBy(...) cannot be updated before start(clock). Start it "
                        + "first, normally by enqueueing it in a TaskRunner.");
            }
            if (complete) return;

            double nowSec = nowSec(clock, startSec);
            double elapsedSec = elapsedSince(startSec, nowSec);
            PlantTargetResolution resolution = plant.getTargetResolution();
            boolean reached = command.request() == startedRequest
                    && resolution != null
                    && resolution.satisfiesSemanticCommand(command, startedRequest)
                    && plant.atTarget(resolution.target());
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
            else if (timeoutSec > 0.0 && elapsedSec >= timeoutSec) {
                finish(TaskOutcome.TIMEOUT);
            }
        }

        private void finish(TaskOutcome result) {
            outcome = result;
            complete = true;
        }

        @Override
        public void cancel() {
            if (!started || complete) return;

            // Become terminal before the optional persistent request change.
            outcome = TaskOutcome.CANCELLED;
            complete = true;
            if (cancellationRequest != null) command.publish(cancellationRequest);
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
            return "SemanticScalarTasks.set(" + request.semantic() + ").untilReachedBy("
                    + plant.getClass().getSimpleName() + ")";
        }
    }

    private static void validateFeedbackObserver(
            SemanticScalarCommand<?> command,
            Plant plant) {
        Plant actualPlant = Objects.requireNonNull(plant, "plant");
        if (!actualPlant.carriesSemanticCommand(command)) {
            throw new IllegalArgumentException("SemanticScalarTasks.set(...).untilReachedBy(plant) "
                    + "requires a Plant whose final target graph carries this exact "
                    + "SemanticScalarCommand. Bind it with PlantTargets.exact(command), use it "
                    + "as the stable base of an overlay, or preserve it through "
                    + "equivalentPositionsOf(...).");
        }
        if (!actualPlant.hasFeedback()) {
            throw new IllegalStateException("SemanticScalarTasks.set(...).untilReachedBy(plant) "
                    + "requires authoritative Plant feedback. Use build() for a write-once "
                    + "request or forSeconds(...) for timed open-loop behavior.");
        }
    }

    private static double nowSec(LoopClock clock, double fallbackSec) {
        return clock != null ? clock.nowSec() : fallbackSec;
    }

    private static double elapsedSince(double intervalStartSec, double nowSec) {
        return Math.max(0.0, nowSec - intervalStartSec);
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
