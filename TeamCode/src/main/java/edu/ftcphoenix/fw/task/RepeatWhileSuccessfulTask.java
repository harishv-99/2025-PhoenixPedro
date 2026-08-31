package edu.ftcphoenix.fw.task;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Internal bounded fresh-child composition exposed through
 * {@link Tasks#repeatWhileSuccessful(String, int, BooleanSource, Supplier)}.
 *
 * <p>This implementation is package-private so {@link Tasks} remains the one public generic
 * Task-construction surface.</p>
 */
final class RepeatWhileSuccessfulTask implements Task {

    /** Stable lifecycle phases used by diagnostics and reentry guards. */
    private enum Phase {
        NOT_STARTED,
        ADMITTING,
        BUILDING,
        STARTING_CHILD,
        ACTIVE_CHILD,
        AWAITING_ADMISSION,
        COMPLETE,
        CANCELLED,
        FAILED
    }

    /** Stable terminal reasons; {@link #NONE} applies only before terminal completion. */
    private enum StopReason {
        NONE,
        LIMIT,
        CONDITION_FALSE,
        CHILD_OUTCOME,
        CANCELLED,
        FAILED
    }

    private final String debugName;
    private final int maxIterations;
    private final BooleanSource mayStartIteration;
    private final Supplier<? extends Task> taskFactory;
    private final List<Task> iterationIdentities;

    private boolean startAttempted;
    private boolean started;
    private boolean lifecycleCallInProgress;
    private boolean childStartAttempted;
    private boolean childCancellationAttempted;
    private int iterationsStarted;
    private int iterationsCompleted;
    private int admissionEvaluations;
    private long lastUpdateCycle = Long.MIN_VALUE;
    private long lastProposalCycle = Long.MIN_VALUE;
    private long awaitingAdmissionSinceCycle = Long.MIN_VALUE;
    private Phase phase = Phase.NOT_STARTED;
    private StopReason stopReason = StopReason.NONE;
    private Task currentChild;
    private TaskOutcome currentChildOutcome = TaskOutcome.NOT_DONE;
    private TaskOutcome lastChildOutcome = TaskOutcome.NOT_DONE;
    private TaskOutcome outcome = TaskOutcome.NOT_DONE;
    private RuntimeException retainedLifecycleFailure;

    /** Retain validated policy without sampling either callback. */
    RepeatWhileSuccessfulTask(String debugName,
                              int maxIterations,
                              BooleanSource mayStartIteration,
                              Supplier<? extends Task> taskFactory) {
        if (debugName == null || debugName.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "Tasks.repeatWhileSuccessful debugName must be nonblank, got '"
                            + debugName + "'.");
        }
        if (maxIterations <= 0) {
            throw new IllegalArgumentException(
                    "Tasks.repeatWhileSuccessful maxIterations must be > 0, got "
                            + maxIterations + ".");
        }
        this.debugName = debugName;
        this.maxIterations = maxIterations;
        this.mayStartIteration = Objects.requireNonNull(
                mayStartIteration,
                "Tasks.repeatWhileSuccessful requires a non-null mayStartIteration source.");
        this.taskFactory = Objects.requireNonNull(
                taskFactory,
                "Tasks.repeatWhileSuccessful requires a non-null Task factory.");
        // Grow only as iterations actually start; a valid large safety bound must not allocate a
        // correspondingly large array at construction time.
        this.iterationIdentities = new ArrayList<>();
    }

    /** Evaluate first admission, then build and start at most one child in this cycle. */
    @Override
    public void start(LoopClock clock) {
        markStartAttempt();
        started = true;
        lifecycleCallInProgress = true;
        try {
            requireClock(clock);
            proposeIteration(clock);
            throwRetainedFailureIfPresent();
        } catch (RuntimeException failure) {
            throw failClosed(failure);
        } finally {
            lifecycleCallInProgress = false;
        }
    }

    /**
     * Advance only the current child, or admit one child after a completed-cycle handoff.
     */
    @Override
    public void update(LoopClock clock) {
        if (!started) {
            throw TaskLifecycle.updateBeforeStart(
                    "Tasks.repeatWhileSuccessful('" + debugName + "')");
        }
        throwRetainedFailureIfPresent();
        if (isTerminalPhase(phase)) {
            return;
        }
        if (lifecycleCallInProgress) {
            throw failClosed(new IllegalStateException(
                    "Tasks.repeatWhileSuccessful '" + debugName
                            + "' received a reentrant lifecycle callback. Task callbacks must not "
                            + "start or update their owning repeat Task."));
        }

        try {
            requireClock(clock);
        } catch (RuntimeException failure) {
            throw failClosed(failure);
        }

        long cycle = clock.cycle();
        if (cycle == lastUpdateCycle) {
            return;
        }
        // Claim the cycle before invoking a child, policy source, or factory callback.
        lastUpdateCycle = cycle;

        lifecycleCallInProgress = true;
        try {
            switch (phase) {
                case ACTIVE_CHILD:
                    if (finishCurrentChildIfComplete(clock)) {
                        break;
                    }
                    currentChild.update(clock);
                    throwRetainedFailureIfPresent();
                    if (!isTerminalPhase(phase)) {
                        finishCurrentChildIfComplete(clock);
                    }
                    break;

                case AWAITING_ADMISSION:
                    if (cycle != awaitingAdmissionSinceCycle) {
                        proposeIteration(clock);
                    }
                    break;

                default:
                    // Transitional phases occur only inside a guarded lifecycle callback.
                    break;
            }
            throwRetainedFailureIfPresent();
        } catch (RuntimeException failure) {
            throw failClosed(failure);
        } finally {
            lifecycleCallInProgress = false;
        }
    }

    /**
     * Terminalize first, then cancel and verify only the currently start-attempted child.
     */
    @Override
    public void cancel() {
        if (!started || isTerminalPhase(phase)) {
            return;
        }

        Task childToCancel = childStartAttempted ? currentChild : null;
        phase = Phase.CANCELLED;
        stopReason = StopReason.CANCELLED;
        outcome = TaskOutcome.CANCELLED;

        if (childToCancel == null || childCancellationAttempted) {
            return;
        }

        childCancellationAttempted = true;
        try {
            childToCancel.cancel();
            if (!childToCancel.isComplete()) {
                throw new IllegalStateException(
                        "Tasks.repeatWhileSuccessful '" + debugName
                                + "' cancelled its active child, but child cancel() returned "
                                + "without making the child terminal. Active Task cancellation "
                                + "must make isComplete() return true.");
            }
        } catch (RuntimeException failure) {
            retainLifecycleFailure(failure);
            throw retainedLifecycleFailure;
        }
    }

    /** Return only cached wrapper state; no child or policy callback is sampled. */
    @Override
    public boolean isComplete() {
        return isTerminalPhase(phase);
    }

    /** Return the exact abnormal child outcome, or this wrapper's selected terminal result. */
    @Override
    public TaskOutcome getOutcome() {
        return isTerminalPhase(phase) ? outcome : TaskOutcome.NOT_DONE;
    }

    /** Return the stable caller-supplied diagnostic name. */
    @Override
    public String getDebugName() {
        return debugName;
    }

    /** Report cached repeat state without sampling the admission policy. */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "repeatWhileSuccessful" : prefix;
        dbg.addData(p + ".name", debugName)
                .addData(p + ".maxIterations", maxIterations)
                .addData(p + ".phase", phase)
                .addData(p + ".iterationsStarted", iterationsStarted)
                .addData(p + ".iterationsCompleted", iterationsCompleted)
                .addData(p + ".admissionEvaluations", admissionEvaluations)
                .addData(p + ".currentChildPresent", currentChild != null)
                .addData(p + ".currentChildOutcome", currentChildOutcome)
                .addData(p + ".lastChildOutcome", lastChildOutcome)
                .addData(p + ".outcome", getOutcome())
                .addData(p + ".stopReason", stopReason)
                .addData(p + ".hasLifecycleFailure", retainedLifecycleFailure != null);
        if (currentChild != null) {
            currentChild.debugDump(dbg, p + ".current");
        }
    }

    /** Evaluate one admission decision and, when permitted, start one fresh child. */
    private void proposeIteration(LoopClock clock) {
        if (isTerminalPhase(phase)) {
            return;
        }
        long cycle = clock.cycle();
        if (cycle == lastProposalCycle) {
            return;
        }
        lastProposalCycle = cycle;

        phase = Phase.ADMITTING;
        admissionEvaluations++;
        boolean admitted = mayStartIteration.getAsBoolean(clock);
        throwRetainedFailureIfPresent();
        if (isTerminalPhase(phase)) {
            return;
        }
        if (!admitted) {
            finishSuccessfully(StopReason.CONDITION_FALSE);
            return;
        }

        phase = Phase.BUILDING;
        Task builtChild = taskFactory.get();
        throwRetainedFailureIfPresent();
        if (isTerminalPhase(phase)) {
            return;
        }
        validateFreshChild(builtChild);

        iterationIdentities.add(builtChild);
        currentChild = builtChild;
        currentChildOutcome = TaskOutcome.NOT_DONE;
        childStartAttempted = true;
        childCancellationAttempted = false;
        iterationsStarted++;
        phase = Phase.STARTING_CHILD;

        builtChild.start(clock);
        throwRetainedFailureIfPresent();
        if (isTerminalPhase(phase)) {
            return;
        }
        phase = Phase.ACTIVE_CHILD;
        finishCurrentChildIfComplete(clock);
    }

    /** Capture one natural child completion and choose the repeat's next phase. */
    private boolean finishCurrentChildIfComplete(LoopClock clock) {
        if (phase != Phase.ACTIVE_CHILD || currentChild == null) {
            return isTerminalPhase(phase);
        }
        boolean childComplete = currentChild.isComplete();
        throwRetainedFailureIfPresent();
        if (isTerminalPhase(phase)) {
            return true;
        }
        if (!childComplete) {
            return false;
        }

        TaskOutcome childOutcome = currentChild.getOutcome();
        throwRetainedFailureIfPresent();
        if (isTerminalPhase(phase)) {
            return true;
        }
        if (childOutcome == null || childOutcome == TaskOutcome.NOT_DONE) {
            throw new IllegalStateException(
                    "Tasks.repeatWhileSuccessful '" + debugName
                            + "' observed a complete child reporting " + childOutcome
                            + ". A terminal child Task must report SUCCESS, TIMEOUT, CANCELLED, "
                            + "or UNKNOWN from getOutcome(). Fix the child Task's lifecycle "
                            + "contract.");
        }

        currentChildOutcome = childOutcome;
        lastChildOutcome = childOutcome;
        iterationsCompleted++;
        currentChild = null;
        childStartAttempted = false;
        childCancellationAttempted = false;

        if (childOutcome != TaskOutcome.SUCCESS) {
            phase = Phase.COMPLETE;
            stopReason = StopReason.CHILD_OUTCOME;
            outcome = childOutcome;
            return true;
        }
        if (iterationsStarted >= maxIterations) {
            finishSuccessfully(StopReason.LIMIT);
            return true;
        }

        phase = Phase.AWAITING_ADMISSION;
        awaitingAdmissionSinceCycle = clock.cycle();
        return true;
    }

    /** Reject null, self, or any identity previously returned by this bounded factory. */
    private void validateFreshChild(Task candidate) {
        if (candidate == null) {
            throw new IllegalStateException(
                    "Tasks.repeatWhileSuccessful '" + debugName
                            + "' factory returned null. Return a fresh Task for every admitted "
                            + "iteration.");
        }
        if (candidate == this) {
            throw new IllegalStateException(
                    "Tasks.repeatWhileSuccessful '" + debugName
                            + "' factory returned its own wrapper. Return a distinct fresh child "
                            + "Task; a Task cannot own itself.");
        }
        for (int index = 0; index < iterationIdentities.size(); index++) {
            if (candidate == iterationIdentities.get(index)) {
                throw new IllegalStateException(
                        "Tasks.repeatWhileSuccessful '" + debugName
                                + "' factory reused the Task instance from iteration "
                                + (index + 1) + ". Return a fresh Task for every admitted "
                                + "iteration.");
            }
        }
    }

    /** Select a normal successful ending without another policy or factory callback. */
    private void finishSuccessfully(StopReason reason) {
        phase = Phase.COMPLETE;
        stopReason = reason;
        outcome = TaskOutcome.SUCCESS;
    }

    /**
     * Preserve the first lifecycle failure, fail terminally, and best-effort clean the exact
     * start-attempted child. Cleanup failures are suppressed on the retained primary failure.
     */
    private RuntimeException failClosed(RuntimeException failure) {
        retainLifecycleFailure(failure);
        if (phase != Phase.CANCELLED) {
            phase = Phase.FAILED;
            stopReason = StopReason.FAILED;
            outcome = TaskOutcome.CANCELLED;
        }

        if (currentChild != null && childStartAttempted && !childCancellationAttempted) {
            childCancellationAttempted = true;
            try {
                currentChild.cancel();
                if (!currentChild.isComplete()) {
                    suppressCleanupFailure(new IllegalStateException(
                            "Tasks.repeatWhileSuccessful '" + debugName
                                    + "' could not settle its child after a lifecycle failure; "
                                    + "child cancel() returned without making it terminal."));
                }
            } catch (RuntimeException cleanupFailure) {
                suppressCleanupFailure(cleanupFailure);
            }
        }
        return retainedLifecycleFailure;
    }

    /** Preserve only the first callback or lifecycle failure. */
    private void retainLifecycleFailure(RuntimeException failure) {
        if (retainedLifecycleFailure == null) {
            retainedLifecycleFailure = Objects.requireNonNull(failure, "failure");
        }
    }

    /** Attach a later cleanup failure without replacing or self-suppressing the primary failure. */
    private void suppressCleanupFailure(RuntimeException cleanupFailure) {
        if (cleanupFailure != retainedLifecycleFailure) {
            retainedLifecycleFailure.addSuppressed(cleanupFailure);
        }
    }

    /** Make a caught reentrant failure observable even if the callback swallowed its exception. */
    private void throwRetainedFailureIfPresent() {
        if (retainedLifecycleFailure != null) {
            throw retainedLifecycleFailure;
        }
    }

    /** Consume the one permitted wrapper start before any policy, factory, or child callback. */
    private void markStartAttempt() {
        if (!startAttempted) {
            startAttempted = true;
            return;
        }

        IllegalStateException failure = new IllegalStateException(
                "Tasks.repeatWhileSuccessful '" + debugName
                        + "' is single-use and start(...) was called more than once. Create a "
                        + "fresh task with its builder or macro method, a Supplier<Task>, or an "
                        + "OutputTaskFactory.");
        if (lifecycleCallInProgress && !isTerminalPhase(phase)) {
            throw failClosed(failure);
        }
        throw failure;
    }

    /** Require the real shared loop clock used by the owning runner. */
    private static void requireClock(LoopClock clock) {
        if (clock == null) {
            throw new IllegalArgumentException(
                    "Tasks.repeatWhileSuccessful requires a non-null LoopClock; start and update "
                            + "it through the owning TaskRunner.");
        }
    }

    /** Return whether this wrapper owns a terminal result independent of further callbacks. */
    private static boolean isTerminalPhase(Phase phase) {
        return phase == Phase.COMPLETE
                || phase == Phase.CANCELLED
                || phase == Phase.FAILED;
    }
}
