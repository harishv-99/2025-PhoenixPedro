package edu.ftcsushi.fw.task;

import java.util.Arrays;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Internal sequential composition used by {@link Tasks#sequence(Task...)} and
 * {@link Tasks#sequenceOnCompletion(Task...)}.
 *
 * <p>The ordinary sequence advances only after exact {@link TaskOutcome#SUCCESS}. The explicit
 * completion-continuation form advances after every valid natural terminal outcome and retains the
 * first non-success result. Both forms cache each completed child's outcome once, preserve
 * same-callback advancement through immediately successful children, and never advance after
 * direct cancellation or a child lifecycle failure.</p>
 *
 * <p>This type remains package-private so {@link Tasks} is the one public construction surface.</p>
 */
final class SequenceTask implements Task {

    private final Task[] tasks;
    private final TaskOutcome[] completedOutcomes;
    private final boolean continueOnCompletion;
    private final String factoryCall;

    private boolean startAttempted = false;
    private boolean started = false;
    private boolean complete = false;
    private TaskOutcome outcome = TaskOutcome.NOT_DONE;
    private TaskOutcome firstNonSuccess = null;
    /** Index of the current child, or {@code -1} before one is selected. */
    private int index = -1;

    /**
     * Create one defensively copied sequential graph.
     *
     * @param continueOnCompletion whether valid abnormal completion may start the next child
     * @param tasks ordered array of distinct, fresh child Tasks
     */
    SequenceTask(boolean continueOnCompletion, Task... tasks) {
        this.continueOnCompletion = continueOnCompletion;
        this.factoryCall = continueOnCompletion
                ? "Tasks.sequenceOnCompletion(...)"
                : "Tasks.sequence(...)";
        if (tasks == null) {
            throw new IllegalArgumentException(factoryCall + " tasks are required");
        }
        this.tasks = Arrays.copyOf(tasks, tasks.length);
        validateDistinctChildren(this.tasks, factoryCall);
        this.completedOutcomes = new TaskOutcome[this.tasks.length];
    }

    /**
     * Start the first child and drain immediately completed children according to this sequence's
     * continuation policy.
     */
    @Override
    public void start(LoopClock clock) {
        markStartAttempt();
        started = true;
        try {
            if (tasks.length == 0) {
                finishNaturally(TaskOutcome.SUCCESS);
                return;
            }
            index = 0;
            startCurrentAndDrain(clock);
        } catch (RuntimeException failure) {
            throw failClosed(failure);
        }
    }

    /**
     * Update only the current child. A child that became terminal between cycles is observed before
     * another update, and an immediately completed successor is drained in this same callback.
     */
    @Override
    public void update(LoopClock clock) {
        if (!started) {
            throw TaskLifecycle.updateBeforeStart("Task returned by " + factoryCall);
        }
        if (complete) {
            return;
        }

        try {
            Task current = tasks[index];
            boolean childComplete = current.isComplete();
            if (complete) {
                return;
            }
            if (!childComplete) {
                current.update(clock);
                if (complete) {
                    return;
                }
                childComplete = current.isComplete();
                if (complete) {
                    return;
                }
            }
            if (childComplete) {
                finishCurrentAndAdvance(clock);
            }
        } catch (RuntimeException failure) {
            throw failClosed(failure);
        }
    }

    /**
     * Make this sequence terminal before best-effort cancellation of only its current child.
     * Later children never start.
     */
    @Override
    public void cancel() {
        if (!started || complete) {
            return;
        }
        Task current = currentTaskOrNull();
        complete = true;
        outcome = TaskOutcome.CANCELLED;
        if (current != null) {
            current.cancel();
        }
    }

    /** {@inheritDoc} */
    @Override
    public boolean isComplete() {
        return complete;
    }

    /** Return the retained exact outcome without re-querying completed children. */
    @Override
    public TaskOutcome getOutcome() {
        return complete ? outcome : TaskOutcome.NOT_DONE;
    }

    /** Identify this internal implementation by the public factory robot code calls. */
    @Override
    public String getDebugName() {
        return factoryCall;
    }

    /** Dump cached wrapper state without re-sampling completed child outcomes. */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "sequence" : prefix;
        dbg.addData(p + ".started", started)
                .addData(p + ".complete", complete)
                .addData(p + ".size", tasks.length)
                .addData(p + ".index", index)
                .addData(p + ".outcome", getOutcome());
        if (index >= 0 && index < tasks.length) {
            dbg.addData(p + ".currentName", tasks[index].getDebugName())
                    .addData(p + ".currentCapturedOutcome", completedOutcomes[index]);
        }
    }

    /**
     * Start the selected child and keep draining children that complete within their own start.
     */
    private void startCurrentAndDrain(LoopClock clock) {
        while (!complete && index < tasks.length) {
            Task current = tasks[index];
            current.start(clock);
            if (complete) {
                return;
            }
            boolean childComplete = current.isComplete();
            if (complete || !childComplete) {
                return;
            }
            if (!captureAndApplyCurrentOutcome()) {
                return;
            }
            index++;
            if (index >= tasks.length) {
                finishNaturally(finalNaturalOutcome());
            }
        }
    }

    /** Capture the current child's terminal result, apply policy, and start later work if allowed. */
    private void finishCurrentAndAdvance(LoopClock clock) {
        if (!captureAndApplyCurrentOutcome()) {
            return;
        }
        index++;
        if (index >= tasks.length) {
            finishNaturally(finalNaturalOutcome());
            return;
        }
        startCurrentAndDrain(clock);
    }

    /**
     * Capture and validate one terminal child outcome exactly once.
     *
     * @return {@code true} when this policy permits a later child to start
     */
    private boolean captureAndApplyCurrentOutcome() {
        TaskOutcome childOutcome = completedOutcomes[index];
        if (childOutcome == null) {
            childOutcome = tasks[index].getOutcome();
            if (complete) {
                // A custom outcome callback may have directly cancelled this wrapper.
                return false;
            }
            if (childOutcome == null || childOutcome == TaskOutcome.NOT_DONE) {
                throw malformedChildOutcome(index, childOutcome);
            }
            completedOutcomes[index] = childOutcome;
        }

        if (childOutcome != TaskOutcome.SUCCESS) {
            if (!continueOnCompletion) {
                finishNaturally(childOutcome);
                return false;
            }
            if (firstNonSuccess == null) {
                firstNonSuccess = childOutcome;
            }
        }
        return true;
    }

    /** Return this sequence's aggregate natural result after every permitted child has completed. */
    private TaskOutcome finalNaturalOutcome() {
        return firstNonSuccess == null ? TaskOutcome.SUCCESS : firstNonSuccess;
    }

    /** Record a natural terminal outcome before any later callback can re-enter this wrapper. */
    private void finishNaturally(TaskOutcome terminalOutcome) {
        outcome = terminalOutcome;
        complete = true;
    }

    /**
     * Terminalize after a lifecycle failure, best-effort cancel the current child, and preserve the
     * original failure as the exception visible to the caller.
     */
    private RuntimeException failClosed(RuntimeException failure) {
        if (complete) {
            return failure;
        }
        Task current = currentTaskOrNull();
        complete = true;
        outcome = TaskOutcome.CANCELLED;
        if (current != null) {
            try {
                current.cancel();
            } catch (RuntimeException cleanupFailure) {
                if (cleanupFailure != failure) {
                    failure.addSuppressed(cleanupFailure);
                }
            }
        }
        return failure;
    }

    /** Return the current selected child without invoking its lifecycle. */
    private Task currentTaskOrNull() {
        return index >= 0 && index < tasks.length ? tasks[index] : null;
    }

    /** Reject nulls and direct aliases before any child can acquire state. */
    private static void validateDistinctChildren(Task[] children, String factoryCall) {
        for (int i = 0; i < children.length; i++) {
            Task child = children[i];
            if (child == null) {
                throw new IllegalArgumentException(
                        factoryCall + " children must not contain null; found null at index " + i);
            }
            for (int previous = 0; previous < i; previous++) {
                if (child == children[previous]) {
                    throw new IllegalArgumentException(
                            factoryCall + " child at index " + i
                                    + " reuses the same Task instance as index " + previous + ". "
                                    + "Each child must be a distinct, fresh task; create it with "
                                    + "its builder or macro method, a Supplier<Task>, or an "
                                    + "OutputTaskFactory.");
                }
            }
        }
    }

    /** Build an actionable error for a complete child with a non-terminal result. */
    private IllegalStateException malformedChildOutcome(int childIndex,
                                                        TaskOutcome reportedOutcome) {
        return new IllegalStateException(
                factoryCall + " child at index " + childIndex + " is complete but reported "
                        + reportedOutcome + ". A completed child Task must report SUCCESS, TIMEOUT, "
                        + "CANCELLED, or UNKNOWN from getOutcome(). Fix that child's lifecycle "
                        + "contract.");
    }

    /** Consume the one permitted wrapper start before invoking any child callback. */
    private void markStartAttempt() {
        if (startAttempted) {
            throw new IllegalStateException(
                    "The Task returned by " + factoryCall
                            + " is single-use and start(...) was called more than once. "
                            + "Create a fresh task with its builder or macro method, a "
                            + "Supplier<Task>, or an OutputTaskFactory.");
        }
        startAttempted = true;
    }
}
