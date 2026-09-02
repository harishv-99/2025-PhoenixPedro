package edu.ftcsushi.fw.task;

import java.util.Arrays;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Internal wait-all composition for {@link Tasks#parallelAll(Task...)}.
 *
 * <p>Every valid child starts and the group remains active until every child finishes. Completed
 * child outcomes are validated and cached once. All-success reports {@link TaskOutcome#SUCCESS};
 * identical non-success outcomes are retained; mixed non-success kinds report
 * {@link TaskOutcome#UNKNOWN}. This is deliberately not a fail-fast or race composition.</p>
 *
 * <p>This type remains package-private so {@link Tasks} is the one public construction surface.</p>
 */
final class ParallelAllTask implements Task {

    private final Task[] tasks;
    private final boolean[] childStartAttempted;
    private final TaskOutcome[] completedOutcomes;

    private boolean startAttempted = false;
    private boolean started = false;
    private boolean complete = false;
    private TaskOutcome outcome = TaskOutcome.NOT_DONE;

    /** Create one defensively copied all-children parallel graph. */
    ParallelAllTask(Task... tasks) {
        if (tasks == null) {
            throw new IllegalArgumentException("Tasks.parallelAll(...) tasks are required");
        }
        this.tasks = Arrays.copyOf(tasks, tasks.length);
        validateDistinctChildren(this.tasks);
        this.childStartAttempted = new boolean[this.tasks.length];
        this.completedOutcomes = new TaskOutcome[this.tasks.length];
    }

    /** Start each valid child in declaration order, then capture any immediate completions. */
    @Override
    public void start(LoopClock clock) {
        markStartAttempt();
        started = true;
        try {
            for (int i = 0; i < tasks.length; i++) {
                if (complete) {
                    return;
                }
                childStartAttempted[i] = true;
                tasks[i].start(clock);
            }
            observeAllCompletions();
        } catch (RuntimeException failure) {
            throw failClosed(failure);
        }
    }

    /**
     * Update every still-active child once in declaration order. Completion is checked before each
     * update so a child that became terminal between cycles receives no extra update.
     */
    @Override
    public void update(LoopClock clock) {
        if (!started) {
            throw TaskLifecycle.updateBeforeStart("Task returned by Tasks.parallelAll(...)");
        }
        if (complete) {
            return;
        }

        try {
            for (int i = 0; i < tasks.length; i++) {
                if (complete) {
                    return;
                }
                if (completedOutcomes[i] != null) {
                    continue;
                }

                Task child = tasks[i];
                boolean childComplete = child.isComplete();
                if (complete) {
                    return;
                }
                if (!childComplete) {
                    child.update(clock);
                    if (complete) {
                        return;
                    }
                    childComplete = child.isComplete();
                    if (complete) {
                        return;
                    }
                }
                if (childComplete) {
                    captureOutcome(i);
                }
            }
            finishIfAllComplete();
        } catch (RuntimeException failure) {
            throw failClosed(failure);
        }
    }

    /** Terminalize first, then best-effort cancel every direct child. */
    @Override
    public void cancel() {
        if (!started || complete) {
            return;
        }
        complete = true;
        outcome = TaskOutcome.CANCELLED;
        RuntimeException cleanupFailure = cancelChildren(null, false);
        if (cleanupFailure != null) {
            throw cleanupFailure;
        }
    }

    /** {@inheritDoc} */
    @Override
    public boolean isComplete() {
        return complete;
    }

    /** Return the cached aggregate without re-querying completed children. */
    @Override
    public TaskOutcome getOutcome() {
        return complete ? outcome : TaskOutcome.NOT_DONE;
    }

    /** Identify this internal implementation by the public factory robot code calls. */
    @Override
    public String getDebugName() {
        return "Tasks.parallelAll(...)";
    }

    /** Dump cached group and child-completion state without re-sampling child outcomes. */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "parallelAll" : prefix;
        int completedCount = 0;
        for (TaskOutcome childOutcome : completedOutcomes) {
            if (childOutcome != null) {
                completedCount++;
            }
        }
        dbg.addData(p + ".started", started)
                .addData(p + ".complete", complete)
                .addData(p + ".size", tasks.length)
                .addData(p + ".completedCount", completedCount)
                .addData(p + ".outcome", getOutcome());
        for (int i = 0; i < tasks.length; i++) {
            dbg.addData(p + ".child" + i + ".name", tasks[i].getDebugName())
                    .addData(p + ".child" + i + ".startAttempted", childStartAttempted[i])
                    .addData(p + ".child" + i + ".capturedOutcome", completedOutcomes[i]);
        }
    }

    /** Observe and cache every child that is already complete. */
    private void observeAllCompletions() {
        for (int i = 0; i < tasks.length; i++) {
            if (complete || completedOutcomes[i] != null) {
                continue;
            }
            boolean childComplete = tasks[i].isComplete();
            if (complete) {
                return;
            }
            if (childComplete) {
                captureOutcome(i);
            }
        }
        finishIfAllComplete();
    }

    /** Capture and validate one completed child result exactly once. */
    private void captureOutcome(int childIndex) {
        if (completedOutcomes[childIndex] != null) {
            return;
        }
        TaskOutcome childOutcome = tasks[childIndex].getOutcome();
        if (complete) {
            // A custom outcome callback may have directly cancelled this wrapper.
            return;
        }
        if (childOutcome == null || childOutcome == TaskOutcome.NOT_DONE) {
            throw malformedChildOutcome(childIndex, childOutcome);
        }
        completedOutcomes[childIndex] = childOutcome;
    }

    /** Finish and aggregate only after every child has one cached terminal outcome. */
    private void finishIfAllComplete() {
        if (complete) {
            return;
        }
        for (TaskOutcome childOutcome : completedOutcomes) {
            if (childOutcome == null) {
                return;
            }
        }

        TaskOutcome retainedNonSuccess = null;
        for (TaskOutcome childOutcome : completedOutcomes) {
            if (childOutcome == TaskOutcome.SUCCESS) {
                continue;
            }
            if (retainedNonSuccess == null) {
                retainedNonSuccess = childOutcome;
            } else if (retainedNonSuccess != childOutcome) {
                retainedNonSuccess = TaskOutcome.UNKNOWN;
                break;
            }
        }
        outcome = retainedNonSuccess == null ? TaskOutcome.SUCCESS : retainedNonSuccess;
        complete = true;
    }

    /**
     * Fail closed on a child lifecycle error, attempt every started child's cleanup, and keep the
     * original error primary.
     */
    private RuntimeException failClosed(RuntimeException failure) {
        if (complete) {
            return failure;
        }
        complete = true;
        outcome = TaskOutcome.CANCELLED;
        cancelChildren(failure, true);
        return failure;
    }

    /** Best-effort cancellation with later failures suppressed on the supplied primary failure. */
    private RuntimeException cancelChildren(RuntimeException primaryFailure,
                                            boolean startedChildrenOnly) {
        RuntimeException retainedFailure = primaryFailure;
        for (int i = 0; i < tasks.length; i++) {
            if (startedChildrenOnly && !childStartAttempted[i]) {
                continue;
            }
            try {
                tasks[i].cancel();
            } catch (RuntimeException cleanupFailure) {
                if (retainedFailure == null) {
                    retainedFailure = cleanupFailure;
                } else if (cleanupFailure != retainedFailure) {
                    retainedFailure.addSuppressed(cleanupFailure);
                }
            }
        }
        return retainedFailure;
    }

    /** Reject nulls and direct aliases before any child can acquire state. */
    private static void validateDistinctChildren(Task[] children) {
        for (int i = 0; i < children.length; i++) {
            Task child = children[i];
            if (child == null) {
                throw new IllegalArgumentException(
                        "Tasks.parallelAll(...) children must not contain null; found null at index "
                                + i);
            }
            for (int previous = 0; previous < i; previous++) {
                if (child == children[previous]) {
                    throw new IllegalArgumentException(
                            "Tasks.parallelAll(...) child at index " + i
                                    + " reuses the same Task instance as index " + previous + ". "
                                    + "Each child must be a distinct, fresh task; create it with "
                                    + "its builder or macro method, a Supplier<Task>, or an "
                                    + "OutputTaskFactory.");
                }
            }
        }
    }

    /** Build an actionable error for a complete child with a non-terminal result. */
    private static IllegalStateException malformedChildOutcome(int childIndex,
                                                               TaskOutcome reportedOutcome) {
        return new IllegalStateException(
                "Tasks.parallelAll(...) child at index " + childIndex
                        + " is complete but reported " + reportedOutcome
                        + ". A completed child Task must report SUCCESS, TIMEOUT, CANCELLED, or "
                        + "UNKNOWN from getOutcome(). Fix that child's lifecycle contract.");
    }

    /** Consume the one permitted wrapper start before invoking any child callback. */
    private void markStartAttempt() {
        if (startAttempted) {
            throw new IllegalStateException(
                    "The Task returned by Tasks.parallelAll(...) is single-use and start(...) was "
                            + "called more than once. Create a fresh task with its builder or macro "
                            + "method, a Supplier<Task>, or an OutputTaskFactory.");
        }
        startAttempted = true;
    }
}
