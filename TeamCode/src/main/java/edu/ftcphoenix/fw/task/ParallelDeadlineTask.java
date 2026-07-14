package edu.ftcphoenix.fw.task;

import java.util.Arrays;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Internal structured-concurrency Task whose deadline child owns the lifetime of its companions.
 *
 * <p>The deadline is started and updated first. When it completes, this Task captures its terminal
 * outcome, becomes terminal, and best-effort cancels every companion whose start was attempted.
 * Companion completion never ends the group and companion outcomes never replace the deadline's
 * outcome.</p>
 *
 * <p>This type is deliberately package-private. Robot code should use
 * {@link Tasks#parallelDeadline(Task, Task...)} so the framework exposes one composition API rather
 * than a second public concrete Task class.</p>
 */
final class ParallelDeadlineTask implements Task {

    private final Task deadline;
    private final Task[] companions;
    private final boolean[] companionStartAttempted;

    private boolean startAttempted = false;
    private boolean started = false;
    private boolean deadlineStartAttempted = false;
    private boolean complete = false;
    private TaskOutcome outcome = TaskOutcome.NOT_DONE;

    /**
     * Create one deadline-owned parallel group.
     *
     * <p>The companion array is defensively copied before it is retained. Direct child aliases are
     * rejected before any child can start; hidden aliases in nested composites remain protected by
     * each Task's single-use start guard.</p>
     *
     * @param deadline   child whose completion and outcome end the group
     * @param companions children that run concurrently only while the deadline remains active
     * @throws IllegalArgumentException if an input is null or a direct child identity is reused
     */
    ParallelDeadlineTask(Task deadline, Task... companions) {
        if (deadline == null) {
            throw new IllegalArgumentException(
                    "Tasks.parallelDeadline requires a deadline Task; deadline must not be null.");
        }
        if (companions == null) {
            throw new IllegalArgumentException(
                    "Tasks.parallelDeadline companions must not be null; omit companion arguments "
                            + "to use no companions.");
        }

        this.deadline = deadline;
        this.companions = Arrays.copyOf(companions, companions.length);
        validateDistinctChildren(deadline, this.companions);
        this.companionStartAttempted = new boolean[this.companions.length];
    }

    /**
     * Start the deadline first, then each companion while the deadline remains active.
     *
     * <p>If the deadline completes in its own start callback, no companion starts. A child start
     * attempt is recorded before invoking that child so runner fail-stop can clean partially
     * acquired state when a callback throws.</p>
     */
    @Override
    public void start(LoopClock clock) {
        markStartAttempt();
        started = true;
        outcome = TaskOutcome.NOT_DONE;

        deadlineStartAttempted = true;
        deadline.start(clock);
        if (complete || finishIfDeadlineComplete()) {
            return;
        }

        for (int i = 0; i < companions.length; i++) {
            companionStartAttempted[i] = true;
            companions[i].start(clock);
            if (complete || finishIfDeadlineComplete()) {
                return;
            }
        }
    }

    /**
     * Update the deadline first, then each still-active companion in declaration order.
     *
     * <p>Completion is checked before and after each update. This prevents an extra update when an
     * outer lifecycle owner completes the deadline between cycles, and lets reentrant callbacks end
     * the group before a later companion receives work.</p>
     */
    @Override
    public void update(LoopClock clock) {
        if (!started) {
            throw TaskLifecycle.updateBeforeStart("Task returned by Tasks.parallelDeadline(...)");
        }
        if (complete || finishIfDeadlineComplete()) {
            return;
        }

        deadline.update(clock);
        if (complete || finishIfDeadlineComplete()) {
            return;
        }

        for (int i = 0; i < companions.length; i++) {
            Task companion = companions[i];
            boolean companionComplete = companion.isComplete();
            if (complete || finishIfDeadlineComplete()) {
                return;
            }
            if (companionComplete) {
                continue;
            }

            companion.update(clock);
            if (complete || finishIfDeadlineComplete()) {
                return;
            }
        }
    }

    /**
     * Cancel the active group and best-effort cancel every direct child whose start was attempted.
     *
     * <p>The group becomes terminal before child callbacks. Cleanup deliberately does not query
     * child completion because a failed completion hook may be the reason the runner is cancelling
     * this Task. Under the Task contract, cancellation of an already-terminal child is a no-op.</p>
     */
    @Override
    public void cancel() {
        if (!started || complete) {
            return;
        }

        outcome = TaskOutcome.CANCELLED;
        complete = true;
        cancelAttemptedChildren(true);
    }

    /** {@inheritDoc} */
    @Override
    public boolean isComplete() {
        return complete;
    }

    /**
     * Return the deadline's captured outcome, or this group's direct cancellation result.
     *
     * <p>Companion outcomes do not affect this value.</p>
     */
    @Override
    public TaskOutcome getOutcome() {
        return complete ? outcome : TaskOutcome.NOT_DONE;
    }

    /** Identify this internal implementation by the public factory robot code calls. */
    @Override
    public String getDebugName() {
        return "Tasks.parallelDeadline(...)";
    }

    /**
     * Dump aggregate lifecycle state followed by the deadline and every retained companion.
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "parallelDeadline" : prefix;
        dbg.addData(p + ".started", started)
                .addData(p + ".complete", complete)
                .addData(p + ".outcome", getOutcome())
                .addData(p + ".deadlineStartAttempted", deadlineStartAttempted)
                .addData(p + ".companionCount", companions.length);

        deadline.debugDump(dbg, p + ".deadline");
        for (int i = 0; i < companions.length; i++) {
            String childPrefix = p + ".companion" + i;
            dbg.addData(childPrefix + ".startAttempted", companionStartAttempted[i]);
            companions[i].debugDump(dbg, childPrefix);
        }
    }

    /**
     * Finish naturally if the deadline now reports complete.
     *
     * <p>The terminal outcome is queried and validated before this group becomes terminal. If that
     * query fails, the group remains active so {@link TaskRunner}'s fail-stop path can cancel every
     * start-attempted child. Once capture succeeds, the group becomes terminal before companion
     * cleanup begins.</p>
     *
     * @return {@code true} if the group is terminal after this check
     */
    private boolean finishIfDeadlineComplete() {
        boolean deadlineComplete = deadline.isComplete();
        if (complete) {
            return true;
        }
        if (!deadlineComplete) {
            return false;
        }

        TaskOutcome capturedOutcome = deadline.getOutcome();
        if (complete) {
            // The outcome callback may have reentrantly cancelled this group.
            return true;
        }
        if (capturedOutcome == null || capturedOutcome == TaskOutcome.NOT_DONE) {
            throw malformedDeadlineOutcome(capturedOutcome);
        }

        outcome = capturedOutcome;
        complete = true;
        cancelAttemptedChildren(false);
        return true;
    }

    /**
     * Best-effort cancel the selected direct children in stable declaration order.
     *
     * @param includeDeadline whether the active deadline also receives cancellation
     */
    private void cancelAttemptedChildren(boolean includeDeadline) {
        RuntimeException firstFailure = null;
        if (includeDeadline && deadlineStartAttempted) {
            firstFailure = cancelAndAccumulate(deadline, firstFailure);
        }
        for (int i = 0; i < companions.length; i++) {
            if (companionStartAttempted[i]) {
                firstFailure = cancelAndAccumulate(companions[i], firstFailure);
            }
        }
        if (firstFailure != null) {
            throw firstFailure;
        }
    }

    /** Call one child cancellation hook and retain failures without skipping later children. */
    private static RuntimeException cancelAndAccumulate(Task child,
                                                         RuntimeException firstFailure) {
        try {
            child.cancel();
            return firstFailure;
        } catch (RuntimeException failure) {
            if (firstFailure == null) {
                return failure;
            }
            if (failure != firstFailure) {
                firstFailure.addSuppressed(failure);
            }
            return firstFailure;
        }
    }

    /** Reject direct aliases before the first child can acquire state. */
    private static void validateDistinctChildren(Task deadline, Task[] companions) {
        for (int i = 0; i < companions.length; i++) {
            Task companion = companions[i];
            if (companion == null) {
                throw new IllegalArgumentException(
                        "Tasks.parallelDeadline companion at index " + i + " must not be null.");
            }
            if (companion == deadline) {
                throw duplicateChild(
                        "deadline", "companion at index " + i);
            }
            for (int previous = 0; previous < i; previous++) {
                if (companion == companions[previous]) {
                    throw duplicateChild(
                            "companion at index " + previous, "companion at index " + i);
                }
            }
        }
    }

    /** Build one actionable direct-alias construction error. */
    private static IllegalArgumentException duplicateChild(String firstRole, String secondRole) {
        return new IllegalArgumentException(
                "Tasks.parallelDeadline requires distinct Task instances, but " + firstRole
                        + " and " + secondRole + " reference the same object. Create each child "
                        + "as a fresh task with its builder or macro method, a Supplier<Task>, or "
                        + "an OutputTaskFactory.");
    }

    /** Build an actionable error for a completed deadline with a non-terminal outcome. */
    private static IllegalStateException malformedDeadlineOutcome(TaskOutcome reportedOutcome) {
        return new IllegalStateException(
                "Tasks.parallelDeadline deadline is complete but reported " + reportedOutcome
                        + ". A completed deadline Task must report SUCCESS, TIMEOUT, CANCELLED, or "
                        + "UNKNOWN from getOutcome(). Fix the deadline Task's lifecycle contract.");
    }

    /** Consume the one permitted wrapper start before invoking any child callback. */
    private void markStartAttempt() {
        if (startAttempted) {
            throw new IllegalStateException(
                    "The Task returned by Tasks.parallelDeadline(...) is single-use and "
                            + "start(...) was called more than once. "
                            + "Create a fresh task with its builder or macro method, a "
                            + "Supplier<Task>, or an OutputTaskFactory.");
        }
        startAttempted = true;
    }
}
