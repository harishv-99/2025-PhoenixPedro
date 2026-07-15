package edu.ftcphoenix.fw.task;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Internal hard-time-limit decorator exposed through {@link Tasks#withTimeout(Task, double)}.
 *
 * <p>The timeout belongs to this wrapper, not to the wrapped operation. When the limit is reached,
 * this Task invokes the child's ordinary active {@link Task#cancel()} path and reports
 * {@link TaskOutcome#TIMEOUT} only after that cancellation returns and the child exposes a valid
 * terminal lifecycle. A task-specific timeout may intentionally behave differently; for example,
 * it may retain a more precise status or apply timeout-specific mechanism requests.</p>
 *
 * <p>This implementation is package-private so generic Task construction has one supported public
 * surface in {@link Tasks}.</p>
 */
final class TimeoutTask implements Task {

    private final Task child;
    private final double timeoutSec;

    private boolean startAttempted;
    private boolean started;
    private boolean startInProgress;
    private boolean updateInProgress;
    private boolean childStartAttempted;
    private boolean childCancellationAttempted;
    private boolean timeoutCancellationInProgress;
    private boolean complete;
    private boolean timeoutFired;
    private double startSec;
    private double elapsedSec;
    private long lastUpdatedCycle = Long.MIN_VALUE;
    private TaskOutcome outcome = TaskOutcome.NOT_DONE;
    private TaskOutcome retainedChildOutcome = TaskOutcome.NOT_DONE;
    private RuntimeException retainedLifecycleFailure;

    /** Create one single-use timeout decorator. */
    TimeoutTask(Task child, double timeoutSec) {
        this.child = Objects.requireNonNull(child,
                "Tasks.withTimeout requires a child Task; task must not be null.");
        if (!Double.isFinite(timeoutSec) || timeoutSec < 0.0) {
            throw new IllegalArgumentException(
                    "Tasks.withTimeout timeoutSec must be finite and >= 0, got " + timeoutSec);
        }
        this.timeoutSec = timeoutSec;
    }

    /**
     * Start the wrapper timer and, for a positive limit, start the child at the same clock
     * boundary. A zero limit completes without starting or cancelling the child.
     */
    @Override
    public void start(LoopClock clock) {
        markStartAttempt();
        requireClock(clock);

        started = true;
        startInProgress = true;
        startSec = clock.nowSec();
        elapsedSec = 0.0;
        outcome = TaskOutcome.NOT_DONE;
        retainedChildOutcome = TaskOutcome.NOT_DONE;

        try {
            if (timeoutSec == 0.0) {
                timeoutFired = true;
                outcome = TaskOutcome.TIMEOUT;
                complete = true;
                return;
            }

            childStartAttempted = true;
            try {
                child.start(clock);
            } catch (RuntimeException failure) {
                throw retainLifecycleFailure(failure);
            }
            if (complete) {
                return;
            }
            finishNaturallyIfChildComplete();
        } finally {
            startInProgress = false;
        }
    }

    /**
     * Observe an already-terminal child first, then enforce the limit before allowing another
     * child update. This gives a child that completed between cycles precedence at the exact
     * boundary while preventing new child work once the hard budget has elapsed.
     */
    @Override
    public void update(LoopClock clock) {
        if (!started) {
            throw TaskLifecycle.updateBeforeStart("Task returned by Tasks.withTimeout(...)");
        }
        if (complete || startInProgress || updateInProgress) {
            return;
        }
        requireClock(clock);

        long cycle = clock.cycle();
        if (cycle == lastUpdatedCycle) {
            return;
        }
        lastUpdatedCycle = cycle;

        updateInProgress = true;
        try {
            if (retainedLifecycleFailure != null) {
                throw retainedLifecycleFailure;
            }
            elapsedSec = Math.max(0.0, clock.nowSec() - startSec);
            if (finishNaturallyIfChildComplete()) {
                return;
            }

            if (elapsedSec >= timeoutSec) {
                finishByTimeout();
                return;
            }

            try {
                child.update(clock);
            } catch (RuntimeException failure) {
                throw retainLifecycleFailure(failure);
            }
            if (complete) {
                return;
            }
            finishNaturallyIfChildComplete();
        } finally {
            updateInProgress = false;
        }
    }

    /**
     * Direct active cancellation is terminal and uses the child's ordinary cancellation path at
     * most once. A reentrant cancellation during timeout cleanup cannot relabel the already-chosen
     * timeout ending.
     */
    @Override
    public void cancel() {
        if (!started || complete || timeoutCancellationInProgress) {
            return;
        }

        outcome = TaskOutcome.CANCELLED;
        complete = true;
        if (!childStartAttempted || childCancellationAttempted) {
            return;
        }

        childCancellationAttempted = true;
        child.cancel();
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
        return "Tasks.withTimeout(...)";
    }

    /** Retain wrapper timing and the child snapshot even after terminal completion. */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "withTimeout" : prefix;
        dbg.addData(p + ".timeoutSec", timeoutSec)
                .addData(p + ".startAttempted", startAttempted)
                .addData(p + ".started", started)
                .addData(p + ".childStartAttempted", childStartAttempted)
                .addData(p + ".childCancellationAttempted", childCancellationAttempted)
                .addData(p + ".timeoutFired", timeoutFired)
                .addData(p + ".startSec", startSec)
                .addData(p + ".elapsedSec", elapsedSec)
                .addData(p + ".complete", complete)
                .addData(p + ".outcome", getOutcome())
                .addData(p + ".retainedChildOutcome", retainedChildOutcome)
                .addData(p + ".hasLifecycleFailure", retainedLifecycleFailure != null);
        child.debugDump(dbg, p + ".child");
    }

    /** Capture and validate the child's natural terminal outcome. */
    private boolean finishNaturallyIfChildComplete() {
        final boolean childComplete;
        try {
            childComplete = child.isComplete();
        } catch (RuntimeException failure) {
            throw retainLifecycleFailure(failure);
        }
        if (complete) {
            return true;
        }
        if (!childComplete) {
            return false;
        }

        TaskOutcome capturedOutcome = readTerminalChildOutcome("completed");
        if (complete) {
            return true;
        }
        retainedChildOutcome = capturedOutcome;
        outcome = capturedOutcome;
        complete = true;
        return true;
    }

    /**
     * Attempt timeout cancellation once and publish TIMEOUT only after valid terminal cleanup.
     * Any failure remains latched and nonterminal until an outer fail-stop owner directly cancels
     * this wrapper, preventing a sequence continuation from being released after failed cleanup.
     */
    private void finishByTimeout() {
        if (childCancellationAttempted) {
            throw retainedLifecycleFailure != null
                    ? retainedLifecycleFailure
                    : retainLifecycleFailure(new IllegalStateException(
                            "Tasks.withTimeout cannot retry child cancellation after a failed "
                                    + "timeout cleanup attempt."));
        }

        timeoutFired = true;
        childCancellationAttempted = true;
        timeoutCancellationInProgress = true;
        try {
            try {
                child.cancel();
            } catch (RuntimeException failure) {
                throw retainLifecycleFailure(failure);
            }

            final boolean childComplete;
            try {
                childComplete = child.isComplete();
            } catch (RuntimeException failure) {
                throw retainLifecycleFailure(failure);
            }
            if (!childComplete) {
                throw retainLifecycleFailure(new IllegalStateException(
                        "Tasks.withTimeout reached its limit, but child cancel() returned without "
                                + "making the child terminal. Active Task cancellation must make "
                                + "isComplete() return true."));
            }

            retainedChildOutcome = readTerminalChildOutcome("cancelled at the timeout");
            outcome = TaskOutcome.TIMEOUT;
            complete = true;
        } finally {
            timeoutCancellationInProgress = false;
        }
    }

    /** Read one child outcome and reject null/NOT_DONE after terminal completion. */
    private TaskOutcome readTerminalChildOutcome(String ending) {
        final TaskOutcome capturedOutcome;
        try {
            capturedOutcome = child.getOutcome();
        } catch (RuntimeException failure) {
            throw retainLifecycleFailure(failure);
        }
        if (complete) {
            return capturedOutcome;
        }
        if (capturedOutcome == null || capturedOutcome == TaskOutcome.NOT_DONE) {
            throw retainLifecycleFailure(new IllegalStateException(
                    "Tasks.withTimeout child was " + ending + " but reported " + capturedOutcome
                            + ". A terminal child Task must report SUCCESS, TIMEOUT, CANCELLED, or "
                            + "UNKNOWN from getOutcome(). Fix the child Task's lifecycle contract."));
        }
        return capturedOutcome;
    }

    /** Preserve the first lifecycle failure so repeated direct updates cannot release a child. */
    private RuntimeException retainLifecycleFailure(RuntimeException failure) {
        if (retainedLifecycleFailure == null) {
            retainedLifecycleFailure = failure;
        }
        return retainedLifecycleFailure;
    }

    /** Consume the one permitted wrapper start before invoking the child. */
    private void markStartAttempt() {
        if (startAttempted) {
            throw new IllegalStateException(
                    "The Task returned by Tasks.withTimeout(...) is single-use and start(...) was "
                            + "called more than once. Create a fresh task with its builder or "
                            + "macro method, a Supplier<Task>, or an OutputTaskFactory.");
        }
        startAttempted = true;
    }

    /** Require the real shared loop clock used by TaskRunner. */
    private static void requireClock(LoopClock clock) {
        if (clock == null) {
            throw new IllegalArgumentException(
                    "Tasks.withTimeout requires a non-null LoopClock; start and update it through "
                            + "the owning TaskRunner.");
        }
    }
}
