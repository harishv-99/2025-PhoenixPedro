package edu.ftcsushi.fw.task;

import java.util.Objects;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Hard-time-limit decorator created through Tasks.withTimeout, using the shared Task lifecycle.
 *
 * <p>The wrapper observes an already-completed child first; otherwise it enforces the budget
 * before another child update. Its TIMEOUT uses the child's ordinary cancellation policy, unlike
 * an operation-owned timeout. Zero duration starts no child. Failed cancellation or malformed
 * terminal evidence remains an exception and cannot release either sequence policy.</p>
 */
final class TimeoutTask extends AbstractTask {
    private final Task child;
    private final double timeoutSec;
    private boolean childStartAttempted;
    private boolean childCancellationAttempted;
    private boolean childTerminalObserved;
    private boolean childCallbackInProgress;
    private boolean childCancellationValidationPending;
    private boolean timeoutFired;
    private double startSec;
    private double elapsedSec;
    private TaskOutcome retainedChildOutcome = TaskOutcome.NOT_DONE;

    /** Validate the finite non-negative budget without invoking the child. */
    TimeoutTask(Task child, double timeoutSec) {
        super("Tasks.withTimeout(...)");
        this.child = Objects.requireNonNull(child,
                "Tasks.withTimeout requires a child Task; task must not be null.");
        if (!Double.isFinite(timeoutSec) || timeoutSec < 0.0) {
            throw new IllegalArgumentException(
                    "Tasks.withTimeout timeoutSec must be finite and >= 0, got " + timeoutSec);
        }
        this.timeoutSec = timeoutSec;
    }

    /** Start this budget at its own boundary; an empty budget never starts the child. */
    @Override
    protected void onStart(LoopClock clock) {
        startSec = clock.nowSec();
        if (timeoutSec == 0.0) {
            timeoutFired = true;
            complete(TaskOutcome.TIMEOUT);
            return;
        }
        childStartAttempted = true;
        invokeChild(() -> child.start(clock));
        checkFailure();
        if (isActive()) {
            finishNaturallyIfChildComplete();
        }
    }

    /** Preserve natural-completion-before-budget and budget-before-next-update precedence. */
    @Override
    protected void onUpdate(LoopClock clock) {
        elapsedSec = Math.max(0.0, clock.nowSec() - startSec);
        if (finishNaturallyIfChildComplete() || !isActive()) {
            return;
        }
        if (elapsedSec >= timeoutSec) {
            timeoutFired = true;
            complete(TaskOutcome.TIMEOUT);
            return;
        }
        invokeChild(() -> child.update(clock));
        checkFailure();
        if (isActive()) {
            finishNaturallyIfChildComplete();
        }
    }

    /** Abort the exact started child, without selecting a timeout on direct cancellation/failure. */
    @Override
    protected void onCancel() {
        cancelChildOnce();
    }

    /** A timeout's ending policy cancels the child before a TIMEOUT result can become consumable. */
    @Override
    protected void onFinish() {
        if (timeoutFired) {
            cancelChildOnce();
        }
    }

    /** Retain one already-terminal natural result without querying after callback cancellation. */
    private boolean finishNaturallyIfChildComplete() {
        boolean terminal = child.isComplete();
        checkFailure();
        if (!isActive()) {
            return true;
        }
        if (!terminal) {
            return false;
        }
        childTerminalObserved = true;
        TaskOutcome captured = child.getOutcome();
        checkFailure();
        if (isActive()) {
            retainedChildOutcome = requireChildOutcome(captured, "completed");
            complete(retainedChildOutcome);
        }
        return true;
    }

    /** Stop only owned work, recording the attempt before callbacks and validating terminality. */
    private void cancelChildOnce() {
        if (!childStartAttempted || childTerminalObserved || childCancellationAttempted) {
            return;
        }
        childCancellationAttempted = true;
        child.cancel();
        childCancellationValidationPending = true;
        if (!childCallbackInProgress) {
            validateChildCancellation();
        }
    }

    /** Let a reentrantly cancelled child's outer callback settle before reading its result. */
    private void invokeChild(Runnable action) {
        childCallbackInProgress = true;
        try {
            action.run();
        } finally {
            childCallbackInProgress = false;
        }
        if (childCancellationValidationPending) {
            validateChildCancellation();
        }
    }

    /** Validate deferred cancellation without retrying a child's cancellation action. */
    private void validateChildCancellation() {
        childCancellationValidationPending = false;
        if (!child.isComplete()) {
            throw new IllegalStateException(
                    "Tasks.withTimeout child cancel() returned without making the child terminal. "
                            + "Active Task cancellation must make isComplete() return true.");
        }
        childTerminalObserved = true;
        retainedChildOutcome = requireChildOutcome(child.getOutcome(), "cancelled");
    }

    /** Reject a child that calls itself complete without a valid terminal outcome. */
    private static TaskOutcome requireChildOutcome(TaskOutcome outcome, String ending) {
        if (outcome == null || outcome == TaskOutcome.NOT_DONE) {
            throw new IllegalStateException("Tasks.withTimeout child was " + ending
                    + " but reported " + outcome + ". A terminal child Task must report SUCCESS, "
                    + "TIMEOUT, CANCELLED, or UNKNOWN from getOutcome(). Fix its lifecycle contract.");
        }
        return outcome;
    }

    /** Publish cached timing and child evidence without inspecting failed/pending outcomes. */
    @Override
    protected void debugState(DebugSink dbg, String prefix) {
        dbg.addData(prefix + ".timeoutSec", timeoutSec)
                .addData(prefix + ".childStartAttempted", childStartAttempted)
                .addData(prefix + ".childCancellationAttempted", childCancellationAttempted)
                .addData(prefix + ".timeoutFired", timeoutFired)
                .addData(prefix + ".startSec", startSec)
                .addData(prefix + ".elapsedSec", elapsedSec)
                .addData(prefix + ".retainedChildOutcome", retainedChildOutcome);
        if (isStarted() && !hasFailure() && (!isComplete() || isEndingSettled())) {
            child.debugDump(dbg, prefix + ".child");
        }
    }
}
