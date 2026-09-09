package edu.ftcsushi.fw.task;

import java.util.Objects;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;

/** Adds one caller-owned ending action using the same lifecycle as built-in timed Tasks. */
final class CleanupTask extends AbstractTask {
    private final Task child;
    private final Runnable cleanup;
    private boolean childStartAttempted;
    private boolean childCancellationAttempted;
    private boolean childTerminalObserved;
    private boolean childCallbackInProgress;
    private boolean childCancellationValidationPending;
    private boolean cleanupAttempted;
    private TaskOutcome childOutcome = TaskOutcome.NOT_DONE;

    /** Capture collaborators without starting work or acquiring cleanup responsibility. */
    CleanupTask(Task child, Runnable cleanup) {
        super("Tasks.withCleanup(...)");
        this.child = Objects.requireNonNull(child,
                "Tasks.withCleanup requires a non-null child Task.");
        this.cleanup = Objects.requireNonNull(cleanup,
                "Tasks.withCleanup requires a non-null synchronous cleanup action.");
    }

    /** Arm the child's cancellation responsibility before attempting its start. */
    @Override
    protected void onStart(LoopClock clock) {
        childStartAttempted = true;
        invokeChild(() -> child.start(clock));
        checkFailure();
        if (isActive()) {
            observeNaturalCompletion();
        }
    }

    /** Observe completion before and after advancing the child. */
    @Override
    protected void onUpdate(LoopClock clock) {
        observeNaturalCompletion();
        if (isActive()) {
            invokeChild(() -> child.update(clock));
            checkFailure();
            if (isActive()) {
                observeNaturalCompletion();
            }
        }
    }

    /** Cancel only started, not already-proven-terminal child work and validate its ending. */
    @Override
    protected void onCancel() {
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

    /** A reentrant cancel cannot consume the child's result until its outer callback returns. */
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

    /** Validate a returned child ending once, after any in-flight child callback has settled. */
    private void validateChildCancellation() {
        childCancellationValidationPending = false;
        if (!child.isComplete()) {
            throw new IllegalStateException(
                    "Tasks.withCleanup child cancel() returned without making the child "
                            + "terminal. Fix active cancellation so isComplete() returns true "
                            + "before cancel() returns.");
        }
        childTerminalObserved = true;
        childOutcome = requireChildOutcome(child.getOutcome());
    }

    /** The base attempts this action even when child cancellation throws. */
    @Override
    protected void onFinish() {
        cleanupAttempted = true;
        cleanup.run();
    }

    /** Publish cached owner facts and only eligible, non-advancing child diagnostics. */
    @Override
    protected void debugState(DebugSink dbg, String prefix) {
        dbg.addData(prefix + ".childStartAttempted", childStartAttempted)
                .addData(prefix + ".childCancellationAttempted", childCancellationAttempted)
                .addData(prefix + ".childTerminalObserved", childTerminalObserved)
                .addData(prefix + ".cleanupAttempted", cleanupAttempted)
                .addData(prefix + ".childOutcome", childOutcome);
        if (isStarted() && !hasFailure() && (!isComplete() || isEndingSettled())) {
            try {
                child.debugDump(dbg, prefix + ".child");
            } catch (RuntimeException diagnosticFailure) {
                dbg.addData(prefix + ".childDiagnosticsUnavailable",
                        diagnosticFailure.getClass().getSimpleName() + ": "
                                + diagnosticFailure.getMessage());
            }
        }
    }

    /** Capture one truthful natural ending without querying a child after reentrant cancellation. */
    private void observeNaturalCompletion() {
        boolean terminal = child.isComplete();
        checkFailure();
        if (!isActive() || !terminal) {
            return;
        }
        childTerminalObserved = true;
        TaskOutcome observed = child.getOutcome();
        checkFailure();
        if (isActive()) {
            childOutcome = requireChildOutcome(observed);
            complete(childOutcome);
        }
    }

    /** Reject malformed terminal child outcomes before they can release a continuation. */
    private static TaskOutcome requireChildOutcome(TaskOutcome outcome) {
        if (outcome == null || outcome == TaskOutcome.NOT_DONE) {
            throw new IllegalStateException("Tasks.withCleanup child is terminal but reported "
                    + outcome + ". A completed Task must report SUCCESS, TIMEOUT, CANCELLED, "
                    + "or UNKNOWN; fix the child's getOutcome() contract.");
        }
        return outcome;
    }
}
