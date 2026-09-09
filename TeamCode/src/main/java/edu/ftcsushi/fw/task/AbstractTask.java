package edu.ftcsushi.fw.task;

import java.util.Objects;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.lifecycle.CleanupActions;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Advanced implementation base for cooperative Tasks with one guarded execution lifetime.
 *
 * <p>Ordinary robot code uses the existing Task factories and builders. This protected-constructor
 * extension seam shares lifecycle mechanics across framework packages without adding a second
 * ordinary construction path. Implementations supply behavior and owned ending policy, not a
 * runner, clock, exception accumulator, or competing hardware writer.</p>
 *
 * <p>An ending claim immediately prevents further updates and repeated cancellation. Its outcome
 * is consumable only after ending actions and the outermost synchronous lifecycle callback return.
 * This includes an acquisition that returns after reentrant cancellation. Runtime failures remain
 * exceptions, never successful cleanup or ordinary cancellation. Java {@link Error} is not caught.</p>
 */
public abstract class AbstractTask implements Task {
    private final String debugName;
    private boolean startAttempted;
    private boolean started;
    private boolean starting;
    private boolean ending;
    private boolean endingSettled;
    private boolean endingActionsReturned;
    private boolean cancellationInProgress;
    private boolean cancellationRequested;
    private boolean abortAttempted;
    private boolean finishAttempted;
    private int callbackDepth;
    private long lastUpdateCycle = Long.MIN_VALUE;
    private TaskOutcome selectedOutcome = TaskOutcome.NOT_DONE;
    private RuntimeException retainedFailure;

    /** Capture an actionable diagnostic identity without invoking any implementation hook. */
    protected AbstractTask(String debugName) {
        if (debugName == null || debugName.trim().isEmpty()) {
            throw new IllegalArgumentException("AbstractTask requires a nonblank debug name.");
        }
        this.debugName = debugName;
    }

    /** Consume one start attempt, validate the clock, then arm the implementation's lifetime. */
    @Override
    public final void start(LoopClock clock) {
        if (startAttempted) {
            throw new IllegalStateException(getDebugName()
                    + " is single-use; start(...) was called more than once. "
                    + "Create a fresh Task with its factory, builder, or macro method.");
        }
        startAttempted = true;
        requireClock(clock);
        started = true;
        starting = true;
        try {
            guarded(() -> onStart(clock));
        } finally {
            starting = false;
        }
    }

    /**
     * Advance once per cycle. The first update may share the start cycle; recursive updates are
     * inert. A retained failure is checked before cycle deduplication so an effect is never retried.
     */
    @Override
    public final void update(LoopClock clock) {
        if (!started) {
            throw TaskLifecycle.updateBeforeStart(getDebugName());
        }
        checkFailure();
        if (ending || callbackDepth != 0) {
            return;
        }
        guarded(() -> {
            requireClock(clock);
            if (lastUpdateCycle == clock.cycle()) {
                return;
            }
            lastUpdateCycle = clock.cycle();
            onUpdate(clock);
        });
    }

    /**
     * Classify existing terminal evidence, then abort active work and finalize once. Cancellation
     * before start and repeated/terminal cancellation are inert, including after a failed ending.
     */
    @Override
    public final void cancel() {
        if (!started || ending || cancellationInProgress) {
            return;
        }
        guarded(() -> {
            cancellationInProgress = true;
            cancellationRequested = true;
            try {
                onBeforeCancel();
                checkFailure();
                if (!ending) {
                    end(TaskOutcome.CANCELLED, true);
                }
            } finally {
                cancellationInProgress = false;
            }
        });
    }

    /** Return the cached irreversible ending claim, not a promise that cleanup succeeded. */
    @Override
    public final boolean isComplete() {
        return ending;
    }

    /** Return the settled result, or rethrow failure instead of releasing a continuation. */
    @Override
    public final TaskOutcome getOutcome() {
        requireOutcomeAvailable();
        return ending ? selectedOutcome : TaskOutcome.NOT_DONE;
    }

    /** Return the implementation's stable diagnostic name. */
    @Override
    public String getDebugName() {
        return debugName;
    }

    /**
     * Publish cached lifecycle facts even after failure. A diagnostic-only RuntimeException is
     * reported without replacing the lifecycle result; diagnostic hooks must not advance behavior.
     */
    @Override
    public final void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = prefix == null || prefix.isEmpty() ? debugName : prefix;
        dbg.addData(p + ".name", getDebugName())
                .addData(p + ".startAttempted", startAttempted)
                .addData(p + ".started", started)
                .addData(p + ".complete", ending)
                .addData(p + ".endingSettled", endingSettled)
                .addData(p + ".selectedOutcome", selectedOutcome)
                .addData(p + ".outcomeAvailable", endingSettled && retainedFailure == null)
                .addData(p + ".hasLifecycleFailure", retainedFailure != null);
        if (retainedFailure != null) {
            dbg.addData(p + ".failure", retainedFailure.getClass().getSimpleName()
                    + ": " + retainedFailure.getMessage());
        }
        try {
            debugState(dbg, p);
        } catch (RuntimeException diagnosticFailure) {
            dbg.addData(p + ".diagnosticsUnavailable", diagnosticFailure.getClass().getSimpleName()
                    + ": " + diagnosticFailure.getMessage());
        }
    }

    /** Begin implementation work after the common start guard and clock validation. */
    protected abstract void onStart(LoopClock clock);

    /** Advance implementation work once in this cycle, using this unchanged shared clock. */
    protected abstract void onUpdate(LoopClock clock);

    /**
     * Observe exact externally completed work before selecting cancellation, when applicable.
     * A route may call {@link #complete(TaskOutcome)} here to preserve its already-terminal result.
     */
    protected void onBeforeCancel() {
    }

    /**
     * Apply the explicitly chosen active-cancellation policy after ending is claimed. Do not
     * return early merely because {@link #isComplete()} is now true. Called at most once.
     */
    protected abstract void onCancel();

    /**
     * Abort owned work after a lifecycle failure. The default uses the same policy as cancellation;
     * an integration may instead retain its precise failed execution status. Never invent ownership.
     */
    protected void onFailure(RuntimeException failure) {
        onCancel();
    }

    /** Apply an optional all-ending action once, after any active cancellation/failure action. */
    protected void onFinish() {
    }

    /** Add implementation-specific cached diagnostics, without querying this Task's outcome. */
    protected void debugState(DebugSink dbg, String prefix) {
    }

    /** Whether a valid start clock has armed this lifetime, including an ending lifetime. */
    protected final boolean isStarted() {
        return started;
    }

    /** Whether another ordinary implementation effect is still permitted. */
    protected final boolean isActive() {
        return started && !ending && retainedFailure == null;
    }

    /** Whether a lifecycle failure is retained; useful for cached diagnostic eligibility. */
    protected final boolean hasFailure() {
        return retainedFailure != null;
    }

    /** Whether ending actions and the outer synchronous callback have both finished returning. */
    protected final boolean isEndingSettled() {
        return endingSettled;
    }

    /** Propagate a previously retained failure without replaying any effect. */
    protected final void checkFailure() {
        if (retainedFailure != null) {
            throw retainedFailure;
        }
    }

    /**
     * Guard a typed policy/result getter. Pending-ending inspection is itself retained as a
     * lifecycle failure, even if a callback catches it; cached diagnostics use a separate path.
     */
    protected final void requireOutcomeAvailable() {
        checkFailure();
        if (ending && !endingSettled) {
            throw failClosed(new IllegalStateException(getDebugName()
                    + " outcome is not available during terminal cleanup or pending acquisition. "
                    + "Do not inspect the owning Task's outcome or advance its parent from a "
                    + "cleanup callback; wait for the outer lifecycle call to return."));
        }
    }

    /**
     * Observe externally updated status without consuming the effectful update-cycle allowance.
     * Before start, after normal ending, or during another hook, observation is inert. Failures
     * use the same fail-closed path as updates. This is not permission to advance hardware.
     */
    protected final void observe(Runnable observation) {
        Objects.requireNonNull(observation, "Task status observation is required");
        requireOutcomeAvailable();
        if (!started || ending || callbackDepth != 0) {
            return;
        }
        guarded(observation);
    }

    /**
     * Claim a natural outcome inside a lifecycle or {@link #observe(Runnable)} hook, then run the
     * optional ending action once. External evidence must enter through the guarded observation seam.
     */
    protected final void complete(TaskOutcome outcome) {
        requireTerminalOutcome(outcome);
        if (!started) {
            throw new IllegalStateException(getDebugName() + " cannot complete before start(clock).");
        }
        requireGuardedCompletion();
        if (!ending) {
            end(outcome, false);
        }
    }

    /**
     * Finish classification of a handle returned by an in-flight start acquisition. A pending
     * cancellation may be refined to the handle's already-terminal result, before any result is
     * consumable. The owner must first stop/release any acquired active handle as appropriate.
     * This never reactivates work or repeats global cleanup, and is not a general result setter.
     */
    protected final void completeAfterAcquisition(TaskOutcome outcome) {
        requireTerminalOutcome(outcome);
        checkFailure();
        requireGuardedCompletion();
        if (!ending) {
            complete(outcome);
        } else if (starting && callbackDepth > 0 && cancellationRequested
                && !endingSettled && selectedOutcome == TaskOutcome.CANCELLED) {
            selectedOutcome = outcome;
        } else {
            throw new IllegalStateException(getDebugName()
                    + " can refine an ending only for a pending start acquisition cancellation.");
        }
    }

    /** Run one synchronous hook and defer result settlement until the complete callback stack exits. */
    private void guarded(Runnable action) {
        callbackDepth++;
        try {
            action.run();
            checkFailure();
        } catch (RuntimeException failure) {
            throw failClosed(failure);
        } finally {
            callbackDepth--;
            if (callbackDepth == 0 && ending && endingActionsReturned) {
                endingSettled = true;
            }
        }
    }

    /** Claim terminality before callbacks and share synchronous best-effort exception mechanics. */
    private void end(TaskOutcome outcome, boolean abort) {
        ending = true;
        selectedOutcome = outcome;
        try {
            if (retainedFailure != null) {
                CleanupActions.attemptAllAfterFailure(retainedFailure,
                        () -> abortOnce(abort), this::finishOnce);
            } else {
                CleanupActions.attemptAll(() -> abortOnce(abort), this::finishOnce);
            }
        } catch (RuntimeException failure) {
            retainFailure(failure);
        }
        // An Error deliberately bypasses this publication and remaining cleanup actions.
        endingActionsReturned = true;
        checkFailure();
    }

    /** Invoke the one eligible abort hook, recording its attempt before calling external code. */
    private void abortOnce(boolean abort) {
        if (!abort || abortAttempted) {
            return;
        }
        abortAttempted = true;
        try {
            if (retainedFailure != null) {
                onFailure(retainedFailure);
            } else {
                onCancel();
            }
        } catch (RuntimeException failure) {
            // Publish the first failure before the next ending action can inspect this owner.
            // CleanupActions attaches secondary failures; do not attach the same one twice here.
            if (retainedFailure == null) {
                retainedFailure = failure;
            }
            throw failure;
        }
    }

    /** Invoke the optional ending policy once, recording its attempt before external code. */
    private void finishOnce() {
        if (!finishAttempted) {
            finishAttempted = true;
            try {
                onFinish();
            } catch (RuntimeException failure) {
                if (retainedFailure == null) {
                    retainedFailure = failure;
                }
                throw failure;
            }
        }
    }

    /** Preserve the primary failure before best-effort abort and cleanup. */
    private RuntimeException failClosed(RuntimeException failure) {
        retainFailure(failure);
        if (!ending) {
            end(TaskOutcome.CANCELLED, true);
        }
        return retainedFailure;
    }

    /** Retain each distinct propagated failure without self-suppression or retry duplication. */
    private void retainFailure(RuntimeException failure) {
        if (retainedFailure == null) {
            retainedFailure = failure;
        } else if (retainedFailure != failure) {
            for (Throwable suppressed : retainedFailure.getSuppressed()) {
                if (suppressed == failure) {
                    return;
                }
            }
            retainedFailure.addSuppressed(failure);
        }
    }

    /** Validate a result where the implementation claims a terminal boundary. */
    private void requireTerminalOutcome(TaskOutcome outcome) {
        if (outcome == null || outcome == TaskOutcome.NOT_DONE) {
            throw new IllegalStateException(getDebugName() + " cannot finish with " + outcome
                    + "; report SUCCESS, TIMEOUT, CANCELLED, or UNKNOWN.");
        }
    }

    /** Reject missing clock ownership before implementation effects. */
    private void requireClock(LoopClock clock) {
        Objects.requireNonNull(clock, getDebugName()
                + " requires the owning runner's non-null LoopClock.");
    }

    /** Reject an unguarded advanced implementation call before claiming an incomplete ending. */
    private void requireGuardedCompletion() {
        if (callbackDepth == 0) {
            throw new IllegalStateException(getDebugName()
                    + " must complete inside a lifecycle or observe(...) hook, not outside "
                    + "the guarded callback boundary.");
        }
    }
}
