package edu.ftcphoenix.fw.task;

import java.util.Objects;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Internal single-use wrapper exposed only through {@link Tasks#buildAtStart(String, Supplier)}.
 *
 * <p>The wrapper owns the deferred-construction boundary and retains the exact child returned for
 * that start. Keeping this implementation package-private leaves {@link Tasks} as the one public
 * generic Task-construction layer.</p>
 */
final class BuildAtStartTask implements Task {

    /** Wrapper lifecycle states; declared at class scope for Java 8 compatibility. */
    private enum State {
        NOT_STARTED,
        STARTING,
        ACTIVE,
        NATURAL_COMPLETE,
        CANCELLED,
        FAILED
    }

    private final String debugName;
    private final Supplier<? extends Task> factory;

    private boolean startAttempted;
    private boolean childStartAttempted;
    private boolean childCancellationAttempted;
    private boolean completionObservationInProgress;
    private boolean completionObservationFailed;
    private State state = State.NOT_STARTED;
    private Task child;

    /** Create one deferred wrapper while retaining, but not sampling, its factory. */
    BuildAtStartTask(String debugName, Supplier<? extends Task> factory) {
        if (debugName == null || debugName.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "Tasks.buildAtStart debugName must be nonblank, got '" + debugName + "'.");
        }
        this.debugName = debugName;
        this.factory = Objects.requireNonNull(
                factory,
                "Tasks.buildAtStart requires a non-null Task factory.");
    }

    /** Build, retain, and start the exact child once at this wrapper's start boundary. */
    @Override
    public void start(LoopClock clock) {
        markStartAttempt();
        state = State.STARTING;

        final Task builtChild;
        try {
            builtChild = factory.get();
        } catch (RuntimeException failure) {
            state = State.FAILED;
            throw failure;
        }

        if (builtChild == null) {
            state = State.FAILED;
            throw new IllegalStateException(
                    "Tasks.buildAtStart '" + debugName + "' factory returned null. Return a fresh "
                            + "Task for this start.");
        }
        if (builtChild == this) {
            state = State.FAILED;
            throw new IllegalStateException(
                    "Tasks.buildAtStart '" + debugName + "' factory returned its own wrapper. "
                            + "Return a distinct fresh child Task; a Task cannot own itself.");
        }

        // Retain the exact child before start so partial acquisition always has a cleanup path.
        child = builtChild;
        if (state != State.STARTING) {
            // Reentrant active cancellation while the factory ran made the wrapper terminal. The
            // returned child never started, so its pre-start cancellation hook is not invoked.
            return;
        }

        childStartAttempted = true;
        try {
            child.start(clock);
        } catch (RuntimeException startFailure) {
            failAfterChildStart(startFailure);
            throw startFailure;
        }

        if (state == State.STARTING) {
            state = State.ACTIVE;
        }
    }

    /** Forward active updates to the exact retained child. */
    @Override
    public void update(LoopClock clock) {
        if (state == State.NOT_STARTED) {
            throw TaskLifecycle.updateBeforeStart(
                    "Tasks.buildAtStart('" + debugName + "')");
        }
        if (state != State.ACTIVE) {
            return;
        }

        child.update(clock);
    }

    /**
     * Make active cancellation terminal before invoking the child's cleanup hook. Natural child
     * completion and every already-terminal wrapper state are inert.
     */
    @Override
    public void cancel() {
        if (state == State.NOT_STARTED || isTerminalState(state)) {
            return;
        }

        if (state == State.ACTIVE
                && !completionObservationInProgress
                && !completionObservationFailed) {
            completionObservationInProgress = true;
            final boolean childComplete;
            try {
                childComplete = child.isComplete();
            } catch (RuntimeException observationFailure) {
                completionObservationFailed = true;
                state = State.CANCELLED;
                bestEffortCancelChild(observationFailure);
                throw observationFailure;
            } finally {
                completionObservationInProgress = false;
            }
            if (state != State.ACTIVE) {
                return;
            }
            if (childComplete) {
                state = State.NATURAL_COMPLETE;
                return;
            }
        }

        state = State.CANCELLED;
        cancelChildOnce();
    }

    /** Forward completion while retaining an observed natural terminal state. */
    @Override
    public boolean isComplete() {
        if (isTerminalState(state)) {
            return true;
        }
        if (state != State.ACTIVE || completionObservationInProgress) {
            return false;
        }

        completionObservationInProgress = true;
        try {
            boolean childComplete = child.isComplete();
            completionObservationFailed = false;
            if (state == State.ACTIVE && childComplete) {
                state = State.NATURAL_COMPLETE;
            }
            return isTerminalState(state);
        } catch (RuntimeException failure) {
            completionObservationFailed = true;
            throw failure;
        } finally {
            completionObservationInProgress = false;
        }
    }

    /** Forward the active or naturally completed child's outcome. */
    @Override
    public TaskOutcome getOutcome() {
        switch (state) {
            case CANCELLED:
            case FAILED:
                return TaskOutcome.CANCELLED;
            case ACTIVE:
            case NATURAL_COMPLETE:
                TaskOutcome childOutcome = child.getOutcome();
                return state == State.CANCELLED || state == State.FAILED
                        ? TaskOutcome.CANCELLED
                        : childOutcome;
            case NOT_STARTED:
            case STARTING:
            default:
                return TaskOutcome.NOT_DONE;
        }
    }

    /** Return the caller-supplied stable identity without sampling the factory. */
    @Override
    public String getDebugName() {
        return debugName;
    }

    /** Report cached wrapper state and nest the child only after the factory has produced it. */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "buildAtStart" : prefix;
        dbg.addData(p + ".name", debugName)
                .addData(p + ".state", state)
                .addData(p + ".startAttempted", startAttempted)
                .addData(p + ".childCreated", child != null)
                .addData(p + ".childStartAttempted", childStartAttempted)
                .addData(p + ".childCancellationAttempted", childCancellationAttempted)
                .addData(p + ".complete", isTerminalState(state));
        if (child != null) {
            child.debugDump(dbg, p + ".child");
        }
    }

    /** Consume the wrapper's one permitted start before any factory or child callback. */
    private void markStartAttempt() {
        if (startAttempted) {
            throw new IllegalStateException(
                    "Tasks.buildAtStart '" + debugName + "' is single-use and start(...) was "
                            + "called more than once. Create a fresh task with its builder or "
                            + "macro method, a Supplier<Task>, or an OutputTaskFactory.");
        }
        startAttempted = true;
    }

    /** Terminalize a failed child start, then preserve cleanup failure as suppressed. */
    private void failAfterChildStart(RuntimeException startFailure) {
        state = State.FAILED;
        bestEffortCancelChild(startFailure);
    }

    /** Attempt one child cleanup while keeping an existing lifecycle failure primary. */
    private void bestEffortCancelChild(RuntimeException primaryFailure) {
        try {
            cancelChildOnce();
        } catch (RuntimeException cleanupFailure) {
            if (cleanupFailure != primaryFailure) {
                primaryFailure.addSuppressed(cleanupFailure);
            }
        }
    }

    /** Invoke an active child's cancellation hook at most once. */
    private void cancelChildOnce() {
        if (!childStartAttempted || childCancellationAttempted || child == null) {
            return;
        }
        childCancellationAttempted = true;
        child.cancel();
    }

    /** Return whether the wrapper owns a terminal state independent of further child callbacks. */
    private static boolean isTerminalState(State state) {
        return state == State.NATURAL_COMPLETE
                || state == State.CANCELLED
                || state == State.FAILED;
    }
}
