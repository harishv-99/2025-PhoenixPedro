package edu.ftcsushi.fw.task;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;

/** Internal fail-closed implementation for {@link Tasks#branchOnOutcome(Task, Task, Task)}. */
final class BranchOnOutcomeTask implements Task {

    private enum Phase {
        MOVE,
        BRANCH,
        DONE
    }

    private final Task move;
    private final Task onSuccess;
    private final Task onTimeout;

    private boolean startAttempted = false;
    private boolean started = false;
    private Phase phase = Phase.MOVE;
    private Task current;
    private TaskOutcome outcome = TaskOutcome.NOT_DONE;

    /** Retain three distinct branch children. Validation occurs before any child starts. */
    BranchOnOutcomeTask(Task move, Task onSuccess, Task onTimeout) {
        if (move == null) {
            throw new NullPointerException("Tasks.branchOnOutcome(...) move is required");
        }
        if (onSuccess == null) {
            throw new NullPointerException("Tasks.branchOnOutcome(...) onSuccess is required");
        }
        if (onTimeout == null) {
            throw new NullPointerException("Tasks.branchOnOutcome(...) onTimeout is required");
        }
        requireDistinctTasks(move, onSuccess, onTimeout);
        this.move = move;
        this.onSuccess = onSuccess;
        this.onTimeout = onTimeout;
        this.current = move;
    }

    /** Start only the move; branch selection occurs when its terminal outcome is observed. */
    @Override
    public void start(LoopClock clock) {
        markStartAttempt();
        started = true;
        try {
            current.start(clock);
        } catch (RuntimeException failure) {
            throw failClosed(failure);
        }
    }

    /**
     * Advance the active child once. A child already complete between cycles is observed before an
     * extra update, and a selected branch is never updated in the callback that starts it.
     */
    @Override
    public void update(LoopClock clock) {
        if (!started) {
            throw TaskLifecycle.updateBeforeStart(getDebugName());
        }
        if (phase == Phase.DONE) {
            return;
        }

        try {
            boolean childComplete = current.isComplete();
            if (phase == Phase.DONE) {
                return;
            }
            if (childComplete) {
                finishCurrentOrStartBranch(clock);
                return;
            }

            current.update(clock);
            if (phase == Phase.DONE) {
                return;
            }
            childComplete = current.isComplete();
            if (phase == Phase.DONE) {
                return;
            }
            if (childComplete) {
                finishCurrentOrStartBranch(clock);
            }
        } catch (RuntimeException failure) {
            throw failClosed(failure);
        }
    }

    /** Terminalize first, then best-effort cancel only the active child. */
    @Override
    public void cancel() {
        if (!started || phase == Phase.DONE) {
            return;
        }
        Task childToCancel = current;
        outcome = TaskOutcome.CANCELLED;
        phase = Phase.DONE;
        childToCancel.cancel();
    }

    /** {@inheritDoc} */
    @Override
    public boolean isComplete() {
        return phase == Phase.DONE;
    }

    /** Return a cached terminal outcome; active wrappers consistently report {@code NOT_DONE}. */
    @Override
    public TaskOutcome getOutcome() {
        return phase == Phase.DONE ? outcome : TaskOutcome.NOT_DONE;
    }

    /** Identify this wrapper by the public factory robot code calls. */
    @Override
    public String getDebugName() {
        return "Tasks.branchOnOutcome(...)";
    }

    /** Dump retained wrapper state without invoking child outcome callbacks. */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "branchOnOutcome" : prefix;
        dbg.addData(p + ".started", started)
                .addData(p + ".phase", phase)
                .addData(p + ".currentName", current.getDebugName())
                .addData(p + ".complete", isComplete())
                .addData(p + ".outcome", getOutcome());
    }

    /** Validate one completed child's outcome and either select a branch or finish exactly. */
    private void finishCurrentOrStartBranch(LoopClock clock) {
        TaskOutcome childOutcome = current.getOutcome();
        if (phase == Phase.DONE) {
            // A custom outcome callback may have directly cancelled this wrapper.
            return;
        }
        if (childOutcome == null || childOutcome == TaskOutcome.NOT_DONE) {
            throw malformedChildOutcome(phase, childOutcome);
        }

        if (phase == Phase.BRANCH) {
            outcome = childOutcome;
            phase = Phase.DONE;
            return;
        }

        switch (childOutcome) {
            case SUCCESS:
                startBranch(onSuccess, clock);
                return;
            case TIMEOUT:
                startBranch(onTimeout, clock);
                return;
            case CANCELLED:
            case UNKNOWN:
                outcome = childOutcome;
                phase = Phase.DONE;
                return;
            default:
                // NOT_DONE is rejected above; this keeps the enum switch fail closed if extended.
                throw malformedChildOutcome(phase, childOutcome);
        }
    }

    /** Select the branch before starting it so reentrant cancellation reaches the right child. */
    private void startBranch(Task selectedBranch, LoopClock clock) {
        current = selectedBranch;
        phase = Phase.BRANCH;
        current.start(clock);
    }

    /** Fail closed on lifecycle errors and preserve a later cancellation failure as suppressed. */
    private RuntimeException failClosed(RuntimeException failure) {
        if (phase == Phase.DONE) {
            return failure;
        }
        Task childToCancel = current;
        outcome = TaskOutcome.CANCELLED;
        phase = Phase.DONE;
        try {
            childToCancel.cancel();
        } catch (RuntimeException cleanupFailure) {
            if (cleanupFailure != failure) {
                failure.addSuppressed(cleanupFailure);
            }
        }
        return failure;
    }

    /** Reject direct aliases among the three retained branch children. */
    private static void requireDistinctTasks(Task move, Task onSuccess, Task onTimeout) {
        if (move == onSuccess) {
            throw duplicateChild("move", "onSuccess");
        }
        if (move == onTimeout) {
            throw duplicateChild("move", "onTimeout");
        }
        if (onSuccess == onTimeout) {
            throw duplicateChild("onSuccess", "onTimeout");
        }
    }

    /** Build one actionable direct-alias construction error. */
    private static IllegalArgumentException duplicateChild(String firstRole, String secondRole) {
        return new IllegalArgumentException(
                "Tasks.branchOnOutcome(...) requires distinct Task instances, but " + firstRole
                        + " and " + secondRole + " reference the same object. Create each child as "
                        + "a fresh task with its builder or macro method, a Supplier<Task>, or an "
                        + "OutputTaskFactory.");
    }

    /** Build one actionable error for a complete move or branch with a non-terminal result. */
    private static IllegalStateException malformedChildOutcome(Phase childPhase,
                                                               TaskOutcome reportedOutcome) {
        String role = childPhase == Phase.MOVE ? "move" : "selected branch";
        return new IllegalStateException(
                "Tasks.branchOnOutcome(...) " + role + " is complete but reported "
                        + reportedOutcome + ". A completed child Task must report SUCCESS, TIMEOUT, "
                        + "CANCELLED, or UNKNOWN from getOutcome(). Fix that child's lifecycle "
                        + "contract.");
    }

    /** Consume the one permitted wrapper start before invoking the move. */
    private void markStartAttempt() {
        if (startAttempted) {
            throw new IllegalStateException(
                    getDebugName() + " is single-use and start(...) was called more than once. "
                            + "Create a fresh task with its builder or macro method, a "
                            + "Supplier<Task>, or an OutputTaskFactory.");
        }
        startAttempted = true;
    }
}
