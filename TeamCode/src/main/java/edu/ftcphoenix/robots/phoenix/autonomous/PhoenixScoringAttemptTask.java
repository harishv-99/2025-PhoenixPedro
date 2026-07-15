package edu.ftcphoenix.robots.phoenix.autonomous;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.robots.phoenix.PhoenixCapabilities;

/**
 * Runs Phoenix's target-selection, aiming, and single-shot phases without hiding a failed phase.
 *
 * <p>This package-private lifecycle owner exists behind
 * {@link PhoenixAutoTasks#aimAndShootOne(PhoenixCapabilities,
 * edu.ftcphoenix.fw.drive.DriveCommandSink,
 * edu.ftcphoenix.robots.phoenix.PhoenixProfile.AutoConfig)} so ordinary robot code keeps one short
 * macro call. A phase must report {@link TaskOutcome#SUCCESS} before the dependent phase begins.
 * Timeout, cancellation, and unknown terminal outcomes remain visible to the autonomous routine.
 * If the attempt has requested a shot, every abnormal ending best-effort cancels that transient
 * request.</p>
 */
final class PhoenixScoringAttemptTask implements Task {

    private enum Phase {
        WAIT_FOR_TARGET,
        AIM,
        WAIT_FOR_SHOT,
        DONE
    }

    private final PhoenixCapabilities.Scoring scoring;
    private final Task waitForTargetTask;
    private final Task aimTask;
    private final Task waitForShotTask;

    private boolean startAttempted;
    private boolean started;
    private boolean shotRequested;
    private boolean transientCleanupAttempted;
    private boolean updateInProgress;
    private boolean advanceInProgress;
    private Phase phase = Phase.WAIT_FOR_TARGET;
    private Task activeTask;
    private TaskOutcome outcome = TaskOutcome.NOT_DONE;

    /**
     * Creates one single-use scoring attempt from three distinct, fresh phase Tasks.
     *
     * @param scoring           capability owner used for velocity capture and transient-shot intent
     * @param waitForTargetTask wait that succeeds only after a scoring target is selected
     * @param aimTask           Phoenix targeting Task for the selected target
     * @param waitForShotTask   wait that succeeds after the requested shot drains
     * @throws NullPointerException     if any required role is absent
     * @throws IllegalArgumentException if the same Task instance is assigned to two phase roles
     */
    PhoenixScoringAttemptTask(PhoenixCapabilities.Scoring scoring,
                              Task waitForTargetTask,
                              Task aimTask,
                              Task waitForShotTask) {
        this.scoring = Objects.requireNonNull(
                scoring,
                "PhoenixScoringAttemptTask scoring capability is required"
        );
        this.waitForTargetTask = requireTask(waitForTargetTask, "waitForTargetTask");
        this.aimTask = requireTask(aimTask, "aimTask");
        this.waitForShotTask = requireTask(waitForShotTask, "waitForShotTask");
        requireDistinctTasks();
    }

    /** {@inheritDoc} */
    @Override
    public void start(LoopClock clock) {
        markStartAttempt();
        Objects.requireNonNull(clock, "PhoenixScoringAttemptTask start clock is required");
        started = true;
        phase = Phase.WAIT_FOR_TARGET;
        activeTask = waitForTargetTask;
        outcome = TaskOutcome.NOT_DONE;
        activeTask.start(clock);
        if (phase != Phase.DONE) {
            advanceCompletedPhases(clock);
        }
    }

    /** {@inheritDoc} */
    @Override
    public void update(LoopClock clock) {
        if (!started) {
            throw new IllegalStateException(
                    "PhoenixScoringAttemptTask.update(clock) was called before start(clock). "
                            + "Start the task once before updating it, normally by enqueuing the "
                            + "result of PhoenixAutoTasks.aimAndShootOne(...) in a TaskRunner."
            );
        }
        if (phase == Phase.DONE) {
            return;
        }
        Objects.requireNonNull(clock, "PhoenixScoringAttemptTask update clock is required");
        if (updateInProgress || advanceInProgress) {
            return;
        }

        updateInProgress = true;
        try {
            advanceCompletedPhases(clock);
            if (phase == Phase.DONE) {
                return;
            }

            Task task = activeTask;
            if (task == null) {
                throw malformedState("active phase " + phase + " has no active Task");
            }
            task.update(clock);
            if (phase == Phase.DONE || activeTask != task) {
                return;
            }
            advanceCompletedPhases(clock);
        } finally {
            updateInProgress = false;
        }
    }

    /** Advance through phase Tasks that completed during start or the current update. */
    private void advanceCompletedPhases(LoopClock clock) {
        if (advanceInProgress) {
            return;
        }
        advanceInProgress = true;
        try {
            while (phase != Phase.DONE) {
                Task task = activeTask;
                if (task == null) {
                    throw malformedState("active phase " + phase + " has no active Task");
                }

                boolean complete = task.isComplete();
                if (phase == Phase.DONE || activeTask != task) {
                    return;
                }
                if (!complete) {
                    return;
                }

                TaskOutcome phaseOutcome = terminalOutcome(task, phase);
                if (phase == Phase.DONE || activeTask != task) {
                    return;
                }
                if (phaseOutcome != TaskOutcome.SUCCESS) {
                    finish(phaseOutcome);
                    cleanupTransientShotIfOwned();
                    return;
                }

                switch (phase) {
                    case WAIT_FOR_TARGET:
                        scoring.captureSuggestedShotVelocity();
                        if (phase == Phase.WAIT_FOR_TARGET && activeTask == task) {
                            startPhase(Phase.AIM, aimTask, clock);
                        }
                        break;

                    case AIM:
                        // Mark ownership before the call so fail-stop cleanup also covers a
                        // partially completed capability request that throws.
                        shotRequested = true;
                        scoring.requestSingleShot();
                        if (phase == Phase.AIM && activeTask == task) {
                            startPhase(Phase.WAIT_FOR_SHOT, waitForShotTask, clock);
                        }
                        break;

                    case WAIT_FOR_SHOT:
                        // The wait's success means the owned request has drained naturally.
                        shotRequested = false;
                        finish(TaskOutcome.SUCCESS);
                        break;

                    case DONE:
                    default:
                        return;
                }
            }
        } finally {
            advanceInProgress = false;
        }
    }

    /**
     * {@inheritDoc}
     *
     * <p>Active cancellation terminalizes first, then best-effort cancels the current phase and
     * any transient shot this attempt requested. Pre-start, terminal, and repeated cancellation
     * are side-effect-free no-ops.</p>
     */
    @Override
    public void cancel() {
        if (!started || phase == Phase.DONE) {
            return;
        }

        Task taskToCancel = activeTask;
        finish(TaskOutcome.CANCELLED);

        RuntimeException firstFailure = null;
        try {
            if (taskToCancel != null) {
                taskToCancel.cancel();
            }
        } catch (RuntimeException ex) {
            firstFailure = ex;
        }

        try {
            cleanupTransientShotIfOwned();
        } catch (RuntimeException ex) {
            if (firstFailure == null) {
                firstFailure = ex;
            } else if (ex != firstFailure) {
                firstFailure.addSuppressed(ex);
            }
        }

        if (firstFailure != null) {
            throw firstFailure;
        }
    }

    /** {@inheritDoc} */
    @Override
    public boolean isComplete() {
        return phase == Phase.DONE;
    }

    /** {@inheritDoc} */
    @Override
    public TaskOutcome getOutcome() {
        return phase == Phase.DONE ? outcome : TaskOutcome.NOT_DONE;
    }

    /** {@inheritDoc} */
    @Override
    public String getDebugName() {
        if (phase == Phase.DONE) {
            return "PhoenixScoringAttempt(DONE:" + outcome + ")";
        }
        return "PhoenixScoringAttempt(" + phase + ")";
    }

    /** {@inheritDoc} */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "phoenixScoringAttempt" : prefix;
        dbg.addData(p + ".phase", phase)
                .addData(p + ".outcome", getOutcome())
                .addData(p + ".shotRequested", shotRequested)
                .addData(p + ".transientCleanupAttempted", transientCleanupAttempted)
                .addData(p + ".activeTask", activeTask != null ? activeTask.getDebugName() : "none");
        if (activeTask != null) {
            activeTask.debugDump(dbg, p + ".child");
        }
    }

    /** Start the next phase after first publishing it as the active cancellation owner. */
    private void startPhase(Phase nextPhase, Task nextTask, LoopClock clock) {
        phase = nextPhase;
        activeTask = nextTask;
        activeTask.start(clock);
    }

    /** Record a terminal outcome before performing any cleanup that could fail or reenter. */
    private void finish(TaskOutcome terminalOutcome) {
        outcome = terminalOutcome;
        phase = Phase.DONE;
    }

    /** Cancel this attempt's transient shot at most once after it has been requested. */
    private void cleanupTransientShotIfOwned() {
        if (!shotRequested || transientCleanupAttempted) {
            return;
        }
        transientCleanupAttempted = true;
        shotRequested = false;
        scoring.cancelTransientActions();
    }

    /** Require a completed phase Task to expose a usable terminal outcome. */
    private TaskOutcome terminalOutcome(Task task, Phase completedPhase) {
        TaskOutcome childOutcome = task.getOutcome();
        if (childOutcome == null || childOutcome == TaskOutcome.NOT_DONE) {
            throw new IllegalStateException(
                    "PhoenixScoringAttemptTask phase " + completedPhase + " completed, but "
                            + task.getDebugName() + " reported " + childOutcome + ". A completed "
                            + "phase must report SUCCESS, TIMEOUT, CANCELLED, or UNKNOWN."
            );
        }
        return childOutcome;
    }

    /** Build one actionable error for an impossible internal lifecycle state. */
    private IllegalStateException malformedState(String detail) {
        return new IllegalStateException(
                "PhoenixScoringAttemptTask has malformed lifecycle state: " + detail
                        + ". Create a fresh task with PhoenixAutoTasks.aimAndShootOne(...)."
        );
    }

    /** Consume the one permitted start attempt before starting a child or changing scoring intent. */
    private void markStartAttempt() {
        if (startAttempted) {
            throw new IllegalStateException(
                    "PhoenixScoringAttemptTask is single-use and start(...) was called more than "
                            + "once. Create a fresh task with "
                            + "PhoenixAutoTasks.aimAndShootOne(...)."
            );
        }
        startAttempted = true;
    }

    /** Validate one required phase Task with its role named in any construction error. */
    private static Task requireTask(Task task, String role) {
        return Objects.requireNonNull(
                task,
                "PhoenixScoringAttemptTask " + role + " is required"
        );
    }

    /** Reject obvious aliases before any child can be started. */
    private void requireDistinctTasks() {
        requireDistinct(waitForTargetTask, "waitForTargetTask", aimTask, "aimTask");
        requireDistinct(waitForTargetTask, "waitForTargetTask", waitForShotTask, "waitForShotTask");
        requireDistinct(aimTask, "aimTask", waitForShotTask, "waitForShotTask");
    }

    /** Reject one direct child alias with fresh-graph guidance. */
    private static void requireDistinct(Task first,
                                        String firstRole,
                                        Task second,
                                        String secondRole) {
        if (first == second) {
            throw new IllegalArgumentException(
                    "PhoenixScoringAttemptTask requires distinct Task instances, but "
                            + firstRole + " and " + secondRole + " reference the same object. "
                            + "Create every phase from a fresh Task factory call."
            );
        }
    }
}
