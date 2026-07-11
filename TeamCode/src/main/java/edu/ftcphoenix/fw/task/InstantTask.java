package edu.ftcphoenix.fw.task;

import java.util.Objects;

import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A {@link Task} that runs a single action once, immediately on {@link #start(LoopClock)}, and
 * then completes.
 *
 * <p>Typical usage:</p>
 * <pre>{@code
 * TaskRunner runner = new TaskRunner();
 * runner.enqueue(new InstantTask(() -> telemetry.addLine("Auto start")));
 * }</pre>
 *
 * <p>Semantics:</p>
 * <ul>
 *   <li>The provided {@link Runnable} is guaranteed to run at most once.</li>
 *   <li>A second {@link #start(LoopClock)} call is rejected instead of being silently ignored.</li>
 *   <li>After start, {@link #update(LoopClock)} does nothing; a direct pre-start update is a
 *       lifecycle error. The task is considered complete as soon as the action has been run.</li>
 *   <li>Cancellation before start and after completion is a no-op. The action therefore still runs
 *       after a pre-start cancellation request.</li>
 *   <li>{@link #getOutcome()} reports {@link TaskOutcome#NOT_DONE} before the action has run,
 *       {@link TaskOutcome#SUCCESS} after a normal run, and
 *       {@link TaskOutcome#CANCELLED} after an early cancellation.</li>
 * </ul>
 */
public final class InstantTask implements Task {

    private final Runnable action;
    private boolean startAttempted = false;
    private boolean started = false;
    private boolean finished = false;
    private boolean cancelled = false;

    /**
     * Creates an {@code InstantTask} that runs the given action once when the task starts.
     *
     * @param action action to run once; must not be {@code null}
     */
    public InstantTask(Runnable action) {
        this.action = Objects.requireNonNull(action, "action must not be null");
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void start(LoopClock clock) {
        markStartAttempt();
        started = true;
        finished = false;
        cancelled = false;
        action.run();
        // A callback may have reentrantly cancelled this Task through its owning runner.
        finished = true;
    }

    /** {@inheritDoc} */
    @Override
    public void update(LoopClock clock) {
        if (!started) {
            throw TaskLifecycle.updateBeforeStart("InstantTask");
        }
        // No periodic work; instant tasks finish in start().
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void cancel() {
        if (!started || finished) {
            return;
        }
        finished = true;
        cancelled = true;
    }

    /** {@inheritDoc} */
    @Override
    public boolean isComplete() {
        return finished;
    }

    /**
     * Outcome semantics for an instant task:
     * <ul>
     *   <li>Before the task has run: {@link TaskOutcome#NOT_DONE}.</li>
     *   <li>After normal execution: {@link TaskOutcome#SUCCESS}.</li>
     *   <li>After cancellation of a start attempt whose action failed:
     *       {@link TaskOutcome#CANCELLED}.</li>
     * </ul>
     */
    @Override
    public TaskOutcome getOutcome() {
        if (!finished) {
            return TaskOutcome.NOT_DONE;
        }
        return cancelled ? TaskOutcome.CANCELLED : TaskOutcome.SUCCESS;
    }

    /** Record the single permitted start attempt before running the user action. */
    private void markStartAttempt() {
        if (startAttempted) {
            throw new IllegalStateException(
                    "InstantTask is single-use and start(...) was called more than once. "
                            + "Create a fresh task with its builder or macro method, a "
                            + "Supplier<Task>, or an OutputTaskFactory.");
        }
        startAttempted = true;
    }
}
