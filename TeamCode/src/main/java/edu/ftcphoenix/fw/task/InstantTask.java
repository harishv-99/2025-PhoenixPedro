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
 *   <li>If {@link #start(LoopClock)} is called multiple times due to user error, the action still
 *       only runs on the first call.</li>
 *   <li>{@link #update(LoopClock)} does nothing; the task is considered complete as soon as the
 *       action has been run.</li>
 *   <li>If {@link #cancel()} is called before the task starts, the action is skipped and the task
 *       reports {@link TaskOutcome#CANCELLED}.</li>
 *   <li>{@link #getOutcome()} reports {@link TaskOutcome#NOT_DONE} before the action has run,
 *       {@link TaskOutcome#SUCCESS} after a normal run, and
 *       {@link TaskOutcome#CANCELLED} after an early cancellation.</li>
 * </ul>
 */
public final class InstantTask implements Task {

    private final Runnable action;
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
        if (finished) {
            return;
        }
        action.run();
        finished = true;
        cancelled = false;
    }

    /** {@inheritDoc} */
    @Override
    public void update(LoopClock clock) {
        // No periodic work; instant tasks finish in start().
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void cancel() {
        if (finished) {
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
     *   <li>Before the task has run or been cancelled: {@link TaskOutcome#NOT_DONE}.</li>
     *   <li>After normal execution: {@link TaskOutcome#SUCCESS}.</li>
     *   <li>After cancellation before execution: {@link TaskOutcome#CANCELLED}.</li>
     * </ul>
     */
    @Override
    public TaskOutcome getOutcome() {
        if (!finished) {
            return TaskOutcome.NOT_DONE;
        }
        return cancelled ? TaskOutcome.CANCELLED : TaskOutcome.SUCCESS;
    }
}
