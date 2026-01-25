package edu.ftcphoenix.fw.task;

import java.util.Objects;

import edu.ftcphoenix.fw.util.LoopClock;

/**
 * A {@link Task} that runs a single action once, immediately on
 * {@link #start(LoopClock)}, and then completes.
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
 *   <li>If {@link #start(LoopClock)} is called multiple times (e.g. due to user
 *       error), the action will only run on the first call.</li>
 *   <li>{@link #update(LoopClock)} does nothing; the task is considered complete
 *       as soon as the action has been run.</li>
 *   <li>{@link #getOutcome()} reports {@link TaskOutcome#NOT_DONE} while the task
 *       has not yet run, and {@link TaskOutcome#SUCCESS} once the action has run.</li>
 * </ul>
 */
public final class InstantTask implements Task {

    private final Runnable action;
    private boolean finished = false;

    /**
     * Creates an {@code InstantTask} that runs the given action once when
     * the task starts.
     *
     * @param action action to run once; must not be {@code null}
     */
    public InstantTask(Runnable action) {
        this.action = Objects.requireNonNull(action, "action must not be null");
    }

    @Override
    public void start(LoopClock clock) {
        // Idempotent start: only run the action once.
        if (finished) {
            return;
        }
        action.run();
        finished = true;
    }

    @Override
    public void update(LoopClock clock) {
        // No periodic work; instant tasks finish in start().
    }

    @Override
    public boolean isComplete() {
        return finished;
    }

    /**
     * Outcome semantics for an instant task:
     * <ul>
     *   <li>Before the action has been run: {@link TaskOutcome#NOT_DONE}.</li>
     *   <li>After the action has been run: {@link TaskOutcome#SUCCESS}.</li>
     * </ul>
     */
    @Override
    public TaskOutcome getOutcome() {
        return finished ? TaskOutcome.SUCCESS : TaskOutcome.NOT_DONE;
    }
}
