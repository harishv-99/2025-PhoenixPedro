package edu.ftcphoenix.fw.task;

import edu.ftcphoenix.fw.util.LoopClock;

/**
 * A cooperative, stateful unit of work that is driven by the main robot loop.
 *
 * <p>Typical lifecycle:</p>
 * <ol>
 *   <li>{@link #start(LoopClock)} is called once when the task is first
 *       scheduled.</li>
 *   <li>{@link #update(LoopClock)} is called every loop while
 *       {@link #isComplete()} returns {@code false}.</li>
 *   <li>Once {@link #isComplete()} returns {@code true}, the task is
 *       considered finished and will no longer receive updates.</li>
 * </ol>
 *
 * <p>Tasks are intended to be used with a runner such as
 * {@code TaskRunner}, which will manage calling {@code start()},
 * {@code update()} and checking {@code isComplete()} each iteration.</p>
 */
public interface Task {

    /**
     * Called once when the task is first started.
     *
     * <p>Implementations should perform any initialization here, including
     * capturing the initial time from {@link LoopClock} if needed.</p>
     *
     * @param clock loop timing information for the current iteration
     */
    void start(LoopClock clock);

    /**
     * Called once per loop while the task is running.
     *
     * <p>Implementations should advance their internal state based on the
     * information in {@link LoopClock}, and may mark themselves complete
     * by causing {@link #isComplete()} to return {@code true}.</p>
     *
     * @param clock loop timing information for the current iteration
     */
    void update(LoopClock clock);

    /**
     * @return {@code true} once the task has finished and no longer needs to
     * receive {@link #update(LoopClock)} calls.
     */
    boolean isComplete();

    /**
     * @return a short human-readable label for debugging. The default
     * implementation returns the simple class name.
     */
    default String getDebugName() {
        return getClass().getSimpleName();
    }

    /**
     * Returns the outcome of this task, if it exposes one.
     *
     * <p>The default implementation returns {@link TaskOutcome#UNKNOWN},
     * which is appropriate for simple tasks that do not distinguish between
     * different terminal states.</p>
     *
     * <p>Tasks that care about outcomes (for example, that may finish with
     * success vs timeout) should override this method and follow this
     * convention:</p>
     * <ul>
     *   <li>While the task is still running (before {@link #isComplete()}
     *       becomes {@code true}), return {@link TaskOutcome#NOT_DONE}.</li>
     *   <li>Once the task has completed, return a terminal value such as
     *       {@link TaskOutcome#SUCCESS} or {@link TaskOutcome#TIMEOUT}.</li>
     * </ul>
     *
     * @return the current outcome for this task
     */
    TaskOutcome getOutcome();
}
