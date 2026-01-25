package edu.ftcphoenix.fw.task;

/**
 * Describes the completion status of a {@link Task}.
 *
 * <p>All tasks support outcomes via {@link Task#getOutcome()}, but many
 * simple tasks will just use the default {@link #UNKNOWN} value and never
 * override it. Tasks that care about distinguishing different terminal
 * states (for example, success vs timeout) should override
 * {@code getOutcome()} and return one of the more specific values.</p>
 */
public enum TaskOutcome {

    /**
     * This task does not expose a meaningful outcome, or the outcome is not
     * being tracked. This is the default for generic tasks that don't care
     * about distinguishing success vs timeout.
     */
    UNKNOWN,

    /**
     * The task has not yet completed.
     *
     * <p>Tasks that track outcomes may return this while
     * {@link Task#isComplete()} is still false.</p>
     */
    NOT_DONE,

    /**
     * The task completed normally.
     */
    SUCCESS,

    /**
     * The task terminated because a timeout elapsed (or some other
     * time-based abort condition occurred).
     */
    TIMEOUT
}
