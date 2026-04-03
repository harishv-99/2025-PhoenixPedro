package edu.ftcphoenix.fw.task;

/**
 * Describes the completion status of a {@link Task}.
 *
 * <p>All tasks support outcomes via {@link Task#getOutcome()}, but many simple tasks will just use
 * the default {@link #UNKNOWN} value and never override it. Tasks that care about distinguishing
 * different terminal states, for example success vs timeout vs cancellation, should override
 * {@code getOutcome()} and return one of the more specific values.</p>
 *
 * <p>Most tasks report {@link #NOT_DONE} while running and switch to one of the terminal values
 * once they finish.</p>
 */
public enum TaskOutcome {

    /**
     * This task does not expose a meaningful outcome, or the outcome is not being tracked. This is
     * the default for generic tasks that do not care about distinguishing success vs timeout.
     */
    UNKNOWN,

    /**
     * The task has not yet completed.
     *
     * <p>Tasks that track outcomes may return this while {@link Task#isComplete()} is still
     * false.</p>
     */
    NOT_DONE,

    /**
     * The task completed normally.
     */
    SUCCESS,

    /**
     * The task terminated because a timeout elapsed or some other time-based abort condition
     * occurred.
     */
    TIMEOUT,

    /**
     * The task was stopped early through {@link Task#cancel()} or by a runner-level cancellation
     * helper such as {@link TaskRunner#clearAndCancel()}.
     */
    CANCELLED
}
