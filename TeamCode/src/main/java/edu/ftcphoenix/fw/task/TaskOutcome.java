package edu.ftcphoenix.fw.task;

/**
 * Describes the completion status of a {@link Task}.
 *
 * <p>Every task implementation supplies {@link Task#getOutcome()}. A simple task that does not
 * distinguish terminal states may return {@link #UNKNOWN}; a task that tracks success, timeout,
 * cancellation, or another meaningful state should return the corresponding specific value.</p>
 *
 * <p>Most tasks report {@link #NOT_DONE} while running and switch to one of the terminal values
 * once they finish.</p>
 */
public enum TaskOutcome {

    /**
     * This task does not expose a meaningful outcome, or the outcome is not being tracked. This is
     * appropriate for generic tasks that do not distinguish success from other terminal states.
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
     * The task ended through cancellation or another task-specific fail-closed abort.
     *
     * <p>Consult a task's more-specific status API, when it has one, to distinguish direct
     * {@link Task#cancel()} from other abnormal terminal reasons.</p>
     */
    CANCELLED
}
