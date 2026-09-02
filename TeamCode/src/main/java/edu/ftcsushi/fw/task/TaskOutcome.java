package edu.ftcsushi.fw.task;

/**
 * Describes the completion status of a {@link Task}.
 *
 * <p>Every task implementation supplies {@link Task#getOutcome()}. A simple task that cannot
 * provide more specific terminal evidence may return {@link #UNKNOWN}; composition never treats
 * that uncertainty as evidence of {@link #SUCCESS}.</p>
 *
 * <p>Most tasks report {@link #NOT_DONE} while running and switch to one of the terminal values
 * once they finish.</p>
 */
public enum TaskOutcome {

    /**
     * This task completed, but does not expose a more specific terminal result. This is a valid
     * terminal outcome, but it is not evidence that a dependent step is safe to start.
     */
    UNKNOWN,

    /**
     * The task has not yet completed.
     *
     * <p>Tasks return this while {@link Task#isComplete()} is false. A task that reports complete
     * must instead return one of the other four terminal values; composition rejects a completed
     * child that still reports {@code NOT_DONE}.</p>
     */
    NOT_DONE,

    /**
     * The task satisfied its documented success condition. This is the only outcome that ordinary
     * dependent {@link Tasks#sequence(Task...)} composition accepts before starting its next step.
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
