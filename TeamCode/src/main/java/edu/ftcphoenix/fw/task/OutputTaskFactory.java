package edu.ftcphoenix.fw.task;

import java.util.function.Supplier;

/**
 * Factory for creating fresh {@link OutputTask} instances.
 *
 * <p>{@code OutputTask}s are single-use: once a task has been started, updated, cancelled, or
 * completed, it should not be put back into another queue. Use an {@code OutputTaskFactory} when a
 * queue needs to create the same kind of pulse repeatedly, such as “feed one game piece” while a
 * trigger is held.</p>
 *
 * <h2>Typical usage</h2>
 * <pre>{@code
 * OutputTaskFactory feedOne = Tasks.outputPulse("feedOne")
 *         .startWhen(canShootNow)
 *         .runOutput(0.90)
 *         .forSeconds(0.12)
 *         .cooldownSec(0.05)
 *         .build();
 *
 * feederQueue.whileHigh(clock, shootHeld, 1, feedOne);
 * }</pre>
 *
 * <p>The interface also extends {@link Supplier}, so it can be passed anywhere Phoenix accepts a
 * {@code Supplier<? extends OutputTask>}.</p>
 */
@FunctionalInterface
public interface OutputTaskFactory extends Supplier<OutputTask> {

    /**
     * Create a new, unused output task.
     *
     * @return a fresh task instance; callers should not reuse completed task objects
     */
    OutputTask create();

    /**
     * Supplier bridge for Java library APIs and existing queue helpers.
     */
    @Override
    default OutputTask get() {
        return create();
    }
}
