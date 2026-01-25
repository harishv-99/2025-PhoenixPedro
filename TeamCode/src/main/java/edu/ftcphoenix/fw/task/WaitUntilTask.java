package edu.ftcphoenix.fw.task;

import java.util.Objects;
import java.util.function.BooleanSupplier;

import edu.ftcphoenix.fw.util.LoopClock;

/**
 * A {@link Task} that runs until a condition becomes true, optionally
 * with an optional timeout.
 *
 * <p>Typical usage:
 * <pre>{@code
 * TaskRunner runner = new TaskRunner();
 *
 * // Wait until a sensor reports ready
 * runner.enqueue(new WaitUntilTask(() -> sensorReady()));
 *
 * // Or: wait until ready, but give up after 2 seconds
 * runner.enqueue(new WaitUntilTask(() -> sensorReady(), 2.0));
 * }</pre>
 *
 * <p>Behavior:</p>
 * <ul>
 *   <li>On {@link #start(LoopClock)}, the internal timer and flags are reset.</li>
 *   <li>On each {@link #update(LoopClock)}, the condition is checked first:
 *     <ul>
 *       <li>If it returns {@code true}, the task completes successfully.</li>
 *       <li>Otherwise, if a finite timeout is configured and elapsed time
 *           reaches the timeout, the task completes with {@link #isTimedOut()}
 *           returning {@code true} and {@link #getOutcome()} returning
 *           {@link TaskOutcome#TIMEOUT}.</li>
 *     </ul>
 *   </li>
 *   <li>{@link #isComplete()} returns {@code true} once either the condition
 *       is satisfied or a timeout occurs.</li>
 * </ul>
 */
public final class WaitUntilTask implements Task {

    private final BooleanSupplier condition;
    private final double timeoutSec;

    private boolean finished = false;
    private boolean timedOut = false;
    private double elapsedSec = 0.0;

    /**
     * Create a wait-until task with no timeout.
     *
     * @param condition condition to wait for; task completes when this returns true
     */
    public WaitUntilTask(BooleanSupplier condition) {
        this(condition, Double.POSITIVE_INFINITY);
    }

    /**
     * Create a wait-until task with a timeout.
     *
     * @param condition  condition to wait for; task completes when this returns true,
     *                   or when {@code timeoutSec} elapses
     * @param timeoutSec timeout in seconds; must be &gt;= 0.0
     */
    public WaitUntilTask(BooleanSupplier condition, double timeoutSec) {
        this.condition = Objects.requireNonNull(condition, "condition is required");
        if (timeoutSec < 0.0) {
            throw new IllegalArgumentException("timeoutSec must be >= 0, got " + timeoutSec);
        }
        this.timeoutSec = timeoutSec;
    }

    @Override
    public void start(LoopClock clock) {
        finished = false;
        timedOut = false;
        elapsedSec = 0.0;
    }

    @Override
    public void update(LoopClock clock) {
        if (finished) {
            return;
        }

        // First check condition
        if (condition.getAsBoolean()) {
            finished = true;
            return;
        }

        // Then update elapsed time and check timeout (if finite)
        elapsedSec += clock.dtSec();

        if (elapsedSec >= timeoutSec) {
            finished = true;
            timedOut = true;
        }
    }

    @Override
    public boolean isComplete() {
        return finished;
    }

    @Override
    public TaskOutcome getOutcome() {
        if (!finished) {
            return TaskOutcome.NOT_DONE;
        }
        return timedOut ? TaskOutcome.TIMEOUT : TaskOutcome.SUCCESS;
    }

    /**
     * @return true if the task completed due to timeout rather than the condition.
     */
    public boolean isTimedOut() {
        return timedOut;
    }
}
