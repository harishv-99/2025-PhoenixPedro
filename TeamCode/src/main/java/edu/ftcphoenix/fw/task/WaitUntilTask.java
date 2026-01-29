package edu.ftcphoenix.fw.task;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A {@link Task} that runs until a condition becomes true, optionally with a timeout.
 *
 * <p>This is the "wait until X" primitive for Phoenix tasks. It is intentionally expressed in terms
 * of {@link BooleanSource} (not a raw {@code BooleanSupplier}) so it composes naturally with
 * Phoenix's clocked signal pipeline (debounce, hysteresis, memoization, edges, etc.).</p>
 *
 * <p>Typical usage:
 * <pre>{@code
 * TaskRunner runner = new TaskRunner();
 *
 * // Wait until a sensor gate is ready.
 * BooleanSource ready = BooleanSource.of(() -> sensorReady());
 * runner.enqueue(new WaitUntilTask(ready));
 *
 * // Or: wait until ready, but give up after 2 seconds.
 * runner.enqueue(new WaitUntilTask(ready, 2.0));
 * }</pre>
 */
public final class WaitUntilTask implements Task {

    private final BooleanSource condition;
    private final double timeoutSec;

    private boolean finished = false;
    private boolean timedOut = false;
    private double elapsedSec = 0.0;

    // Debug helper: the last observed condition value (sampled during update()).
    private boolean lastCondition = false;

    /**
     * Create a wait-until task with no timeout.
     */
    public WaitUntilTask(BooleanSource condition) {
        this(condition, Double.POSITIVE_INFINITY);
    }

    /**
     * Create a wait-until task with a timeout.
     *
     * @param condition  condition to wait for; task completes when this becomes true
     * @param timeoutSec timeout in seconds; must be &gt;= 0.0
     */
    public WaitUntilTask(BooleanSource condition, double timeoutSec) {
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
        lastCondition = false;
    }

    @Override
    public void update(LoopClock clock) {
        if (finished) {
            return;
        }

        // First check condition
        boolean cond = condition.getAsBoolean(clock);
        lastCondition = cond;
        if (cond) {
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

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "waitUntil" : prefix;

        dbg.addData(p + ".finished", finished)
                .addData(p + ".timedOut", timedOut)
                .addData(p + ".condition", lastCondition)
                .addData(p + ".elapsedSec", elapsedSec)
                .addData(p + ".timeoutSec", timeoutSec);

        condition.debugDump(dbg, p + ".cond");
    }

    /**
     * @return true if the task completed due to timeout rather than the condition.
     */
    public boolean isTimedOut() {
        return timedOut;
    }
}
