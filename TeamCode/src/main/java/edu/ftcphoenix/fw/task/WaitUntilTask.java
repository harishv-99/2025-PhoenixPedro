package edu.ftcphoenix.fw.task;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A {@link Task} that runs until a condition becomes true, optionally with a timeout.
 *
 * <p>This is the "wait until X" primitive for Phoenix tasks. It is intentionally expressed in
 * terms of {@link BooleanSource}, not a raw boolean supplier, so it composes naturally with the
 * clocked signal pipeline: debounce, hysteresis, memoization, edges, and related helpers.</p>
 *
 * <p>Typical usage:</p>
 * <pre>{@code
 * TaskRunner runner = new TaskRunner();
 *
 * // Wait until a sensor gate is ready.
 * runner.enqueue(new WaitUntilTask(ready));
 *
 * // Or: wait until ready, but give up after 2 seconds.
 * runner.enqueue(new WaitUntilTask(ready, 2.0));
 * }</pre>
 *
 * <p>Cancellation ends the wait immediately and reports {@link TaskOutcome#CANCELLED}.</p>
 *
 * <p>Timeout elapsed time is measured from the {@link LoopClock#nowSec()} captured at task start,
 * so the loop interval before scheduling is never charged to the wait. The condition is sampled
 * before the timeout comparison, including at the exact timeout boundary.</p>
 */
public final class WaitUntilTask implements Task {

    private final BooleanSource condition;
    private final double timeoutSec;

    private boolean startAttempted = false;
    private boolean finished = false;
    private boolean timedOut = false;
    private boolean cancelled = false;
    private double startSec = 0.0;
    private double elapsedSec = 0.0;
    /**
     * Last observed condition value, sampled during {@link #update(LoopClock)}.
     */
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
     * @param timeoutSec timeout in seconds; must be {@code >= 0}
     */
    public WaitUntilTask(BooleanSource condition, double timeoutSec) {
        this.condition = Objects.requireNonNull(condition, "condition is required");
        if (timeoutSec < 0.0) {
            throw new IllegalArgumentException("timeoutSec must be >= 0, got " + timeoutSec);
        }
        this.timeoutSec = timeoutSec;
    }

    /** {@inheritDoc} */
    @Override
    public void start(LoopClock clock) {
        markStartAttempt();
        finished = false;
        timedOut = false;
        cancelled = false;
        startSec = clock.nowSec();
        elapsedSec = 0.0;
        lastCondition = false;
    }

    /** {@inheritDoc} */
    @Override
    public void update(LoopClock clock) {
        if (finished) {
            return;
        }
        boolean cond = condition.getAsBoolean(clock);
        lastCondition = cond;
        if (cond) {
            finished = true;
            return;
        }
        elapsedSec = Math.max(0.0, clock.nowSec() - startSec);
        if (elapsedSec >= timeoutSec) {
            finished = true;
            timedOut = true;
        }
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
        timedOut = false;
    }

    /** {@inheritDoc} */
    @Override
    public boolean isComplete() {
        return finished;
    }

    /**
     * {@inheritDoc}
     *
     * <p>Returns {@link TaskOutcome#TIMEOUT} if the timeout elapsed before the condition became
     * true, or {@link TaskOutcome#CANCELLED} if the wait was ended early through
     * {@link #cancel()}.</p>
     */
    @Override
    public TaskOutcome getOutcome() {
        if (!finished) {
            return TaskOutcome.NOT_DONE;
        }
        if (cancelled) {
            return TaskOutcome.CANCELLED;
        }
        return timedOut ? TaskOutcome.TIMEOUT : TaskOutcome.SUCCESS;
    }

    /** {@inheritDoc} */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "waitUntil" : prefix;

        dbg.addData(p + ".finished", finished)
                .addData(p + ".timedOut", timedOut)
                .addData(p + ".cancelled", cancelled)
                .addData(p + ".condition", lastCondition)
                .addData(p + ".startSec", startSec)
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

    /** Record the single permitted start attempt before resetting task state. */
    private void markStartAttempt() {
        if (startAttempted) {
            throw new IllegalStateException(
                    "WaitUntilTask is single-use and start(...) was called more than once. "
                            + "Create a fresh task with its builder or macro method, a "
                            + "Supplier<Task>, or an OutputTaskFactory.");
        }
        startAttempted = true;
    }
}
