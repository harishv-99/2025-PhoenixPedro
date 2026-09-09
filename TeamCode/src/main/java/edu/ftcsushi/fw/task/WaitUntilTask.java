package edu.ftcsushi.fw.task;

import java.util.Objects;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Condition wait created through Tasks.waitUntil, with shared lifecycle/failure handling.
 *
 * <p>The condition is sampled before the optional timeout comparison, including at the exact
 * deadline. The no-timeout factory explicitly selects an unbounded wait. Condition exceptions
 * inside Task updates are lifecycle failures, not retryable Task updates. Independent source
 * consumers retain their own source-cache contract.</p>
 */
public final class WaitUntilTask extends AbstractTask {
    private final BooleanSource condition;
    private final double timeoutSec;
    private double startSec;
    private double elapsedSec;
    private boolean lastCondition;
    private boolean timedOut;

    /** Select the explicitly unbounded factory form. */
    WaitUntilTask(BooleanSource condition) {
        this(condition, Double.POSITIVE_INFINITY, false);
    }

    /** Select a finite non-negative timeout in seconds. */
    WaitUntilTask(BooleanSource condition, double timeoutSec) {
        this(condition, timeoutSec, true);
    }

    /** Validate authored timing without sampling the borrowed condition. */
    private WaitUntilTask(BooleanSource condition, double timeoutSec, boolean bounded) {
        super("WaitUntilTask");
        this.condition = Objects.requireNonNull(condition, "condition is required");
        if (bounded && (!Double.isFinite(timeoutSec) || timeoutSec < 0.0)) {
            throw new IllegalArgumentException(
                    "timeoutSec must be finite and >= 0; use Tasks.waitUntil(condition) "
                            + "for an unbounded wait, got " + timeoutSec);
        }
        this.timeoutSec = timeoutSec;
    }

    /** Anchor this wait without charging time from before it started. */
    @Override
    protected void onStart(LoopClock clock) {
        startSec = clock.nowSec();
    }

    /** Observe once in the eligible update, allowing condition success to win the deadline tie. */
    @Override
    protected void onUpdate(LoopClock clock) {
        boolean value = condition.getAsBoolean(clock);
        if (!isActive()) {
            return;
        }
        lastCondition = value;
        if (value) {
            complete(TaskOutcome.SUCCESS);
            return;
        }
        elapsedSec = Math.max(0.0, clock.nowSec() - startSec);
        if (elapsedSec >= timeoutSec) {
            timedOut = true;
            complete(TaskOutcome.TIMEOUT);
        }
    }

    /** Waiting owns no resource or persistent request to restore. */
    @Override
    protected void onCancel() {
    }

    /** Return timeout evidence only when the Task's result is safe to consume. */
    public boolean isTimedOut() {
        requireOutcomeAvailable();
        return timedOut;
    }

    /** Expose cached observation/timing facts; borrowed diagnostics remain non-advancing. */
    @Override
    protected void debugState(DebugSink dbg, String prefix) {
        dbg.addData(prefix + ".finished", isComplete())
                .addData(prefix + ".timedOut", timedOut)
                .addData(prefix + ".condition", lastCondition)
                .addData(prefix + ".startSec", startSec)
                .addData(prefix + ".elapsedSec", elapsedSec)
                .addData(prefix + ".timeoutSec", timeoutSec);
        if (isStarted() && !hasFailure() && (!isComplete() || isEndingSettled())) {
            condition.debugDump(dbg, prefix + ".cond");
        }
    }
}
