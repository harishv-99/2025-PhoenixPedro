package edu.ftcsushi.fw.task;

import java.util.function.Consumer;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Advanced fixed-duration callback Task, with the shared guarded Task lifecycle.
 *
 * <p>Ordinary mechanisms use ScalarTasks or SemanticScalarTasks. This constructor is the advanced
 * seam when an operation genuinely needs its own per-cycle callback. Callbacks run synchronously
 * on the owning loop, never on a worker thread. They must be short and stop producing effects after
 * requesting cancellation; no wrapper can undo arbitrary code that resumes after cancelling itself.</p>
 *
 * <p>Start captures this Task's own nowSec boundary before onStart. Zero duration runs onStart and
 * onFinish without an update. Positive duration allows the first update in the start cycle, and
 * invokes onUpdate before testing expiry, including the expiry cycle. Further updates in that same
 * cycle are inert. The preceding loop's dtSec is never charged to the new interval.</p>
 *
 * <p>The optional onFinish action runs once on normal completion, active cancellation, or an armed
 * lifecycle RuntimeException. It is not success-only work. A failed callback or ending remains an
 * exception through later update/outcome inspection, while cached diagnostics remain available.</p>
 */
public final class RunForSecondsTask extends AbstractTask {
    private final double durationSec;
    private final Runnable startAction;
    private final Consumer<LoopClock> updateAction;
    private final Runnable finishAction;
    private double startSec;
    private double elapsedSec;

    /**
     * Capture optional callbacks without executing them.
     *
     * @param durationSec finite duration in seconds, greater than or equal to zero
     * @param onStart optional action at start, or null
     * @param onUpdate optional action per eligible update, or null; its clock is unchanged, so
     *                 callback-owned timers compare nowSec rather than consuming the prior dtSec
     * @param onFinish optional short synchronous all-ending action, or null
     * @throws IllegalArgumentException if duration is negative or non-finite
     */
    public RunForSecondsTask(double durationSec, Runnable onStart,
                             Consumer<LoopClock> onUpdate, Runnable onFinish) {
        super("RunForSecondsTask");
        if (!Double.isFinite(durationSec) || durationSec < 0.0) {
            throw new IllegalArgumentException(
                    "durationSec must be finite and >= 0, got " + durationSec);
        }
        this.durationSec = durationSec;
        this.startAction = onStart;
        this.updateAction = onUpdate;
        this.finishAction = onFinish;
    }

    /** Anchor the interval before callbacks, preserving the zero-duration start/finish order. */
    @Override
    protected void onStart(LoopClock clock) {
        startSec = clock.nowSec();
        elapsedSec = 0.0;
        if (startAction != null) {
            startAction.run();
        }
        if (isActive() && durationSec == 0.0) {
            complete(TaskOutcome.SUCCESS);
        }
    }

    /** Preserve the callback-before-expiry order without continuing after callback cancellation. */
    @Override
    protected void onUpdate(LoopClock clock) {
        if (updateAction != null) {
            updateAction.accept(clock);
        }
        if (!isActive()) {
            return;
        }
        elapsedSec = Math.max(0.0, clock.nowSec() - startSec);
        if (elapsedSec >= durationSec) {
            complete(TaskOutcome.SUCCESS);
        }
    }

    /** The timed callback's configured ending action owns restoration, not a separate abort write. */
    @Override
    protected void onCancel() {
    }

    /** Invoke the optional ending action through the common exactly-once/failure boundary. */
    @Override
    protected void onFinish() {
        if (finishAction != null) {
            finishAction.run();
        }
    }

    /** Return cached remaining seconds; failed or pending endings cannot look like elapsed time. */
    public double getRemainingSec() {
        requireOutcomeAvailable();
        return cachedRemainingSec();
    }

    /** Keep diagnostic timing readable without consuming lifecycle result evidence. */
    private double cachedRemainingSec() {
        return !isStarted() || isComplete() ? 0.0 : Math.max(durationSec - elapsedSec, 0.0);
    }

    /** Expose cached timing only; diagnostics never invoke a callback or inspect a failed outcome. */
    @Override
    protected void debugState(DebugSink dbg, String prefix) {
        dbg.addData(prefix + ".durationSec", durationSec)
                .addData(prefix + ".finished", isComplete())
                .addData(prefix + ".startSec", startSec)
                .addData(prefix + ".elapsedSec", elapsedSec)
                .addData(prefix + ".remainingSec", cachedRemainingSec())
                .addData(prefix + ".hasOnStart", startAction != null)
                .addData(prefix + ".hasOnUpdate", updateAction != null)
                .addData(prefix + ".hasOnFinish", finishAction != null);
    }
}
