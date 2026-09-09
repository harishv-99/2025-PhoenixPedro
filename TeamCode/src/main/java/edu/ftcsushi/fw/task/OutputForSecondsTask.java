package edu.ftcsushi.fw.task;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Constant scalar proposal for a fixed duration, created through Tasks.outputForSeconds.
 *
 * <p>A positive interval remains observable for its start cycle; zero completes at start and is
 * never selected by the output queue. This Task does not write hardware or supply the queue's idle
 * value. getOutput remains the constant value even outside the active lifetime; OutputTaskRunner
 * selects its own idle value when no Task is active.</p>
 */
public final class OutputForSecondsTask extends AbstractTask implements OutputTask {
    private final double output;
    private final double durationSec;
    private double startSec;
    private double elapsedSec;

    /** Capture a constant proposal and validate its finite, non-negative duration in seconds. */
    OutputForSecondsTask(String name, double output, double durationSec) {
        super(name == null || name.trim().isEmpty() ? "OutputForSeconds" : name);
        if (!Double.isFinite(durationSec) || durationSec < 0.0) {
            throw new IllegalArgumentException(
                    "durationSec must be finite and >= 0, got " + durationSec);
        }
        this.output = output;
        this.durationSec = durationSec;
    }

    /** Anchor this pulse and make an empty pulse terminal without exposing a run window. */
    @Override
    protected void onStart(LoopClock clock) {
        startSec = clock.nowSec();
        if (durationSec == 0.0) {
            complete(TaskOutcome.SUCCESS);
        }
    }

    /** End after this pulse's own interval, not the loop interval preceding start. */
    @Override
    protected void onUpdate(LoopClock clock) {
        elapsedSec = Math.max(0.0, clock.nowSec() - startSec);
        if (elapsedSec >= durationSec) {
            complete(TaskOutcome.SUCCESS);
        }
    }

    /** The queue owns idle selection; this constant proposal owns no terminal output write. */
    @Override
    protected void onCancel() {
    }

    /** Return the constant proposal without advancing or querying lifecycle state. */
    @Override
    public double getOutput() {
        return output;
    }

    /** Publish cached timing/proposal facts even when lifecycle inspection would throw. */
    @Override
    protected void debugState(DebugSink dbg, String prefix) {
        dbg.addData(prefix + ".output", output)
                .addData(prefix + ".startSec", startSec)
                .addData(prefix + ".elapsedSec", elapsedSec)
                .addData(prefix + ".durationSec", durationSec);
    }
}
