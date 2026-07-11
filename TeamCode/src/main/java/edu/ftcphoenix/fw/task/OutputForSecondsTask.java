package edu.ftcphoenix.fw.task;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * An {@link OutputTask} that outputs a constant value for a fixed duration.
 *
 * <p>This is the output-producing sibling of {@link RunForSecondsTask}. It is useful for short
 * pulses such as "run feeder for 120ms" and for sensorless fallback behavior when no real
 * completion sensor exists.</p>
 *
 * <p>Active cancellation ends the pulse immediately and reports
 * {@link TaskOutcome#CANCELLED}; pre-start and terminal cancellation are no-ops.</p>
 *
 * <p>Elapsed time is measured from the {@link LoopClock#nowSec()} captured at start. Consequently,
 * a positive duration remains active and exposes {@link #getOutput()} for at least its start cycle,
 * even when the preceding loop delta is larger than the duration. A zero duration completes at
 * start and is never selected by {@link OutputTaskRunner}.</p>
 */
public final class OutputForSecondsTask implements OutputTask {

    private final String name;
    private final double output;
    private final double durationSec;

    private boolean startAttempted = false;
    private boolean started = false;
    private boolean finished = false;
    private boolean cancelled = false;
    private double startSec = 0.0;
    private double elapsedSec = 0.0;

    /**
     * Create a constant-output timed task.
     *
     * @param name        debug label
     * @param output      output value while the task runs
     * @param durationSec duration in seconds; must be {@code >= 0}
     */
    public OutputForSecondsTask(String name, double output, double durationSec) {
        if (durationSec < 0.0) {
            throw new IllegalArgumentException("durationSec must be >= 0, got " + durationSec);
        }
        this.name = (name == null || name.isEmpty()) ? "OutputForSeconds" : name;
        this.output = output;
        this.durationSec = durationSec;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void start(LoopClock clock) {
        markStartAttempt();
        started = true;
        finished = (durationSec == 0.0);
        cancelled = false;
        startSec = clock.nowSec();
        elapsedSec = 0.0;
    }

    /** {@inheritDoc} */
    @Override
    public void update(LoopClock clock) {
        if (!started) {
            throw TaskLifecycle.updateBeforeStart(name);
        }
        if (finished) {
            return;
        }
        elapsedSec = Math.max(0.0, clock.nowSec() - startSec);
        if (elapsedSec >= durationSec) {
            finished = true;
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void cancel() {
        if (!started || finished) {
            return;
        }
        cancelled = true;
        finished = true;
    }

    /** {@inheritDoc} */
    @Override
    public boolean isComplete() {
        return finished;
    }

    /**
     * Outcome semantics for a constant-output timed task:
     * <ul>
     *   <li>While the task is active: {@link TaskOutcome#NOT_DONE}.</li>
     *   <li>After the full duration elapses: {@link TaskOutcome#SUCCESS}.</li>
     *   <li>After early cancellation: {@link TaskOutcome#CANCELLED}.</li>
     * </ul>
     */
    @Override
    public TaskOutcome getOutcome() {
        if (!finished) {
            return TaskOutcome.NOT_DONE;
        }
        return cancelled ? TaskOutcome.CANCELLED : TaskOutcome.SUCCESS;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String getDebugName() {
        return name;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double getOutput() {
        return output;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "outputForSeconds" : prefix;

        dbg.addData(p + ".name", name)
                .addData(p + ".output", output)
                .addData(p + ".startSec", startSec)
                .addData(p + ".elapsedSec", elapsedSec)
                .addData(p + ".durationSec", durationSec)
                .addData(p + ".cancelled", cancelled)
                .addData(p + ".complete", finished)
                .addData(p + ".outcome", getOutcome());
    }

    /** Record the single permitted start attempt before resetting task state. */
    private void markStartAttempt() {
        if (startAttempted) {
            throw new IllegalStateException(
                    name + " is a single-use OutputForSecondsTask and start(...) was called "
                            + "more than once. Create a fresh task with its builder or macro "
                            + "method, a Supplier<Task>, or an OutputTaskFactory.");
        }
        startAttempted = true;
    }
}
