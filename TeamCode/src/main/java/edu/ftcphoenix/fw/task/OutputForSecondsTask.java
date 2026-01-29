package edu.ftcphoenix.fw.task;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * An {@link OutputTask} that outputs a constant value for a fixed duration.
 *
 * <p>This is the output-producing sibling of {@link RunForSecondsTask}. It is useful for
 * short pulses ("run feeder for 120ms") and for sensorless fallback behavior.
 */
public final class OutputForSecondsTask implements OutputTask {

    private final String name;
    private final double output;
    private final double durationSec;

    private boolean finished = false;
    private double elapsedSec = 0.0;

    /**
     * @param name        debug label
     * @param output      output value while the task runs
     * @param durationSec duration in seconds; must be &gt;= 0
     */
    public OutputForSecondsTask(String name, double output, double durationSec) {
        if (durationSec < 0.0) {
            throw new IllegalArgumentException("durationSec must be >= 0, got " + durationSec);
        }
        this.name = (name == null || name.isEmpty()) ? "OutputForSeconds" : name;
        this.output = output;
        this.durationSec = durationSec;
    }

    @Override
    public void start(LoopClock clock) {
        finished = (durationSec == 0.0);
        elapsedSec = 0.0;
    }

    @Override
    public void update(LoopClock clock) {
        if (finished) {
            return;
        }
        elapsedSec += clock.dtSec();
        if (elapsedSec >= durationSec) {
            finished = true;
        }
    }

    @Override
    public boolean isComplete() {
        return finished;
    }

    @Override
    public TaskOutcome getOutcome() {
        return finished ? TaskOutcome.SUCCESS : TaskOutcome.NOT_DONE;
    }

    @Override
    public String getDebugName() {
        return name;
    }

    @Override
    public double getOutput() {
        return output;
    }

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "outputForSeconds" : prefix;

        dbg.addData(p + ".name", name)
                .addData(p + ".output", output)
                .addData(p + ".elapsedSec", elapsedSec)
                .addData(p + ".durationSec", durationSec)
                .addData(p + ".complete", finished)
                .addData(p + ".outcome", getOutcome());
    }
}
