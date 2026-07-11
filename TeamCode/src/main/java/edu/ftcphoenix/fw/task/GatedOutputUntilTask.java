package edu.ftcphoenix.fw.task;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * An {@link OutputTask} that waits for a gate condition, then outputs a run value until a done
 * condition (or timeout) is reached.
 *
 * <p>This is the core primitive for "feed one", "spit out", and similar short, reactive
 * behaviors. It is designed to work with or without sensors:</p>
 *
 * <ul>
 *   <li><b>Sensor-based</b>: set {@code doneWhen} to a real condition such as
 *       "piece left the gate" or "touch sensor released".</li>
 *   <li><b>Sensorless fallback</b>: set {@code doneWhen} to
 *       {@code BooleanSource.constant(false)} and rely on {@code maxRunSec} to stop after a fixed
 *       time.</li>
 * </ul>
 *
 * <h2>Phases</h2>
 * <ol>
 *   <li><b>WAIT</b>: output {@code idleOutput} until {@code startWhen} becomes true.</li>
 *   <li><b>RUN</b>: output {@code runOutput} until {@code doneWhen} is true <em>and</em> the
 *       minimum run time has elapsed, or until the maximum run time elapses.</li>
 *   <li><b>COOLDOWN</b> (optional): output {@code idleOutput} for {@code cooldownSec} seconds.</li>
 * </ol>
 *
 * <p>RUN and COOLDOWN each capture their own {@link LoopClock#nowSec()} anchor. When a gate opens,
 * a condition that is already done with no required minimum run completes at idle. Otherwise a
 * positive required RUN window publishes {@code runOutput} in that same runner cycle, guaranteeing
 * one downstream observation even when the configured duration is shorter than the next loop. A
 * zero RUN window stays at {@code idleOutput} and completes immediately (or enters its configured
 * cooldown) without publishing the run value.</p>
 *
 * <h2>Cancellation</h2>
 * <p>{@link #cancel()} immediately transitions the task to DONE, restores {@code idleOutput}, and
 * reports {@link TaskOutcome#CANCELLED}. This makes it safe to clear queued output tasks during
 * driver override, mechanism shutdown, or mode transitions.</p>
 */
public final class GatedOutputUntilTask implements OutputTask {

    private enum Phase {
        WAIT,
        RUN,
        COOLDOWN,
        DONE
    }

    private final String name;
    private final BooleanSource startWhen;
    private final BooleanSource doneWhen;
    private final ScalarSource runOutput;

    private final double idleOutput;
    private final double minRunSec;
    private final double maxRunSec;
    private final double cooldownSec;

    private Phase phase = Phase.WAIT;
    private double runStartedSec = 0.0;
    private double runElapsedSec = 0.0;
    private double cooldownStartedSec = 0.0;
    private double cooldownElapsedSec = 0.0;
    private double currentOutput = 0.0;
    private TaskOutcome finalOutcome = TaskOutcome.NOT_DONE;

    /**
     * Create a gated output task.
     *
     * @param name        debug label
     * @param startWhen   gate condition to begin RUN
     * @param doneWhen    completion condition evaluated during RUN
     * @param runOutput   output while RUNning
     * @param idleOutput  output while waiting or cooling down
     * @param minRunSec   minimum run time in seconds, must be {@code >= 0}
     * @param maxRunSec   maximum run time in seconds, must be {@code >= minRunSec}
     * @param cooldownSec cooldown time in seconds after completion, must be {@code >= 0}
     */
    public GatedOutputUntilTask(String name,
                                BooleanSource startWhen,
                                BooleanSource doneWhen,
                                ScalarSource runOutput,
                                double idleOutput,
                                double minRunSec,
                                double maxRunSec,
                                double cooldownSec) {
        Objects.requireNonNull(startWhen, "startWhen is required");
        Objects.requireNonNull(doneWhen, "doneWhen is required");
        Objects.requireNonNull(runOutput, "runOutput is required");
        if (minRunSec < 0.0) {
            throw new IllegalArgumentException("minRunSec must be >= 0, got " + minRunSec);
        }
        if (maxRunSec < minRunSec) {
            throw new IllegalArgumentException("maxRunSec must be >= minRunSec, got max=" + maxRunSec + " min=" + minRunSec);
        }
        if (cooldownSec < 0.0) {
            throw new IllegalArgumentException("cooldownSec must be >= 0, got " + cooldownSec);
        }

        this.name = (name == null || name.isEmpty()) ? "GatedOutput" : name;
        this.startWhen = startWhen;
        this.doneWhen = doneWhen;
        this.runOutput = runOutput;
        this.idleOutput = idleOutput;
        this.minRunSec = minRunSec;
        this.maxRunSec = maxRunSec;
        this.cooldownSec = cooldownSec;
        this.currentOutput = idleOutput;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void start(LoopClock clock) {
        phase = Phase.WAIT;
        runStartedSec = 0.0;
        runElapsedSec = 0.0;
        cooldownStartedSec = 0.0;
        cooldownElapsedSec = 0.0;
        currentOutput = idleOutput;
        finalOutcome = TaskOutcome.NOT_DONE;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void update(LoopClock clock) {
        switch (phase) {
            case WAIT:
                currentOutput = idleOutput;
                if (startWhen.getAsBoolean(clock)) {
                    beginRun(clock);
                }
                break;
            case RUN:
                runElapsedSec = elapsedSince(runStartedSec, clock);
                currentOutput = runOutput.getAsDouble(clock);
                boolean done = doneWhen.getAsBoolean(clock);
                if (done && runElapsedSec >= minRunSec) {
                    finishRun(TaskOutcome.SUCCESS, clock);
                    break;
                }
                if (runElapsedSec >= maxRunSec) {
                    finishRun(TaskOutcome.TIMEOUT, clock);
                }
                break;
            case COOLDOWN:
                currentOutput = idleOutput;
                cooldownElapsedSec = elapsedSince(cooldownStartedSec, clock);
                if (cooldownElapsedSec >= cooldownSec) {
                    phase = Phase.DONE;
                }
                break;
            case DONE:
            default:
                currentOutput = idleOutput;
                break;
        }
    }

    /**
     * Enter the RUN phase at the current absolute loop time.
     *
     * <p>An already-satisfied done condition with no positive minimum completes at idle. Otherwise
     * a positive required run window publishes its output immediately so the downstream
     * output/Plant phase can observe it in this runner cycle. A zero run window completes without
     * ever publishing the run output.</p>
     */
    private void beginRun(LoopClock clock) {
        phase = Phase.RUN;
        runStartedSec = clock.nowSec();
        runElapsedSec = 0.0;

        boolean done = doneWhen.getAsBoolean(clock);
        if (done && minRunSec <= 0.0) {
            finishRun(TaskOutcome.SUCCESS, clock);
            return;
        }

        if (maxRunSec <= 0.0) {
            finishRun(TaskOutcome.TIMEOUT, clock);
            return;
        }

        currentOutput = runOutput.getAsDouble(clock);
    }

    /** End RUN and either begin cooldown or complete immediately. */
    private void finishRun(TaskOutcome outcome, LoopClock clock) {
        finalOutcome = outcome;
        currentOutput = idleOutput;
        if (cooldownSec > 0.0) {
            phase = Phase.COOLDOWN;
            cooldownStartedSec = clock.nowSec();
            cooldownElapsedSec = 0.0;
        } else {
            phase = Phase.DONE;
        }
    }

    /** Return non-negative elapsed time from one task-owned interval anchor. */
    private static double elapsedSince(double startedSec, LoopClock clock) {
        return Math.max(0.0, clock.nowSec() - startedSec);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void cancel() {
        if (phase == Phase.DONE) {
            return;
        }
        phase = Phase.DONE;
        currentOutput = idleOutput;
        finalOutcome = TaskOutcome.CANCELLED;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean isComplete() {
        return phase == Phase.DONE;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public TaskOutcome getOutcome() {
        if (!isComplete()) {
            return TaskOutcome.NOT_DONE;
        }
        return finalOutcome;
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
        return currentOutput;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "gatedOutput" : prefix;
        dbg.addData(p + ".name", name)
                .addData(p + ".phase", phase)
                .addData(p + ".output", currentOutput)
                .addData(p + ".runStartedSec", runStartedSec)
                .addData(p + ".runElapsedSec", runElapsedSec)
                .addData(p + ".cooldownStartedSec", cooldownStartedSec)
                .addData(p + ".cooldownElapsedSec", cooldownElapsedSec)
                .addData(p + ".minRunSec", minRunSec)
                .addData(p + ".maxRunSec", maxRunSec)
                .addData(p + ".cooldownSec", cooldownSec)
                .addData(p + ".complete", isComplete())
                .addData(p + ".outcome", getOutcome());
        startWhen.debugDump(dbg, p + ".startWhen");
        doneWhen.debugDump(dbg, p + ".doneWhen");
        runOutput.debugDump(dbg, p + ".runOutput");
    }
}
