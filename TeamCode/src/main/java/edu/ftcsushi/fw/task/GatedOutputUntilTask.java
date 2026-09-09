package edu.ftcsushi.fw.task;

import java.util.Objects;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.time.LoopClock;

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
 * <p>Active {@link #cancel()} immediately transitions the task to DONE, restores
 * {@code idleOutput}, and reports {@link TaskOutcome#CANCELLED}. Pre-start and terminal
 * cancellation are no-ops. This makes active output safe to abort during driver override,
 * mechanism shutdown, or mode transitions.</p>
 *
 * <p>Create this leaf Task through the staged {@link Tasks#outputPulse(String)} recipe.</p>
 */
public final class GatedOutputUntilTask extends AbstractTask implements OutputTask {

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
    private TaskOutcome runOutcome = TaskOutcome.NOT_DONE;

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
    GatedOutputUntilTask(String name,
                         BooleanSource startWhen,
                         BooleanSource doneWhen,
                         ScalarSource runOutput,
                         double idleOutput,
                         double minRunSec,
                         double maxRunSec,
                         double cooldownSec) {
        super(name == null || name.trim().isEmpty() ? "GatedOutput" : name);
        Objects.requireNonNull(startWhen, "startWhen is required");
        Objects.requireNonNull(doneWhen, "doneWhen is required");
        Objects.requireNonNull(runOutput, "runOutput is required");
        if (!Double.isFinite(minRunSec) || minRunSec < 0.0) {
            throw new IllegalArgumentException("minRunSec must be finite and >= 0, got " + minRunSec);
        }
        if (!Double.isFinite(maxRunSec) || maxRunSec < minRunSec) {
            throw new IllegalArgumentException("maxRunSec must be finite and >= minRunSec, got max=" + maxRunSec + " min=" + minRunSec);
        }
        if (!Double.isFinite(cooldownSec) || cooldownSec < 0.0) {
            throw new IllegalArgumentException("cooldownSec must be finite and >= 0, got " + cooldownSec);
        }

        this.name = (name == null || name.trim().isEmpty()) ? "GatedOutput" : name;
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
    protected void onStart(LoopClock clock) {
        phase = Phase.WAIT;
        runStartedSec = 0.0;
        runElapsedSec = 0.0;
        cooldownStartedSec = 0.0;
        cooldownElapsedSec = 0.0;
        currentOutput = idleOutput;
        runOutcome = TaskOutcome.NOT_DONE;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void onUpdate(LoopClock clock) {
        switch (phase) {
            case WAIT:
                currentOutput = idleOutput;
                boolean start = startWhen.getAsBoolean(clock);
                if (phase == Phase.DONE) {
                    break;
                }
                if (start) {
                    beginRun(clock);
                }
                break;
            case RUN:
                runElapsedSec = elapsedSince(runStartedSec, clock);
                double output = runOutput.getAsDouble(clock);
                if (phase == Phase.DONE) {
                    break;
                }
                currentOutput = output;
                boolean done = doneWhen.getAsBoolean(clock);
                if (phase == Phase.DONE) {
                    break;
                }
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
                    complete(runOutcome);
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
        if (phase == Phase.DONE) {
            return;
        }
        if (done && minRunSec <= 0.0) {
            finishRun(TaskOutcome.SUCCESS, clock);
            return;
        }

        if (maxRunSec <= 0.0) {
            finishRun(TaskOutcome.TIMEOUT, clock);
            return;
        }

        double output = runOutput.getAsDouble(clock);
        if (phase == Phase.DONE) {
            currentOutput = idleOutput;
            return;
        }
        currentOutput = output;
    }

    /** End RUN and either begin cooldown or complete immediately. */
    private void finishRun(TaskOutcome outcome, LoopClock clock) {
        runOutcome = outcome;
        currentOutput = idleOutput;
        if (cooldownSec > 0.0) {
            phase = Phase.COOLDOWN;
            cooldownStartedSec = clock.nowSec();
            cooldownElapsedSec = 0.0;
        } else {
            phase = Phase.DONE;
            complete(runOutcome);
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
    protected void onCancel() {
        if (phase == Phase.DONE) {
            return;
        }
        phase = Phase.DONE;
        currentOutput = idleOutput;
        runOutcome = TaskOutcome.CANCELLED;
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
    protected void debugState(DebugSink dbg, String prefix) {
        String p = prefix;
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
                .addData(p + ".runOutcome", runOutcome);
        if (!hasFailure() && (!isComplete() || isEndingSettled())) {
            startWhen.debugDump(dbg, p + ".startWhen");
            doneWhen.debugDump(dbg, p + ".doneWhen");
            runOutput.debugDump(dbg, p + ".runOutput");
        }
    }

}
