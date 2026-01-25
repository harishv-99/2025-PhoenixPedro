package edu.ftcphoenix.fw.task;

import java.util.function.Consumer;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Task that runs a set of callbacks for a fixed amount of time.
 *
 * <p>This is a small building block for things like macros:
 * "drive with this command for 0.8 seconds", "run intake for 0.5 seconds", etc.</p>
 *
 * <h2>Lifecycle</h2>
 * <ul>
 *   <li>{@link #start(LoopClock)} – called once when the task begins.
 *       <ul>
 *         <li>Runs the {@code onStart} callback if non-null.</li>
 *         <li>Initializes internal timer to {@code durationSec}.</li>
 *         <li>If {@code durationSec == 0}, immediately runs {@code onFinish}
 *             and marks the task finished.</li>
 *       </ul>
 *   </li>
 *   <li>{@link #update(LoopClock)} – called every loop while the task is active.
 *       <ul>
 *         <li>Runs the {@code onUpdate} callback if non-null.</li>
 *         <li>Subtracts {@code clock.dtSec()} from the remaining time.</li>
 *         <li>When time reaches zero, runs {@code onFinish} (if non-null)
 *             and marks the task finished.</li>
 *       </ul>
 *   </li>
 *   <li>{@link #isComplete()} – returns true once the duration has elapsed.</li>
 * </ul>
 *
 * <h2>Typical usage</h2>
 *
 * <pre>{@code
 * // Example: run intakePlant at +1.0 for 0.7 seconds, then stop.
 * Task intakePulse = new RunForSecondsTask(
 *     0.7,
 *     // onStart: set target once
 *     () -> intakePlant.setTarget(+1.0),
 *     // onUpdate: nothing (we're just holding constant)
 *     null,
 *     // onFinish: stop the intake
 *     () -> intakePlant.setTarget(0.0)
 * );
 *
 * TaskRunner runner = new TaskRunner();
 * runner.enqueue(intakePulse);
 * }</pre>
 */
public final class RunForSecondsTask implements Task {

    private final double durationSec;
    private final Runnable onStart;
    private final Consumer<LoopClock> onUpdate;
    private final Runnable onFinish;

    private boolean started = false;
    private boolean finished = false;
    private double remainingSec = 0.0;

    /**
     * Create a new timed task.
     *
     * @param durationSec how long the task should run (in seconds), must be &gt;= 0.
     * @param onStart     optional callback run once when the task starts (may be null).
     * @param onUpdate    optional callback run each loop while the task is active (may be null).
     *                    Receives the current {@link LoopClock}.
     * @param onFinish    optional callback run once when time elapses (may be null).
     */
    public RunForSecondsTask(double durationSec,
                             Runnable onStart,
                             Consumer<LoopClock> onUpdate,
                             Runnable onFinish) {
        if (durationSec < 0.0) {
            throw new IllegalArgumentException("durationSec must be >= 0, got " + durationSec);
        }
        this.durationSec = durationSec;
        this.onStart = onStart;
        this.onUpdate = onUpdate;
        this.onFinish = onFinish;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void start(LoopClock clock) {
        if (started) {
            // TaskRunner should not call start() twice on the same instance,
            // but guard defensively.
            return;
        }

        started = true;
        finished = false;
        remainingSec = durationSec;

        if (onStart != null) {
            onStart.run();
        }

        // If duration is zero, finish immediately.
        if (remainingSec <= 0.0) {
            if (onFinish != null) {
                onFinish.run();
            }
            finished = true;
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void update(LoopClock clock) {
        if (!started || finished) {
            return;
        }

        if (onUpdate != null) {
            onUpdate.accept(clock);
        }

        remainingSec -= clock.dtSec();
        if (remainingSec <= 0.0 && !finished) {
            if (onFinish != null) {
                onFinish.run();
            }
            finished = true;
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean isComplete() {
        return finished;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public TaskOutcome getOutcome() {
        if (!finished) {
            return TaskOutcome.NOT_DONE;
        }
        return TaskOutcome.SUCCESS;
    }


    /**
     * {@inheritDoc}
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "runForSeconds" : prefix;

        dbg.addData(p + ".durationSec", durationSec)
                .addData(p + ".started", started)
                .addData(p + ".finished", finished)
                .addData(p + ".remainingSec", getRemainingSec())
                .addData(p + ".hasOnStart", onStart != null)
                .addData(p + ".hasOnUpdate", onUpdate != null)
                .addData(p + ".hasOnFinish", onFinish != null);
    }

    /**
     * @return remaining time in seconds (clamped at 0) once the task has started,
     * or 0 if the task is finished or has not yet been started.
     * This is mainly intended for debugging / telemetry.
     */
    public double getRemainingSec() {
        if (!started || finished) {
            return 0.0;
        }
        return Math.max(remainingSec, 0.0);
    }
}
