package edu.ftcphoenix.fw.task;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Sequential runner for {@link OutputTask} instances that exposes the active task's output.
 *
 * <p>This is intentionally built on top of {@link TaskRunner} rather than introducing a new
 * scheduler. Phoenix keeps one non-blocking timing abstraction: {@link Task}.</p>
 *
 * <h2>Usage</h2>
 * <pre>{@code
 * OutputTaskRunner feederQueue = new OutputTaskRunner(0.0);
 *
 * // Enqueue a "feed one" pulse or any other output-producing task.
 * feederQueue.enqueue(OutputTasks.outputForSeconds("feed", 0.9, 0.12));
 *
 * // In your loop:
 * feederQueue.update(clock);            // idempotent by clock.cycle()
 * double feederPower = feederQueue.getAsDouble(clock);
 * transferShooterPlant.setTarget(feederPower);
 * }
 * </pre>
 *
 * <p>{@link #getAsDouble(LoopClock)} calls {@link #update(LoopClock)} internally and returns the
 * most recent output, so the minimum safe usage is:
 * {@code double out = queue.getAsDouble(clock);}.
 * Prefer calling {@link #update(LoopClock)} explicitly near the top of your loop so ordering is
 * obvious.</p>
 */
public final class OutputTaskRunner implements ScalarSource {

    private final TaskRunner runner = new TaskRunner();
    private final double idleOutput;

    // Cache output per cycle so debug + multiple consumers don't re-read and risk ordering confusion.
    private long lastOutputCycle = Long.MIN_VALUE;
    private double lastOutput = 0.0;

    /**
     * Create a queue with a fixed idle output.
     *
     * @param idleOutput output returned when no task is active
     */
    public OutputTaskRunner(double idleOutput) {
        this.idleOutput = idleOutput;
        this.lastOutput = idleOutput;
    }

    /**
     * Enqueue an output-producing task.
     */
    public void enqueue(OutputTask task) {
        runner.enqueue(task);
    }

    /**
     * Clear the queue and forget the current task.
     */
    public void clear() {
        runner.clear();
        lastOutputCycle = Long.MIN_VALUE;
        lastOutput = idleOutput;
    }

    /**
     * @return true if there is no current task and no queued tasks.
     */
    public boolean isIdle() {
        return runner.isIdle();
    }

    /**
     * @return the number of tasks remaining in the queue (not counting the active task).
     */
    public int queuedCount() {
        return runner.queuedCount();
    }

    /**
     * @return true if a task is currently active (started and not complete).
     */
    public boolean hasActiveTask() {
        return runner.hasActiveTask();
    }

    /**
     * Advance the queue by one loop.
     *
     * <p>This is idempotent by {@link LoopClock#cycle()} because {@link TaskRunner#update(LoopClock)}
     * is idempotent.</p>
     */
    public void update(LoopClock clock) {
        runner.update(clock);
    }

    /**
     * The current output value.
     *
     * <p>This samples the active task's {@link OutputTask#getOutput()} if present, otherwise
     * returns the configured idle output.</p>
     */
    public double output(LoopClock clock) {
        long cyc = clock.cycle();
        if (cyc == lastOutputCycle) {
            return lastOutput;
        }
        lastOutputCycle = cyc;

        if (!runner.hasActiveTask()) {
            lastOutput = idleOutput;
            return lastOutput;
        }

        Task cur = runner.currentTaskOrNull();
        if (cur instanceof OutputTask) {
            lastOutput = ((OutputTask) cur).getOutput();
        } else {
            // This should never happen because we only enqueue OutputTask, but fail safe.
            lastOutput = idleOutput;
        }
        return lastOutput;
    }

    /**
     * ScalarSource integration.
     *
     * <p>This calls {@link #update(LoopClock)} and then returns {@link #output(LoopClock)}.</p>
     */
    @Override
    public double getAsDouble(LoopClock clock) {
        update(clock);
        return output(clock);
    }

    @Override
    public void reset() {
        clear();
    }

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "outputQueue" : prefix;

        dbg.addLine(p)
                .addData(p + ".idleOutput", idleOutput)
                .addData(p + ".lastOutput", lastOutput)
                .addData(p + ".hasActive", runner.hasActiveTask())
                .addData(p + ".queuedCount", runner.queuedCount());

        runner.debugDump(dbg, p + ".runner");
    }
}
