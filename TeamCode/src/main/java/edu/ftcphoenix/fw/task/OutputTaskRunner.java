package edu.ftcphoenix.fw.task;

import java.util.Objects;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.source.BooleanSource;
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
 * feederQueue.enqueue(Tasks.outputForSeconds("feed", 0.9, 0.12));
 *
 * // In your loop:
 * feederQueue.update(clock);            // idempotent by clock.cycle()
 * double feederPower = feederQueue.getAsDouble(clock);
 * transferShooterTarget.set(feederPower);
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
     * Clear the queue and forget the current task without invoking cancellation hooks.
     *
     * <p>Prefer {@link #cancelAndClear()} when aborting automation.</p>
     */
    public void clear() {
        runner.clear();
        lastOutputCycle = Long.MIN_VALUE;
        lastOutput = idleOutput;
    }

    /**
     * Cancel the active task, if any, and keep queued tasks intact.
     */
    public boolean cancelCurrent() {
        boolean cancelled = runner.cancelCurrent();
        lastOutputCycle = Long.MIN_VALUE;
        lastOutput = idleOutput;
        return cancelled;
    }

    /**
     * Cancel the active task (if any) and clear queued tasks.
     */
    public void cancelAndClear() {
        runner.cancelAndClear();
        lastOutputCycle = Long.MIN_VALUE;
        lastOutput = idleOutput;
    }

    /**
     * Total backlog count = active task (if any) + queued tasks.
     *
     * <p>This is useful when you want to keep exactly 1 “feed one” task buffered while a
     * trigger is held, without accidentally queueing up dozens of pulses.</p>
     */
    public int backlogCount() {
        return (runner.hasActiveTask() ? 1 : 0) + runner.queuedCount();
    }

    /**
     * Ensure that this queue has at least {@code desiredBacklog} tasks either active or queued.
     *
     * <p>This is the core helper for "repeat while held" behavior:
     * if a driver holds a trigger, you can keep a single feed pulse buffered so the next shot
     * starts immediately when the previous one ends.</p>
     *
     * <p><b>Ordering note:</b> this method <em>does not</em> advance the queue.
     * It only enqueues new tasks if backlog is low.
     *
     * <p>This is intentional: callers should control when {@link #update(LoopClock)} happens so
     * task progression doesn't occur "accidentally" in a helper like this. That matters when the
     * queued tasks depend on other subsystem state (for example a shooter feed task that waits on
     * {@code Plant.atTarget()} after the shooter flywheel target is updated for the current loop).
     *
     * <p>If you need {@code desiredBacklog} to reflect tasks that might complete in the current
     * loop, call {@link #update(LoopClock)} earlier in the cycle before calling this method.</p>
     *
     * <p>Safe to call multiple times. This method only enqueues tasks when the current
     * backlog is below {@code desiredBacklog}.</p>
     *
     * <p><b>Important:</b> tasks are assumed to be single-use, so {@code taskFactory} must produce
     * a <em>new</em> {@link OutputTask} instance each time it is called.</p>
     *
     * @param clock          loop clock (required)
     * @param desiredBacklog minimum number of tasks to keep active+queued (>= 0)
     * @param taskFactory    factory to create a new task when more backlog is needed (required)
     */
    public void ensureBacklog(LoopClock clock, int desiredBacklog, Supplier<? extends OutputTask> taskFactory) {
        Objects.requireNonNull(clock, "clock");
        Objects.requireNonNull(taskFactory, "taskFactory");
        if (desiredBacklog < 0) {
            throw new IllegalArgumentException("desiredBacklog must be >= 0, got " + desiredBacklog);
        }

        int backlog = backlogCount();
        while (backlog < desiredBacklog) {
            OutputTask t = taskFactory.get();
            if (t == null) {
                throw new IllegalArgumentException("taskFactory returned null OutputTask");
            }
            enqueue(t);
            backlog++;
        }
    }

    /**
     * Maintain a queue backlog while a request signal is high; cancel and clear when it is low.
     *
     * <p>Example:</p>
     * <pre>{@code
     * feederQueue.whileHigh(clock, requestShoot, 1, feedOneFactory);
     * }</pre>
     *
     * <p><b>Design note:</b> {@code request} should usually be a <em>request</em> signal
     * (driver intent), not a readiness gate. Readiness should live inside the produced task
     * (for example as {@code startWhen}). That way, the task can wait safely without producing
     * output, and your code keeps one buffered task ready to run the moment sensors allow.</p>
     *
     * @param clock       loop clock (required)
     * @param request     request signal; when low, the queue is cleared (required)
     * @param backlog     number of tasks to keep buffered while requested (>= 0)
     * @param taskFactory factory that produces a <b>new</b> task instance each time (required)
     */
    public void whileHigh(
            LoopClock clock,
            BooleanSource request,
            int backlog,
            Supplier<? extends OutputTask> taskFactory
    ) {
        Objects.requireNonNull(clock, "clock");
        Objects.requireNonNull(request, "request");
        Objects.requireNonNull(taskFactory, "taskFactory");
        if (backlog < 0) {
            throw new IllegalArgumentException("backlog must be >= 0, got " + backlog);
        }

        if (!request.getAsBoolean(clock)) {
            cancelAndClear();
            return;
        }

        ensureBacklog(clock, backlog, taskFactory);
    }

    /**
     * Maintain a queue backlog while a request signal is low; cancel and clear when it is high.
     *
     * <p>This mirrors {@code Bindings.whileLow(...)} and keeps boolean signal vocabulary parallel
     * across input bindings and output queues.</p>
     */
    public void whileLow(
            LoopClock clock,
            BooleanSource request,
            int backlog,
            Supplier<? extends OutputTask> taskFactory
    ) {
        Objects.requireNonNull(clock, "clock");
        Objects.requireNonNull(request, "request");
        Objects.requireNonNull(taskFactory, "taskFactory");
        if (backlog < 0) {
            throw new IllegalArgumentException("backlog must be >= 0, got " + backlog);
        }

        if (request.getAsBoolean(clock)) {
            cancelAndClear();
            return;
        }

        ensureBacklog(clock, backlog, taskFactory);
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
     * A {@link BooleanSource} view of {@link #hasActiveTask()}.
     *
     * <p>This is useful for building clean, declarative output selection rules:
     * "if the queue is active, use queue output; otherwise use base output".
     * Because this is a {@code BooleanSource}, you can combine it with other signals
     * and use {@link BooleanSource#choose(edu.ftcphoenix.fw.core.source.ScalarSource, edu.ftcphoenix.fw.core.source.ScalarSource)}.
     * </p>
     *
     * <p>The returned source calls {@link #update(LoopClock)} and memoizes by
     * {@link LoopClock#cycle()} so it stays consistent within a loop.</p>
     */
    public BooleanSource activeSource() {
        OutputTaskRunner self = this;
        return new BooleanSource() {
            private long lastCycle = Long.MIN_VALUE;
            private boolean last = false;

            /**
             * {@inheritDoc}
             */
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                long cyc = clock.cycle();
                if (cyc == lastCycle) {
                    return last;
                }
                lastCycle = cyc;

                // Ensure the queue state is current.
                self.update(clock);
                last = self.hasActiveTask();
                return last;
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                self.reset();
                lastCycle = Long.MIN_VALUE;
                last = false;
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) {
                    return;
                }
                String p = (prefix == null || prefix.isEmpty()) ? "active" : prefix;
                dbg.addData(p + ".class", "OutputQueueActive")
                        .addData(p + ".active", last);
                self.debugDump(dbg, p + ".queue");
            }
        };
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

    /**
     * {@inheritDoc}
     */
    @Override
    public void reset() {
        cancelAndClear();
    }

    /**
     * {@inheritDoc}
     */
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
                .addData(p + ".queuedCount", runner.queuedCount())
                .addData(p + ".backlogCount", backlogCount());

        runner.debugDump(dbg, p + ".runner");
    }
}
