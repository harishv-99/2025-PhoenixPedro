package edu.ftcsushi.fw.task;

import java.util.Objects;
import java.util.function.Supplier;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Sequential runner for {@link OutputTask} instances that exposes the active task's output.
 *
 * <p>This is intentionally built on top of {@link TaskRunner} rather than introducing a new
 * scheduler. Sushi keeps one non-blocking timing abstraction: {@link Task}.</p>
 *
 * <h2>Usage</h2>
 * <pre>{@code
 * OutputTaskRunner feederQueue = Tasks.outputQueue(0.0);
 *
 * OutputTaskFactory feedOne = Tasks.outputPulse("feed")
 *         .startWhen(shooterReady.and(aimLocked))
 *         .runOutput(0.9)
 *         .forSeconds(0.12)
 *         .build();
 *
 * PlantTargetResolver finalTransferTarget = PlantTargets.overlay(baseTransferTarget)
 *         .add("feedPulse", feederQueue.activeSource(), feederQueue)
 *         .build();
 *
 * Plant transfer = FtcActuators.plant(hardwareMap)
 *         .crServo("transfer", Direction.FORWARD)
 *         .power()
 *         .targetFromResolver(finalTransferTarget)
 *         .build();
 *
 * // In your loop: manage requests, advance the queue, then update the Plant.
 * feederQueue.whileHigh(clock, shootHeld, 1, feedOne);
 * feederQueue.update(clock);
 * transfer.update(clock);
 * }
 * </pre>
 *
 * <p>{@link #getAsDouble(LoopClock)} calls {@link #update(LoopClock)} internally and returns the
 * most recent output, so the minimum safe usage is:
 * {@code double out = queue.getAsDouble(clock);}.
 * Prefer calling {@link #update(LoopClock)} explicitly near the top of your loop so ordering is
 * obvious.</p>
 *
 * <p>Built-in positive-duration output tasks remain active with their run output available for at
 * least one runner cycle, even when their configured duration is shorter than one loop. A
 * zero-duration run never replaces {@code idleOutput}; a configured cooldown may keep its task
 * active at that idle value.</p>
 *
 * <p>Total cancellation and lifecycle-failure paths invalidate per-cycle source caches
 * immediately and report the configured {@code idleOutput}, even if active output was sampled
 * earlier in the same cycle. {@link #cancelCurrent()} also invalidates caches, but intentionally
 * retains queued work; a later update, including another top-level update in the same cycle, may
 * therefore start the next queued output Task. If cancellation, reset, or another invalidating
 * lifecycle action re-enters an in-progress output/active observation, that observation fails
 * rather than overwriting the invalidation; a later nonrecursive observation reports the runner's
 * actual new state.</p>
 */
public final class OutputTaskRunner implements ScalarSource {

    private final TaskRunner runner = new TaskRunner();
    private final double idleOutput;

    // Cache output per cycle so debug + multiple consumers don't re-read and risk ordering confusion.
    private long lastOutputCycle = Long.MIN_VALUE;
    private long lastOutputRevision = Long.MIN_VALUE;
    private double lastOutput = 0.0;

    /** Shared revision lets every retained activeSource observe abort/failure invalidation. */
    private long invalidationRevision = 0L;

    /** Shared by output and every active view so cross-kind recursion is also rejected. */
    private boolean observationInProgress = false;

    /**
     * Create a queue with a fixed idle output.
     *
     * @param idleOutput output returned when no task is active
     */
    OutputTaskRunner(double idleOutput) {
        this.idleOutput = idleOutput;
        this.lastOutput = idleOutput;
    }

    /**
     * Enqueue an output-producing task.
     *
     * @throws IllegalStateException if this exact instance is already current or queued; repeated
     *                               output behavior requires a fresh task from an
     *                               {@link OutputTaskFactory}
     */
    public void enqueue(OutputTask task) {
        runner.enqueue(task);
    }

    /**
     * Cancel the active task, if any, and keep queued tasks intact.
     *
     * <p>If cancellation throws, queued work is discarded and the failure is rethrown.</p>
     */
    public boolean cancelCurrent() {
        try {
            return runner.cancelCurrent();
        } finally {
            invalidateOutput();
        }
    }

    /**
     * Cancel the active task (if any) and clear queued tasks.
     *
     * <p>The runner is empty afterward even if the cancellation hook throws.</p>
     */
    public void cancelAndClear() {
        try {
            runner.cancelAndClear();
        } finally {
            invalidateOutput();
        }
    }

    /**
     * Total backlog count = active task (if any) + queued tasks.
     *
     * <p>This is useful when you want to keep exactly 1 “feed one” task buffered while a
     * trigger is held, without accidentally queueing up dozens of pulses.</p>
     */
    public int backlogCount() {
        try {
            return (runner.hasActiveTask() ? 1 : 0) + runner.queuedCount();
        } catch (RuntimeException failure) {
            invalidateOutput();
            throw failure;
        }
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
     * <p><b>Important:</b> tasks are single-use, so {@code taskFactory} must produce a
     * <em>new</em> {@link OutputTask} instance each time it is called. Duplicate instances are
     * rejected while current or queued, and a previously-started framework task rejects another
     * start. A reusable
     * {@link OutputTaskFactory} from {@link Tasks#outputPulse(String)} is the usual way to satisfy
     * that rule for repeated pulses.</p>
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
        try {
            return runner.hasActiveTask();
        } catch (RuntimeException failure) {
            invalidateOutput();
            throw failure;
        }
    }

    /**
     * A {@link BooleanSource} view of {@link #hasActiveTask()}.
     *
     * <p>This is useful for building clean, declarative output selection rules:
     * "if the queue is active, let the queue override the base target". Because this is a
     * {@code BooleanSource}, it can be used directly as an enable condition in a
     * {@code PlantTargets.overlay(...)} layer.</p>
     *
     * <p>The returned source calls {@link #update(LoopClock)} and memoizes by
     * {@link LoopClock#cycle()} plus this runner's lifecycle-invalidation revision. It publishes
     * only after update and active-state inspection both succeed, so a failed observation can be
     * retried in the same cycle. Recursive observation is rejected.</p>
     */
    public BooleanSource activeSource() {
        OutputTaskRunner self = this;
        return new BooleanSource() {
            private long lastCycle = Long.MIN_VALUE;
            private long lastRevision = Long.MIN_VALUE;
            private boolean last = false;

            /**
             * {@inheritDoc}
             */
            @Override
            public boolean getAsBoolean(LoopClock clock) {
                if (self.observationInProgress) {
                    throw reentrantObservation("active state");
                }
                Objects.requireNonNull(clock, "clock");
                long cyc = clock.cycle();
                long revision = self.invalidationRevision;
                if (cyc == lastCycle && revision == lastRevision) {
                    return last;
                }

                self.observationInProgress = true;
                try {
                    // Ensure the queue state is current.
                    self.update(clock);
                    boolean candidate = self.hasActiveTask();
                    if (revision != self.invalidationRevision) {
                        throw changedDuringObservation("active state");
                    }

                    last = candidate;
                    lastRevision = revision;
                    lastCycle = cyc;
                    return last;
                } finally {
                    self.observationInProgress = false;
                }
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void reset() {
                try {
                    self.reset();
                } finally {
                    lastCycle = Long.MIN_VALUE;
                    lastRevision = Long.MIN_VALUE;
                    last = false;
                }
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
                boolean liveActive = false;
                String stateDebugError = null;
                try {
                    liveActive = self.hasActiveTask();
                } catch (RuntimeException failure) {
                    stateDebugError = describeFailure(failure);
                }
                dbg.addData(p + ".class", "OutputQueueActive")
                        .addData(p + ".active", liveActive);
                if (stateDebugError != null) {
                    dbg.addData(p + ".stateDebugError", stateDebugError);
                }
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
        try {
            runner.update(clock);
        } catch (RuntimeException failure) {
            invalidateOutput();
            throw failure;
        }
    }

    /**
     * The current output value.
     *
     * <p>This samples the active task's {@link OutputTask#getOutput()} if present, otherwise
     * returns the configured idle output. A complete successful observation is cached by cycle
     * and lifecycle-invalidation revision. A failing value getter leaves the previous committed
     * cache intact and may be retried in the same cycle. Recursive observation is rejected.</p>
     */
    public double output(LoopClock clock) {
        if (observationInProgress) {
            throw reentrantObservation("output");
        }
        Objects.requireNonNull(clock, "clock");

        observationInProgress = true;
        try {
            return observeOutput(clock);
        } finally {
            observationInProgress = false;
        }
    }

    /** Observe output while the caller owns {@link #observationInProgress}. */
    private double observeOutput(LoopClock clock) {
        long cyc = clock.cycle();
        long revision = invalidationRevision;
        if (cyc == lastOutputCycle && revision == lastOutputRevision) {
            return lastOutput;
        }

        final boolean active;
        try {
            active = runner.hasActiveTask();
        } catch (RuntimeException failure) {
            // Completion is Task lifecycle, so TaskRunner has fail-stopped and this owner must
            // expose its real idle state rather than retaining the old queue observation.
            invalidateOutput();
            throw failure;
        }

        double candidate = idleOutput;
        if (active) {
            Task cur = runner.currentTaskOrNull();
            if (cur instanceof OutputTask) {
                // This is a retryable value observation, not Task lifecycle. Do not invalidate or
                // detach the Task merely because its getter threw.
                candidate = ((OutputTask) cur).getOutput();
            }
        }

        if (revision != invalidationRevision) {
            // A getter or lifecycle callback cancelled/reset the queue. Preserve the idle
            // invalidation it published; never overwrite it with the detached Task's candidate.
            throw changedDuringObservation("output");
        }

        lastOutput = candidate;
        lastOutputRevision = revision;
        lastOutputCycle = cyc;
        return lastOutput;
    }

    /**
     * ScalarSource integration.
     *
     * <p>This calls {@link #update(LoopClock)} and then returns {@link #output(LoopClock)}.</p>
     */
    @Override
    public double getAsDouble(LoopClock clock) {
        if (observationInProgress) {
            throw reentrantObservation("output");
        }
        Objects.requireNonNull(clock, "clock");

        observationInProgress = true;
        try {
            long revision = invalidationRevision;
            update(clock);
            if (revision != invalidationRevision) {
                throw changedDuringObservation("output");
            }
            return observeOutput(clock);
        } finally {
            observationInProgress = false;
        }
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

        boolean hasActive = false;
        String stateDebugError = null;
        try {
            hasActive = runner.hasActiveTask();
        } catch (RuntimeException failure) {
            // The runner has already fail-stopped. Keep diagnostics non-throwing and describe the
            // lifecycle failure while reporting the now-idle state.
            invalidateOutput();
            stateDebugError = describeFailure(failure);
        }
        int queued = runner.queuedCount();

        dbg.addLine(p)
                .addData(p + ".idleOutput", idleOutput)
                .addData(p + ".lastOutput", lastOutput)
                .addData(p + ".hasActive", hasActive)
                .addData(p + ".queuedCount", queued)
                .addData(p + ".backlogCount", (hasActive ? 1 : 0) + queued);
        if (stateDebugError != null) {
            dbg.addData(p + ".stateDebugError", stateDebugError);
        }

        runner.debugDump(dbg, p + ".runner");
    }

    /** Return output sampling to the configured idle state, including within the current cycle. */
    private void invalidateOutput() {
        lastOutputCycle = Long.MIN_VALUE;
        lastOutputRevision = Long.MIN_VALUE;
        lastOutput = idleOutput;
        invalidationRevision++;
    }

    /** Explain a recursive value observation without invoking arbitrary Task formatting code. */
    private static IllegalStateException reentrantObservation(String observation) {
        return new IllegalStateException(
                "OutputTaskRunner cannot observe " + observation + " reentrantly; "
                        + "OutputTask value getters and Task lifecycle callbacks must not sample "
                        + "the same runner recursively."
        );
    }

    /** Preserve a lifecycle invalidation that occurred during a staged value observation. */
    private static IllegalStateException changedDuringObservation(String observation) {
        return new IllegalStateException(
                "OutputTaskRunner " + observation + " changed while it was being observed; "
                        + "do not cancel, reset, or otherwise mutate the runner from an "
                        + "OutputTask value getter or Task lifecycle callback. Retry after the "
                        + "lifecycle action completes."
        );
    }

    /** Compact diagnostic text that avoids invoking arbitrary task formatting code. */
    private static String describeFailure(RuntimeException failure) {
        String message = failure.getMessage();
        return failure.getClass().getSimpleName()
                + ((message == null || message.isEmpty()) ? "" : ": " + message);
    }
}
