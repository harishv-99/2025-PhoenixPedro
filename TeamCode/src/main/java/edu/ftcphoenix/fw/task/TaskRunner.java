package edu.ftcphoenix.fw.task;

import java.util.ArrayList;
import java.util.List;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Simple sequential task runner.
 *
 * <p>Responsibilities:</p>
 * <ul>
 *   <li>Maintain a queue of {@link Task} instances.</li>
 *   <li>Start tasks one at a time in FIFO order.</li>
 *   <li>Update the current task each loop until it reports complete.</li>
 *   <li>Expose a small API to enqueue, cancel, clear, and inspect tasks.</li>
 * </ul>
 *
 * <p>Tasks are assumed to be single-use: once a {@link Task} has completed
 * ({@link Task#isComplete()} returns true), it should not be enqueued again.</p>
 *
 * <h2>Per-cycle idempotency</h2>
 * <p>{@link #update(LoopClock)} is idempotent by {@link LoopClock#cycle()}.</p>
 *
 * <p>This protects against accidental double-updates within a single OpMode loop cycle
 * (for example, nested helpers that both call {@code runner.update(clock)}). Without
 * idempotency, a double-update can cause tasks to advance twice as fast, timeouts to
 * elapse early, or short tasks to be skipped unexpectedly.</p>
 *
 * <h2>Typical usage</h2>
 * <pre>{@code
 * TaskRunner runner = new TaskRunner();
 * runner.enqueue(Tasks.waitSeconds(0.2));
 * runner.enqueue(Tasks.instant(intake::start));
 *
 * // In the loop:
 * runner.update(clock);
 *
 * // If driver input or safety logic needs to abort automation:
 * runner.cancelAndClear();
 * }</pre>
 */
public final class TaskRunner {

    private final List<Task> queue = new ArrayList<>();
    private Task current = null;

    /**
     * Tracks which loop cycle we last updated for, to prevent tasks from being advanced multiple
     * times in the same cycle.
     */
    private long lastUpdatedCycle = Long.MIN_VALUE;

    /**
     * Enqueue a task to be run after all currently queued tasks.
     *
     * @param task task to enqueue (must not be {@code null})
     */
    public void enqueue(Task task) {
        if (task == null) {
            throw new IllegalArgumentException("task must not be null");
        }
        queue.add(task);
    }

    /**
     * Clear the queue and forget any current task without invoking cancellation hooks.
     *
     * <p>This is mainly a legacy "drop everything immediately" helper. Prefer
     * {@link #cancelAndClear()} when aborting automation so the active task can release hardware,
     * stop child tasks, and report {@link TaskOutcome#CANCELLED} cleanly.</p>
     */
    public void clear() {
        queue.clear();
        current = null;
        lastUpdatedCycle = Long.MIN_VALUE;
    }

    /**
     * Cancel the active task, if any, while leaving queued tasks intact.
     *
     * <p>This is useful when the caller wants to stop the current action but keep later queued
     * work available for retry or reuse.</p>
     *
     * @return {@code true} if a task was active and received {@link Task#cancel()}; otherwise
     * {@code false}
     */
    public boolean cancelCurrent() {
        if (current == null) {
            return false;
        }
        current.cancel();
        current = null;
        lastUpdatedCycle = Long.MIN_VALUE;
        return true;
    }

    /**
     * Cancel the active task (if any) and clear queued tasks.
     *
     * <p>This is the safest general-purpose abort helper for TeleOp takeovers, safety interlocks,
     * and route interruptions.</p>
     */
    public void cancelAndClear() {
        cancelCurrent();
        queue.clear();
        lastUpdatedCycle = Long.MIN_VALUE;
    }

    /**
     * @return true if there is no current task and no queued tasks.
     */
    public boolean isIdle() {
        return current == null && queue.isEmpty();
    }

    /**
     * @return the number of tasks remaining in the queue (not counting the current task, if any).
     */
    public int queuedCount() {
        return queue.size();
    }

    /**
     * @return true if a task is currently active (started and not complete).
     */
    public boolean hasActiveTask() {
        return current != null && !current.isComplete();
    }

    /**
     * @return the current task instance, or {@code null} if none has been started.
     *
     * <p>This is intentionally exposed for advanced integrations that need to inspect the active
     * task. Most robot code should not need this.</p>
     */
    public Task currentTaskOrNull() {
        return current;
    }

    /**
     * @return the next queued task (FIFO head), or {@code null} if the queue is empty.
     */
    public Task nextQueuedTaskOrNull() {
        return queue.isEmpty() ? null : queue.get(0);
    }

    /**
     * Update the task runner and the current task.
     *
     * <p>Semantics:</p>
     * <ul>
     *   <li>If there is no current task, or the current task is complete, the runner will pull the
     *       next task from the queue and call its {@link Task#start(LoopClock)} method.</li>
     *   <li>If the task completes immediately in {@code start()}, the runner will advance to the
     *       next queued task (if any) before returning.</li>
     *   <li>If there is an active (not complete) current task after this process, its
     *       {@link Task#update(LoopClock)} method is called exactly once.</li>
     * </ul>
     *
     * <p>This method is idempotent by {@link LoopClock#cycle()}: if called twice in the same loop
     * cycle, the second call is a no-op.</p>
     *
     * @param clock loop clock (must not be {@code null})
     */
    public void update(LoopClock clock) {
        if (clock == null) {
            throw new IllegalArgumentException("clock must not be null");
        }

        long c = clock.cycle();
        if (c == lastUpdatedCycle) {
            return;
        }
        lastUpdatedCycle = c;

        while ((current == null || current.isComplete()) && !queue.isEmpty()) {
            current = queue.remove(0);
            current.start(clock);
            if (current.isComplete()) {
                current = null;
            }
        }

        if (current != null && !current.isComplete()) {
            current.update(clock);
        }
    }

    /**
     * Emit a compact summary of queue + current task for debugging.
     *
     * @param dbg    debug sink (may be {@code null}; if null, no output is produced)
     * @param prefix base key prefix, e.g. {@code "tasks"}
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "tasks" : prefix;

        dbg.addLine(p)
                .addData(p + ".queueSize", queue.size())
                .addData(p + ".hasCurrent", current != null)
                .addData(p + ".lastUpdatedCycle", lastUpdatedCycle);

        if (current != null) {
            dbg.addData(p + ".currentName", current.getDebugName())
                    .addData(p + ".currentClass", current.getClass().getSimpleName())
                    .addData(p + ".currentComplete", current.isComplete())
                    .addData(p + ".currentOutcome", current.getOutcome());
            current.debugDump(dbg, p + ".current");
        }

        if (!queue.isEmpty()) {
            Task next = queue.get(0);
            dbg.addData(p + ".nextName", next.getDebugName())
                    .addData(p + ".nextClass", next.getClass().getSimpleName())
                    .addData(p + ".queuedCount", queue.size());
        }
    }
}
