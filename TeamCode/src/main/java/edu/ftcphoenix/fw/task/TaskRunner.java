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
 *   <li>Expose a small API to enqueue, cancel, and inspect tasks.</li>
 * </ul>
 *
 * <p>Framework Tasks enforce a single-use start lifecycle. This runner additionally rejects the
 * same object identity when that Task is already current or queued, catching aliases before they
 * reach {@link Task#start(LoopClock)}. The runner deliberately keeps no permanent identity
 * history; once an instance is no longer owned here it can be enqueued, but a previously-started
 * framework Task will reject the later start attempt. Use a builder or macro method,
 * {@code Supplier<Task>}, or {@link OutputTaskFactory} to create fresh instances for repetition.</p>
 *
 * <h2>Per-cycle idempotency</h2>
 * <p>{@link #update(LoopClock)} is idempotent by {@link LoopClock#cycle()}.</p>
 *
 * <p>This protects against accidental double-updates within a single OpMode loop cycle
 * (for example, nested helpers that both call {@code runner.update(clock)}). Without
 * idempotency, a double-update can cause tasks to advance twice as fast, timeouts to
 * elapse early, or short tasks to be skipped unexpectedly.</p>
 *
 * <p>When a queued task starts, the runner also gives it its first {@code update(clock)} in the
 * same call. Timed task implementations therefore anchor their own intervals to
 * {@link LoopClock#nowSec()}; the current {@link LoopClock#dtSec()} belongs to the interval before
 * that newly started task.</p>
 *
 * <h2>Failure semantics</h2>
 * <p>If a task throws a {@link RuntimeException} from {@code start(...)}, {@code update(...)}, or
 * {@code isComplete()}, the runner fails closed: it detaches and best-effort cancels that task,
 * discards every queued or reentrantly-enqueued follow-up, resets its cycle state, and rethrows the
 * original exception. If cancellation also throws, that cleanup failure is attached to the
 * original as a suppressed exception. If an explicit {@link #cancelCurrent()} hook throws, its
 * queued follow-ups are also discarded; {@link #cancelAndClear()} always ends empty.</p>
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

    /** Prevent reentrant updates from starting follow-up work while cancellation is in progress. */
    private boolean suppressUpdates = false;

    /** Prevent lifecycle callbacks from recursively advancing this same runner. */
    private boolean updateInProgress = false;

    /**
     * Enqueue a task to be run after all currently queued tasks.
     *
     * @param task task to enqueue (must not be {@code null})
     * @throws IllegalStateException if this exact instance is already current or queued
     */
    public void enqueue(Task task) {
        if (task == null) {
            throw new IllegalArgumentException("task must not be null");
        }
        if (task == current) {
            throw alreadyOwned(task, "the current task");
        }
        for (Task queued : queue) {
            if (task == queued) {
                throw alreadyOwned(task, "the queue");
            }
        }
        queue.add(task);
    }

    /**
     * Cancel the active task, if any, while leaving queued tasks intact.
     *
     * <p>This is useful when the caller wants to stop the current action but keep later queued
     * work available for later execution. If the current Task already completed, it is detached
     * without another cancellation callback and this method returns {@code false}. If active
     * cancellation throws, queued follow-ups are cleared and the failure is rethrown so work cannot
     * continue after failed cleanup.</p>
     *
     * @return {@code true} if a task was active and received {@link Task#cancel()}; otherwise
     * {@code false}
     */
    public boolean cancelCurrent() {
        Task task = current;
        if (task == null) {
            return false;
        }

        boolean complete = checkComplete(task);
        if (current != task) {
            // A custom completion query re-entered the runner and changed ownership.
            return false;
        }
        if (complete) {
            current = null;
            lastUpdatedCycle = Long.MIN_VALUE;
            return false;
        }

        current = null;
        lastUpdatedCycle = Long.MIN_VALUE;

        boolean previousSuppressUpdates = suppressUpdates;
        suppressUpdates = true;
        try {
            task.cancel();
            return true;
        } catch (RuntimeException failure) {
            // A follow-up must not run after cleanup of its prerequisite failed.
            queue.clear();
            current = null;
            lastUpdatedCycle = Long.MIN_VALUE;
            throw failure;
        } finally {
            suppressUpdates = previousSuppressUpdates;
        }
    }

    /**
     * Cancel the active task (if any) and clear queued tasks.
     *
     * <p>This is the safest general-purpose abort helper for TeleOp takeovers, safety interlocks,
     * and route interruptions.</p>
     */
    public void cancelAndClear() {
        Task task = current;
        // Detach first so cancellation cannot observe itself as still owned. Clear both before and
        // after the hook so reentrant enqueue/cancel calls cannot escape a total abort.
        current = null;
        queue.clear();
        lastUpdatedCycle = Long.MIN_VALUE;

        boolean previousSuppressUpdates = suppressUpdates;
        suppressUpdates = true;
        try {
            if (task != null) {
                task.cancel();
            }
        } finally {
            current = null;
            queue.clear();
            lastUpdatedCycle = Long.MIN_VALUE;
            suppressUpdates = previousSuppressUpdates;
        }
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
        Task task = current;
        return task != null && !checkComplete(task);
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
     *       {@link Task#update(LoopClock)} method is called exactly once, including when it was
     *       started earlier in this same runner call.</li>
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

        if (suppressUpdates || updateInProgress) {
            return;
        }

        long c = clock.cycle();
        if (c == lastUpdatedCycle) {
            return;
        }
        lastUpdatedCycle = c;

        updateInProgress = true;
        try {
            while (current == null || checkComplete(current)) {
                if (current != null) {
                    current = null;
                }
                if (queue.isEmpty()) {
                    break;
                }

                Task next = queue.remove(0);
                current = next;
                startTask(next, clock);

                // A lifecycle callback may cooperatively abort or otherwise detach itself.
                if (current != next) {
                    continue;
                }
            }

            Task task = current;
            if (task != null && !checkComplete(task)) {
                updateTask(task, clock);
            }
        } finally {
            updateInProgress = false;
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

        Task task = current;
        if (task != null) {
            dbg.addData(p + ".currentClass", task.getClass().getSimpleName());
            try {
                dbg.addData(p + ".currentName", task.getDebugName());
            } catch (RuntimeException failure) {
                dbg.addData(p + ".currentNameError", describeFailure(failure));
            }

            boolean lifecycleReadable = true;
            try {
                // Completion is a Task lifecycle query even when telemetry requests it. Preserve
                // fail-stop cleanup, but report rather than propagate the programming failure.
                dbg.addData(p + ".currentComplete", checkComplete(task))
                        .addData(p + ".currentOutcome", task.getOutcome());
            } catch (RuntimeException failure) {
                lifecycleReadable = false;
                dbg.addData(p + ".currentDebugError", describeFailure(failure));
            }
            if (lifecycleReadable) {
                try {
                    task.debugDump(dbg, p + ".current");
                } catch (RuntimeException failure) {
                    dbg.addData(p + ".currentDumpError", describeFailure(failure));
                }
            }
        }

        if (!queue.isEmpty()) {
            Task next = queue.get(0);
            dbg.addData(p + ".nextClass", next.getClass().getSimpleName())
                    .addData(p + ".queuedCount", queue.size());
            try {
                dbg.addData(p + ".nextName", next.getDebugName());
            } catch (RuntimeException failure) {
                dbg.addData(p + ".nextDebugError", describeFailure(failure));
            }
        }
    }

    /** Build an actionable duplicate-identity error without retaining task history. */
    private static IllegalStateException alreadyOwned(Task task, String location) {
        String name = task.getDebugName();
        if (name == null || name.isEmpty()) {
            name = task.getClass().getSimpleName();
        }
        if (name == null || name.isEmpty()) {
            name = "unnamed Task";
        }
        return new IllegalStateException(
                "TaskRunner already owns task \"" + name + "\" in " + location
                        + "; the same Task instance cannot be enqueued twice at once. "
                        + "Create a fresh task with its builder or macro method, a "
                        + "Supplier<Task>, or an OutputTaskFactory.");
    }

    /** Compact diagnostic text that avoids invoking arbitrary task formatting code. */
    private static String describeFailure(RuntimeException failure) {
        String message = failure.getMessage();
        return failure.getClass().getSimpleName()
                + ((message == null || message.isEmpty()) ? "" : ": " + message);
    }

    /** Invoke one start hook and fail-stop the runner if it throws. */
    private void startTask(Task task, LoopClock clock) {
        try {
            task.start(clock);
        } catch (RuntimeException failure) {
            throw failStop(task, failure);
        }
    }

    /** Invoke one update hook and fail-stop the runner if it throws. */
    private void updateTask(Task task, LoopClock clock) {
        try {
            task.update(clock);
        } catch (RuntimeException failure) {
            throw failStop(task, failure);
        }
    }

    /** Query completion and fail-stop the runner if the query throws. */
    private boolean checkComplete(Task task) {
        try {
            return task.isComplete();
        } catch (RuntimeException failure) {
            throw failStop(task, failure);
        }
    }

    /**
     * Reset all runner-owned state, best-effort cancel the failed task, and preserve the original
     * lifecycle failure as the exception visible to the caller.
     */
    private RuntimeException failStop(Task failedTask, RuntimeException failure) {
        current = null;
        queue.clear();
        lastUpdatedCycle = Long.MIN_VALUE;

        boolean previousSuppressUpdates = suppressUpdates;
        suppressUpdates = true;
        try {
            if (failedTask != null) {
                failedTask.cancel();
            }
        } catch (RuntimeException cleanupFailure) {
            if (cleanupFailure != failure) {
                failure.addSuppressed(cleanupFailure);
            }
        } finally {
            // Cancellation is user code: clear again in case it re-entered enqueue/cancel APIs.
            current = null;
            queue.clear();
            lastUpdatedCycle = Long.MIN_VALUE;
            suppressUpdates = previousSuppressUpdates;
        }
        return failure;
    }
}
