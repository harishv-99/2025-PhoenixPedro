package edu.ftcphoenix.fw.task;

import java.util.ArrayList;
import java.util.List;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Internal {@link Task} implementation for {@link Tasks#sequence(Task...)}.
 *
 * <p>Semantics:</p>
 * <ul>
 *   <li>On {@link #start(LoopClock)}, the first child is started.</li>
 *   <li>On each {@link #update(LoopClock)}, the current child is updated.</li>
 *   <li>When the current child finishes, the next child starts before that update returns; a child
 *       that finishes in its own {@code start()} method is skipped through immediately.</li>
 *   <li>The sequence finishes when all children have finished.</li>
 *   <li>Active {@link #cancel()} marks the sequence terminal, then asks its current child to
 *       cancel. Pre-start and terminal cancellation are no-ops.</li>
 * </ul>
 *
 * <p>Robot code constructs this composition through {@link Tasks}:</p>
 * <pre>{@code
 * TaskRunner runner = new TaskRunner();
 *
 * runner.enqueue(Tasks.sequence(
 *     new InstantTask(() -> log("start")),
 *     new WaitUntilTask(readySensor),
 *     new InstantTask(() -> log("done"))
 * ));
 * }</pre>
 *
 * <p>This is the standard Phoenix "do A, then B, then C" implementation for macros. It remains
 * package-private so composition has one public construction surface through {@link Tasks}.</p>
 *
 * <p>A sequence is single-use, and each child position must contain a distinct Task instance.
 * Repeated behavior should construct a fresh sequence with fresh children.</p>
 */
final class SequenceTask implements Task {

    private final List<Task> tasks = new ArrayList<>();
    private boolean startAttempted = false;
    private boolean started = false;
    private boolean cancelled = false;
    /**
     * Index of the current task; {@code -1} before the first task is started.
     */
    private int index = -1;

    /**
     * Create a sequence from an ordered array of tasks.
     *
     * <p>The array is copied; subsequent modifications to {@code tasks} do not affect this
     * sequence.</p>
     *
     * @param tasks ordered array of distinct child tasks; must not be {@code null} or contain
     *              {@code null} or duplicate instances
     */
    SequenceTask(Task... tasks) {
        if (tasks == null) {
            throw new IllegalArgumentException("tasks is required");
        }
        List<Task> copiedTasks = new ArrayList<>(tasks.length);
        for (Task task : tasks) {
            copiedTasks.add(task);
        }
        validateDistinctChildren(copiedTasks);
        this.tasks.addAll(copiedTasks);
    }

    /**
     * {@inheritDoc}
     *
     * <p>Initializes the sequence and starts the first available child task
     * immediately. If one or more children finish in their own {@code start()} calls, the sequence
     * keeps advancing until it finds a still-running child or runs out of tasks.</p>
     */
    @Override
    public void start(LoopClock clock) {
        markStartAttempt();
        started = true;
        cancelled = false;
        index = -1;
        advanceToNextTask(clock);
    }

    /**
     * {@inheritDoc}
     *
     * <p>Updates only the current child task. When that child finishes, the sequence advances to
     * the next child before returning.</p>
     */
    @Override
    public void update(LoopClock clock) {
        if (!started) {
            throw TaskLifecycle.updateBeforeStart("Task returned by Tasks.sequence(...)");
        }
        Task current = getCurrentTask();
        if (current == null) {
            return;
        }
        current.update(clock);
        if (current.isComplete()) {
            advanceToNextTask(clock);
        }
    }

    /**
     * {@inheritDoc}
     *
     * <p>Marks the whole sequence complete with {@link TaskOutcome#CANCELLED}, then asks the
     * current child to cancel. Later children are never started.</p>
     */
    @Override
    public void cancel() {
        if (!started || isComplete()) {
            return;
        }
        Task current = getCurrentTask();
        // Establish terminal state before child cleanup so a throwing hook cannot reopen the graph.
        cancelled = true;
        index = tasks.size();
        // The Task contract makes pre-start and terminal cancellation safe no-ops. Do not query
        // child completion here: this path may be cleaning up that exact failed lifecycle query.
        if (current != null) {
            current.cancel();
        }
    }

    /**
     * {@inheritDoc}
     *
     * <p>The sequence is complete once it has advanced past its final child, or immediately after a
     * direct cancellation.</p>
     */
    @Override
    public boolean isComplete() {
        return started && index >= tasks.size();
    }

    /**
     * Report the aggregated outcome for this sequence.
     *
     * <p>Semantics:</p>
     * <ul>
     *   <li>While the sequence is still running, this returns {@link TaskOutcome#NOT_DONE}.</li>
     *   <li>If the sequence itself was cancelled, this returns {@link TaskOutcome#CANCELLED}.</li>
     *   <li>Otherwise, once all children have completed, if <b>any</b> child reports
     *       {@link TaskOutcome#TIMEOUT}, this returns {@link TaskOutcome#TIMEOUT}.</li>
     *   <li>Otherwise this returns {@link TaskOutcome#SUCCESS}, regardless of whether individual
     *       children report {@link TaskOutcome#SUCCESS} or {@link TaskOutcome#UNKNOWN}.</li>
     * </ul>
     */
    @Override
    public TaskOutcome getOutcome() {
        if (!isComplete()) {
            return TaskOutcome.NOT_DONE;
        }
        if (cancelled) {
            return TaskOutcome.CANCELLED;
        }
        for (Task task : tasks) {
            if (task.getOutcome() == TaskOutcome.TIMEOUT) {
                return TaskOutcome.TIMEOUT;
            }
        }
        return TaskOutcome.SUCCESS;
    }

    /** Identify this internal implementation by the public factory robot code calls. */
    @Override
    public String getDebugName() {
        return "Tasks.sequence(...)";
    }

    /**
     * {@inheritDoc}
     *
     * <p>Dumps aggregate sequence state plus the currently active child, if one exists.</p>
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "sequence" : prefix;
        dbg.addData(p + ".started", started)
                .addData(p + ".cancelled", cancelled)
                .addData(p + ".size", tasks.size())
                .addData(p + ".index", index)
                .addData(p + ".complete", isComplete())
                .addData(p + ".outcome", getOutcome());
        Task current = getCurrentTask();
        if (current != null) {
            dbg.addData(p + ".currentName", current.getDebugName())
                    .addData(p + ".currentComplete", current.isComplete())
                    .addData(p + ".currentOutcome", current.getOutcome());
            current.debugDump(dbg, p + ".current");
        }
    }

    /**
     * @return the current child task, or {@code null} if there is none.
     */
    private Task getCurrentTask() {
        if (!started || index < 0 || index >= tasks.size()) {
            return null;
        }
        return tasks.get(index);
    }

    /**
     * Advance to the next task and start it immediately.
     *
     * <p>If the next task finishes during its own {@code start()}, this method continues advancing
     * until it finds a non-finished task or runs out of children.</p>
     */
    private void advanceToNextTask(LoopClock clock) {
        while (true) {
            index++;
            if (index >= tasks.size()) {
                return;
            }
            Task next = tasks.get(index);
            next.start(clock);
            if (!next.isComplete()) {
                return;
            }
        }
    }

    /** Reject child aliases that would start one stateful Task instance more than once. */
    private static void validateDistinctChildren(List<Task> children) {
        for (int i = 0; i < children.size(); i++) {
            Task child = children.get(i);
            if (child == null) {
                throw new IllegalArgumentException(
                        "Tasks.sequence children must not contain null; found null at index " + i);
            }
            for (int previous = 0; previous < i; previous++) {
                if (child == children.get(previous)) {
                    throw new IllegalArgumentException(
                            "Tasks.sequence child at index " + i
                                    + " reuses the same Task instance as index " + previous + ". "
                                    + "Each child must be a distinct, fresh task; create it with "
                                    + "its builder or macro method, a Supplier<Task>, or an "
                                    + "OutputTaskFactory.");
                }
            }
        }
    }

    /** Record the single permitted start attempt before starting any child. */
    private void markStartAttempt() {
        if (startAttempted) {
            throw new IllegalStateException(
                    "The Task returned by Tasks.sequence(...) is single-use and start(...) was "
                            + "called more than once. "
                            + "Create a fresh task with its builder or macro method, a "
                            + "Supplier<Task>, or an OutputTaskFactory.");
        }
        startAttempted = true;
    }
}
