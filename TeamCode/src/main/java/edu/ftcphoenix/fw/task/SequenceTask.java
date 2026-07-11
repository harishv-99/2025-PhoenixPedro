package edu.ftcphoenix.fw.task;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A {@link Task} that runs a sequence of child tasks one after another.
 *
 * <p>Semantics:</p>
 * <ul>
 *   <li>On {@link #start(LoopClock)}, the first child is started.</li>
 *   <li>On each {@link #update(LoopClock)}, the current child is updated.</li>
 *   <li>When the current child finishes, the next child is started on the following update, or
 *       immediately if the child finishes in its own {@code start()} method.</li>
 *   <li>The sequence finishes when all children have finished.</li>
 *   <li>Active {@link #cancel()} marks the sequence terminal, then asks its current child to
 *       cancel. Pre-start and terminal cancellation are no-ops.</li>
 * </ul>
 *
 * <p>Typical usage:</p>
 * <pre>{@code
 * TaskRunner runner = new TaskRunner();
 *
 * runner.enqueue(SequenceTask.of(
 *     new InstantTask(() -> log("start")),
 *     new WaitUntilTask(readySensor),
 *     new InstantTask(() -> log("done"))
 * ));
 * }</pre>
 *
 * <p>This is the standard Phoenix "do A, then B, then C" primitive for macros. It is also the
 * main building block used by helpers such as {@link Tasks#sequence(Task...)} and
 * {@link Tasks#branchOnOutcome(Task, Task, Task)}.</p>
 *
 * <p>A sequence is single-use, and each child position must contain a distinct Task instance.
 * Repeated behavior should construct a fresh sequence with fresh children.</p>
 */
public final class SequenceTask implements Task {

    private final List<Task> tasks = new ArrayList<>();
    private boolean startAttempted = false;
    private boolean started = false;
    private boolean cancelled = false;
    /**
     * Index of the current task; {@code -1} before the first task is started.
     */
    private int index = -1;

    /**
     * Create a sequence from a list of tasks.
     *
     * <p>The list is copied; subsequent modifications to {@code tasks} do not affect this
     * sequence.</p>
     *
     * @param tasks ordered list of distinct child tasks; must not be {@code null} or contain
     *              {@code null} or duplicate instances
     */
    public SequenceTask(List<Task> tasks) {
        if (tasks == null) {
            throw new IllegalArgumentException("tasks is required");
        }
        validateDistinctChildren(tasks);
        this.tasks.addAll(tasks);
    }

    /**
     * Convenience factory for a sequence from varargs.
     *
     * <p>Each element in {@code tasks} must be non-null and refer to a distinct Task instance. The
     * array is copied into an internal list.</p>
     *
     * @param tasks ordered distinct child tasks to run; must not be {@code null} or contain
     *              {@code null} or duplicate instances
     * @return a new {@link SequenceTask} running the given children in order
     */
    public static SequenceTask of(Task... tasks) {
        if (tasks == null) {
            throw new IllegalArgumentException("tasks is required");
        }
        List<Task> list = new ArrayList<>(tasks.length);
        for (Task t : tasks) {
            if (t == null) {
                throw new IllegalArgumentException("tasks must not contain null elements");
            }
            list.add(t);
        }
        return new SequenceTask(list);
    }

    /**
     * Convenience factory that obtains sequence children from suppliers.
     *
     * <p>Suppliers are invoked eagerly, exactly once each, while this method constructs the
     * sequence. They are not retained or invoked again when the sequence starts. Each supplier
     * must return a non-null, fresh Task instance distinct from every other child.</p>
     *
     * @param taskSuppliers non-null suppliers that each create a distinct, fresh Task
     * @return a {@link SequenceTask} using newly-created tasks
     */
    @SafeVarargs
    public static SequenceTask fromSuppliers(Supplier<Task>... taskSuppliers) {
        if (taskSuppliers == null) {
            throw new IllegalArgumentException("taskSuppliers is required");
        }
        List<Task> list = new ArrayList<>(taskSuppliers.length);
        for (Supplier<Task> supplier : taskSuppliers) {
            if (supplier == null) {
                throw new IllegalArgumentException("taskSuppliers must not contain null elements");
            }
            Task t = supplier.get();
            if (t == null) {
                throw new IllegalArgumentException("taskSuppliers must not return null");
            }
            list.add(t);
        }
        return new SequenceTask(list);
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
            throw TaskLifecycle.updateBeforeStart("SequenceTask");
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
    public Task getCurrentTask() {
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
                        "SequenceTask children must not contain null; found null at index " + i);
            }
            for (int previous = 0; previous < i; previous++) {
                if (child == children.get(previous)) {
                    throw new IllegalArgumentException(
                            "SequenceTask child at index " + i
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
                    "SequenceTask is single-use and start(...) was called more than once. "
                            + "Create a fresh task with its builder or macro method, a "
                            + "Supplier<Task>, or an OutputTaskFactory.");
        }
        startAttempted = true;
    }
}
