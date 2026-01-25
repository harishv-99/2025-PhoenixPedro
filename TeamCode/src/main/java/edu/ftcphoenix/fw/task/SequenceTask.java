package edu.ftcphoenix.fw.task;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A {@link Task} that runs a sequence of child tasks one after another.
 *
 * <p>Semantics:
 * <ul>
 *   <li>On {@link #start(LoopClock)}, the first child is started.</li>
 *   <li>On each {@link #update(LoopClock)}, the current child is updated.</li>
 *   <li>When the current child finishes, the next child is started on the
 *       following update (or immediately if the child finishes in its
 *       {@code start()} method).</li>
 *   <li>The sequence finishes when all children have finished.</li>
 * </ul>
 *
 * <p>Typical usage:
 * <pre>{@code
 * TaskRunner runner = new TaskRunner();
 *
 * runner.enqueue(SequenceTask.of(
 *     new InstantTask(() -> log("start")),
 *     new WaitUntilTask(() -> sensorReady()),
 *     new InstantTask(() -> log("done"))
 * ));
 * }</pre>
 */
public final class SequenceTask implements Task {

    private final List<Task> tasks = new ArrayList<Task>();

    private boolean started = false;
    /**
     * Index of current task; -1 before the first task is started.
     */
    private int index = -1;

    /**
     * Create a sequence from a list of tasks.
     *
     * <p>The list is copied; subsequent modifications to {@code tasks}
     * do not affect this sequence.</p>
     *
     * @param tasks ordered list of child tasks; must not be {@code null}
     */
    public SequenceTask(List<Task> tasks) {
        if (tasks == null) {
            throw new IllegalArgumentException("tasks is required");
        }
        this.tasks.addAll(tasks);
    }

    /**
     * Convenience factory for a sequence from varargs.
     *
     * <p>Each element in {@code tasks} must be non-null. The array is copied
     * into an internal list.</p>
     *
     * @param tasks ordered child tasks to run; must not be {@code null}
     * @return a new SequenceTask running the given children in order
     */
    public static SequenceTask of(Task... tasks) {
        if (tasks == null) {
            throw new IllegalArgumentException("tasks is required");
        }
        List<Task> list = new ArrayList<Task>(tasks.length);
        for (Task t : tasks) {
            if (t == null) {
                throw new IllegalArgumentException("tasks must not contain null elements");
            }
            list.add(t);
        }
        return new SequenceTask(list);
    }

    /**
     * Convenience factory that builds tasks lazily via suppliers.
     *
     * <p>This can be used to avoid reusing {@link Task} instances, since
     * each supplier is called once per sequence run to create a fresh task.</p>
     *
     * @param taskSuppliers suppliers that create new tasks
     * @return a SequenceTask using newly-created tasks for each run
     */
    @SafeVarargs
    public static SequenceTask fromSuppliers(Supplier<Task>... taskSuppliers) {
        if (taskSuppliers == null) {
            throw new IllegalArgumentException("taskSuppliers is required");
        }
        List<Task> list = new ArrayList<Task>(taskSuppliers.length);
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
     */
    @Override
    public void start(LoopClock clock) {
        started = true;
        index = -1;
        advanceToNextTask(clock);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void update(LoopClock clock) {
        if (!started) {
            // Defensive: if update is called before start, behave as if started now.
            start(clock);
            return;
        }

        Task current = getCurrentTask();
        if (current == null) {
            // No tasks; sequence is already complete.
            return;
        }

        // Update current task
        current.update(clock);

        // If it completed during update, advance to the next one.
        if (current.isComplete()) {
            advanceToNextTask(clock);
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean isComplete() {
        // Finished when we've advanced past the last child.
        return started && index >= tasks.size();
    }

    /**
     * Report the aggregated outcome for this sequence.
     *
     * <p>Semantics:</p>
     * <ul>
     *   <li>While the sequence is still running (before {@link #isComplete()}
     *       returns {@code true}), this returns {@link TaskOutcome#NOT_DONE}.</li>
     *   <li>Once all children have completed, if <b>any</b> child reports
     *       {@link TaskOutcome#TIMEOUT}, this returns {@link TaskOutcome#TIMEOUT}.</li>
     *   <li>Otherwise this returns {@link TaskOutcome#SUCCESS}, regardless of
     *       whether individual children report {@link TaskOutcome#SUCCESS} or
     *       {@link TaskOutcome#UNKNOWN}.</li>
     * </ul>
     *
     * <p>This behavior is designed so that higher-level code (for example,
     * {@code Tasks.branchOnOutcome(...)}) can treat a sequence as
     * "timed out" if any of its children timed out.</p>
     */
    @Override
    public TaskOutcome getOutcome() {
        // While the sequence is still running, report NOT_DONE.
        if (!isComplete()) {
            return TaskOutcome.NOT_DONE;
        }

        // Once complete, propagate TIMEOUT if any child timed out; otherwise SUCCESS.
        for (int i = 0; i < tasks.size(); i++) {
            TaskOutcome child = tasks.get(i).getOutcome();
            if (child == TaskOutcome.TIMEOUT) {
                return TaskOutcome.TIMEOUT;
            }
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
        String p = (prefix == null || prefix.isEmpty()) ? "sequence" : prefix;

        dbg.addData(p + ".started", started)
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
     * Advance to the next task, starting it immediately.
     *
     * <p>If the next task finishes in its {@code start()}, this method will
     * continue advancing until it finds a non-finished task or runs out of
     * tasks.</p>
     */
    private void advanceToNextTask(LoopClock clock) {
        while (true) {
            index++;

            if (index >= tasks.size()) {
                // No more tasks; sequence is finished.
                return;
            }

            Task next = tasks.get(index);
            next.start(clock);

            // If this task finished immediately in start(), loop to pick the next one.
            if (!next.isComplete()) {
                return;
            }
        }
    }
}
