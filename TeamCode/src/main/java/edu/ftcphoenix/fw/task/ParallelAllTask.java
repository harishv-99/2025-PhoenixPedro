package edu.ftcphoenix.fw.task;

import java.util.ArrayList;
import java.util.List;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A {@link Task} that runs multiple child tasks in parallel and finishes
 * only when <b>all</b> children have finished.
 *
 * <p>Semantics:</p>
 * <ul>
 *   <li>On {@link #start(LoopClock)}, all children are started.</li>
 *   <li>On each {@link #update(LoopClock)}, all children that are not yet
 *       finished are updated once.</li>
 *   <li>The parallel group finishes when every child task reports complete.</li>
 * </ul>
 *
 * <p>Note that this class does not support cancellation semantics by itself;
 * if you wish to stop a running parallel group early, you should implement
 * that logic in your own code (for example, by not calling
 * {@link #update(LoopClock)} any longer).</p>
 */
public final class ParallelAllTask implements Task {

    private final List<Task> tasks = new ArrayList<>();

    private boolean started = false;
    private boolean finished = false;

    /**
     * Create a parallel group from a list of tasks.
     *
     * <p>The list is copied; subsequent modifications to {@code tasks}
     * do not affect this parallel group.</p>
     *
     * @param tasks list of child tasks to run in parallel; must not be {@code null}
     */
    public ParallelAllTask(List<Task> tasks) {
        if (tasks == null) {
            throw new IllegalArgumentException("tasks is required");
        }
        this.tasks.addAll(tasks);
    }

    /**
     * Convenience factory for a parallel group from varargs.
     *
     * @param tasks child tasks to run in parallel; must not be {@code null}
     *              and must not contain {@code null} elements
     * @return a new {@link ParallelAllTask} containing the given children
     */
    public static ParallelAllTask of(Task... tasks) {
        if (tasks == null) {
            throw new IllegalArgumentException("tasks is required");
        }
        List<Task> list = new ArrayList<Task>(tasks.length);
        for (Task t : tasks) {
            if (t == null) {
                throw new IllegalArgumentException("task element must not be null");
            }
            list.add(t);
        }
        return new ParallelAllTask(list);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void start(LoopClock clock) {
        if (started) {
            return; // single-use; ignore repeated start calls
        }
        started = true;

        // Start all children.
        for (int i = 0; i < tasks.size(); i++) {
            Task t = tasks.get(i);
            t.start(clock);
        }

        // If all children finished immediately in start(), mark finished.
        if (allFinished()) {
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

        // Update all children that are not yet complete.
        for (int i = 0; i < tasks.size(); i++) {
            Task t = tasks.get(i);
            if (!t.isComplete()) {
                t.update(clock);
            }
        }

        // Check if all have now completed.
        if (allFinished()) {
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
     * Report the aggregated outcome for this parallel group.
     *
     * <p>Semantics:</p>
     * <ul>
     *   <li>While the group is still running (before {@link #isComplete()}
     *       returns {@code true}), this returns {@link TaskOutcome#NOT_DONE}.</li>
     *   <li>Once all children have completed, if <b>any</b> child reports
     *       {@link TaskOutcome#TIMEOUT}, this returns {@link TaskOutcome#TIMEOUT}.</li>
     *   <li>Otherwise this returns {@link TaskOutcome#SUCCESS}, regardless of
     *       whether individual children report {@link TaskOutcome#SUCCESS} or
     *       {@link TaskOutcome#UNKNOWN}.</li>
     * </ul>
     *
     * <p>This behavior is designed so that higher-level code (for example,
     * {@code Tasks.branchOnOutcome(...)}) can treat a parallel group as
     * "timed out" if any of its children timed out.</p>
     */
    @Override
    public TaskOutcome getOutcome() {
        if (!finished) {
            return TaskOutcome.NOT_DONE;
        }

        // If any child task reports a timeout, propagate TIMEOUT so that
        // callers can branch on it (for example, using Tasks.branchOnOutcome).
        for (int i = 0; i < tasks.size(); i++) {
            TaskOutcome childOutcome = tasks.get(i).getOutcome();
            if (childOutcome == TaskOutcome.TIMEOUT) {
                return TaskOutcome.TIMEOUT;
            }
        }

        // Otherwise treat the parallel group as a normal success. Individual
        // children may still report UNKNOWN or SUCCESS, but since no timeout
        // occurred we consider the group successful for control-flow purposes.
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
        String p = (prefix == null || prefix.isEmpty()) ? "parallelAll" : prefix;

        int completeCount = 0;
        for (int i = 0; i < tasks.size(); i++) {
            if (tasks.get(i).isComplete()) {
                completeCount++;
            }
        }

        dbg.addData(p + ".started", started)
                .addData(p + ".finished", finished)
                .addData(p + ".size", tasks.size())
                .addData(p + ".completeCount", completeCount)
                .addData(p + ".complete", isComplete())
                .addData(p + ".outcome", getOutcome());

        for (int i = 0; i < tasks.size(); i++) {
            Task t = tasks.get(i);
            String childPrefix = p + ".child" + i;
            t.debugDump(dbg, childPrefix);
        }
    }

    // --------------------------------------------------------------------
    // Internal helpers
    // --------------------------------------------------------------------

    private boolean allFinished() {
        for (int i = 0; i < tasks.size(); i++) {
            if (!tasks.get(i).isComplete()) {
                return false;
            }
        }
        return true;
    }
}
