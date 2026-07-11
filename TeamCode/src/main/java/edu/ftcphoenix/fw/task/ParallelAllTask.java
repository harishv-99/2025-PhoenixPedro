package edu.ftcphoenix.fw.task;

import java.util.ArrayList;
import java.util.List;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A {@link Task} that runs multiple child tasks in parallel and finishes only when <b>all</b>
 * children have finished.
 *
 * <p>Semantics:</p>
 * <ul>
 *   <li>On {@link #start(LoopClock)}, all children are started.</li>
 *   <li>On each {@link #update(LoopClock)}, all children that are not yet finished are updated
 *       once.</li>
 *   <li>The parallel group finishes when every child task reports complete.</li>
 *   <li>Active {@link #cancel()} marks the group terminal, then asks every child to cancel.
 *       Pre-start and terminal child cancellation are no-ops.</li>
 * </ul>
 *
 * <p>Typical usage:</p>
 * <pre>{@code
 * TaskRunner runner = new TaskRunner();
 * runner.enqueue(ParallelAllTask.of(
 *     new WaitUntilTask(sensorReady, 1.5),
 *     new OutputForSecondsTask("intakePulse", 1.0, 0.20)
 * ));
 * }</pre>
 *
 * <p>A parallel group is single-use, and each child position must contain a distinct Task
 * instance. Repeated behavior should construct a fresh group with fresh children.</p>
 */
public final class ParallelAllTask implements Task {

    private final List<Task> tasks = new ArrayList<>();
    private boolean startAttempted = false;
    private boolean started = false;
    private boolean finished = false;
    private boolean cancelled = false;

    /**
     * Create a parallel group from a list of tasks.
     *
     * <p>The list is copied; subsequent modifications to {@code tasks} do not affect this parallel
     * group.</p>
     *
     * @param tasks list of distinct child tasks to run in parallel; must not be {@code null} or
     *              contain {@code null} or duplicate instances
     */
    public ParallelAllTask(List<Task> tasks) {
        if (tasks == null) {
            throw new IllegalArgumentException("tasks is required");
        }
        validateDistinctChildren(tasks);
        this.tasks.addAll(tasks);
    }

    /**
     * Convenience factory for a parallel group from varargs.
     *
     * @param tasks distinct child tasks to run in parallel; must not be {@code null} or contain
     *              {@code null} or duplicate instances
     * @return a new {@link ParallelAllTask} containing the given children
     */
    public static ParallelAllTask of(Task... tasks) {
        if (tasks == null) {
            throw new IllegalArgumentException("tasks is required");
        }
        List<Task> list = new ArrayList<>(tasks.length);
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
     *
     * <p>Starts every child task once. If all children finish immediately during their own
     * {@code start()} calls, the parallel group also becomes complete immediately.</p>
     */
    @Override
    public void start(LoopClock clock) {
        markStartAttempt();
        if (started) {
            return;
        }
        started = true;
        finished = false;
        cancelled = false;
        for (Task t : tasks) {
            if (finished) {
                // A child's start callback may have reentrantly cancelled this group.
                break;
            }
            t.start(clock);
        }
        if (!finished && allFinished()) {
            finished = true;
        }
    }

    /**
     * {@inheritDoc}
     *
     * <p>Updates every child that is not yet complete. Children that have already finished are not
     * updated again.</p>
     */
    @Override
    public void update(LoopClock clock) {
        if (!started) {
            throw TaskLifecycle.updateBeforeStart("ParallelAllTask");
        }
        if (finished) {
            return;
        }
        for (Task t : tasks) {
            if (finished) {
                break;
            }
            boolean childComplete = t.isComplete();
            if (finished) {
                break;
            }
            if (!childComplete) {
                t.update(clock);
            }
        }
        if (!finished && allFinished()) {
            finished = true;
        }
    }

    /**
     * {@inheritDoc}
     *
     * <p>The group becomes terminal first, then cancellation is best-effort propagated to every
     * child. Pre-start and terminal child cancellation are defined no-ops, so this avoids querying
     * a child's possibly-failed completion hook during cleanup. If multiple child cleanup hooks
     * throw, the later failures are suppressed on the first.</p>
     */
    @Override
    public void cancel() {
        if (!started || finished) {
            return;
        }

        // Establish terminal state before child cleanup and still give every child a cleanup
        // attempt when an earlier hook fails.
        cancelled = true;
        finished = true;
        RuntimeException firstFailure = null;
        for (Task t : tasks) {
            try {
                // A child's isComplete() may be the lifecycle hook whose failure led here.
                t.cancel();
            } catch (RuntimeException failure) {
                if (firstFailure == null) {
                    firstFailure = failure;
                } else if (failure != firstFailure) {
                    firstFailure.addSuppressed(failure);
                }
            }
        }
        if (firstFailure != null) {
            throw firstFailure;
        }
    }

    /**
     * {@inheritDoc}
     *
     * <p>The parallel group is complete once all children have completed, or immediately after a
     * direct cancellation.</p>
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
     *   <li>While the group is still running, this returns {@link TaskOutcome#NOT_DONE}.</li>
     *   <li>If the group itself was cancelled, this returns {@link TaskOutcome#CANCELLED}.</li>
     *   <li>Otherwise, once all children have completed, if <b>any</b> child reports
     *       {@link TaskOutcome#TIMEOUT}, this returns {@link TaskOutcome#TIMEOUT}.</li>
     *   <li>Otherwise this returns {@link TaskOutcome#SUCCESS}, regardless of whether individual
     *       children report {@link TaskOutcome#SUCCESS} or {@link TaskOutcome#UNKNOWN}.</li>
     * </ul>
     */
    @Override
    public TaskOutcome getOutcome() {
        if (!finished) {
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
     * <p>Dumps aggregate state plus each child task under a numbered child prefix.</p>
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "parallelAll" : prefix;
        int completeCount = 0;
        for (Task task : tasks) {
            if (task.isComplete()) {
                completeCount++;
            }
        }
        dbg.addData(p + ".started", started)
                .addData(p + ".finished", finished)
                .addData(p + ".cancelled", cancelled)
                .addData(p + ".size", tasks.size())
                .addData(p + ".completeCount", completeCount)
                .addData(p + ".complete", isComplete())
                .addData(p + ".outcome", getOutcome());
        for (int i = 0; i < tasks.size(); i++) {
            tasks.get(i).debugDump(dbg, p + ".child" + i);
        }
    }

    /**
     * @return true once every child task reports complete.
     */
    private boolean allFinished() {
        for (Task t : tasks) {
            if (!t.isComplete()) {
                return false;
            }
            if (finished) {
                return true;
            }
        }
        return true;
    }

    /** Reject child aliases that would start or update one stateful Task instance twice. */
    private static void validateDistinctChildren(List<Task> children) {
        for (int i = 0; i < children.size(); i++) {
            Task child = children.get(i);
            if (child == null) {
                throw new IllegalArgumentException(
                        "ParallelAllTask children must not contain null; found null at index " + i);
            }
            for (int previous = 0; previous < i; previous++) {
                if (child == children.get(previous)) {
                    throw new IllegalArgumentException(
                            "ParallelAllTask child at index " + i
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
                    "ParallelAllTask is single-use and start(...) was called more than once. "
                            + "Create a fresh task with its builder or macro method, a "
                            + "Supplier<Task>, or an OutputTaskFactory.");
        }
        startAttempted = true;
    }
}
