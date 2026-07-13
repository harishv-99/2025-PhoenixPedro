package edu.ftcphoenix.fw.drive.route;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;

/**
 * Autonomous-style {@link Task} wrapper around an external {@link RouteFollower}.
 *
 * <p>This lets Phoenix task runners sequence route following together with mechanism actions,
 * waits, and other tasks without the framework taking ownership of a specific route library.</p>
 *
 * <p>The Task retains the exact {@link RouteExecution} returned for its start. Use
 * {@link #getRouteStatus()} when routine policy needs the precise terminal reason. The broader
 * {@link #getOutcome()} maps completed routes to success, follower or Task timeouts to timeout,
 * and other abnormal endings to the fail-closed cancelled bucket.</p>
 *
 * <p>This Task calls the follower's update hook while it is active, but it is not a persistent
 * lifecycle owner. An external follower that must keep updating during hold-end, mechanism, or
 * wait phases needs one composition-root heartbeat; its adapter should deduplicate the Task's
 * same-cycle call.</p>
 *
 * <p>A {@code RouteTask} instance is single-use. Create a fresh task with
 * {@link RouteTasks#follow(String, RouteFollower, Object, Config)} or a
 * {@code Supplier<Task>} each time a route should run.</p>
 *
 * <p>Typical usage:</p>
 * <pre>{@code
 * RouteTask.Config cfg = new RouteTask.Config();
 * cfg.timeoutSec = 4.0;
 *
 * Task auto = Tasks.sequence(
 *         RouteTasks.follow("outbound", pedroAdapter, outboundPath, cfg),
 *         Tasks.runOnce(scoringSupervisor::requestSingleShot),
 *         RouteTasks.follow("return", pedroAdapter, returnPath, cfg)
 * );
 * }</pre>
 * <p>This snippet demonstrates Task composition only. Robot-owned policy must gate any later
 * position-dependent action on the route's precise result.</p>
 *
 * @param <R> route object type understood by the wrapped {@link RouteFollower}
 */
public final class RouteTask<R> implements Task {

    /**
     * Task-level configuration for a route follow.
     */
    public static final class Config {

        /**
         * Maximum time allowed for the route follow before timing out.
         *
         * <p>Set {@code <= 0} to disable the timeout.</p>
         */
        public double timeoutSec = 10.0;
    }

    private final String debugName;
    private final RouteFollower<R> follower;
    private final R route;
    private final Config cfg;

    private boolean startAttempted = false;
    private boolean started = false;
    private boolean complete = false;
    private TaskOutcome outcome = TaskOutcome.NOT_DONE;
    private RouteStatus routeStatus = RouteStatus.NOT_STARTED;
    private RouteExecution execution;
    private double startTimeSec = 0.0;

    /**
     * Creates a named route-follow task.
     *
     * @param debugName human-readable task label used for debugging
     * @param follower route follower adapter to command
     * @param route route object to follow
     * @param cfg task-level timeout config; when {@code null}, defaults are used
     */
    public RouteTask(String debugName,
                     RouteFollower<R> follower,
                     R route,
                     Config cfg) {
        this.debugName = (debugName != null && !debugName.isEmpty()) ? debugName : "RouteTask";
        this.follower = Objects.requireNonNull(follower, "follower");
        this.route = Objects.requireNonNull(route, "route");
        this.cfg = (cfg != null) ? cfg : new Config();
    }

    /**
     * Creates a route-follow task with the default debug name.
     *
     * @param follower route follower adapter to command
     * @param route route object to follow
     * @param cfg task-level timeout config; when {@code null}, defaults are used
     */
    public RouteTask(RouteFollower<R> follower,
                     R route,
                     Config cfg) {
        this("RouteTask", follower, route, cfg);
    }

    @Override
    public String getDebugName() {
        return debugName;
    }

    @Override
    public void start(LoopClock clock) {
        if (startAttempted) {
            throw new IllegalStateException("RouteTask '" + debugName
                    + "' is single-use and has already been started. Create a fresh task with "
                    + "RouteTasks.follow(...) or a Supplier<Task> for each run.");
        }
        startAttempted = true;
        started = true;
        complete = false;
        outcome = TaskOutcome.NOT_DONE;
        routeStatus = RouteStatus.NOT_STARTED;
        startTimeSec = (clock != null) ? clock.nowSec() : 0.0;
        try {
            execution = follower.follow(route);
        } catch (RuntimeException startFailure) {
            markFailedTerminal();
            throw startFailure;
        }
        if (execution == null) {
            markFailedTerminal();
            throw new IllegalStateException("RouteFollower.follow(...) returned null for RouteTask '"
                    + debugName + "'. Return a RouteExecution for the route that was started.");
        }
        observeAndApplyStatus();
    }

    @Override
    public void update(LoopClock clock) {
        if (!started) {
            throw new IllegalStateException("RouteTask '" + debugName + "' cannot be updated "
                    + "before start(clock). Start it first, normally by enqueueing it in a "
                    + "TaskRunner.");
        }
        if (complete) {
            return;
        }
        if (clock == null) {
            return;
        }

        observeAndApplyStatus();
        if (complete) {
            return;
        }

        try {
            follower.update(clock);
        } catch (RuntimeException updateFailure) {
            retainStatusAfterUpdateFailure(updateFailure);
            throw updateFailure;
        }
        if (complete) {
            return;
        }

        observeAndApplyStatus();
        if (complete) {
            return;
        }

        if (cfg.timeoutSec > 0.0 && (clock.nowSec() - startTimeSec) > cfg.timeoutSec) {
            complete = true;
            outcome = TaskOutcome.TIMEOUT;
            routeStatus = RouteStatus.TASK_TIMEOUT;
            execution.cancelForTimeout();
        }
    }

    @Override
    public void cancel() {
        if (!started || complete) {
            return;
        }
        // A root-owned heartbeat may have terminalized this exact execution since the Task's
        // previous update. Preserve that reason instead of relabeling a completed/replaced route
        // as Task cancellation.
        observeAndApplyStatus();
        if (complete) {
            return;
        }
        complete = true;
        outcome = TaskOutcome.CANCELLED;
        routeStatus = RouteStatus.CANCELLED;
        if (execution != null) {
            execution.cancelAfterActiveObservation();
        }
    }

    @Override
    public boolean isComplete() {
        return complete;
    }

    @Override
    public TaskOutcome getOutcome() {
        return outcome;
    }

    /**
     * Returns the precise backend-neutral status for this route attempt.
     *
     * <p>This preserves why a route ended even when the broader {@link TaskOutcome} maps several
     * fail-closed terminal reasons to {@link TaskOutcome#CANCELLED}.</p>
     *
     * @return current or retained terminal route status
     */
    public RouteStatus getRouteStatus() {
        return routeStatus;
    }

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        Task.super.debugDump(dbg, prefix);
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "task" : prefix;
        dbg.addData(p + ".routeStatus", routeStatus)
                .addData(p + ".routeClass", route.getClass().getSimpleName())
                .addData(p + ".timeoutSec", cfg.timeoutSec)
                .addData(p + ".startedAtSec", startTimeSec);
    }

    private RouteStatus readExecutionStatus() {
        RouteStatus status;
        try {
            status = execution.status();
        } catch (RuntimeException failure) {
            throw new IllegalStateException("RouteTask '" + debugName
                    + "' could not read its RouteExecution status. " + failure.getMessage(),
                    failure);
        }
        if (status == null) {
            throw new IllegalStateException("RouteExecution.status() returned null for RouteTask '"
                    + debugName + "'. Return a backend-neutral RouteStatus.");
        }
        return status;
    }

    private void applyObservedStatus(RouteStatus observedStatus) {
        routeStatus = observedStatus;
        switch (observedStatus) {
            case NOT_STARTED:
                throw new IllegalStateException("RouteFollower.follow(...) returned a NOT_STARTED "
                        + "execution for RouteTask '" + debugName + "'. follow(...) must begin the "
                        + "route synchronously and return ACTIVE or a retained terminal status.");
            case ACTIVE:
                return;
            case COMPLETED:
                complete = true;
                outcome = TaskOutcome.SUCCESS;
                return;
            case FOLLOWER_TIMEOUT_OR_STALL:
            case TASK_TIMEOUT:
                complete = true;
                outcome = TaskOutcome.TIMEOUT;
                return;
            case INTERRUPTED:
            case REPLACED:
            case CANCELLED:
            case FAILED:
            case UNKNOWN_TERMINAL:
                complete = true;
                outcome = TaskOutcome.CANCELLED;
                return;
            default:
                throw new IllegalStateException("Unhandled RouteStatus " + observedStatus
                        + " for RouteTask '" + debugName + "'.");
        }
    }

    private void retainStatusAfterUpdateFailure(RuntimeException updateFailure) {
        try {
            applyObservedStatus(readExecutionStatus());
        } catch (RuntimeException statusFailure) {
            markFailedTerminal();
            execution.failClosed(updateFailure);
            addSuppressedIfDistinct(updateFailure, statusFailure);
            return;
        }

        if (!complete) {
            // The exact execution was still active when its owner threw. With no more-specific
            // retained terminal evidence, fail closed and keep the update failure primary.
            markFailedTerminal();
            execution.failClosed(updateFailure);
        }
    }

    private void observeAndApplyStatus() {
        try {
            applyObservedStatus(readExecutionStatus());
        } catch (RuntimeException statusFailure) {
            markFailedTerminal();
            execution.failClosed(statusFailure);
            throw statusFailure;
        }
    }

    private void markFailedTerminal() {
        complete = true;
        outcome = TaskOutcome.CANCELLED;
        routeStatus = RouteStatus.FAILED;
    }

    private static void addSuppressedIfDistinct(RuntimeException primary,
                                                RuntimeException secondary) {
        if (secondary != primary) {
            primary.addSuppressed(secondary);
        }
    }
}
