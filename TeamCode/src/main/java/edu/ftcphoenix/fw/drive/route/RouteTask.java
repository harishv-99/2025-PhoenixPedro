package edu.ftcphoenix.fw.drive.route;

import java.util.Objects;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.task.Tasks;

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
 * <p>When a bounded mechanism Task should run only while this route is active, use this route as
 * the deadline in {@link Tasks#parallelDeadline(Task, Task...)}. The companion must make its own
 * active cancellation safe; persistent mechanism requests should remain capability or service
 * state instead of a forever-running companion.</p>
 *
 * <p>A {@code RouteTask} instance is single-use. Create a fresh task with
 * {@link RouteTasks#follow(String, RouteFollower, Object, Config)},
 * {@link RouteTasks#followBuiltAtStart(String, RouteFollower, Supplier, Config)}, or a
 * {@code Supplier<Task>} each time a route should run. The built-at-start form resolves its route
 * factory exactly once at this Task's own {@link #start(LoopClock)} boundary, which lets a
 * robot-owned path factory use current pose or vision state without moving route-library types into
 * framework core.</p>
 *
 * <p>Typical usage:</p>
 * <pre>{@code
 * RouteTask.Config cfg = new RouteTask.Config();
 * cfg.timeoutSec = 4.0;
 *
 * Task auto = Tasks.sequence(
 *         RouteTasks.follow("outbound", pedroAdapter, outboundPath, cfg),
 *         Tasks.runOnce(scoringSupervisor::requestSingleShot),
 *         RouteTasks.followBuiltAtStart(
 *                 "return",
 *                 pedroAdapter,
 *                 () -> pathFactory.buildReturnFromCurrentPose(returnPose),
 *                 cfg)
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
    private final Supplier<? extends R> routeFactory;
    private final Config cfg;

    private R route;

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
        this(debugName,
                follower,
                Objects.requireNonNull(route, "route"),
                null,
                cfg);
    }

    /**
     * Creates the internal built-at-start form used by {@link RouteTasks}.
     *
     * <p>Keeping this as a package-private factory avoids a public constructor overload whose
     * {@code Supplier} argument could be confused with an eager route type that is itself a
     * {@code Supplier}.</p>
     */
    static <R> RouteTask<R> builtAtStart(String debugName,
                                         RouteFollower<R> follower,
                                         Supplier<? extends R> routeFactory,
                                         Config cfg) {
        return new RouteTask<R>(
                debugName,
                follower,
                null,
                Objects.requireNonNull(routeFactory, "routeFactory"),
                cfg);
    }

    /** Initialize exactly one of the eager-route or built-at-start route sources. */
    private RouteTask(String debugName,
                      RouteFollower<R> follower,
                      R route,
                      Supplier<? extends R> routeFactory,
                      Config cfg) {
        this.debugName = (debugName != null && !debugName.isEmpty()) ? debugName : "RouteTask";
        this.follower = Objects.requireNonNull(follower, "follower");
        this.route = route;
        this.routeFactory = routeFactory;
        if ((route == null) == (routeFactory == null)) {
            throw new IllegalArgumentException(
                    "RouteTask requires exactly one eager route or built-at-start route factory");
        }
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
                    + "RouteTasks.follow(...), RouteTasks.followBuiltAtStart(...), or a "
                    + "Supplier<Task> for each run.");
        }
        startAttempted = true;
        started = true;
        complete = false;
        outcome = TaskOutcome.NOT_DONE;
        routeStatus = RouteStatus.NOT_STARTED;
        startTimeSec = (clock != null) ? clock.nowSec() : 0.0;

        if (routeFactory != null) {
            R builtRoute;
            try {
                builtRoute = routeFactory.get();
            } catch (RuntimeException factoryFailure) {
                if (!complete) {
                    markFailedTerminal();
                }
                throw routeFactoryFailure(factoryFailure);
            }

            // A factory may re-enter robot policy that cancels this Task. Active cancellation is
            // terminal, so do not begin following a route after that cancellation returns.
            if (complete) {
                return;
            }
            if (builtRoute == null) {
                markFailedTerminal();
                throw new IllegalStateException(
                        "RouteTasks.followBuiltAtStart(...) route factory returned null for "
                                + "RouteTask '" + debugName + "'. Return a non-null route object.");
            }
            route = builtRoute;
        }

        try {
            execution = follower.follow(route);
        } catch (RuntimeException startFailure) {
            if (!complete) {
                markFailedTerminal();
            }
            throw startFailure;
        }
        if (execution == null) {
            if (!complete) {
                markFailedTerminal();
            }
            throw new IllegalStateException("RouteFollower.follow(...) returned null for RouteTask '"
                    + debugName + "'. Return a RouteExecution for the route that was started.");
        }
        if (complete) {
            // Cancellation may re-enter from follower initialization before follow(...) returns its
            // exact handle. Apply that retained cancellation to an active execution once, while
            // still preserving a truthful terminal status the follower may have returned.
            finishReentrantCancellationAfterFollow();
            return;
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
        if (execution == null) {
            // start(clock) records an active Task before resolving a built-at-start route or
            // acquiring its RouteExecution. Reentrant cancellation is terminal immediately; the
            // start path either avoids follow(...) or cancels the exact handle as soon as it is
            // returned.
            markCancelledTerminal();
            return;
        }
        // A root-owned heartbeat may have terminalized this exact execution since the Task's
        // previous update. Preserve that reason instead of relabeling a completed/replaced route
        // as Task cancellation.
        observeAndApplyStatus();
        if (complete) {
            return;
        }
        markCancelledTerminal();
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
                .addData(p + ".routeSource", routeFactory == null ? "eager" : "builtAtStart")
                .addData(p + ".routeClass",
                        route == null ? "pending" : route.getClass().getSimpleName())
                .addData(p + ".timeoutSec", cfg.timeoutSec)
                .addData(p + ".startedAtSec", startTimeSec);
    }

    /** Add Task identity and actionable guidance while retaining the factory's original failure. */
    private IllegalStateException routeFactoryFailure(RuntimeException factoryFailure) {
        String causeMessage = factoryFailure.getMessage();
        String causeDetail = (causeMessage == null || causeMessage.isEmpty())
                ? factoryFailure.getClass().getSimpleName()
                : causeMessage;
        return new IllegalStateException(
                "RouteTask '" + debugName + "' could not build its route at start. "
                        + "Check the route factory and the live state it reads. Cause: "
                        + causeDetail,
                factoryFailure);
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

    /** Finish cancellation that re-entered before follower.follow(...) returned its exact handle. */
    private void finishReentrantCancellationAfterFollow() {
        RouteStatus returnedStatus;
        try {
            returnedStatus = readExecutionStatus();
        } catch (RuntimeException statusFailure) {
            if (!complete) {
                markFailedTerminal();
            }
            execution.failClosed(statusFailure);
            throw statusFailure;
        }
        if (returnedStatus == RouteStatus.ACTIVE) {
            execution.cancelAfterActiveObservation();
            return;
        }
        try {
            applyObservedStatus(returnedStatus);
        } catch (RuntimeException statusFailure) {
            if (!complete) {
                markFailedTerminal();
            }
            execution.failClosed(statusFailure);
            throw statusFailure;
        }
    }

    private void markFailedTerminal() {
        complete = true;
        outcome = TaskOutcome.CANCELLED;
        routeStatus = RouteStatus.FAILED;
    }

    /** Mark active Task cancellation terminal before invoking any external cleanup hook. */
    private void markCancelledTerminal() {
        complete = true;
        outcome = TaskOutcome.CANCELLED;
        routeStatus = RouteStatus.CANCELLED;
    }

    private static void addSuppressedIfDistinct(RuntimeException primary,
                                                RuntimeException secondary) {
        if (secondary != primary) {
            primary.addSuppressed(secondary);
        }
    }
}
