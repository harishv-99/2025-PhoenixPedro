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
 * {@link RouteTasks#follow(String, RouteFollower, Object, double)},
 * {@link RouteTasks#followWithoutTaskTimeout(String, RouteFollower, Object)},
 * {@link RouteTasks#followBuiltAtStart(String, RouteFollower, Supplier, double)},
 * {@link RouteTasks#followBuiltAtStartWithoutTaskTimeout(String, RouteFollower, Supplier)}, or a
 * {@code Supplier<Task>} each time a route should run. The built-at-start forms resolve their route
 * factory exactly once at this Task's own {@link #start(LoopClock)} boundary, which lets a
 * robot-owned path factory use current pose or vision state without moving route-library types
 * into framework core.</p>
 *
 * <p>Typical usage:</p>
 * <pre>{@code
 * RouteTask&lt;MyRoute&gt; outbound =
 *         RouteTasks.follow("outbound", routeAdapter, outboundPath, 4.0);
 * RouteTask&lt;MyRoute&gt; livePoseReturn = RouteTasks.followBuiltAtStart(
 *         "return",
 *         routeAdapter,
 *         () -> pathFactory.buildReturnFromCurrentPose(returnPose),
 *         4.0);
 * }</pre>
 * <p>Pass these fresh status-bearing Tasks and the mechanism action to a robot-owned routine
 * helper. That policy must gate any position-dependent action on the route's precise result;
 * generic sequence composition does not choose continue, fallback, or abort semantics.</p>
 *
 * @param <R> route object type understood by the wrapped {@link RouteFollower}
 */
public final class RouteTask<R> implements Task {

    private final String debugName;
    private final RouteFollower<R> follower;
    private final Supplier<? extends R> routeFactory;
    private final boolean taskTimeoutEnabled;
    private final double taskTimeoutSec;

    private R route;

    private boolean startAttempted = false;
    private boolean started = false;
    private boolean complete = false;
    private boolean statusObservationInProgress = false;
    private TaskOutcome outcome = TaskOutcome.NOT_DONE;
    private RouteStatus routeStatus = RouteStatus.NOT_STARTED;
    private RouteExecution execution;
    private double startTimeSec = 0.0;

    /** Creates the package-internal eager form selected only through {@link RouteTasks}. */
    static <R> RouteTask<R> eager(String debugName,
                                  RouteFollower<R> follower,
                                  R route,
                                  boolean taskTimeoutEnabled,
                                  double taskTimeoutSec) {
        return new RouteTask<R>(
                debugName,
                follower,
                Objects.requireNonNull(route, "route"),
                null,
                taskTimeoutEnabled,
                taskTimeoutSec);
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
                                         boolean taskTimeoutEnabled,
                                         double taskTimeoutSec) {
        return new RouteTask<R>(
                debugName,
                follower,
                null,
                Objects.requireNonNull(routeFactory, "routeFactory"),
                taskTimeoutEnabled,
                taskTimeoutSec);
    }

    /** Initialize exactly one of the eager-route or built-at-start route sources. */
    private RouteTask(String debugName,
                      RouteFollower<R> follower,
                      R route,
                      Supplier<? extends R> routeFactory,
                      boolean taskTimeoutEnabled,
                      double taskTimeoutSec) {
        this.debugName = requireDebugName(debugName);
        this.taskTimeoutEnabled = taskTimeoutEnabled;
        this.taskTimeoutSec = taskTimeoutEnabled
                ? requireTaskTimeoutSec(taskTimeoutSec, routeFactory != null)
                : 0.0;
        this.follower = Objects.requireNonNull(follower, "follower");
        this.route = route;
        this.routeFactory = routeFactory;
        if ((route == null) == (routeFactory == null)) {
            throw new IllegalArgumentException(
                    "RouteTask requires exactly one eager route or built-at-start route factory");
        }
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
                    + "the matching RouteTasks factory or a Supplier<Task> for each run.");
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

        if (taskTimeoutEnabled
                && (clock.nowSec() - startTimeSec) >= taskTimeoutSec) {
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
     * fail-closed terminal reasons to {@link TaskOutcome#CANCELLED}. If the retained execution is
     * still active in this Task's cache, this method observes its current status first. That
     * observation may terminalize this Task. If the execution supplies an invalid status or throws,
     * the Task fails closed and this method throws rather than returning stale policy input. This
     * keeps policy truthful when a composition-root follower heartbeat terminalized the execution
     * before the Task's later phase update.</p>
     *
     * @return current or retained terminal route status
     * @throws IllegalStateException if the retained execution's current status is null, invalid, or
     *                               cannot be read
     */
    public RouteStatus getRouteStatus() {
        // A composition-root follower heartbeat may terminalize the exact execution before this
        // Task receives its phase update. Return the current execution truth so robot policy can
        // distinguish that terminal reason from a cancellation it is about to apply.
        if (started && !complete && execution != null) {
            observeAndApplyStatus();
        }
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
                .addData(p + ".timeoutSec", taskTimeoutDebugValue())
                .addData(p + ".startedAtSec", startTimeSec);
    }

    /** Return a readable debug value without exposing a numeric no-timeout sentinel. */
    private Object taskTimeoutDebugValue() {
        return taskTimeoutEnabled ? Double.valueOf(taskTimeoutSec) : "none";
    }

    /** Require the diagnostic identity needed for actionable route status and failures. */
    private static String requireDebugName(String debugName) {
        if (debugName == null || debugName.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "RouteTasks debugName must be nonblank so this route attempt can be "
                            + "identified, got '" + debugName + "'");
        }
        return debugName;
    }

    /** Validate the explicit Task-owned deadline selected by a bounded route factory. */
    private static double requireTaskTimeoutSec(double taskTimeoutSec, boolean builtAtStart) {
        if (!Double.isFinite(taskTimeoutSec) || taskTimeoutSec <= 0.0) {
            String noTimeoutFactory = builtAtStart
                    ? "RouteTasks.followBuiltAtStartWithoutTaskTimeout(...)"
                    : "RouteTasks.followWithoutTaskTimeout(...)";
            throw new IllegalArgumentException(
                    "RouteTasks taskTimeoutSec must be finite and > 0 seconds, got "
                            + taskTimeoutSec + ". Use " + noTimeoutFactory
                            + " when no Task-level timeout is intended.");
        }
        return taskTimeoutSec;
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
        if (statusObservationInProgress) {
            return;
        }
        statusObservationInProgress = true;
        try {
            applyObservedStatus(readExecutionStatus());
        } catch (RuntimeException statusFailure) {
            markFailedTerminal();
            execution.failClosed(statusFailure);
            throw statusFailure;
        } finally {
            statusObservationInProgress = false;
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
