package edu.ftcsushi.fw.drive.route;

import java.util.Objects;
import java.util.function.Supplier;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.task.AbstractTask;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.task.Tasks;

/**
 * Autonomous-style {@link Task} wrapper around an external {@link RouteFollower}.
 *
 * <p>This lets Sushi task runners sequence route following together with mechanism actions,
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
public final class RouteTask<R> extends AbstractTask {

    private final String debugName;
    private final RouteFollower<R> follower;
    private final Supplier<? extends R> routeFactory;
    private final boolean taskTimeoutEnabled;
    private final double taskTimeoutSec;

    private R route;

    private boolean statusObservationInProgress;
    private boolean statusReadFailed;
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
        super(requireDebugName(debugName));
        this.debugName = debugName;
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
    protected void onStart(LoopClock clock) {
        routeStatus = RouteStatus.NOT_STARTED;
        startTimeSec = clock.nowSec();
        if (routeFactory != null) {
            R builtRoute;
            try {
                builtRoute = routeFactory.get();
            } catch (RuntimeException factoryFailure) {
                throw routeFactoryFailure(factoryFailure);
            }
            if (!isActive()) {
                return;
            }
            if (builtRoute == null) {
                throw new IllegalStateException(
                        "RouteTasks.followBuiltAtStart(...) route factory returned null for "
                                + "RouteTask '" + debugName + "'. Return a non-null route object.");
            }
            route = builtRoute;
        }

        execution = follower.follow(route);
        if (execution == null) {
            throw new IllegalStateException("RouteFollower.follow(...) returned null for RouteTask '"
                    + debugName + "'. Return a RouteExecution for the route that was started.");
        }
        if (!isActive()) {
            // Ownership arrived after cancellation. Settle only after classifying and, if needed,
            // cancelling this exact returned handle; never act on the follower's replacement.
            finishReentrantCancellationAfterFollow();
            return;
        }
        observeAndApplyStatus();
    }

    @Override
    protected void onUpdate(LoopClock clock) {
        observeAndApplyStatus();
        if (!isActive()) {
            return;
        }
        follower.update(clock);
        if (!isActive()) {
            return;
        }
        observeAndApplyStatus();
        if (!isActive()) {
            return;
        }
        if (taskTimeoutEnabled && clock.nowSec() - startTimeSec >= taskTimeoutSec) {
            routeStatus = RouteStatus.TASK_TIMEOUT;
            complete(TaskOutcome.TIMEOUT);
            execution.cancelForTimeout();
        }
    }

    /** Preserve terminal evidence published by the root heartbeat before choosing cancellation. */
    @Override
    protected void onBeforeCancel() {
        if (execution != null) {
            observeAndApplyStatus();
        }
    }

    @Override
    protected void onCancel() {
        routeStatus = RouteStatus.CANCELLED;
        if (execution != null) {
            execution.cancelAfterActiveObservation();
        }
    }

    /** Retain precise route evidence, but never translate a thrown lifecycle failure into success. */
    @Override
    protected void onFailure(RuntimeException primaryFailure) {
        if (execution == null) {
            routeStatus = RouteStatus.FAILED;
            return; // follow() owns fail-closed cleanup when it did not return a handle.
        }
        if (!statusReadFailed) {
            try {
                RouteStatus observed = readExecutionStatus();
                routeStatus = observed;
                if (observed != RouteStatus.ACTIVE) {
                    return;
                }
            } catch (RuntimeException statusFailure) {
                addSuppressedIfDistinct(primaryFailure, statusFailure);
            }
        }
        routeStatus = RouteStatus.FAILED;
        execution.failClosed(primaryFailure);
    }

    /**
     * Returns current exact-execution evidence, or rethrows a retained lifecycle failure.
     *
     * <p>This cheap observation is independent of effectful update deduplication: a root heartbeat
     * may publish a terminal status later in the same cycle. It never advances the follower. A
     * final result is unavailable while cancellation/acquisition cleanup is still settling; use
     * cached debug rows, or the integration's own execution status, inside cleanup callbacks.</p>
     */
    public RouteStatus getRouteStatus() {
        requireOutcomeAvailable();
        if (isStarted() && isActive() && execution != null) {
            observe(this::observeAndApplyStatus);
        }
        requireOutcomeAvailable();
        return routeStatus;
    }

    @Override
    protected void debugState(DebugSink dbg, String prefix) {
        dbg.addData(prefix + ".routeStatus", routeStatus)
                .addData(prefix + ".routeSource", routeFactory == null ? "eager" : "builtAtStart")
                .addData(prefix + ".routeClass",
                        route == null ? "pending" : route.getClass().getSimpleName())
                .addData(prefix + ".timeoutSec", taskTimeoutDebugValue())
                .addData(prefix + ".startedAtSec", startTimeSec);
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

    /** Validate one non-advancing execution observation and remember a broken status source. */
    private RouteStatus readExecutionStatus() {
        try {
            RouteStatus status = execution.status();
            if (status == null || status == RouteStatus.NOT_STARTED) {
                throw new IllegalStateException("RouteExecution.status() returned " + status
                        + " for RouteTask '" + debugName
                        + "'. Return ACTIVE or a retained terminal RouteStatus.");
            }
            return status;
        } catch (RuntimeException failure) {
            statusReadFailed = true;
            throw new IllegalStateException("RouteTask '" + debugName
                    + "' could not read its RouteExecution status. " + failure.getMessage(), failure);
        }
    }

    /** Map valid domain evidence without manufacturing an exception from a FAILED status value. */
    private void applyObservedStatus(RouteStatus observedStatus, boolean afterAcquisition) {
        routeStatus = observedStatus;
        TaskOutcome result;
        switch (observedStatus) {
            case ACTIVE:
                return;
            case COMPLETED:
                result = TaskOutcome.SUCCESS;
                break;
            case FOLLOWER_TIMEOUT_OR_STALL:
            case TASK_TIMEOUT:
                result = TaskOutcome.TIMEOUT;
                break;
            case INTERRUPTED:
            case REPLACED:
            case CANCELLED:
            case FAILED:
            case UNKNOWN_TERMINAL:
                result = TaskOutcome.CANCELLED;
                break;
            default:
                throw new IllegalStateException("Unhandled RouteStatus " + observedStatus
                        + " for RouteTask '" + debugName + "'.");
        }
        if (afterAcquisition) {
            completeAfterAcquisition(result);
        } else {
            complete(result);
        }
    }

    /** The lifecycle guard owns errors; this narrow guard prevents recursive status resampling. */
    private void observeAndApplyStatus() {
        if (statusObservationInProgress) {
            return;
        }
        statusObservationInProgress = true;
        try {
            RouteStatus observed = readExecutionStatus();
            if (isActive()) {
                applyObservedStatus(observed, false);
            }
        } finally {
            statusObservationInProgress = false;
        }
    }

    /** Handle late acquisition without letting a pending cancellation erase returned route truth. */
    private void finishReentrantCancellationAfterFollow() {
        RouteStatus returnedStatus;
        try {
            returnedStatus = readExecutionStatus();
        } catch (RuntimeException statusFailure) {
            // The common abort already ran while no handle existed. This newly owned handle
            // still needs its one exact fail-closed attempt before start can settle.
            routeStatus = RouteStatus.FAILED;
            execution.failClosed(statusFailure);
            throw statusFailure;
        }
        if (returnedStatus == RouteStatus.ACTIVE) {
            execution.cancelAfterActiveObservation();
        } else {
            applyObservedStatus(returnedStatus, true);
        }
    }

    private static void addSuppressedIfDistinct(RuntimeException primary,
                                                RuntimeException secondary) {
        if (secondary != primary) {
            primary.addSuppressed(secondary);
        }
    }
}
