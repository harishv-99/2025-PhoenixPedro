package edu.ftcphoenix.fw.drive.route;

import java.util.function.Supplier;

/**
 * Convenience factory methods for external route-following tasks.
 *
 * <p>This is the route-library sibling of {@code DriveTasks} and {@code DriveGuidanceTasks}.
 * Robot code can sequence external route followers using the Phoenix task system without exposing
 * any one library's API beyond the project-specific adapter. Every route Task has an explicit
 * diagnostic name and explicitly selects either a finite Task-level timeout or no Task-level
 * timeout.</p>
 */
public final class RouteTasks {

    private RouteTasks() {
        // utility class
    }

    /**
     * Creates a named route-follow task with a finite Task-level timeout.
     *
     * <p>The timeout begins at this Task's own start boundary. If the exact route execution remains
     * active at the deadline, the returned Task reports {@link RouteStatus#TASK_TIMEOUT}. A
     * follower completion or failure observed on that update takes precedence.</p>
     *
     * @param debugName nonblank human-readable debug label
     * @param follower adapter that knows how to follow the supplied route type
     * @param route route object to follow
     * @param taskTimeoutSec finite Task-level timeout in seconds; must be {@code > 0}
     * @param <R> route type
     * @return typed task that retains the precise route status
     * @throws IllegalArgumentException if {@code debugName} is blank or
     *                                  {@code taskTimeoutSec} is non-finite or not positive
     * @throws NullPointerException if {@code follower} or {@code route} is {@code null}
     */
    public static <R> RouteTask<R> follow(String debugName,
                                          RouteFollower<R> follower,
                                          R route,
                                          double taskTimeoutSec) {
        return RouteTask.eager(debugName, follower, route, true, taskTimeoutSec);
    }

    /**
     * Creates a named route-follow task without a Task-level timeout.
     *
     * <p>This disables only {@link RouteStatus#TASK_TIMEOUT}. The follower may still report
     * {@link RouteStatus#FOLLOWER_TIMEOUT_OR_STALL} from its own route constraints or health
     * checks.</p>
     *
     * @param debugName nonblank human-readable debug label
     * @param follower adapter that knows how to follow the supplied route type
     * @param route route object to follow
     * @param <R> route type
     * @return typed task that retains the precise route status
     * @throws IllegalArgumentException if {@code debugName} is blank
     * @throws NullPointerException if {@code follower} or {@code route} is {@code null}
     */
    public static <R> RouteTask<R> followWithoutTaskTimeout(
            String debugName,
            RouteFollower<R> follower,
            R route) {
        return RouteTask.eager(debugName, follower, route, false, 0.0);
    }

    /**
     * Creates a named route-follow task with a finite Task-level timeout that builds its route when
     * the Task starts.
     *
     * <p>The factory is retained without being sampled and is invoked exactly once at the returned
     * Task's own {@link RouteTask#start(edu.ftcphoenix.fw.core.time.LoopClock)} boundary. Use this
     * form when a robot-owned path factory needs current pose, vision, or strategy state. Keep the
     * factory quick and non-blocking; it should only construct and return the route, not start or
     * update the follower.</p>
     *
     * <p>If the factory throws or returns {@code null}, the Task becomes terminal with
     * {@link RouteStatus#FAILED}, the follower is not called, and an actionable exception identifies
     * the Task. Debug inspection and pre-start cancellation never sample the factory.</p>
     *
     * @param debugName nonblank human-readable debug label
     * @param follower adapter that knows how to follow the supplied route type
     * @param routeFactory factory sampled exactly once when this Task starts
     * @param taskTimeoutSec finite Task-level timeout in seconds; must be {@code > 0}
     * @param <R> route type
     * @return typed task that retains the precise route status
     * @throws IllegalArgumentException if {@code debugName} is blank or
     *                                  {@code taskTimeoutSec} is non-finite or not positive
     * @throws NullPointerException if {@code follower} or {@code routeFactory} is {@code null}
     */
    public static <R> RouteTask<R> followBuiltAtStart(
            String debugName,
            RouteFollower<R> follower,
            Supplier<? extends R> routeFactory,
            double taskTimeoutSec) {
        return RouteTask.builtAtStart(
                debugName,
                follower,
                routeFactory,
                true,
                taskTimeoutSec
        );
    }

    /**
     * Creates a named route-follow task without a Task-level timeout that builds its route when the
     * Task starts.
     *
     * <p>The factory is sampled once by {@code start(clock)}, never by construction, cancellation
     * before start, updates, status reads, or debug output. This disables only
     * {@link RouteStatus#TASK_TIMEOUT}; the follower may still report
     * {@link RouteStatus#FOLLOWER_TIMEOUT_OR_STALL}.</p>
     *
     * @param debugName nonblank human-readable debug label
     * @param follower adapter that knows how to follow the supplied route type
     * @param routeFactory factory sampled exactly once when this Task starts
     * @param <R> route type
     * @return typed task that retains the precise route status
     * @throws IllegalArgumentException if {@code debugName} is blank
     * @throws NullPointerException if {@code follower} or {@code routeFactory} is {@code null}
     */
    public static <R> RouteTask<R> followBuiltAtStartWithoutTaskTimeout(
            String debugName,
            RouteFollower<R> follower,
            Supplier<? extends R> routeFactory) {
        return RouteTask.builtAtStart(debugName, follower, routeFactory, false, 0.0);
    }
}
