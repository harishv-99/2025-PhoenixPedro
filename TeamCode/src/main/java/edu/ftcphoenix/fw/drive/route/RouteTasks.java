package edu.ftcphoenix.fw.drive.route;

import java.util.function.Supplier;

/**
 * Convenience factory methods for external route-following tasks.
 *
 * <p>This is the route-library sibling of {@code DriveTasks} and {@code DriveGuidanceTasks}.
 * Robot code can sequence external route followers using the Phoenix task system without exposing
 * any one library's API beyond the project-specific adapter.</p>
 */
public final class RouteTasks {

    private RouteTasks() {
        // utility class
    }

    /**
     * Creates a route-follow task with a default debug name.
     *
     * @param follower adapter that knows how to follow the supplied route type
     * @param route route object to follow
     * @param cfg task-level config (timeout); may be {@code null}
     * @param <R> route type
     * @return typed task that retains the precise route status
     */
    public static <R> RouteTask<R> follow(RouteFollower<R> follower,
                                          R route,
                                          RouteTask.Config cfg) {
        return new RouteTask<R>(follower, route, cfg);
    }

    /**
     * Creates a named route-follow task.
     *
     * @param debugName human-readable debug label
     * @param follower adapter that knows how to follow the supplied route type
     * @param route route object to follow
     * @param cfg task-level config (timeout); may be {@code null}
     * @param <R> route type
     * @return typed task that retains the precise route status
     */
    public static <R> RouteTask<R> follow(String debugName,
                                          RouteFollower<R> follower,
                                          R route,
                                          RouteTask.Config cfg) {
        return new RouteTask<R>(debugName, follower, route, cfg);
    }

    /**
     * Creates a route-follow task that builds its route when the Task starts.
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
     * @param follower adapter that knows how to follow the supplied route type
     * @param routeFactory factory sampled exactly once when this Task starts
     * @param cfg task-level config (timeout); may be {@code null}
     * @param <R> route type
     * @return typed task that retains the precise route status
     * @throws NullPointerException if {@code follower} or {@code routeFactory} is {@code null}
     */
    public static <R> RouteTask<R> followBuiltAtStart(
            RouteFollower<R> follower,
            Supplier<? extends R> routeFactory,
            RouteTask.Config cfg) {
        return RouteTask.builtAtStart("RouteTask", follower, routeFactory, cfg);
    }

    /**
     * Creates a named route-follow task that builds its route when the Task starts.
     *
     * <p>This is the named sibling of
     * {@link #followBuiltAtStart(RouteFollower, Supplier, RouteTask.Config)}. The factory is sampled
     * once by {@code start(clock)}, never by construction, cancellation before start, updates,
     * status reads, or debug output.</p>
     *
     * @param debugName human-readable debug label
     * @param follower adapter that knows how to follow the supplied route type
     * @param routeFactory factory sampled exactly once when this Task starts
     * @param cfg task-level config (timeout); may be {@code null}
     * @param <R> route type
     * @return typed task that retains the precise route status
     * @throws NullPointerException if {@code follower} or {@code routeFactory} is {@code null}
     */
    public static <R> RouteTask<R> followBuiltAtStart(
            String debugName,
            RouteFollower<R> follower,
            Supplier<? extends R> routeFactory,
            RouteTask.Config cfg) {
        return RouteTask.builtAtStart(debugName, follower, routeFactory, cfg);
    }
}
