package edu.ftcphoenix.fw.drive.route;

import edu.ftcphoenix.fw.task.Task;

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
     * @param route    route object to follow
     * @param cfg      task-level config (timeout); may be {@code null}
     * @param <R>      route type
     * @return task that follows the supplied route until completion or timeout
     */
    public static <R> Task follow(RouteFollower<R> follower,
                                  R route,
                                  RouteTask.Config cfg) {
        return new RouteTask<R>(follower, route, cfg);
    }

    /**
     * Creates a named route-follow task.
     *
     * @param debugName human-readable debug label
     * @param follower  adapter that knows how to follow the supplied route type
     * @param route     route object to follow
     * @param cfg       task-level config (timeout); may be {@code null}
     * @param <R>       route type
     * @return task that follows the supplied route until completion or timeout
     */
    public static <R> Task follow(String debugName,
                                  RouteFollower<R> follower,
                                  R route,
                                  RouteTask.Config cfg) {
        return new RouteTask<R>(debugName, follower, route, cfg);
    }
}
