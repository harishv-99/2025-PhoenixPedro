package edu.ftcphoenix.fw.drive.route;

/**
 * Describes the current or terminal state of one route attempt as seen by its Task and execution.
 *
 * <p>Terminal values are retained by the execution that produced them. They must not be replaced
 * with the state of a newer route, which lets a {@link RouteTask} observe and cancel exactly the
 * route it started.</p>
 */
public enum RouteStatus {

    /** The owning RouteTask has not called its follower yet; integrations must not return this. */
    NOT_STARTED,

    /** The route is actively being followed. */
    ACTIVE,

    /** The follower reached the route's intended endpoint normally. */
    COMPLETED,

    /** The follower ended the route because its own timeout or stall safeguard fired. */
    FOLLOWER_TIMEOUT_OR_STALL,

    /** The route was stopped by an external policy, callback, or manual takeover. */
    INTERRUPTED,

    /** A newer route replaced this execution. */
    REPLACED,

    /** The owning {@link RouteTask}'s timeout elapsed. */
    TASK_TIMEOUT,

    /** The owning Task or execution was actively cancelled. */
    CANCELLED,

    /** The follower failed while starting or updating this execution. */
    FAILED,

    /** The follower stopped, but its integration could not prove why. */
    UNKNOWN_TERMINAL
}
