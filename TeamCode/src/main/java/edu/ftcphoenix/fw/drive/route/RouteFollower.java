package edu.ftcphoenix.fw.drive.route;

import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Minimal Phoenix-owned seam for following an external route object.
 *
 * <p>This interface exists for lane-5 integrations such as Pedro Pathing, Road Runner, or a
 * team-owned follower. Phoenix intentionally does <b>not</b> define the route type itself. The
 * generic type parameter {@code R} is whatever the external package uses to represent a route
 * (for example, a Pedro {@code PathChain}).</p>
 *
 * <p>The goal is the same as {@link edu.ftcphoenix.fw.drive.DriveCommandSink}: keep framework code
 * dependent on the smallest useful seam instead of depending on one specific route library.</p>
 *
 * <p>Typical usage:</p>
 * <pre>{@code
 * public final class PedroRouteAdapter implements RouteFollower<PathChain> {
 *     private final Follower follower;
 *
 *     public PedroRouteAdapter(Follower follower) {
 *         this.follower = follower;
 *     }
 *
 *     @Override
 *     public void update(LoopClock clock) {
 *         follower.update();
 *     }
 *
 *     @Override
 *     public void follow(PathChain route) {
 *         follower.followPath(route);
 *     }
 *
 *     @Override
 *     public boolean isBusy() {
 *         return follower.isBusy();
 *     }
 *
 *     @Override
 *     public void cancel() {
 *         follower.breakFollowing();
 *     }
 * }
 * }</pre>
 *
 * @param <R> route object type owned by the external route library
 */
public interface RouteFollower<R> {

    /**
     * Optional lifecycle hook for per-loop updates.
     *
     * <p>The default implementation is a no-op so simple adapters only need to implement it when
     * their follower must be advanced each loop.</p>
     *
     * @param clock shared loop clock for the current cycle
     */
    default void update(LoopClock clock) {
        // default no-op
    }

    /**
     * Begin following the supplied route.
     *
     * @param route external route object to follow
     */
    void follow(R route);

    /**
     * Returns whether the follower is still actively executing its current route.
     *
     * @return {@code true} while the route is still in progress
     */
    boolean isBusy();

    /**
     * Interrupt the current route and leave the follower in a predictable stopped state.
     */
    void cancel();
}
