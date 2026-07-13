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
 * dependent on the smallest useful seam instead of depending on one specific route library. Each
 * start returns a {@link RouteExecution} so status and cancellation remain attached to that exact
 * route even when another route replaces it.</p>
 *
 * <p>A follower whose heartbeat must continue during mechanism and wait Tasks needs a stable
 * composition-root owner in addition to this Task-facing seam. Its {@link #update(LoopClock)}
 * implementation must be cycle-idempotent so both callers are safe. Phoenix production Pedro code
 * gets the checked-in adapter from its validated runtime rather than independently constructing a
 * Follower/localizer graph:</p>
 * <pre>{@code
 * RouteFollower<PathChain> routes = pedroRuntime.driveAdapter();
 * }</pre>
 *
 * @param <R> route object type owned by the external route library
 */
public interface RouteFollower<R> {

    /**
     * Optional Task-facing lifecycle hook for per-loop updates.
     *
     * <p>The default implementation is a no-op so simple adapters only need to implement it when
     * their follower must be advanced while the route Task is active. A vendor follower that must
     * continue updating during later mechanism/wait Tasks also needs a composition-root owner;
     * implement this hook idempotently by {@link LoopClock#cycle()} in that case.</p>
     *
     * @param clock shared loop clock for the current cycle
     */
    default void update(LoopClock clock) {
        // default no-op
    }

    /**
     * Begin following the supplied route.
     *
     * <p>The returned execution belongs only to this start. If a later call replaces an active
     * route, the older execution must retain {@link RouteStatus#REPLACED}; cancelling that older
     * handle must not affect the replacement. If start throws before returning a handle, the
     * implementation must fail closed because callers have no execution to cancel. This method
     * begins the route synchronously, so a returned execution must be active or already terminal;
     * {@link RouteStatus#NOT_STARTED} is reserved for a {@link RouteTask} before its start.</p>
     *
     * @param route external route object to follow
     * @return non-null status and cancellation handle for this exact route attempt
     */
    RouteExecution follow(R route);
}
