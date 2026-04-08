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

    private boolean started = false;
    private boolean complete = false;
    private TaskOutcome outcome = TaskOutcome.NOT_DONE;
    private double startTimeSec = 0.0;

    /**
     * Creates a named route-follow task.
     *
     * @param debugName human-readable task label used for debugging
     * @param follower  route follower adapter to command
     * @param route     route object to follow
     * @param cfg       task-level timeout config; when {@code null}, defaults are used
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
        started = true;
        complete = false;
        outcome = TaskOutcome.NOT_DONE;
        startTimeSec = (clock != null) ? clock.nowSec() : 0.0;
        follower.follow(route);
    }

    @Override
    public void update(LoopClock clock) {
        if (complete) {
            return;
        }
        if (!started) {
            start(clock);
        }
        if (clock == null) {
            return;
        }

        follower.update(clock);

        if (cfg.timeoutSec > 0.0 && (clock.nowSec() - startTimeSec) > cfg.timeoutSec) {
            follower.cancel();
            complete = true;
            outcome = TaskOutcome.TIMEOUT;
            return;
        }

        if (!follower.isBusy()) {
            complete = true;
            outcome = TaskOutcome.SUCCESS;
        }
    }

    @Override
    public void cancel() {
        if (complete) {
            return;
        }
        follower.cancel();
        complete = true;
        outcome = TaskOutcome.CANCELLED;
    }

    @Override
    public boolean isComplete() {
        return complete;
    }

    @Override
    public TaskOutcome getOutcome() {
        return outcome;
    }

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        Task.super.debugDump(dbg, prefix);
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "task" : prefix;
        dbg.addData(p + ".followerBusy", follower.isBusy())
                .addData(p + ".routeClass", route.getClass().getSimpleName())
                .addData(p + ".timeoutSec", cfg.timeoutSec)
                .addData(p + ".startedAtSec", startTimeSec);
    }
}
