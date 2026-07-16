package edu.ftcphoenix.robots.phoenix.autonomous.pedro;

import com.pedropathing.paths.PathChain;

import java.util.Objects;

import edu.ftcphoenix.fw.drive.route.RouteTask;
import edu.ftcphoenix.fw.drive.route.RouteTasks;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoStrategyId;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoTasks;

/**
 * Builds Phoenix autonomous task sequences for the Pedro route adapter.
 *
 * <p>Static OpModes and selector OpModes should feed this factory a {@link PhoenixPedroAutoContext}
 * rather than building task sequences themselves. That keeps OpModes as FTC entry points while this
 * class owns strategy-to-routine mapping.</p>
 *
 * <p>The checked-in routines use one private robot-owned coordinator to apply explicit route,
 * scoring, and match-budget policy. Position-dependent scoring starts only after outbound
 * completion. Outbound/scoring timeouts and the match-time cutoff select one live-pose return/park
 * attempt, while cancellation-like results abort. Once that final route starts, the pre-park timer
 * is gone; a timeout or other failure of the final attempt terminates the routine rather than
 * starting another fallback. Generic Task and route APIs continue to report facts without choosing
 * Phoenix strategy.</p>
 *
 * <p>The placeholder outbound route is fixed during pre-start construction. Return/park geometry is built once when
 * that route Task starts, allowing its path to begin at the current Pedro pose without exposing the
 * raw Follower or Pedro builder in strategy code.</p>
 */
public final class PhoenixPedroAutoRoutineFactory {

    private PhoenixPedroAutoRoutineFactory() {
        // Utility class.
    }

    /**
     * Build the routine selected by the context's {@code PhoenixAutoSpec}.
     */
    public static Task build(PhoenixPedroAutoContext ctx) {
        Objects.requireNonNull(ctx, "ctx");
        PhoenixAutoStrategyId strategy = ctx.spec().strategy;
        switch (strategy) {
            case SAFE_PRELOAD:
                return safePreload(ctx);
            case PRELOAD_AND_PARK:
                return preloadAndPark(ctx);
            case PARTNER_AWARE_CYCLE:
                return partnerAwareCycle(ctx);
            case PEDRO_INTEGRATION_TEST:
            default:
                return pedroIntegrationTest(ctx);
        }
    }

    private static Task safePreload(PhoenixPedroAutoContext ctx) {
        return buildRoutine(
                ctx,
                "phoenix.safePreload",
                "phoenix.safePreload.outbound",
                "phoenix.safePreload.returnOrPark"
        );
    }

    private static Task preloadAndPark(PhoenixPedroAutoContext ctx) {
        return buildRoutine(
                ctx,
                "phoenix.preloadAndPark",
                "phoenix.preloadAndPark.outbound",
                "phoenix.preloadAndPark.parkPlaceholder"
        );
    }

    private static Task partnerAwareCycle(PhoenixPedroAutoContext ctx) {
        // Structure exists now; real partner-aware lane geometry belongs in PhoenixPedroPathFactory.
        return buildRoutine(
                ctx,
                "phoenix.partnerAware",
                "phoenix.partnerAware.outbound",
                "phoenix.partnerAware.safeReturnPlaceholder"
        );
    }

    private static Task pedroIntegrationTest(PhoenixPedroAutoContext ctx) {
        return buildRoutine(
                ctx,
                "pedro.integrationTest",
                "pedro.outbound12in",
                "pedro.returnToStart"
        );
    }

    /** Construct the private policy owners behind the existing one-method public routine factory. */
    private static Task buildRoutine(PhoenixPedroAutoContext ctx,
                                     String routineName,
                                     String outboundDebugName,
                                     String returnDebugName) {
        double parkTakeoverElapsedSec = requireParkTakeoverElapsedSec(
                ctx.profile().auto.parkTakeoverElapsedSec,
                routineName
        );
        RouteTask<PathChain> outbound = followOutbound(ctx, outboundDebugName);
        Task scoringAttempt = PhoenixAutoTasks.aimAndShootOne(
                ctx.capabilities(),
                ctx.driveAdapter(),
                ctx.profile().auto
        );
        RouteTask<PathChain> returnOrPark = followReturn(ctx, returnDebugName);
        PhoenixPedroPreParkTask prePark = new PhoenixPedroPreParkTask(
                routineName,
                outbound,
                scoringAttempt,
                ctx.capabilities().scoring(),
                ctx.driveAdapter()
        );
        return new PhoenixPedroAutoRoutineTask(
                routineName,
                prePark,
                parkTakeoverElapsedSec,
                returnOrPark
        );
    }

    /** Follow the fixed INIT-built outbound path through the normal eager route helper. */
    private static RouteTask<PathChain> followOutbound(PhoenixPedroAutoContext ctx,
                                                       String debugName) {
        return RouteTasks.follow(debugName,
                ctx.driveAdapter(),
                ctx.paths().outboundPath,
                PhoenixAutoTasks.routeConfig(ctx.profile().auto));
    }

    /** Build the return path from the current Pedro pose when this Task actually starts. */
    private static RouteTask<PathChain> followReturn(PhoenixPedroAutoContext ctx,
                                                     String debugName) {
        return RouteTasks.followBuiltAtStart(debugName,
                ctx.driveAdapter(),
                () -> ctx.pathFactory().buildReturnFromCurrentPose(ctx.paths().pedroStartPose),
                PhoenixAutoTasks.routeConfig(ctx.profile().auto));
    }

    /** Reject a disabled or malformed match cutoff before allocating any Task graph. */
    private static double requireParkTakeoverElapsedSec(double value, String routineName) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(
                    "Phoenix Pedro routine '" + routineName
                            + "' requires profile.auto.parkTakeoverElapsedSec to be finite and "
                            + "> 0 seconds, got " + value
            );
        }
        return value;
    }
}
