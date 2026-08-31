package edu.ftcsushi.robots.phoenix.autonomous.pedro;

import com.pedropathing.paths.PathChain;

import java.util.Objects;

import edu.ftcsushi.fw.drive.route.RouteTask;
import edu.ftcsushi.fw.drive.route.RouteTasks;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.robots.phoenix.PhoenixAutoConfig;
import edu.ftcsushi.robots.phoenix.autonomous.PhoenixAutoStrategyId;
import edu.ftcsushi.robots.phoenix.autonomous.PhoenixAutoTasks;

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
        PhoenixAutoConfig autoConfig = ctx.autoConfig();
        PhoenixAutoStrategyId strategy = ctx.spec().strategy;
        switch (strategy) {
            case SAFE_PRELOAD:
                return safePreload(ctx, autoConfig);
            case PRELOAD_AND_PARK:
                return preloadAndPark(ctx, autoConfig);
            case PARTNER_AWARE_CYCLE:
                return partnerAwareCycle(ctx, autoConfig);
            case PEDRO_INTEGRATION_TEST:
            default:
                return pedroIntegrationTest(ctx, autoConfig);
        }
    }

    private static Task safePreload(PhoenixPedroAutoContext ctx,
                                    PhoenixAutoConfig autoConfig) {
        return buildRoutine(
                ctx,
                autoConfig,
                "phoenix.safePreload",
                "phoenix.safePreload.outbound",
                "phoenix.safePreload.returnOrPark"
        );
    }

    private static Task preloadAndPark(PhoenixPedroAutoContext ctx,
                                       PhoenixAutoConfig autoConfig) {
        return buildRoutine(
                ctx,
                autoConfig,
                "phoenix.preloadAndPark",
                "phoenix.preloadAndPark.outbound",
                "phoenix.preloadAndPark.parkPlaceholder"
        );
    }

    private static Task partnerAwareCycle(PhoenixPedroAutoContext ctx,
                                          PhoenixAutoConfig autoConfig) {
        // Structure exists now; real partner-aware lane geometry belongs in PhoenixPedroPathFactory.
        return buildRoutine(
                ctx,
                autoConfig,
                "phoenix.partnerAware",
                "phoenix.partnerAware.outbound",
                "phoenix.partnerAware.safeReturnPlaceholder"
        );
    }

    private static Task pedroIntegrationTest(PhoenixPedroAutoContext ctx,
                                             PhoenixAutoConfig autoConfig) {
        return buildRoutine(
                ctx,
                autoConfig,
                "pedro.integrationTest",
                "pedro.outbound12in",
                "pedro.returnToStart"
        );
    }

    /** Construct the private policy owners behind the existing one-method public routine factory. */
    private static Task buildRoutine(PhoenixPedroAutoContext ctx,
                                     PhoenixAutoConfig autoConfig,
                                     String routineName,
                                     String outboundDebugName,
                                     String returnDebugName) {
        RoutineConfig auto = RoutineConfig.capture(autoConfig, routineName);
        RouteTask<PathChain> outbound = followOutbound(
                ctx,
                outboundDebugName,
                auto.routeTimeoutSec
        );
        Task scoringAttempt = PhoenixAutoTasks.aimAndShootOne(
                ctx.capabilities(),
                ctx.driveAdapter(),
                autoConfig
        );
        RouteTask<PathChain> returnOrPark = followReturn(
                ctx,
                returnDebugName,
                auto.routeTimeoutSec
        );
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
                auto.parkTakeoverElapsedSec,
                returnOrPark
        );
    }

    /** Follow the fixed INIT-built outbound path through the normal eager route helper. */
    private static RouteTask<PathChain> followOutbound(PhoenixPedroAutoContext ctx,
                                                       String debugName,
                                                       double routeTimeoutSec) {
        return RouteTasks.follow(debugName,
                ctx.driveAdapter(),
                ctx.paths().outboundPath,
                routeTimeoutSec);
    }

    /** Build the return path from the current Pedro pose when this Task actually starts. */
    private static RouteTask<PathChain> followReturn(PhoenixPedroAutoContext ctx,
                                                     String debugName,
                                                     double routeTimeoutSec) {
        return RouteTasks.followBuiltAtStart(debugName,
                ctx.driveAdapter(),
                () -> ctx.pathFactory().buildReturnFromCurrentPose(ctx.paths().pedroStartPose),
                routeTimeoutSec);
    }

    /** Immutable routine-policy slice captured before any route or scoring Task is allocated. */
    private static final class RoutineConfig {
        final double parkTakeoverElapsedSec;
        final double routeTimeoutSec;

        private RoutineConfig(double parkTakeoverElapsedSec, double routeTimeoutSec) {
            this.parkTakeoverElapsedSec = parkTakeoverElapsedSec;
            this.routeTimeoutSec = routeTimeoutSec;
        }

        /** Capture both retained primitives, then validate them in PhoenixAutoConfig source order. */
        static RoutineConfig capture(PhoenixAutoConfig source, String routineName) {
            PhoenixAutoConfig required = Objects.requireNonNull(
                    source,
                    "Phoenix Pedro routine '" + routineName + "' autoConfig is required"
            );
            double parkTakeoverElapsedSec = required.parkTakeoverElapsedSec;
            double routeTimeoutSec = required.routeTimeoutSec;

            requireFinitePositive(
                    "parkTakeoverElapsedSec",
                    parkTakeoverElapsedSec,
                    routineName
            );
            requireFinitePositive("routeTimeoutSec", routeTimeoutSec, routineName);
            return new RoutineConfig(parkTakeoverElapsedSec, routeTimeoutSec);
        }
    }

    private static void requireFinitePositive(String fieldName,
                                              double value,
                                              String routineName) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(
                    "Phoenix Pedro routine '" + routineName + "' requires PhoenixAutoConfig."
                            + fieldName + " to be finite and > 0, got " + value
            );
        }
    }
}
