package edu.ftcphoenix.robots.phoenix.autonomous.pedro;

import java.util.Objects;

import edu.ftcphoenix.fw.drive.route.RouteTasks;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoStrategyId;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoTasks;

/**
 * Builds Phoenix autonomous task sequences for the Pedro route adapter.
 *
 * <p>Static OpModes and selector OpModes should feed this factory a {@link PhoenixPedroAutoContext}
 * rather than building task sequences themselves. That keeps OpModes as FTC entry points while this
 * class owns strategy-to-routine mapping.</p>
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
        return Tasks.sequence(
                followOutbound(ctx, "phoenix.safePreload.outbound"),
                PhoenixAutoTasks.aimAndShootOne(ctx.capabilities(), ctx.driveAdapter(), ctx.profile().auto),
                followReturn(ctx, "phoenix.safePreload.returnOrPark"),
                PhoenixAutoTasks.disableFlywheel(ctx.capabilities())
        );
    }

    private static Task preloadAndPark(PhoenixPedroAutoContext ctx) {
        return Tasks.sequence(
                followOutbound(ctx, "phoenix.preloadAndPark.outbound"),
                PhoenixAutoTasks.aimAndShootOne(ctx.capabilities(), ctx.driveAdapter(), ctx.profile().auto),
                followReturn(ctx, "phoenix.preloadAndPark.parkPlaceholder"),
                PhoenixAutoTasks.disableFlywheel(ctx.capabilities())
        );
    }

    private static Task partnerAwareCycle(PhoenixPedroAutoContext ctx) {
        // Structure exists now; real partner-aware lane geometry belongs in PhoenixPedroPathFactory.
        return Tasks.sequence(
                followOutbound(ctx, "phoenix.partnerAware.outbound"),
                PhoenixAutoTasks.aimAndShootOne(ctx.capabilities(), ctx.driveAdapter(), ctx.profile().auto),
                followReturn(ctx, "phoenix.partnerAware.safeReturnPlaceholder"),
                PhoenixAutoTasks.disableFlywheel(ctx.capabilities())
        );
    }

    private static Task pedroIntegrationTest(PhoenixPedroAutoContext ctx) {
        return Tasks.sequence(
                followOutbound(ctx, "pedro.outbound12in"),
                PhoenixAutoTasks.aimAndShootOne(ctx.capabilities(), ctx.driveAdapter(), ctx.profile().auto),
                followReturn(ctx, "pedro.returnToStart"),
                PhoenixAutoTasks.disableFlywheel(ctx.capabilities())
        );
    }

    private static Task followOutbound(PhoenixPedroAutoContext ctx, String debugName) {
        return RouteTasks.follow(debugName,
                ctx.driveAdapter(),
                ctx.paths().outboundPath,
                PhoenixAutoTasks.routeConfig(ctx.profile().auto));
    }

    private static Task followReturn(PhoenixPedroAutoContext ctx, String debugName) {
        return RouteTasks.follow(debugName,
                ctx.driveAdapter(),
                ctx.paths().returnPath,
                PhoenixAutoTasks.routeConfig(ctx.profile().auto));
    }
}
