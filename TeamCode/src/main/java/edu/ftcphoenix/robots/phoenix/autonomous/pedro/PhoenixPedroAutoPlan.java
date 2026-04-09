package edu.ftcphoenix.robots.phoenix.autonomous.pedro;

import com.pedropathing.paths.PathChain;

import java.util.Objects;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcphoenix.fw.drive.route.RouteTask;
import edu.ftcphoenix.fw.drive.route.RouteTasks;
import edu.ftcphoenix.fw.integrations.pedro.PedroPathingDriveAdapter;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.robots.phoenix.PhoenixCapabilities;

/**
 * Example autonomous mode client for Phoenix's Pedro integration test.
 *
 * <p>
 * This class is the autonomous-side parallel to {@code PhoenixTeleOpControls}: both are mode
 * clients that consume {@link PhoenixCapabilities}. The difference is that TeleOp maps human input
 * into capability calls, while this plan composes timed/conditional tasks over the same vocabulary.
 * </p>
 */
public final class PhoenixPedroAutoPlan {

    private final PhoenixCapabilities capabilities;
    private final PedroPathingDriveAdapter driveAdapter;

    /**
     * Creates the Pedro auto plan helper.
     *
     * @param capabilities shared Phoenix capability families
     * @param driveAdapter Pedro bridge used for route following and aim tasks
     */
    public PhoenixPedroAutoPlan(PhoenixCapabilities capabilities,
                                PedroPathingDriveAdapter driveAdapter) {
        this.capabilities = Objects.requireNonNull(capabilities, "capabilities");
        this.driveAdapter = Objects.requireNonNull(driveAdapter, "driveAdapter");
    }

    /**
     * Builds the checked-in outbound/aim/shoot/return integration test routine.
     *
     * @param outboundPath outbound Pedro path segment
     * @param returnPath   return Pedro path segment
     * @return composed autonomous task sequence
     */
    public Task build(PathChain outboundPath, PathChain returnPath) {
        Objects.requireNonNull(outboundPath, "outboundPath");
        Objects.requireNonNull(returnPath, "returnPath");

        final PhoenixCapabilities.Scoring scoring = capabilities.scoring();
        final PhoenixCapabilities.Targeting targeting = capabilities.targeting();

        RouteTask.Config routeCfg = new RouteTask.Config();
        routeCfg.timeoutSec = 4.0;

        DriveGuidanceTask.Config aimCfg = new DriveGuidanceTask.Config();
        aimCfg.headingTolRad = Math.toRadians(2.0);
        aimCfg.timeoutSec = 1.75;
        aimCfg.maxNoGuidanceSec = 0.75;

        Task aimAndMaybeShoot = Tasks.branchOnOutcome(
                Tasks.waitUntil(new BooleanSource() {
                    @Override
                    public boolean getAsBoolean(LoopClock clock) {
                        return targeting.status(clock).selection.hasSelection;
                    }
                }, 0.75),
                Tasks.sequence(
                        Tasks.runOnce(scoring::captureSuggestedShotVelocity),
                        Tasks.branchOnOutcome(
                                targeting.aimTask(driveAdapter, aimCfg),
                                Tasks.sequence(
                                        Tasks.runOnce(scoring::requestSingleShot),
                                        Tasks.waitUntil(new BooleanSource() {
                                            @Override
                                            public boolean getAsBoolean(LoopClock clock) {
                                                return !scoring.hasPendingShots();
                                            }
                                        }, 2.5)
                                ),
                                Tasks.noop()
                        )
                ),
                Tasks.noop()
        );

        return Tasks.sequence(
                RouteTasks.follow("pedro.outbound12in", driveAdapter, outboundPath, routeCfg),
                aimAndMaybeShoot,
                RouteTasks.follow("pedro.returnToStart", driveAdapter, returnPath, routeCfg),
                Tasks.runOnce(new Runnable() {
                    @Override
                    public void run() {
                        scoring.setFlywheelEnabled(false);
                    }
                })
        );
    }
}
