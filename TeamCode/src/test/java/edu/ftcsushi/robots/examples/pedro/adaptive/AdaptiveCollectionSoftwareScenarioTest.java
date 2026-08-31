package edu.ftcsushi.robots.examples.pedro.adaptive;

import com.pedropathing.paths.PathChain;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.actuation.Plants;
import edu.ftcsushi.fw.core.hal.PowerOutput;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.drive.route.RouteExecution;
import edu.ftcsushi.fw.drive.route.RouteFollower;
import edu.ftcsushi.fw.drive.route.RouteStatus;
import edu.ftcsushi.fw.task.TaskRunner;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.robots.examples.pedro.capability.intake.BasicPedroAutoMechanism;
import edu.ftcsushi.robots.examples.pedro.capability.intake.BasicPedroMechanismTestFactory;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Runs the complete example story without a camera, Control Hub, drivetrain, or Pedro follower. */
public final class AdaptiveCollectionSoftwareScenarioTest {

    @Test
    public void selectedObjectWaitsForSafeInventoryThenReturnsFromLivePhase() {
        Scenario scenario = new Scenario(
                AdaptiveCollectionVisionService.Decision.selectedForHardwareNeutralTest(9.0)
        );
        scenario.runCycle(0.0);

        scenario.inventoryFull.set(true);
        scenario.runCycle(0.02);
        assertEquals(1, scenario.follower.executions.size());
        assertFalse(scenario.attempt.status().complete());

        scenario.milestones.latchSafeToLeave();
        scenario.runCycle(0.02);
        assertEquals(2, scenario.follower.executions.size());
        assertEquals(RouteStatus.CANCELLED,
                scenario.attempt.status().collectionRouteStatus());
        assertEquals(0.0, scenario.intakePlant.commandTarget().get(), 0.0);

        scenario.follower.executions.get(1).integrationStatus = RouteStatus.COMPLETED;
        scenario.runCycle(0.02);

        AdaptiveCollectionAttempt.Status ended = scenario.attempt.status();
        assertTrue(ended.complete());
        assertFalse(ended.usedFallback());
        assertEquals(AdaptiveCollectionAttempt.ExitReason.INVENTORY_FULL_AFTER_SAFE,
                ended.exitReason());
        assertEquals(RouteStatus.CANCELLED, ended.collectionRouteStatus());
        assertEquals(RouteStatus.COMPLETED, ended.returnRouteStatus());
        assertEquals(1, scenario.returnBuildCount);
    }

    @Test
    public void unavailableSelectionUsesFallbackButAbnormalRouteDoesNotReturn() {
        Scenario scenario = new Scenario(
                AdaptiveCollectionVisionService.Decision.unavailableForHardwareNeutralTest(
                        AdaptiveCollectionVisionService.UnavailableReason.ZERO_DETECTIONS
                )
        );
        scenario.runCycle(0.0);
        scenario.follower.executions.get(0).integrationStatus =
                RouteStatus.FOLLOWER_TIMEOUT_OR_STALL;
        scenario.runCycle(0.02);

        AdaptiveCollectionAttempt.Status ended = scenario.attempt.status();
        assertTrue(ended.complete());
        assertTrue(ended.usedFallback());
        assertEquals(AdaptiveCollectionAttempt.ExitReason.FOLLOWER_TIMEOUT_OR_STALL,
                ended.exitReason());
        assertEquals(RouteStatus.FOLLOWER_TIMEOUT_OR_STALL,
                ended.collectionRouteStatus());
        assertEquals(RouteStatus.NOT_STARTED, ended.returnRouteStatus());
        assertEquals(1, scenario.follower.executions.size());
        assertEquals(0, scenario.returnBuildCount);
        assertEquals(0.0, scenario.intakePlant.commandTarget().get(), 0.0);
    }

    private static final class Scenario {
        final ManualLoopClock time = new ManualLoopClock();
        final AtomicBoolean inventoryFull = new AtomicBoolean();
        final ScenarioFollower follower = new ScenarioFollower();
        final Plant intakePlant = Plants.fromOutputs()
                .power(new InertPowerOutput())
                .targetFromNewCommand(0.0)
                .build();
        final AdaptiveCollectionAttempt attempt;
        final TaskRunner runner = new TaskRunner();

        AdaptiveCollectionPaths.Milestones milestones;
        int returnBuildCount;

        Scenario(AdaptiveCollectionVisionService.Decision decision) {
            AdaptiveCollectionPaths paths = new AdaptiveCollectionPaths(
                    follower,
                    (frozenDecision, routeMilestones) -> {
                        milestones = routeMilestones;
                        return new PathChain();
                    },
                    () -> {
                        returnBuildCount++;
                        return new PathChain();
                    }
            );
            BasicPedroAutoMechanism intake =
                    BasicPedroMechanismTestFactory.fromPlant(intakePlant);
            attempt = new AdaptiveCollectionAttempt(
                    () -> decision,
                    paths,
                    BooleanSource.of(inventoryFull::get),
                    intake,
                    AdaptiveCollectionAttempt.Config.defaults()
            );
            runner.enqueue(attempt.task());
        }

        void runCycle(double dtSec) {
            if (dtSec > 0.0) {
                time.nextCycle(dtSec);
            }
            runner.update(time.clock());
        }
    }

    private static final class ScenarioFollower implements RouteFollower<PathChain> {
        final List<ScenarioExecution> executions = new ArrayList<ScenarioExecution>();

        @Override
        public RouteExecution follow(PathChain route) {
            ScenarioExecution execution = new ScenarioExecution();
            executions.add(execution);
            return execution;
        }
    }

    private static final class ScenarioExecution extends RouteExecution {
        RouteStatus integrationStatus = RouteStatus.ACTIVE;

        @Override
        protected RouteStatus integrationStatus() {
            return integrationStatus;
        }

        @Override
        protected void cancelActive() {
            // The base RouteExecution retains the exact Task-owned terminal reason.
        }
    }

    private static final class InertPowerOutput implements PowerOutput {
        private double commandedPower;

        @Override
        public void setPower(double power) {
            commandedPower = power;
        }

        @Override
        public double getCommandedPower() {
            return commandedPower;
        }
    }
}
