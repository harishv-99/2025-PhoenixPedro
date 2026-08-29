package edu.ftcphoenix.robots.examples.pedro.adaptive;

import com.pedropathing.paths.PathChain;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.Plants;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.drive.route.RouteExecution;
import edu.ftcphoenix.fw.drive.route.RouteFollower;
import edu.ftcphoenix.fw.drive.route.RouteStatus;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.task.TaskRunner;
import edu.ftcphoenix.fw.testing.ManualLoopClock;
import edu.ftcphoenix.robots.examples.pedro.capability.intake.BasicPedroAutoMechanism;
import edu.ftcphoenix.robots.examples.pedro.capability.intake.BasicPedroMechanismTestFactory;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the example's exact collection-exit, cancellation, and conditional-return policy. */
public final class AdaptiveCollectionAttemptTest {

    @Test
    public void taskIsStableDecisionFreezesOnceAndUnavailableUsesExplicitFallback() {
        AdaptiveCollectionVisionService.Decision unavailable =
                AdaptiveCollectionVisionService.Decision.unavailableForHardwareNeutralTest(
                        AdaptiveCollectionVisionService.UnavailableReason.ZERO_DETECTIONS
                );
        Fixture fixture = new Fixture(unavailable);
        AdaptiveCollectionAttempt.Status beforeStart = fixture.attempt.status();

        assertSame(fixture.task, fixture.attempt.task());
        assertFalse(beforeStart.complete());
        assertFalse(beforeStart.decisionFrozen());
        assertGuardedDecision(beforeStart);

        fixture.start();
        fixture.update(0.02);
        fixture.update(0.02);

        AdaptiveCollectionAttempt.Status active = fixture.attempt.status();
        assertEquals(1, fixture.collectionBuildCount);
        assertEquals(1, fixture.follower.followCount());
        assertSame(unavailable, fixture.builtDecision);
        assertSame(unavailable, active.frozenDecision());
        assertTrue(active.usedFallback());
        assertFalse(active.complete());

        try {
            fixture.task.start(fixture.time.clock());
            fail("expected the stable root Task to remain single-use");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("single-use"));
        }
    }

    @Test
    public void fullBeforeSafeAndSafeAloneWaitThenSafePlusFullReturns() {
        Fixture fixture = new Fixture(selectedDecision());
        fixture.inventoryFull = true;
        fixture.startAndFirstUpdate();

        assertFalse(fixture.attempt.status().complete());
        assertEquals(1, fixture.follower.followCount());

        fixture.inventoryFull = false;
        fixture.milestones().latchSafeToLeave();
        fixture.update(0.02);
        assertFalse(fixture.attempt.status().complete());

        fixture.inventoryFull = true;
        fixture.update(0.02);

        AdaptiveCollectionAttempt.Status returning = fixture.attempt.status();
        assertEquals(AdaptiveCollectionAttempt.ExitReason.INVENTORY_FULL_AFTER_SAFE,
                returning.exitReason());
        assertFalse(returning.complete());
        assertTrue(returning.inventoryFull());
        assertEquals(RouteStatus.CANCELLED, returning.collectionRouteStatus());
        assertEquals(RouteStatus.ACTIVE, returning.returnRouteStatus());
        assertEquals(1, fixture.collectionExecution().cancelCount);
        assertEquals(1, fixture.returnBuildCount);
        assertEquals(2, fixture.follower.followCount());
        assertEquals(0.0, fixture.plant.commandTarget().get(), 0.0);
    }

    @Test
    public void exactRouteThenNearEndThenSafeFullPrecedenceIsDeterministic() {
        Fixture routeWins = new Fixture(selectedDecision());
        routeWins.startAndFirstUpdate();
        routeWins.inventoryFull = true;
        routeWins.milestones().latchSafeToLeave();
        routeWins.milestones().latchNearEnd();
        routeWins.collectionExecution().integrationStatus = RouteStatus.COMPLETED;
        int readsBeforeRouteExit = routeWins.inventoryReads;
        routeWins.update(0.02);
        AdaptiveCollectionAttempt.Status completedCollection = routeWins.attempt.status();
        assertEquals(AdaptiveCollectionAttempt.ExitReason.ROUTE_COMPLETED,
                completedCollection.exitReason());
        assertEquals(RouteStatus.COMPLETED, completedCollection.collectionRouteStatus());
        assertEquals(RouteStatus.ACTIVE, completedCollection.returnRouteStatus());
        assertEquals(readsBeforeRouteExit, routeWins.inventoryReads);
        assertEquals(1, routeWins.returnBuildCount);
        assertEquals(2, routeWins.follower.followCount());

        Fixture nearEndWins = new Fixture(selectedDecision());
        nearEndWins.startAndFirstUpdate();
        nearEndWins.inventoryFull = true;
        nearEndWins.milestones().latchSafeToLeave();
        nearEndWins.milestones().latchNearEnd();
        int readsBeforeNearEnd = nearEndWins.inventoryReads;
        nearEndWins.update(0.02);
        assertEquals(AdaptiveCollectionAttempt.ExitReason.NEAR_END,
                nearEndWins.attempt.status().exitReason());
        assertEquals(readsBeforeNearEnd, nearEndWins.inventoryReads);
    }

    @Test
    public void everyIntegrationTerminalReasonRemainsDistinctAndSuppressesReturn() {
        List<RouteStatus> routeStatuses = Arrays.asList(
                RouteStatus.FOLLOWER_TIMEOUT_OR_STALL,
                RouteStatus.INTERRUPTED,
                RouteStatus.REPLACED,
                RouteStatus.CANCELLED,
                RouteStatus.FAILED,
                RouteStatus.UNKNOWN_TERMINAL
        );
        List<AdaptiveCollectionAttempt.ExitReason> expectedReasons = Arrays.asList(
                AdaptiveCollectionAttempt.ExitReason.FOLLOWER_TIMEOUT_OR_STALL,
                AdaptiveCollectionAttempt.ExitReason.INTERRUPTED,
                AdaptiveCollectionAttempt.ExitReason.REPLACED,
                AdaptiveCollectionAttempt.ExitReason.CANCELLED,
                AdaptiveCollectionAttempt.ExitReason.FAILED,
                AdaptiveCollectionAttempt.ExitReason.UNKNOWN_TERMINAL
        );

        for (int index = 0; index < routeStatuses.size(); index++) {
            Fixture fixture = new Fixture(selectedDecision());
            fixture.startAndFirstUpdate();
            fixture.collectionExecution().integrationStatus = routeStatuses.get(index);
            fixture.update(0.02);

            AdaptiveCollectionAttempt.Status ended = fixture.attempt.status();
            assertTrue("root should finish for " + routeStatuses.get(index),
                    fixture.task.isComplete());
            assertEquals(TaskOutcome.SUCCESS, fixture.task.getOutcome());
            assertEquals(expectedReasons.get(index), ended.exitReason());
            assertEquals(routeStatuses.get(index), ended.collectionRouteStatus());
            assertEquals(RouteStatus.NOT_STARTED, ended.returnRouteStatus());
            assertEquals(0, fixture.returnBuildCount);
            assertEquals(1, fixture.follower.followCount());
        }
    }

    @Test
    public void collectionTaskTimeoutIsDistinctAndUsesSnapshottedBound() {
        AdaptiveCollectionAttempt.Config config = AdaptiveCollectionAttempt.Config.defaults();
        config.collectionRouteTimeoutSec = 0.25;
        Fixture fixture = new Fixture(selectedDecision(), config);
        config.collectionRouteTimeoutSec = 20.0;
        fixture.startAndFirstUpdate();

        fixture.update(0.26);
        assertFalse(fixture.task.isComplete());
        fixture.update(0.0);

        AdaptiveCollectionAttempt.Status ended = fixture.attempt.status();
        assertTrue(fixture.task.isComplete());
        assertEquals(AdaptiveCollectionAttempt.ExitReason.TASK_TIMEOUT, ended.exitReason());
        assertEquals(RouteStatus.TASK_TIMEOUT, ended.collectionRouteStatus());
        assertEquals(1, fixture.collectionExecution().cancelCount);
        assertEquals(0, fixture.returnBuildCount);
        assertEquals(0.0, fixture.plant.commandTarget().get(), 0.0);
    }

    @Test
    public void earlyExitCancelsExactCollectionBeforeBuildingReturnAndKeepsStatusesSeparate() {
        Fixture fixture = new Fixture(selectedDecision());
        fixture.startAndFirstUpdate();
        ControlledExecution collection = fixture.collectionExecution();
        fixture.milestones().latchNearEnd();
        fixture.update(0.02);

        ControlledExecution returning = fixture.returnExecution();
        assertEquals(Arrays.asList(
                        "build.collection",
                        "follow.1",
                        "cancel.1",
                        "build.return",
                        "follow.2"
                ),
                fixture.events);
        assertEquals(1, collection.cancelCount);
        assertEquals(0, returning.cancelCount);

        returning.integrationStatus = RouteStatus.REPLACED;
        fixture.update(0.02);

        AdaptiveCollectionAttempt.Status ended = fixture.attempt.status();
        assertTrue(fixture.task.isComplete());
        assertEquals(AdaptiveCollectionAttempt.ExitReason.NEAR_END, ended.exitReason());
        assertEquals(RouteStatus.CANCELLED, ended.collectionRouteStatus());
        assertEquals(RouteStatus.REPLACED, ended.returnRouteStatus());
    }

    @Test
    public void sameCycleMilestoneIsVisibleAndPriorStatusSnapshotDoesNotChange() {
        Fixture fixture = new Fixture(selectedDecision());
        fixture.start();
        AdaptiveCollectionAttempt.Status beforeCallback = fixture.attempt.status();

        fixture.milestones().latchNearEnd();
        fixture.task.update(fixture.time.clock());

        AdaptiveCollectionAttempt.Status afterCallback = fixture.attempt.status();
        assertFalse(beforeCallback.nearEnd());
        assertEquals(AdaptiveCollectionAttempt.ExitReason.NOT_FINISHED,
                beforeCallback.exitReason());
        assertTrue(afterCallback.nearEnd());
        assertEquals(AdaptiveCollectionAttempt.ExitReason.NEAR_END,
                afterCallback.exitReason());
        assertFalse(afterCallback.complete());
    }

    @Test
    public void directCancellationCancelsExactWorkAndNeverConstructsReturn() {
        Fixture fixture = new Fixture(selectedDecision());
        fixture.startAndFirstUpdate();

        fixture.task.cancel();
        fixture.task.cancel();

        AdaptiveCollectionAttempt.Status cancelled = fixture.attempt.status();
        assertTrue(fixture.task.isComplete());
        assertEquals(TaskOutcome.CANCELLED, fixture.task.getOutcome());
        assertEquals(AdaptiveCollectionAttempt.ExitReason.CANCELLED,
                cancelled.exitReason());
        assertEquals(RouteStatus.CANCELLED, cancelled.collectionRouteStatus());
        assertEquals(RouteStatus.NOT_STARTED, cancelled.returnRouteStatus());
        assertEquals(1, fixture.collectionExecution().cancelCount);
        assertEquals(0, fixture.returnBuildCount);
        assertEquals(1, fixture.follower.followCount());
        assertEquals(0.0, fixture.plant.commandTarget().get(), 0.0);
    }

    @Test
    public void collectionConstructionFailureLeavesFollowerUntouchedAndBuildsNoReturn() {
        Fixture fixture = new Fixture(selectedDecision());
        RuntimeException constructionFailure = new RuntimeException("collection geometry failed");
        fixture.collectionBuildFailure = constructionFailure;
        TaskRunner runner = new TaskRunner();
        runner.enqueue(fixture.task);

        try {
            runner.update(fixture.time.clock());
            fail("expected start-built collection construction to fail");
        } catch (IllegalStateException expected) {
            assertSame(constructionFailure, expected.getCause());
            assertTrue(expected.getMessage().contains("could not build its route at start"));
        }

        AdaptiveCollectionAttempt.Status failed = fixture.attempt.status();
        assertTrue(runner.isIdle());
        assertEquals(AdaptiveCollectionAttempt.ExitReason.FAILED, failed.exitReason());
        assertEquals(RouteStatus.FAILED, failed.collectionRouteStatus());
        assertEquals(0, fixture.follower.followCount());
        assertEquals(0, fixture.returnBuildCount);
        assertEquals(0.0, fixture.plant.commandTarget().get(), 0.0);

        Fixture nullRoute = new Fixture(selectedDecision());
        nullRoute.collectionBuildReturnsNull = true;
        TaskRunner nullRunner = new TaskRunner();
        nullRunner.enqueue(nullRoute.task);
        try {
            nullRunner.update(nullRoute.time.clock());
            fail("expected a null start-built collection route to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("could not build its route at start"));
        }
        AdaptiveCollectionAttempt.Status nullFailed = nullRoute.attempt.status();
        assertTrue(nullRunner.isIdle());
        assertEquals(AdaptiveCollectionAttempt.ExitReason.FAILED, nullFailed.exitReason());
        assertEquals(RouteStatus.FAILED, nullFailed.collectionRouteStatus());
        assertEquals(0, nullRoute.follower.followCount());
        assertEquals(0, nullRoute.returnBuildCount);
    }

    @Test
    public void configRejectsEveryNonPositiveOrNonFiniteTimeout() {
        assertInvalidConfig(null, "AdaptiveCollectionAttempt.Config is required");
        for (double value : new double[]{0.0, -0.1, Double.NaN,
                Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
            AdaptiveCollectionAttempt.Config collection =
                    AdaptiveCollectionAttempt.Config.defaults();
            collection.collectionRouteTimeoutSec = value;
            assertInvalidConfig(collection, "collectionRouteTimeoutSec", "got " + value);

            AdaptiveCollectionAttempt.Config returning =
                    AdaptiveCollectionAttempt.Config.defaults();
            returning.returnRouteTimeoutSec = value;
            assertInvalidConfig(returning, "returnRouteTimeoutSec", "got " + value);
        }
    }

    private static AdaptiveCollectionVisionService.Decision selectedDecision() {
        return AdaptiveCollectionVisionService.Decision.selectedForHardwareNeutralTest(12.0);
    }

    private static void assertGuardedDecision(AdaptiveCollectionAttempt.Status status) {
        try {
            status.frozenDecision();
            fail("expected frozen decision access to be guarded before route start");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("decisionFrozen()"));
        }
        try {
            status.usedFallback();
            fail("expected fallback access to be guarded before route start");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("decisionFrozen()"));
        }
    }

    private static void assertInvalidConfig(AdaptiveCollectionAttempt.Config config,
                                            String... messageFragments) {
        try {
            new Fixture(selectedDecision(), config);
            fail("expected invalid attempt config");
        } catch (RuntimeException expected) {
            for (String fragment : messageFragments) {
                assertTrue(
                        "expected message fragment '" + fragment + "' in "
                                + expected.getMessage(),
                        expected.getMessage() != null
                                && expected.getMessage().contains(fragment)
                );
            }
        }
    }

    private static final class Fixture {
        final ManualLoopClock time = new ManualLoopClock();
        final List<String> events = new ArrayList<String>();
        final ControlledFollower follower = new ControlledFollower(events);
        final List<AdaptiveCollectionPaths.Milestones> milestoneInstances =
                new ArrayList<AdaptiveCollectionPaths.Milestones>();
        final AdaptiveCollectionPaths paths;
        final Plant plant = Plants.fromOutputs()
                .power(new RecordingPowerOutput())
                .targetFromNewCommand(0.0)
                .build();
        final BasicPedroAutoMechanism mechanism =
                BasicPedroMechanismTestFactory.fromPlant(plant);
        final AdaptiveCollectionAttempt attempt;
        final Task task;

        boolean inventoryFull;
        int inventoryReads;
        int collectionBuildCount;
        int returnBuildCount;
        AdaptiveCollectionVisionService.Decision builtDecision;
        RuntimeException collectionBuildFailure;
        boolean collectionBuildReturnsNull;

        Fixture(AdaptiveCollectionVisionService.Decision decision) {
            this(decision, AdaptiveCollectionAttempt.Config.defaults());
        }

        Fixture(AdaptiveCollectionVisionService.Decision decision,
                AdaptiveCollectionAttempt.Config config) {
            paths = new AdaptiveCollectionPaths(
                    follower,
                    (frozenDecision, milestones) -> {
                        collectionBuildCount++;
                        builtDecision = frozenDecision;
                        milestoneInstances.add(milestones);
                        events.add("build.collection");
                        if (collectionBuildFailure != null) {
                            throw collectionBuildFailure;
                        }
                        return collectionBuildReturnsNull ? null : new PathChain();
                    },
                    () -> {
                        returnBuildCount++;
                        events.add("build.return");
                        return new PathChain();
                    }
            );
            BooleanSource inventorySource = BooleanSource.of(() -> {
                inventoryReads++;
                return inventoryFull;
            });
            attempt = new AdaptiveCollectionAttempt(
                    () -> decision,
                    paths,
                    inventorySource,
                    mechanism,
                    config
            );
            task = attempt.task();
        }

        void start() {
            task.start(time.clock());
        }

        void startAndFirstUpdate() {
            start();
            task.update(time.clock());
        }

        void update(double dtSec) {
            task.update(time.nextCycle(dtSec));
        }

        AdaptiveCollectionPaths.Milestones milestones() {
            assertFalse("collection route must be built first", milestoneInstances.isEmpty());
            return milestoneInstances.get(0);
        }

        ControlledExecution collectionExecution() {
            return follower.executions.get(0);
        }

        ControlledExecution returnExecution() {
            return follower.executions.get(1);
        }
    }

    private static final class ControlledFollower implements RouteFollower<PathChain> {
        private final List<String> events;
        final List<ControlledExecution> executions = new ArrayList<ControlledExecution>();

        ControlledFollower(List<String> events) {
            this.events = events;
        }

        @Override
        public RouteExecution follow(PathChain route) {
            ControlledExecution execution = new ControlledExecution(
                    executions.size() + 1,
                    events
            );
            executions.add(execution);
            events.add("follow." + executions.size());
            return execution;
        }

        int followCount() {
            return executions.size();
        }
    }

    private static final class ControlledExecution extends RouteExecution {
        private final int identity;
        private final List<String> events;
        RouteStatus integrationStatus = RouteStatus.ACTIVE;
        int cancelCount;

        ControlledExecution(int identity, List<String> events) {
            this.identity = identity;
            this.events = events;
        }

        @Override
        protected RouteStatus integrationStatus() {
            return integrationStatus;
        }

        @Override
        protected void cancelActive() {
            cancelCount++;
            events.add("cancel." + identity);
        }
    }

    private static final class RecordingPowerOutput implements PowerOutput {
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
