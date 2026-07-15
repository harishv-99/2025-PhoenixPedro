package edu.ftcphoenix.robots.phoenix.autonomous.pedro;

import org.junit.Test;

import java.lang.reflect.Field;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.route.RouteExecution;
import edu.ftcphoenix.fw.drive.route.RouteFollower;
import edu.ftcphoenix.fw.drive.route.RouteStatus;
import edu.ftcphoenix.fw.drive.route.RouteTask;
import edu.ftcphoenix.fw.drive.route.RouteTasks;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.task.TaskRunner;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.fw.testing.ManualLoopClock;
import edu.ftcphoenix.robots.phoenix.PhoenixCapabilities;
import edu.ftcphoenix.robots.phoenix.ScoringPath;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies Phoenix's bounded pre-park policy, one-park handoff, and cleanup contract. */
public final class PhoenixPedroAutoRoutineTaskTest {

    private static final RouteStatus[] CANCELLATION_LIKE_STATUSES = {
            RouteStatus.INTERRUPTED,
            RouteStatus.REPLACED,
            RouteStatus.CANCELLED,
            RouteStatus.FAILED,
            RouteStatus.UNKNOWN_TERMINAL
    };

    private static final RouteStatus[] ROUTE_TIMEOUT_STATUSES = {
            RouteStatus.FOLLOWER_TIMEOUT_OR_STALL,
            RouteStatus.TASK_TIMEOUT
    };

    @Test
    public void rootStartArmsOnlyAndSameCycleUpdatesDoNotStartOutbound() {
        Fixture fixture = new Fixture();

        fixture.routine.start(fixture.clock.clock());
        fixture.routine.update(fixture.clock.clock());
        fixture.routine.update(fixture.clock.clock());

        assertEquals(0, fixture.outboundFollower.followCount);
        assertEquals(0, fixture.scoringAttempt.startCount);
        assertEquals(0, fixture.returnFollower.followCount);

        fixture.advanceAndUpdate(0.01);
        assertEquals(1, fixture.outboundFollower.followCount);
        assertEquals(0, fixture.outboundFollower.updateCount);

        fixture.routine.update(fixture.clock.clock());
        assertEquals(0, fixture.outboundFollower.updateCount);

        fixture.advanceAndUpdate(0.01);
        fixture.routine.update(fixture.clock.clock());
        assertEquals(1, fixture.outboundFollower.updateCount);
    }

    @Test
    public void earlyPreParkCompletionStartsOneReturnThatCrossesThresholdUninterrupted() {
        Fixture fixture = new Fixture(0.05, 100.0);
        fixture.start();
        fixture.finishOutbound(RouteStatus.COMPLETED);
        fixture.finishScoring(TaskOutcome.SUCCESS);

        assertEquals(1, fixture.returnFollower.followCount);
        assertEquals(0, fixture.returnFollower.current.cancelCount);
        assertTrue(fixture.routine.getDebugName().contains("RETURN_OR_PARK"));

        fixture.advanceAndUpdate(0.10);

        assertFalse(fixture.routine.isComplete());
        assertEquals(1, fixture.returnFollower.followCount);
        assertEquals(0, fixture.returnFollower.current.cancelCount);

        fixture.finishReturn(RouteStatus.COMPLETED);
        assertEquals(TaskOutcome.SUCCESS, fixture.routine.getOutcome());
    }

    @Test
    public void cutoffBeforeFirstAutoLoopSkipsOutboundAndStartsParkFromRootStartBudget() {
        Fixture fixture = new Fixture(0.05, 100.0);
        fixture.routine.start(fixture.clock.clock());

        fixture.advanceAndUpdate(0.06);

        assertEquals(0, fixture.outboundFollower.followCount);
        assertEquals(1, fixture.returnFollower.followCount);
        assertFullCleanup(fixture);
        assertTrue(fixture.routine.getDebugName().contains("MATCH_TIME_CUTOFF"));

        fixture.finishReturn(RouteStatus.COMPLETED);
        assertEquals(TaskOutcome.TIMEOUT, fixture.routine.getOutcome());
    }

    @Test
    public void matchCutoffDuringOutboundCancelsItCleansEveryIntentAndStartsParkOnce() {
        Fixture fixture = new Fixture(0.05, 100.0);
        fixture.start();

        fixture.advanceAndUpdate(0.05);

        assertEquals(1, fixture.outboundFollower.current.cancelCount);
        assertEquals(0, fixture.scoringAttempt.startCount);
        assertEquals(1, fixture.returnFollower.followCount);
        assertFullCleanup(fixture);
        assertTrue(fixture.routine.getDebugName().contains("MATCH_TIME_CUTOFF"));

        fixture.advanceAndUpdate(0.10);
        assertEquals(1, fixture.returnFollower.followCount);
        assertEquals(0, fixture.returnFollower.current.cancelCount);

        fixture.finishReturn(RouteStatus.COMPLETED);
        assertEquals(TaskOutcome.TIMEOUT, fixture.routine.getOutcome());
    }

    @Test
    public void matchCutoffDuringScoringCancelsScoringAndStartsParkOnce() {
        Fixture fixture = new Fixture(0.05, 100.0);
        fixture.start();
        fixture.finishOutbound(RouteStatus.COMPLETED);
        assertEquals(1, fixture.scoringAttempt.startCount);

        fixture.advanceAndUpdate(0.05);

        assertEquals(1, fixture.scoringAttempt.cancelCount);
        assertEquals(1, fixture.returnFollower.followCount);
        assertFullCleanup(fixture);
        assertTrue(fixture.routine.getDebugName().contains("MATCH_TIME_CUTOFF"));

        fixture.finishReturn(RouteStatus.COMPLETED);
        assertEquals(TaskOutcome.TIMEOUT, fixture.routine.getOutcome());
    }

    @Test
    public void cancellationLikeOutboundHeartbeatAtExactCutoffSuppressesPark() {
        for (RouteStatus status : CANCELLATION_LIKE_STATUSES) {
            Fixture fixture = new Fixture(0.05, 100.0);
            fixture.start();

            // Pedro's composition-root heartbeat can terminalize the execution before the Task
            // phase gets its update on this exact threshold cycle.
            fixture.outboundFollower.finish(status);
            fixture.clock.nextCycle(0.04);
            fixture.routine.update(fixture.clock.clock());

            assertEquals(status.toString(), 0, fixture.returnFollower.followCount);
            assertEquals(status.toString(), TaskOutcome.CANCELLED, fixture.routine.getOutcome());
            assertTrue(status.toString(), fixture.routine.getDebugName().contains(status.name()));
            assertFullCleanup(fixture);

            CapturingDebugSink sink = new CapturingDebugSink();
            fixture.routine.debugDump(sink, "auto.policy");
            assertEquals(status.toString(), status, sink.values.get("auto.policy.lastRouteStatus"));
        }
    }

    @Test
    public void cancellationLikeScoringResultAtExactCutoffSuppressesPark() {
        TaskOutcome[] outcomes = {TaskOutcome.CANCELLED, TaskOutcome.UNKNOWN};
        for (TaskOutcome scoringOutcome : outcomes) {
            Fixture fixture = new Fixture(0.05, 100.0);
            fixture.start();
            fixture.finishOutbound(RouteStatus.COMPLETED);

            // The mechanism Task may terminalize after its previous policy update but before the
            // timeout decorator enforces the same exact match-time boundary.
            fixture.scoringAttempt.completeWith(scoringOutcome);
            fixture.clock.nextCycle(0.03);
            fixture.routine.update(fixture.clock.clock());

            assertEquals(scoringOutcome.toString(), 0, fixture.returnFollower.followCount);
            assertEquals(
                    scoringOutcome.toString(),
                    TaskOutcome.CANCELLED,
                    fixture.routine.getOutcome()
            );
            assertTrue(
                    scoringOutcome.toString(),
                    fixture.routine.getDebugName().contains("SCORING_" + scoringOutcome)
            );
            assertFullCleanup(fixture);
        }
    }

    @Test
    public void eachLocalOutboundTimeoutSkipsScoringAndRetainsTimeoutAfterPark() {
        for (RouteStatus status : ROUTE_TIMEOUT_STATUSES) {
            Fixture fixture = new Fixture(25.0, 0.05);
            fixture.start();
            fixture.finishOutbound(status);

            assertEquals(status.toString(), 0, fixture.scoringAttempt.startCount);
            assertEquals(status.toString(), 1, fixture.returnFollower.followCount);
            assertEquals(status.toString(), 1, fixture.scoring.flywheelDisableCount);
            assertEquals(status.toString(), 0, fixture.scoring.cancelTransientCount);
            assertEquals(status.toString(), 0, fixture.drive.stopCount);
            assertTrue(status.toString(), fixture.routine.getDebugName().contains(status.name()));

            fixture.finishReturn(RouteStatus.COMPLETED);
            assertEquals(status.toString(), TaskOutcome.TIMEOUT, fixture.routine.getOutcome());
        }
    }

    @Test
    public void scoringSuccessAndTimeoutAreBothParkEligibleWithTruthfulOutcome() {
        TaskOutcome[] outcomes = {TaskOutcome.SUCCESS, TaskOutcome.TIMEOUT};
        for (TaskOutcome scoringOutcome : outcomes) {
            Fixture fixture = new Fixture();
            fixture.start();
            fixture.finishOutbound(RouteStatus.COMPLETED);
            fixture.finishScoring(scoringOutcome);

            assertEquals(scoringOutcome.toString(), 1, fixture.returnFollower.followCount);
            assertEquals(scoringOutcome.toString(), 1, fixture.scoring.flywheelDisableCount);
            assertEquals(scoringOutcome.toString(), 0, fixture.scoringAttempt.updateCount);
            fixture.finishReturn(RouteStatus.COMPLETED);

            assertEquals(
                    scoringOutcome.toString(),
                    scoringOutcome == TaskOutcome.TIMEOUT
                            ? TaskOutcome.TIMEOUT
                            : TaskOutcome.SUCCESS,
                    fixture.routine.getOutcome()
            );
        }
    }

    @Test
    public void eachCancellationLikeOutboundResultSuppressesPark() {
        for (RouteStatus status : CANCELLATION_LIKE_STATUSES) {
            Fixture fixture = new Fixture();
            fixture.start();
            fixture.finishOutbound(status);

            assertTrue(status.toString(), fixture.routine.isComplete());
            assertEquals(status.toString(), TaskOutcome.CANCELLED, fixture.routine.getOutcome());
            assertEquals(status.toString(), 0, fixture.scoringAttempt.startCount);
            assertEquals(status.toString(), 0, fixture.returnFollower.followCount);
            assertEquals(status.toString(), 1, fixture.scoring.flywheelDisableCount);
        }
    }

    @Test
    public void scoringCancellationAndUnknownSuppressPark() {
        TaskOutcome[] outcomes = {TaskOutcome.CANCELLED, TaskOutcome.UNKNOWN};
        for (TaskOutcome scoringOutcome : outcomes) {
            Fixture fixture = new Fixture();
            fixture.start();
            fixture.finishOutbound(RouteStatus.COMPLETED);
            fixture.finishScoring(scoringOutcome);

            assertTrue(scoringOutcome.toString(), fixture.routine.isComplete());
            assertEquals(scoringOutcome.toString(), TaskOutcome.CANCELLED, fixture.routine.getOutcome());
            assertEquals(scoringOutcome.toString(), 0, fixture.returnFollower.followCount);
            assertTrue(
                    scoringOutcome.toString(),
                    fixture.routine.getDebugName().contains("SCORING_" + scoringOutcome)
            );
        }
    }

    @Test
    public void everyFinalRouteTimeoutFinishesWithoutSecondFallback() {
        for (RouteStatus status : ROUTE_TIMEOUT_STATUSES) {
            Fixture fixture = fixtureAtReturn(TaskOutcome.SUCCESS);
            fixture.finishReturn(status);

            assertTrue(status.toString(), fixture.routine.isComplete());
            assertEquals(status.toString(), TaskOutcome.TIMEOUT, fixture.routine.getOutcome());
            assertEquals(status.toString(), 1, fixture.returnFollower.followCount);
        }
    }

    @Test
    public void everyCancellationLikeFinalRouteResultWinsOverSuccessOrRetainedTimeout() {
        for (RouteStatus status : CANCELLATION_LIKE_STATUSES) {
            Fixture afterSuccess = fixtureAtReturn(TaskOutcome.SUCCESS);
            afterSuccess.finishReturn(status);
            assertEquals(status.toString(), TaskOutcome.CANCELLED, afterSuccess.routine.getOutcome());
            assertEquals(status.toString(), 1, afterSuccess.returnFollower.followCount);

            Fixture afterTimeout = fixtureAtReturn(TaskOutcome.TIMEOUT);
            afterTimeout.finishReturn(status);
            assertEquals(status.toString(), TaskOutcome.CANCELLED, afterTimeout.routine.getOutcome());
            assertEquals(status.toString(), 1, afterTimeout.returnFollower.followCount);
        }
    }

    @Test
    public void directCancellationBeforeStartIsNoOp() {
        Fixture fixture = new Fixture();

        fixture.routine.cancel();

        assertFalse(fixture.routine.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, fixture.routine.getOutcome());
        assertNoFullCleanup(fixture);
        assertEquals(0, fixture.returnFollower.followCount);
    }

    @Test
    public void directCancellationDuringOutboundAndScoringCleansAndNeverParks() {
        Fixture outbound = new Fixture();
        outbound.start();
        outbound.routine.cancel();
        outbound.routine.cancel();
        assertEquals(TaskOutcome.CANCELLED, outbound.routine.getOutcome());
        assertEquals(1, outbound.outboundFollower.current.cancelCount);
        assertEquals(0, outbound.returnFollower.followCount);
        assertFullCleanup(outbound);

        Fixture scoring = new Fixture();
        scoring.start();
        scoring.finishOutbound(RouteStatus.COMPLETED);
        scoring.routine.cancel();
        scoring.routine.cancel();
        assertEquals(TaskOutcome.CANCELLED, scoring.routine.getOutcome());
        assertEquals(1, scoring.scoringAttempt.cancelCount);
        assertEquals(0, scoring.returnFollower.followCount);
        assertFullCleanup(scoring);
    }

    @Test
    public void directCancellationDuringParkCancelsOnlyThatOneParkAndCannotRestartIt() {
        Fixture fixture = fixtureAtReturn(TaskOutcome.SUCCESS);

        fixture.routine.cancel();
        fixture.routine.cancel();

        assertEquals(TaskOutcome.CANCELLED, fixture.routine.getOutcome());
        assertEquals(1, fixture.returnFollower.followCount);
        assertEquals(1, fixture.returnFollower.current.cancelCount);
        // Natural pre-park completion already removed the flywheel request; the completed
        // pre-park Task is not misused as a post-terminal cleanup callback.
        assertEquals(1, fixture.scoring.flywheelDisableCount);
        assertEquals(0, fixture.scoring.cancelTransientCount);
    }

    @Test
    public void cutoffCleanupAttemptsEveryActionAndSuppressesFailuresInOrder() {
        Fixture fixture = new Fixture(0.05, 100.0);
        fixture.start();
        IllegalStateException childFailure = new IllegalStateException("child cancel failed");
        IllegalStateException transientFailure = new IllegalStateException("transient failed");
        IllegalStateException intakeFailure = new IllegalStateException("intake failed");
        IllegalStateException shootingFailure = new IllegalStateException("shooting failed");
        IllegalStateException ejectFailure = new IllegalStateException("eject failed");
        IllegalStateException flywheelFailure = new IllegalStateException("flywheel failed");
        IllegalStateException driveFailure = new IllegalStateException("drive failed");
        fixture.outboundFollower.current.cancelFailure = childFailure;
        fixture.scoring.cancelTransientFailure = transientFailure;
        fixture.scoring.intakeDisableFailure = intakeFailure;
        fixture.scoring.shootingDisableFailure = shootingFailure;
        fixture.scoring.ejectDisableFailure = ejectFailure;
        fixture.scoring.flywheelDisableFailure = flywheelFailure;
        fixture.drive.stopFailure = driveFailure;

        try {
            fixture.advanceAndUpdate(0.05);
            fail("expected aggregate cleanup failure");
        } catch (IllegalStateException actual) {
            assertSame(childFailure, actual);
        }

        assertEquals(1, fixture.outboundFollower.current.cancelCount);
        assertFullCleanup(fixture);
        assertEquals(0, fixture.returnFollower.followCount);
        assertEquals(6, childFailure.getSuppressed().length);
        assertSame(transientFailure, childFailure.getSuppressed()[0]);
        assertSame(intakeFailure, childFailure.getSuppressed()[1]);
        assertSame(shootingFailure, childFailure.getSuppressed()[2]);
        assertSame(ejectFailure, childFailure.getSuppressed()[3]);
        assertSame(flywheelFailure, childFailure.getSuppressed()[4]);
        assertSame(driveFailure, childFailure.getSuppressed()[5]);
    }

    @Test
    public void taskRunnerFailureDuringScoringStartFailsClosedAndSuppressesPark() {
        Fixture fixture = new Fixture();
        TaskRunner runner = new TaskRunner();
        runner.enqueue(fixture.routine);
        runner.update(fixture.clock.clock());
        fixture.clock.nextCycle(0.01);
        runner.update(fixture.clock.clock());

        IllegalStateException startFailure = new IllegalStateException("scoring start failed");
        fixture.scoringAttempt.startFailure = startFailure;
        fixture.outboundFollower.finish(RouteStatus.COMPLETED);
        fixture.clock.nextCycle(0.01);

        try {
            runner.update(fixture.clock.clock());
            fail("expected scoring start failure");
        } catch (IllegalStateException actual) {
            assertSame(startFailure, actual);
        }

        assertTrue(runner.isIdle());
        assertEquals(TaskOutcome.CANCELLED, fixture.routine.getOutcome());
        assertEquals(1, fixture.scoringAttempt.cancelCount);
        assertEquals(0, fixture.returnFollower.followCount);
        assertFullCleanup(fixture);
    }

    @Test
    public void liveParkConstructionFailureFailsClosedWithoutCallingFollower() {
        ManualLoopClock clock = new ManualLoopClock();
        ControlledFollower outboundFollower = new ControlledFollower();
        ControlledFollower returnFollower = new ControlledFollower();
        ControlledTask scoringAttempt = new ControlledTask();
        RecordingScoring scoring = new RecordingScoring();
        RecordingDriveSink drive = new RecordingDriveSink();
        RouteTask<String> outbound = newRoute("outbound", outboundFollower, 100.0);
        IllegalStateException buildFailure = new IllegalStateException("live return unavailable");
        RouteTask<String> liveReturn = RouteTasks.followBuiltAtStart(
                "liveReturn",
                returnFollower,
                () -> {
                    throw buildFailure;
                },
                routeConfig(100.0)
        );
        PhoenixPedroPreParkTask prePark = new PhoenixPedroPreParkTask(
                "returnBuildFailure", outbound, scoringAttempt, scoring, drive);
        PhoenixPedroAutoRoutineTask routine = new PhoenixPedroAutoRoutineTask(
                "returnBuildFailure", prePark, 25.0, liveReturn);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(routine);
        runner.update(clock.clock());
        clock.nextCycle(0.01);
        runner.update(clock.clock());

        outboundFollower.finish(RouteStatus.COMPLETED);
        clock.nextCycle(0.01);
        runner.update(clock.clock());
        scoringAttempt.completeWith(TaskOutcome.SUCCESS);
        clock.nextCycle(0.01);

        try {
            runner.update(clock.clock());
            fail("expected live-return build failure");
        } catch (IllegalStateException actual) {
            assertSame(buildFailure, actual.getCause());
            assertTrue(actual.getMessage().contains("liveReturn"));
        }

        assertTrue(runner.isIdle());
        assertEquals(TaskOutcome.CANCELLED, routine.getOutcome());
        assertEquals(0, returnFollower.followCount);
        assertEquals(1, scoring.flywheelDisableCount);
        CapturingDebugSink sink = new CapturingDebugSink();
        routine.debugDump(sink, "auto.policy");
        assertEquals("FAILED", sink.values.get("auto.policy.trigger"));
        assertEquals(RouteStatus.FAILED, sink.values.get("auto.policy.lastRouteStatus"));
    }

    @Test
    public void liveParkFactoryIsSampledOnceOnlyWhenParkStarts() {
        ManualLoopClock clock = new ManualLoopClock();
        ControlledFollower outboundFollower = new ControlledFollower();
        ControlledFollower returnFollower = new ControlledFollower();
        ControlledTask scoringAttempt = new ControlledTask();
        RecordingScoring scoring = new RecordingScoring();
        RecordingDriveSink drive = new RecordingDriveSink();
        AtomicInteger builds = new AtomicInteger();
        RouteTask<String> outbound = newRoute("outbound", outboundFollower, 100.0);
        RouteTask<String> liveReturn = RouteTasks.followBuiltAtStart(
                "liveReturn",
                returnFollower,
                () -> {
                    builds.incrementAndGet();
                    return "live-route";
                },
                routeConfig(100.0)
        );
        PhoenixPedroPreParkTask prePark = new PhoenixPedroPreParkTask(
                "liveBuild", outbound, scoringAttempt, scoring, drive);
        PhoenixPedroAutoRoutineTask routine = new PhoenixPedroAutoRoutineTask(
                "liveBuild", prePark, 25.0, liveReturn);

        routine.start(clock.clock());
        routine.debugDump(new CapturingDebugSink(), "auto");
        assertEquals(0, builds.get());
        clock.nextCycle(0.01);
        routine.update(clock.clock());
        assertEquals(0, builds.get());

        outboundFollower.finish(RouteStatus.COMPLETED);
        clock.nextCycle(0.01);
        routine.update(clock.clock());
        scoringAttempt.completeWith(TaskOutcome.SUCCESS);
        clock.nextCycle(0.01);
        routine.update(clock.clock());

        assertEquals(1, builds.get());
        assertEquals(1, returnFollower.followCount);
        routine.update(clock.clock());
        assertEquals(1, builds.get());
    }

    @Test
    public void immediateChildrenAdvanceOnlyAfterFirstLaterCycle() {
        Fixture fixture = new Fixture();
        fixture.outboundFollower.statusOnFollow = RouteStatus.COMPLETED;
        fixture.scoringAttempt.completeOnStart = TaskOutcome.SUCCESS;
        fixture.returnFollower.statusOnFollow = RouteStatus.COMPLETED;

        fixture.routine.start(fixture.clock.clock());
        fixture.routine.update(fixture.clock.clock());
        assertFalse(fixture.routine.isComplete());
        assertEquals(0, fixture.outboundFollower.followCount);

        fixture.advanceAndUpdate(0.01);

        assertTrue(fixture.routine.isComplete());
        assertEquals(TaskOutcome.SUCCESS, fixture.routine.getOutcome());
        assertEquals(1, fixture.outboundFollower.followCount);
        assertEquals(1, fixture.scoringAttempt.startCount);
        assertEquals(1, fixture.returnFollower.followCount);
    }

    @Test
    public void reentrantPolicyCallbacksNeverDuplicateOrReleaseParkAfterCancellation() {
        Fixture updateReentry = new Fixture();
        updateReentry.start();
        updateReentry.scoring.onFlywheelDisable =
                () -> updateReentry.routine.update(updateReentry.clock.clock());
        updateReentry.finishOutbound(RouteStatus.FOLLOWER_TIMEOUT_OR_STALL);
        assertEquals(1, updateReentry.scoring.flywheelDisableCount);
        assertEquals(1, updateReentry.returnFollower.followCount);

        Fixture cleanupCancel = new Fixture();
        cleanupCancel.start();
        cleanupCancel.scoring.onFlywheelDisable = cleanupCancel.routine::cancel;
        cleanupCancel.finishOutbound(RouteStatus.FOLLOWER_TIMEOUT_OR_STALL);
        assertEquals(TaskOutcome.CANCELLED, cleanupCancel.routine.getOutcome());
        assertEquals(1, cleanupCancel.scoring.flywheelDisableCount);
        assertEquals(0, cleanupCancel.returnFollower.followCount);

        Fixture outcomeCancel = new Fixture();
        outcomeCancel.start();
        outcomeCancel.finishOutbound(RouteStatus.COMPLETED);
        outcomeCancel.scoringAttempt.completeWith(TaskOutcome.SUCCESS);
        outcomeCancel.scoringAttempt.onGetOutcome = outcomeCancel.routine::cancel;
        outcomeCancel.advanceAndUpdate(0.01);
        assertEquals(TaskOutcome.CANCELLED, outcomeCancel.routine.getOutcome());
        assertEquals(0, outcomeCancel.returnFollower.followCount);
        assertFullCleanup(outcomeCancel);
    }

    @Test
    public void cutoffCancelsParallelDeadlineMechanismWaitAndCompanionBeforePark() {
        ManualLoopClock clock = new ManualLoopClock();
        ControlledFollower outboundFollower = new ControlledFollower();
        ControlledFollower returnFollower = new ControlledFollower();
        RecordingScoring scoring = new RecordingScoring();
        RecordingDriveSink drive = new RecordingDriveSink();
        Task mechanismWait = Tasks.waitForSeconds(100.0);
        ControlledTask companion = new ControlledTask();
        Task scoringGraph = Tasks.parallelDeadline(mechanismWait, companion);
        PhoenixPedroPreParkTask prePark = new PhoenixPedroPreParkTask(
                "parallelCutoff",
                newRoute("outbound", outboundFollower, 100.0),
                scoringGraph,
                scoring,
                drive
        );
        PhoenixPedroAutoRoutineTask routine = new PhoenixPedroAutoRoutineTask(
                "parallelCutoff",
                prePark,
                0.05,
                newRoute("return", returnFollower, 100.0)
        );

        routine.start(clock.clock());
        clock.nextCycle(0.01);
        routine.update(clock.clock());
        outboundFollower.finish(RouteStatus.COMPLETED);
        clock.nextCycle(0.01);
        routine.update(clock.clock());
        assertEquals(1, companion.startCount);

        clock.nextCycle(0.04);
        routine.update(clock.clock());

        assertEquals(TaskOutcome.CANCELLED, mechanismWait.getOutcome());
        assertEquals(1, companion.cancelCount);
        assertEquals(1, returnFollower.followCount);
        assertFullCleanup(scoring, drive);
    }

    @Test
    public void lifecycleAndConstructionFailuresAreActionable() {
        Fixture fixture = new Fixture();
        assertFailureContains(
                () -> fixture.routine.update(fixture.clock.clock()),
                "before start",
                "PhoenixPedroAutoRoutineFactory.build"
        );

        fixture.routine.start(fixture.clock.clock());
        assertFailureContains(
                () -> fixture.routine.start(fixture.clock.clock()),
                "single-use",
                "fresh Task graph"
        );

        ControlledFollower follower = new ControlledFollower();
        RouteTask<String> sharedRoute = newRoute("shared", follower, 100.0);
        ControlledTask ordinary = new ControlledTask();
        RecordingScoring scoring = new RecordingScoring();
        RecordingDriveSink drive = new RecordingDriveSink();

        assertFailureContains(
                () -> new PhoenixPedroPreParkTask(
                        "alias", sharedRoute, sharedRoute, scoring, drive),
                "outboundRoute",
                "scoringAttempt",
                "fresh Task"
        );
        PhoenixPedroPreParkTask prePark = new PhoenixPedroPreParkTask(
                "alias", sharedRoute, ordinary, scoring, drive);
        assertFailureContains(
                () -> new PhoenixPedroAutoRoutineTask("alias", prePark, 25.0, sharedRoute),
                "returnOrParkRoute",
                "pre-park child",
                "fresh"
        );

        double[] invalidThresholds = {0.0, -1.0, Double.NaN, Double.POSITIVE_INFINITY};
        for (double threshold : invalidThresholds) {
            PhoenixPedroPreParkTask freshPrePark = new PhoenixPedroPreParkTask(
                    "invalid", newRoute("outbound", new ControlledFollower(), 100.0),
                    new ControlledTask(), new RecordingScoring(), new RecordingDriveSink());
            assertFailureContains(
                    () -> new PhoenixPedroAutoRoutineTask(
                            "invalid", freshPrePark, threshold,
                            newRoute("return", new ControlledFollower(), 100.0)),
                    "parkTakeoverElapsedSec",
                    "finite and > 0"
            );
        }
    }

    @Test
    public void malformedCompletedChildrenFailClosedWithRoleGuidanceAndNoPark() throws Exception {
        TaskOutcome[] malformedScoring = {null, TaskOutcome.NOT_DONE};
        for (TaskOutcome outcome : malformedScoring) {
            Fixture fixture = new Fixture();
            TaskRunner runner = startWithRunner(fixture);
            finishOutboundWithRunner(fixture, runner, RouteStatus.COMPLETED);
            fixture.scoringAttempt.completeWith(outcome);
            fixture.clock.nextCycle(0.01);

            assertRunnerFailureContains(
                    runner,
                    fixture.clock.clock(),
                    "scoringAttempt",
                    String.valueOf(outcome),
                    "fresh child Task graph"
            );
            assertTrue(runner.isIdle());
            assertEquals(0, fixture.returnFollower.followCount);
            assertFullCleanup(fixture);
        }

        Object[] malformedRouteStatuses = {null, RouteStatus.ACTIVE, RouteStatus.NOT_STARTED};
        for (Object status : malformedRouteStatuses) {
            Fixture fixture = new Fixture();
            TaskRunner runner = startWithRunner(fixture);
            setPrivate(fixture.outboundRoute, "complete", true);
            setPrivate(fixture.outboundRoute, "routeStatus", status);
            fixture.clock.nextCycle(0.01);

            assertRunnerFailureContains(
                    runner,
                    fixture.clock.clock(),
                    "outboundRoute",
                    String.valueOf(status),
                    "terminal RouteStatus"
            );
            assertTrue(runner.isIdle());
            assertEquals(0, fixture.returnFollower.followCount);
            assertFullCleanup(fixture);
        }

        Fixture exactCutoff = new Fixture(0.05, 100.0);
        TaskRunner exactRunner = startWithRunner(exactCutoff);
        finishOutboundWithRunner(exactCutoff, exactRunner, RouteStatus.COMPLETED);
        exactCutoff.scoringAttempt.completeWith(TaskOutcome.NOT_DONE);
        exactCutoff.clock.nextCycle(0.03);
        assertRunnerFailureContains(
                exactRunner,
                exactCutoff.clock.clock(),
                "scoringAttempt",
                "isComplete=true",
                "outcome=NOT_DONE"
        );
        assertTrue(exactRunner.isIdle());
        assertEquals(0, exactCutoff.returnFollower.followCount);
        assertFullCleanup(exactCutoff);
    }

    @Test
    public void debugDumpDistinguishesLocalTimeoutFromMatchCutoff() {
        Fixture local = new Fixture(25.0, 0.05);
        local.start();
        local.finishOutbound(RouteStatus.FOLLOWER_TIMEOUT_OR_STALL);
        CapturingDebugSink localSink = new CapturingDebugSink();
        local.routine.debugDump(localSink, "auto.policy");

        assertEquals("RETURN_OR_PARK", String.valueOf(localSink.values.get("auto.policy.phase")));
        assertEquals("FALLBACK", String.valueOf(localSink.values.get("auto.policy.decision")));
        assertEquals(Boolean.FALSE, localSink.values.get("auto.policy.matchTimeCutoff"));
        assertEquals(
                "FOLLOWER_TIMEOUT_OR_STALL",
                localSink.values.get("auto.policy.trigger")
        );
        assertEquals(
                RouteStatus.FOLLOWER_TIMEOUT_OR_STALL,
                localSink.values.get("auto.policy.lastRouteStatus")
        );
        assertEquals(TaskOutcome.TIMEOUT, localSink.values.get("auto.policy.accumulatedOutcome"));

        Fixture match = new Fixture(0.05, 100.0);
        match.start();
        match.advanceAndUpdate(0.05);
        CapturingDebugSink matchSink = new CapturingDebugSink();
        match.routine.debugDump(matchSink, "auto.policy");

        assertEquals(Boolean.TRUE, matchSink.values.get("auto.policy.matchTimeCutoff"));
        assertEquals("MATCH_TIME_CUTOFF", matchSink.values.get("auto.policy.trigger"));
        assertEquals(TaskOutcome.CANCELLED, matchSink.values.get("auto.policy.preParkOutcome"));
        assertEquals(TaskOutcome.TIMEOUT, matchSink.values.get("auto.policy.accumulatedOutcome"));
    }

    private static Fixture fixtureAtReturn(TaskOutcome scoringOutcome) {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.finishOutbound(RouteStatus.COMPLETED);
        fixture.finishScoring(scoringOutcome);
        return fixture;
    }

    private static void assertFullCleanup(Fixture fixture) {
        assertFullCleanup(fixture.scoring, fixture.drive);
    }

    private static void assertFullCleanup(RecordingScoring scoring, RecordingDriveSink drive) {
        assertEquals(1, scoring.cancelTransientCount);
        assertEquals(1, scoring.intakeDisableCount);
        assertEquals(1, scoring.shootingDisableCount);
        assertEquals(1, scoring.ejectDisableCount);
        assertEquals(1, scoring.flywheelDisableCount);
        assertEquals(1, drive.stopCount);
    }

    private static void assertNoFullCleanup(Fixture fixture) {
        assertEquals(0, fixture.scoring.cancelTransientCount);
        assertEquals(0, fixture.scoring.intakeDisableCount);
        assertEquals(0, fixture.scoring.shootingDisableCount);
        assertEquals(0, fixture.scoring.ejectDisableCount);
        assertEquals(0, fixture.scoring.flywheelDisableCount);
        assertEquals(0, fixture.drive.stopCount);
    }

    private static RouteTask.Config routeConfig(double timeoutSec) {
        RouteTask.Config config = new RouteTask.Config();
        config.timeoutSec = timeoutSec;
        return config;
    }

    private static RouteTask<String> newRoute(String name,
                                               ControlledFollower follower,
                                               double timeoutSec) {
        return RouteTasks.follow(name, follower, name + "-route", routeConfig(timeoutSec));
    }

    private static void assertFailureContains(ThrowingAction action, String... fragments) {
        try {
            action.run();
            fail("expected failure containing " + fragments[0]);
        } catch (RuntimeException expected) {
            for (String fragment : fragments) {
                assertTrue(
                        "missing '" + fragment + "' in: " + expected.getMessage(),
                        expected.getMessage() != null && expected.getMessage().contains(fragment)
                );
            }
        }
    }

    private static TaskRunner startWithRunner(Fixture fixture) {
        TaskRunner runner = new TaskRunner();
        runner.enqueue(fixture.routine);
        runner.update(fixture.clock.clock());
        fixture.clock.nextCycle(0.01);
        runner.update(fixture.clock.clock());
        assertEquals(1, fixture.outboundFollower.followCount);
        return runner;
    }

    private static void finishOutboundWithRunner(Fixture fixture,
                                                 TaskRunner runner,
                                                 RouteStatus status) {
        fixture.outboundFollower.finish(status);
        fixture.clock.nextCycle(0.01);
        runner.update(fixture.clock.clock());
    }

    private static void assertRunnerFailureContains(TaskRunner runner,
                                                    LoopClock clock,
                                                    String... fragments) {
        try {
            runner.update(clock);
            fail("expected failure containing " + fragments[0]);
        } catch (RuntimeException expected) {
            for (String fragment : fragments) {
                assertTrue(
                        "missing '" + fragment + "' in: " + expected.getMessage(),
                        expected.getMessage() != null && expected.getMessage().contains(fragment)
                );
            }
        }
    }

    private static void setPrivate(Object target, String fieldName, Object value) throws Exception {
        Field field = target.getClass().getDeclaredField(fieldName);
        field.setAccessible(true);
        field.set(target, value);
    }

    private interface ThrowingAction {
        void run();
    }

    private static final class Fixture {
        private final double routeTimeoutSec;
        private final ManualLoopClock clock = new ManualLoopClock();
        private final ControlledFollower outboundFollower = new ControlledFollower();
        private final ControlledFollower returnFollower = new ControlledFollower();
        private final RouteTask<String> outboundRoute;
        private final RouteTask<String> returnRoute;
        private final ControlledTask scoringAttempt = new ControlledTask();
        private final RecordingScoring scoring = new RecordingScoring();
        private final RecordingDriveSink drive = new RecordingDriveSink();
        private final PhoenixPedroPreParkTask prePark;
        private final PhoenixPedroAutoRoutineTask routine;

        private Fixture() {
            this(25.0, 0.50);
        }

        private Fixture(double takeoverSec, double routeTimeoutSec) {
            this.routeTimeoutSec = routeTimeoutSec;
            outboundRoute = newRoute("outbound", outboundFollower, routeTimeoutSec);
            returnRoute = newRoute("return", returnFollower, routeTimeoutSec);
            prePark = new PhoenixPedroPreParkTask(
                    "testRoutine", outboundRoute, scoringAttempt, scoring, drive);
            routine = new PhoenixPedroAutoRoutineTask(
                    "testRoutine", prePark, takeoverSec, returnRoute);
        }

        private void start() {
            routine.start(clock.clock());
            routine.update(clock.clock());
            assertEquals(0, outboundFollower.followCount);
            advanceAndUpdate(0.01);
            assertEquals(1, outboundFollower.followCount);
        }

        private void advanceAndUpdate(double dtSec) {
            clock.nextCycle(dtSec);
            routine.update(clock.clock());
        }

        private void finishOutbound(RouteStatus status) {
            finishRoute(outboundFollower, status);
        }

        private void finishScoring(TaskOutcome scoringOutcome) {
            scoringAttempt.completeWith(scoringOutcome);
            advanceAndUpdate(0.01);
        }

        private void finishReturn(RouteStatus status) {
            finishRoute(returnFollower, status);
        }

        private void finishRoute(ControlledFollower follower, RouteStatus status) {
            if (status == RouteStatus.TASK_TIMEOUT) {
                advanceAndUpdate(routeTimeoutSec + 0.01);
            } else {
                follower.finish(status);
                advanceAndUpdate(0.01);
            }
        }
    }

    private static final class ControlledFollower implements RouteFollower<String> {
        private ControlledExecution current;
        private RouteStatus statusOnFollow = RouteStatus.ACTIVE;
        private RuntimeException followFailure;
        private int followCount;
        private int updateCount;

        @Override
        public RouteExecution follow(String route) {
            followCount++;
            if (followFailure != null) {
                throw followFailure;
            }
            current = new ControlledExecution();
            current.integrationStatus = statusOnFollow;
            return current;
        }

        @Override
        public void update(LoopClock clock) {
            updateCount++;
        }

        private void finish(RouteStatus status) {
            if (current == null) {
                throw new IllegalStateException("route has not started");
            }
            if (status == RouteStatus.NOT_STARTED
                    || status == RouteStatus.ACTIVE
                    || status == RouteStatus.TASK_TIMEOUT) {
                throw new IllegalArgumentException("fixture requires an integration terminal status");
            }
            current.integrationStatus = status;
        }
    }

    private static final class ControlledExecution extends RouteExecution {
        private RouteStatus integrationStatus = RouteStatus.ACTIVE;
        private int cancelCount;
        private RuntimeException cancelFailure;

        @Override
        protected RouteStatus integrationStatus() {
            return integrationStatus;
        }

        @Override
        protected void cancelActive() {
            cancelCount++;
            if (integrationStatus == RouteStatus.ACTIVE) {
                integrationStatus = RouteStatus.CANCELLED;
            }
            if (cancelFailure != null) {
                throw cancelFailure;
            }
        }
    }

    private static final class ControlledTask implements Task {
        private boolean startAttempted;
        private boolean started;
        private boolean complete;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;
        private TaskOutcome completeOnStart;
        private RuntimeException startFailure;
        private RuntimeException cancelFailure;
        private Runnable onGetOutcome;
        private int startCount;
        private int updateCount;
        private int cancelCount;

        @Override
        public void start(LoopClock clock) {
            if (startAttempted) {
                throw new IllegalStateException("controlled Task reused");
            }
            startAttempted = true;
            started = true;
            startCount++;
            if (completeOnStart != null) {
                completeWith(completeOnStart);
            }
            if (startFailure != null) {
                throw startFailure;
            }
        }

        @Override
        public void update(LoopClock clock) {
            if (!started) {
                throw new IllegalStateException("controlled Task updated before start");
            }
            if (!complete) {
                updateCount++;
            }
        }

        @Override
        public void cancel() {
            if (!started || complete) {
                return;
            }
            complete = true;
            outcome = TaskOutcome.CANCELLED;
            cancelCount++;
            if (cancelFailure != null) {
                throw cancelFailure;
            }
        }

        @Override
        public boolean isComplete() {
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            Runnable callback = onGetOutcome;
            onGetOutcome = null;
            if (callback != null) {
                callback.run();
            }
            return outcome;
        }

        @Override
        public String getDebugName() {
            return "scoringAttempt";
        }

        private void completeWith(TaskOutcome terminalOutcome) {
            complete = true;
            outcome = terminalOutcome;
        }
    }

    private static final class RecordingScoring implements PhoenixCapabilities.Scoring {
        private int intakeDisableCount;
        private int flywheelDisableCount;
        private int shootingDisableCount;
        private int ejectDisableCount;
        private int cancelTransientCount;
        private RuntimeException intakeDisableFailure;
        private RuntimeException flywheelDisableFailure;
        private RuntimeException shootingDisableFailure;
        private RuntimeException ejectDisableFailure;
        private RuntimeException cancelTransientFailure;
        private Runnable onFlywheelDisable;

        @Override
        public void setIntakeEnabled(boolean enabled) {
            if (!enabled) {
                intakeDisableCount++;
                if (intakeDisableFailure != null) throw intakeDisableFailure;
            }
        }

        @Override
        public void setFlywheelEnabled(boolean enabled) {
            if (!enabled) {
                flywheelDisableCount++;
                Runnable callback = onFlywheelDisable;
                onFlywheelDisable = null;
                if (callback != null) callback.run();
                if (flywheelDisableFailure != null) throw flywheelDisableFailure;
            }
        }

        @Override
        public void setShootingEnabled(boolean enabled) {
            if (!enabled) {
                shootingDisableCount++;
                if (shootingDisableFailure != null) throw shootingDisableFailure;
            }
        }

        @Override
        public void setEjectEnabled(boolean enabled) {
            if (!enabled) {
                ejectDisableCount++;
                if (ejectDisableFailure != null) throw ejectDisableFailure;
            }
        }

        @Override
        public void requestSingleShot() {
        }

        @Override
        public void requestShots(int shotCount) {
        }

        @Override
        public void cancelTransientActions() {
            cancelTransientCount++;
            if (cancelTransientFailure != null) throw cancelTransientFailure;
        }

        @Override
        public void setSelectedVelocityNative(double velocityNative) {
        }

        @Override
        public void adjustSelectedVelocityNative(double deltaNative) {
        }

        @Override
        public void captureSuggestedShotVelocity() {
        }

        @Override
        public boolean hasPendingShots() {
            return false;
        }

        @Override
        public ScoringPath.Status status() {
            return null;
        }
    }

    private static final class RecordingDriveSink implements DriveCommandSink {
        private int stopCount;
        private RuntimeException stopFailure;

        @Override
        public void drive(DriveSignal signal) {
        }

        @Override
        public void stop() {
            stopCount++;
            if (stopFailure != null) throw stopFailure;
        }
    }

    private static final class CapturingDebugSink implements DebugSink {
        private final Map<String, Object> values = new LinkedHashMap<>();

        @Override
        public DebugSink addData(String key, Object value) {
            values.put(key, value);
            return this;
        }

        @Override
        public DebugSink addLine(String text) {
            return this;
        }
    }
}
