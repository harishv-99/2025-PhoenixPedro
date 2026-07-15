package edu.ftcphoenix.robots.phoenix.autonomous.pedro;

import org.junit.Test;

import java.lang.reflect.Field;
import java.util.LinkedHashMap;
import java.util.Map;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.route.RouteExecution;
import edu.ftcphoenix.fw.drive.route.RouteFollower;
import edu.ftcphoenix.fw.drive.route.RouteStatus;
import edu.ftcphoenix.fw.drive.route.RouteTask;
import edu.ftcphoenix.fw.drive.route.RouteTasks;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.task.TaskRunner;
import edu.ftcphoenix.fw.testing.ManualLoopClock;
import edu.ftcphoenix.robots.phoenix.PhoenixCapabilities;
import edu.ftcphoenix.robots.phoenix.ScoringPath;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies Phoenix's private outbound/scoring/return policy and Task lifecycle contract. */
public final class PhoenixPedroAutoRoutineTaskTest {

    private static final RouteStatus[] OUTBOUND_ABORT_STATUSES = {
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
    public void completedOutboundRunsScoringAndCompletedReturnSucceeds() {
        Fixture fixture = new Fixture();
        fixture.start();

        fixture.finishOutbound(RouteStatus.COMPLETED);
        assertEquals(1, fixture.scoringAttempt.startCount);
        assertEquals(0, fixture.returnFollower.followCount);

        fixture.finishScoring(TaskOutcome.SUCCESS);
        assertEquals(1, fixture.scoring.disableCount);
        assertEquals(1, fixture.returnFollower.followCount);

        fixture.finishReturn(RouteStatus.COMPLETED);

        assertTrue(fixture.routine.isComplete());
        assertEquals(TaskOutcome.SUCCESS, fixture.routine.getOutcome());
        assertEquals(0, fixture.scoringAttempt.cancelCount);
        assertEquals(1, fixture.scoring.disableCount);
    }

    @Test
    public void eachOutboundTimeoutSkipsScoringAndRunsLiveReturnFallback() {
        for (RouteStatus status : ROUTE_TIMEOUT_STATUSES) {
            Fixture fixture = new Fixture();
            fixture.start();

            fixture.finishOutbound(status);

            assertEquals(status.toString(), 0, fixture.scoringAttempt.startCount);
            assertEquals(status.toString(), 1, fixture.returnFollower.followCount);
            assertEquals(status.toString(), 1, fixture.scoring.disableCount);
            assertTrue(fixture.routine.getDebugName().contains("FALLBACK"));
            assertTrue(fixture.routine.getDebugName().contains(status.name()));

            fixture.finishReturn(RouteStatus.COMPLETED);
            assertEquals(status.toString(), TaskOutcome.TIMEOUT, fixture.routine.getOutcome());
        }
    }

    @Test
    public void eachCancellationLikeOutboundStatusAbortsWithoutScoringOrFallback() {
        for (RouteStatus status : OUTBOUND_ABORT_STATUSES) {
            Fixture fixture = new Fixture();
            fixture.start();

            fixture.finishOutbound(status);

            assertTrue(status.toString(), fixture.routine.isComplete());
            assertEquals(status.toString(), TaskOutcome.CANCELLED, fixture.routine.getOutcome());
            assertEquals(status.toString(), 0, fixture.scoringAttempt.startCount);
            assertEquals(status.toString(), 0, fixture.returnFollower.followCount);
            assertEquals(status.toString(), 1, fixture.scoring.disableCount);
            assertTrue(fixture.routine.getDebugName().contains("ABORT"));
            assertTrue(fixture.routine.getDebugName().contains(status.name()));
        }
    }

    @Test
    public void scoringTimeoutRunsReturnAndRetainsTimeoutAfterSuccessfulReturn() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.finishOutbound(RouteStatus.COMPLETED);

        fixture.finishScoring(TaskOutcome.TIMEOUT);

        assertEquals(1, fixture.returnFollower.followCount);
        assertEquals(1, fixture.scoring.disableCount);
        assertTrue(fixture.routine.getDebugName().contains("FALLBACK"));
        assertTrue(fixture.routine.getDebugName().contains("SCORING_TIMEOUT"));

        fixture.finishReturn(RouteStatus.COMPLETED);
        assertEquals(TaskOutcome.TIMEOUT, fixture.routine.getOutcome());
    }

    @Test
    public void scoringCancellationAndUnknownAbortWithoutReturn() {
        TaskOutcome[] outcomes = {TaskOutcome.CANCELLED, TaskOutcome.UNKNOWN};
        for (TaskOutcome outcome : outcomes) {
            Fixture fixture = new Fixture();
            fixture.start();
            fixture.finishOutbound(RouteStatus.COMPLETED);

            fixture.finishScoring(outcome);

            assertTrue(outcome.toString(), fixture.routine.isComplete());
            assertEquals(outcome.toString(), TaskOutcome.CANCELLED, fixture.routine.getOutcome());
            assertEquals(outcome.toString(), 0, fixture.returnFollower.followCount);
            assertEquals(outcome.toString(), 1, fixture.scoring.disableCount);
            assertTrue(fixture.routine.getDebugName().contains("SCORING_" + outcome));
        }
    }

    @Test
    public void eachReturnTimeoutFinishesAsTimeout() {
        for (RouteStatus status : ROUTE_TIMEOUT_STATUSES) {
            Fixture fixture = fixtureAtReturn();

            fixture.finishReturn(status);

            assertTrue(status.toString(), fixture.routine.isComplete());
            assertEquals(status.toString(), TaskOutcome.TIMEOUT, fixture.routine.getOutcome());
            assertEquals(status.toString(), 1, fixture.returnFollower.followCount);
        }
    }

    @Test
    public void eachCancellationLikeReturnStatusWinsOverEarlierSuccess() {
        for (RouteStatus status : OUTBOUND_ABORT_STATUSES) {
            Fixture fixture = fixtureAtReturn();

            fixture.finishReturn(status);

            assertTrue(status.toString(), fixture.routine.isComplete());
            assertEquals(status.toString(), TaskOutcome.CANCELLED, fixture.routine.getOutcome());
            assertEquals(status.toString(), 1, fixture.returnFollower.followCount);
            assertTrue(fixture.routine.getDebugName().contains(status.name()));
        }
    }

    @Test
    public void cancellationLikeReturnOverridesRetainedOutboundTimeout() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.finishOutbound(RouteStatus.FOLLOWER_TIMEOUT_OR_STALL);

        fixture.finishReturn(RouteStatus.REPLACED);

        assertEquals(TaskOutcome.CANCELLED, fixture.routine.getOutcome());
        assertTrue(fixture.routine.getDebugName().contains("REPLACED"));
    }

    @Test
    public void returnTimeoutPreservesRetainedOutboundAndScoringTimeouts() {
        Fixture outboundTimeout = new Fixture();
        outboundTimeout.start();
        outboundTimeout.finishOutbound(RouteStatus.FOLLOWER_TIMEOUT_OR_STALL);
        outboundTimeout.finishReturn(RouteStatus.TASK_TIMEOUT);
        assertEquals(TaskOutcome.TIMEOUT, outboundTimeout.routine.getOutcome());

        Fixture scoringTimeout = new Fixture();
        scoringTimeout.start();
        scoringTimeout.finishOutbound(RouteStatus.COMPLETED);
        scoringTimeout.finishScoring(TaskOutcome.TIMEOUT);
        scoringTimeout.finishReturn(RouteStatus.FOLLOWER_TIMEOUT_OR_STALL);
        assertEquals(TaskOutcome.TIMEOUT, scoringTimeout.routine.getOutcome());
    }

    @Test
    public void taskRunnerFailStopCleansFlywheelAfterScoringChildStartFailure() {
        Fixture fixture = new Fixture();
        TaskRunner runner = new TaskRunner();
        runner.enqueue(fixture.routine);
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
        assertEquals(1, fixture.scoring.disableCount);
        assertEquals(0, fixture.returnFollower.followCount);
    }

    @Test
    public void taskRunnerFailStopCleansAfterLiveReturnBuildFailureWithoutStartingFollower() {
        ManualLoopClock clock = new ManualLoopClock();
        ControlledFollower outboundFollower = new ControlledFollower();
        ControlledFollower returnFollower = new ControlledFollower();
        ControlledTask scoringAttempt = new ControlledTask();
        RecordingScoring scoring = new RecordingScoring();
        RouteTask<String> outbound = newRoute("outbound", outboundFollower);
        IllegalStateException buildFailure = new IllegalStateException("live return unavailable");
        RouteTask.Config returnConfig = new RouteTask.Config();
        returnConfig.timeoutSec = 0.05;
        RouteTask<String> liveReturn = RouteTasks.followBuiltAtStart(
                "liveReturn",
                returnFollower,
                () -> {
                    throw buildFailure;
                },
                returnConfig
        );
        PhoenixPedroAutoRoutineTask routine = new PhoenixPedroAutoRoutineTask(
                "returnBuildFailure",
                outbound,
                scoringAttempt,
                liveReturn,
                scoring
        );
        TaskRunner runner = new TaskRunner();
        runner.enqueue(routine);
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
        assertEquals(1, scoring.disableCount);
        assertEquals(0, returnFollower.followCount);
    }

    @Test
    public void directCancellationInEveryPhaseCancelsOnlyActiveChildAndNeverRecovers() {
        Fixture outbound = new Fixture();
        outbound.start();
        outbound.routine.cancel();
        outbound.routine.cancel();
        assertCancelled(outbound);
        assertEquals(1, outbound.outboundFollower.current.cancelCount);
        assertEquals(0, outbound.scoringAttempt.startCount);
        assertEquals(0, outbound.returnFollower.followCount);

        Fixture scoring = new Fixture();
        scoring.start();
        scoring.finishOutbound(RouteStatus.COMPLETED);
        scoring.routine.cancel();
        scoring.routine.cancel();
        assertCancelled(scoring);
        assertEquals(1, scoring.scoringAttempt.cancelCount);
        assertEquals(0, scoring.returnFollower.followCount);

        Fixture returning = fixtureAtReturn();
        returning.routine.cancel();
        returning.routine.cancel();
        assertCancelled(returning);
        assertEquals(1, returning.returnFollower.current.cancelCount);
        assertEquals(1, returning.returnFollower.followCount);
    }

    @Test
    public void cancellationBeforeStartIsNoOpAndTerminalCancellationIsNoOp() {
        Fixture fixture = new Fixture();

        fixture.routine.cancel();
        assertFalse(fixture.routine.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, fixture.routine.getOutcome());
        assertEquals(0, fixture.scoring.disableCount);

        fixture.start();
        fixture.finishOutbound(RouteStatus.INTERRUPTED);
        fixture.routine.cancel();

        assertEquals(TaskOutcome.CANCELLED, fixture.routine.getOutcome());
        assertEquals(1, fixture.scoring.disableCount);
        assertEquals(0, fixture.outboundFollower.current.cancelCount);
    }

    @Test
    public void cancellationReenteredFromFlywheelCleanupDoesNotStartFallback() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.scoring.onDisable = fixture.routine::cancel;

        fixture.finishOutbound(RouteStatus.FOLLOWER_TIMEOUT_OR_STALL);

        assertCancelled(fixture);
        assertEquals(1, fixture.scoring.disableCount);
        assertEquals(0, fixture.scoringAttempt.startCount);
        assertEquals(0, fixture.returnFollower.followCount);
        assertTrue(fixture.routine.getDebugName().contains("DIRECT_CANCEL"));
    }

    @Test
    public void updateReenteredFromFlywheelCleanupStartsEachSelectedReturnOnlyOnce() {
        Fixture outboundTimeout = new Fixture();
        outboundTimeout.start();
        outboundTimeout.scoring.onDisable =
                () -> outboundTimeout.routine.update(outboundTimeout.clock.clock());

        outboundTimeout.finishOutbound(RouteStatus.FOLLOWER_TIMEOUT_OR_STALL);

        assertEquals(1, outboundTimeout.returnFollower.followCount);
        assertEquals(1, outboundTimeout.scoring.disableCount);
        assertTrue(outboundTimeout.routine.getDebugName().contains("FALLBACK"));

        Fixture scoringSuccess = new Fixture();
        scoringSuccess.start();
        scoringSuccess.finishOutbound(RouteStatus.COMPLETED);
        scoringSuccess.scoringAttempt.completeWith(TaskOutcome.SUCCESS);
        scoringSuccess.scoring.onDisable =
                () -> scoringSuccess.routine.update(scoringSuccess.clock.clock());

        scoringSuccess.routine.update(scoringSuccess.clock.clock());

        assertEquals(1, scoringSuccess.returnFollower.followCount);
        assertEquals(1, scoringSuccess.scoring.disableCount);
        assertTrue(scoringSuccess.routine.getDebugName().contains("phase=RETURN_OR_PARK"));
    }

    @Test
    public void cancellationReenteredFromScoringOutcomeDoesNotOverwriteAbortOrStartReturn() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.finishOutbound(RouteStatus.COMPLETED);
        fixture.scoringAttempt.completeWith(TaskOutcome.SUCCESS);
        fixture.scoringAttempt.onGetOutcome = fixture.routine::cancel;

        fixture.routine.update(fixture.clock.clock());

        assertCancelled(fixture);
        assertEquals(1, fixture.scoring.disableCount);
        assertEquals(0, fixture.returnFollower.followCount);
        assertTrue(fixture.routine.getDebugName().contains("decision=ABORT"));
        assertTrue(fixture.routine.getDebugName().contains("DIRECT_CANCEL"));
    }

    @Test
    public void updateReenteredFromScoringOutcomeStartsReturnOnlyOnce() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.finishOutbound(RouteStatus.COMPLETED);
        fixture.scoringAttempt.completeWith(TaskOutcome.SUCCESS);
        fixture.scoringAttempt.onGetOutcome =
                () -> fixture.routine.update(fixture.clock.clock());

        fixture.routine.update(fixture.clock.clock());

        assertFalse(fixture.routine.isComplete());
        assertEquals(1, fixture.returnFollower.followCount);
        assertEquals(1, fixture.scoring.disableCount);
        assertTrue(fixture.routine.getDebugName().contains("phase=RETURN_OR_PARK"));

        fixture.finishReturn(RouteStatus.COMPLETED);
        assertEquals(TaskOutcome.SUCCESS, fixture.routine.getOutcome());
    }

    @Test
    public void childAlreadyCompleteAtUpdateIsAdvancedWithoutTerminalUpdateCall() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.finishOutbound(RouteStatus.COMPLETED);
        fixture.scoringAttempt.completeWith(TaskOutcome.SUCCESS);

        fixture.routine.update(fixture.clock.clock());

        assertEquals(0, fixture.scoringAttempt.updateCount);
        assertEquals(1, fixture.returnFollower.followCount);
    }

    @Test
    public void directCancelAttemptsBothCleanupsOnceAndSuppressesLaterFailure() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.finishOutbound(RouteStatus.COMPLETED);
        IllegalStateException childFailure = new IllegalStateException("child cancel failed");
        IllegalStateException cleanupFailure = new IllegalStateException("flywheel cleanup failed");
        fixture.scoringAttempt.cancelFailure = childFailure;
        fixture.scoring.disableFailure = cleanupFailure;

        try {
            fixture.routine.cancel();
            fail("expected cleanup failure");
        } catch (IllegalStateException actual) {
            assertSame(childFailure, actual);
        }

        assertTrue(fixture.routine.isComplete());
        assertEquals(TaskOutcome.CANCELLED, fixture.routine.getOutcome());
        assertEquals(1, fixture.scoringAttempt.cancelCount);
        assertEquals(1, fixture.scoring.disableCount);
        assertEquals(1, childFailure.getSuppressed().length);
        assertSame(cleanupFailure, childFailure.getSuppressed()[0]);

        fixture.routine.cancel();
        assertEquals(1, fixture.scoringAttempt.cancelCount);
        assertEquals(1, fixture.scoring.disableCount);
    }

    @Test
    public void immediateChildrenAdvanceThroughWholeRoutineAtTheirOwnStartBoundaries() {
        Fixture fixture = new Fixture();
        fixture.outboundFollower.statusOnFollow = RouteStatus.COMPLETED;
        fixture.scoringAttempt.completeOnStart = TaskOutcome.SUCCESS;
        fixture.returnFollower.statusOnFollow = RouteStatus.COMPLETED;

        fixture.start();

        assertTrue(fixture.routine.isComplete());
        assertEquals(TaskOutcome.SUCCESS, fixture.routine.getOutcome());
        assertEquals(1, fixture.outboundFollower.followCount);
        assertEquals(1, fixture.scoringAttempt.startCount);
        assertEquals(1, fixture.returnFollower.followCount);
        assertEquals(1, fixture.scoring.disableCount);
    }

    @Test
    public void immediateOutboundTimeoutFallbackRetainsTimeoutAndSkipsScoring() {
        Fixture fixture = new Fixture();
        fixture.outboundFollower.statusOnFollow = RouteStatus.FOLLOWER_TIMEOUT_OR_STALL;
        fixture.returnFollower.statusOnFollow = RouteStatus.COMPLETED;

        fixture.start();

        assertTrue(fixture.routine.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, fixture.routine.getOutcome());
        assertEquals(0, fixture.scoringAttempt.startCount);
        assertEquals(1, fixture.returnFollower.followCount);
        assertEquals(1, fixture.scoring.disableCount);
    }

    @Test
    public void lifecycleErrorsAndDuplicateAliasesAreActionable() {
        Fixture fixture = new Fixture();

        assertFailureContains(
                () -> fixture.routine.update(fixture.clock.clock()),
                "before start",
                "PhoenixPedroAutoRoutineFactory.build"
        );

        fixture.start();
        int startsBeforeSecondAttempt = fixture.outboundFollower.followCount;
        assertFailureContains(
                () -> fixture.routine.start(fixture.clock.clock()),
                "single-use",
                "fresh Task graph"
        );
        assertEquals(startsBeforeSecondAttempt, fixture.outboundFollower.followCount);

        ControlledFollower follower = new ControlledFollower();
        RouteTask<String> shared = newRoute("shared", follower);
        RouteTask<String> other = newRoute("other", new ControlledFollower());
        RecordingScoring scoring = new RecordingScoring();
        ControlledTask ordinary = new ControlledTask();

        assertFailureContains(
                () -> new PhoenixPedroAutoRoutineTask(
                        "alias", shared, ordinary, shared, scoring),
                "outboundRoute",
                "returnOrParkRoute",
                "fresh Task"
        );
        assertFailureContains(
                () -> new PhoenixPedroAutoRoutineTask(
                        "alias", shared, shared, other, scoring),
                "outboundRoute",
                "scoringAttempt",
                "fresh Task"
        );
        assertFailureContains(
                () -> new PhoenixPedroAutoRoutineTask(
                        "alias", shared, other, other, scoring),
                "scoringAttempt",
                "returnOrParkRoute",
                "fresh Task"
        );
    }

    @Test
    public void constructorRejectsMissingRolesWithNamedGuidance() {
        ControlledFollower follower = new ControlledFollower();
        RouteTask<String> outbound = newRoute("outbound", follower);
        RouteTask<String> returning = newRoute("return", new ControlledFollower());
        ControlledTask scoringTask = new ControlledTask();
        RecordingScoring scoring = new RecordingScoring();

        assertFailureContains(
                () -> new PhoenixPedroAutoRoutineTask(
                        " ", outbound, scoringTask, returning, scoring),
                "routineName",
                "telemetry"
        );
        assertFailureContains(
                () -> new PhoenixPedroAutoRoutineTask(
                        "missing", null, scoringTask, returning, scoring),
                "outboundRoute",
                "fresh Task graph"
        );
        assertFailureContains(
                () -> new PhoenixPedroAutoRoutineTask(
                        "missing", outbound, null, returning, scoring),
                "scoringAttempt",
                "fresh Task graph"
        );
        assertFailureContains(
                () -> new PhoenixPedroAutoRoutineTask(
                        "missing", outbound, scoringTask, null, scoring),
                "returnOrParkRoute",
                "fresh Task graph"
        );
        assertFailureContains(
                () -> new PhoenixPedroAutoRoutineTask(
                        "missing", outbound, scoringTask, returning, null),
                "scoring",
                "fresh Task graph"
        );
    }

    @Test
    public void malformedCompletedScoringOutcomesFailWithRoleAndRecoveryGuidance() {
        TaskOutcome[] malformed = {null, TaskOutcome.NOT_DONE};
        for (TaskOutcome outcome : malformed) {
            Fixture fixture = new Fixture();
            fixture.start();
            fixture.finishOutbound(RouteStatus.COMPLETED);
            fixture.scoringAttempt.completeWith(outcome);

            assertFailureContains(
                    () -> fixture.routine.update(fixture.clock.clock()),
                    "scoringAttempt",
                    String.valueOf(outcome),
                    "fresh child Task graph"
            );
            fixture.routine.cancel();
            assertEquals(1, fixture.scoring.disableCount);
        }
    }

    @Test
    public void malformedCompletedRouteStatusFailsWithoutGuessingPolicy() throws Exception {
        Object[] malformed = {null, RouteStatus.ACTIVE, RouteStatus.NOT_STARTED};
        for (Object status : malformed) {
            Fixture fixture = new Fixture();
            fixture.start();
            setPrivate(fixture.outboundRoute, "complete", true);
            setPrivate(fixture.outboundRoute, "routeStatus", status);

            assertFailureContains(
                    () -> fixture.routine.update(fixture.clock.clock()),
                    "outboundRoute",
                    String.valueOf(status),
                    "terminal RouteStatus"
            );
            fixture.routine.cancel();
            assertEquals(1, fixture.scoring.disableCount);
        }
    }

    @Test
    public void childStartFailureLeavesRoleVisibleAndCancellationCleansOwnedState() {
        Fixture fixture = new Fixture();
        fixture.start();
        IllegalStateException startFailure = new IllegalStateException("scoring start failed");
        fixture.scoringAttempt.startFailure = startFailure;

        try {
            fixture.finishOutbound(RouteStatus.COMPLETED);
            fail("expected child start failure");
        } catch (IllegalStateException actual) {
            assertSame(startFailure, actual);
        }

        assertTrue(fixture.routine.getDebugName().contains("phase=SCORING"));
        fixture.routine.cancel();
        assertEquals(TaskOutcome.CANCELLED, fixture.routine.getOutcome());
        assertEquals(1, fixture.scoringAttempt.cancelCount);
        assertEquals(1, fixture.scoring.disableCount);
        assertEquals(0, fixture.returnFollower.followCount);
    }

    @Test
    public void debugNameAndDumpExposeLiveFallbackStatusAndAccumulatedOutcome() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.finishOutbound(RouteStatus.FOLLOWER_TIMEOUT_OR_STALL);

        String name = fixture.routine.getDebugName();
        assertTrue(name.contains("testRoutine"));
        assertTrue(name.contains("phase=RETURN_OR_PARK"));
        assertTrue(name.contains("decision=FALLBACK"));
        assertTrue(name.contains("FOLLOWER_TIMEOUT_OR_STALL"));

        CapturingDebugSink sink = new CapturingDebugSink();
        fixture.routine.debugDump(sink, "auto.policy");
        fixture.routine.debugDump(null, "ignored");

        assertEquals("testRoutine", sink.values.get("auto.policy.routine"));
        assertEquals("RETURN_OR_PARK", String.valueOf(sink.values.get("auto.policy.phase")));
        assertEquals("FALLBACK", String.valueOf(sink.values.get("auto.policy.decision")));
        assertEquals(
                RouteStatus.FOLLOWER_TIMEOUT_OR_STALL,
                sink.values.get("auto.policy.triggerRouteStatus")
        );
        assertEquals(TaskOutcome.TIMEOUT, sink.values.get("auto.policy.accumulatedOutcome"));
        assertEquals("return", sink.values.get("auto.policy.activeChild"));
    }

    private static Fixture fixtureAtReturn() {
        Fixture fixture = new Fixture();
        fixture.start();
        fixture.finishOutbound(RouteStatus.COMPLETED);
        fixture.finishScoring(TaskOutcome.SUCCESS);
        return fixture;
    }

    private static void assertCancelled(Fixture fixture) {
        assertTrue(fixture.routine.isComplete());
        assertEquals(TaskOutcome.CANCELLED, fixture.routine.getOutcome());
        assertEquals(1, fixture.scoring.disableCount);
        assertTrue(fixture.routine.getDebugName().contains("ABORT"));
        assertTrue(fixture.routine.getDebugName().contains("DIRECT_CANCEL"));
    }

    private static RouteTask<String> newRoute(String name, ControlledFollower follower) {
        RouteTask.Config config = new RouteTask.Config();
        config.timeoutSec = 0.05;
        return RouteTasks.follow(name, follower, name + "-route", config);
    }

    private static void setPrivate(Object target, String fieldName, Object value) throws Exception {
        Field field = target.getClass().getDeclaredField(fieldName);
        field.setAccessible(true);
        field.set(target, value);
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

    private interface ThrowingAction {
        void run();
    }

    private static final class Fixture {
        private final ManualLoopClock clock = new ManualLoopClock();
        private final ControlledFollower outboundFollower = new ControlledFollower();
        private final ControlledFollower returnFollower = new ControlledFollower();
        private final RouteTask<String> outboundRoute = newRoute("outbound", outboundFollower);
        private final RouteTask<String> returnRoute = newRoute("return", returnFollower);
        private final ControlledTask scoringAttempt = new ControlledTask();
        private final RecordingScoring scoring = new RecordingScoring();
        private final PhoenixPedroAutoRoutineTask routine = new PhoenixPedroAutoRoutineTask(
                "testRoutine",
                outboundRoute,
                scoringAttempt,
                returnRoute,
                scoring
        );

        private void start() {
            routine.start(clock.clock());
        }

        private void finishOutbound(RouteStatus status) {
            finishRoute(outboundFollower, status);
        }

        private void finishScoring(TaskOutcome outcome) {
            scoringAttempt.completeWith(outcome);
            routine.update(clock.clock());
        }

        private void finishReturn(RouteStatus status) {
            finishRoute(returnFollower, status);
        }

        private void finishRoute(ControlledFollower follower, RouteStatus status) {
            if (status == RouteStatus.TASK_TIMEOUT) {
                clock.nextCycle(0.06);
            } else {
                follower.finish(status);
            }
            routine.update(clock.clock());
        }
    }

    private static final class ControlledFollower implements RouteFollower<String> {
        private ControlledExecution current;
        private RouteStatus statusOnFollow = RouteStatus.ACTIVE;
        private RuntimeException followFailure;
        private Runnable onFollow;
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
            Runnable callback = onFollow;
            onFollow = null;
            if (callback != null) {
                callback.run();
            }
            return current;
        }

        @Override
        public void update(LoopClock clock) {
            updateCount++;
        }

        private void finish(RouteStatus status) {
            if (status == RouteStatus.NOT_STARTED
                    || status == RouteStatus.ACTIVE
                    || status == RouteStatus.TASK_TIMEOUT) {
                throw new IllegalArgumentException("test fixture requires integration terminal status");
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
        private int disableCount;
        private RuntimeException disableFailure;
        private Runnable onDisable;

        @Override
        public void setIntakeEnabled(boolean enabled) {
        }

        @Override
        public void setFlywheelEnabled(boolean enabled) {
            if (enabled) {
                return;
            }
            disableCount++;
            Runnable callback = onDisable;
            onDisable = null;
            if (callback != null) {
                callback.run();
            }
            if (disableFailure != null) {
                throw disableFailure;
            }
        }

        @Override
        public void setShootingEnabled(boolean enabled) {
        }

        @Override
        public void setEjectEnabled(boolean enabled) {
        }

        @Override
        public void requestSingleShot() {
        }

        @Override
        public void requestShots(int shotCount) {
        }

        @Override
        public void cancelTransientActions() {
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
