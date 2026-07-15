package edu.ftcphoenix.robots.phoenix.autonomous;

import org.junit.Test;

import java.util.Collections;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.task.TaskRunner;
import edu.ftcphoenix.robots.phoenix.PhoenixCapabilities;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;
import edu.ftcphoenix.robots.phoenix.ScoringPath;
import edu.ftcphoenix.robots.phoenix.ScoringTargeting;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies truthful Phoenix scoring-attempt outcomes and owned transient-shot cleanup. */
public final class PhoenixAutoTasksTest {

    private static final DriveCommandSink NOOP_DRIVE = new DriveCommandSink() {
        @Override
        public void drive(DriveSignal signal) {
            // No-op test sink.
        }

        @Override
        public void stop() {
            // No-op test sink.
        }
    };

    @Test
    public void successfulAttemptRunsDependentPhasesInOrder() {
        FakeScoring scoring = new FakeScoring();
        ControlledTask aim = ControlledTask.finishingWith(TaskOutcome.SUCCESS);
        FakeTargeting targeting = new FakeTargeting(true, aim);
        Task attempt = buildFactoryAttempt(scoring, targeting, new PhoenixProfile.AutoConfig());
        LoopClock clock = clockAt(1.0);

        attempt.start(clock);
        tick(attempt, clock, 1.0); // target selected; capture and start aim
        assertEquals(1, scoring.captureCount);
        assertEquals(0, scoring.requestCount);
        assertEquals(1, aim.startCount);

        tick(attempt, clock, 1.1); // aim succeeds; request shot and start drain wait
        assertEquals(1, scoring.requestCount);
        assertTrue(scoring.pendingShots);

        scoring.pendingShots = false;
        tick(attempt, clock, 1.2);

        assertTrue(attempt.isComplete());
        assertEquals(TaskOutcome.SUCCESS, attempt.getOutcome());
        assertEquals(0, scoring.cancelTransientCount);
    }

    @Test
    public void completeInStartPhasesAdvanceWithoutReceivingUpdates() {
        FakeScoring scoring = new FakeScoring();
        CompleteInStartTask target = new CompleteInStartTask(TaskOutcome.SUCCESS);
        CompleteInStartTask aim = new CompleteInStartTask(TaskOutcome.SUCCESS);
        CompleteInStartTask shot = new CompleteInStartTask(TaskOutcome.SUCCESS);
        PhoenixScoringAttemptTask attempt = new PhoenixScoringAttemptTask(
                scoring,
                target,
                aim,
                shot
        );

        attempt.start(clockAt(0.0));

        assertTrue(attempt.isComplete());
        assertEquals(TaskOutcome.SUCCESS, attempt.getOutcome());
        assertEquals(1, scoring.captureCount);
        assertEquals(1, scoring.requestCount);
        assertEquals(0, target.updateCount);
        assertEquals(0, aim.updateCount);
        assertEquals(0, shot.updateCount);
    }

    @Test
    public void reentrantUpdateFromVelocityCaptureDoesNotRepeatActionOrAimStart() {
        FakeScoring scoring = new FakeScoring();
        ControlledTask target = ControlledTask.finishingWith(TaskOutcome.SUCCESS);
        ControlledTask aim = ControlledTask.finishingAfterUpdates(TaskOutcome.SUCCESS, 10);
        PhoenixScoringAttemptTask attempt = new PhoenixScoringAttemptTask(
                scoring,
                target,
                aim,
                ControlledTask.finishingWith(TaskOutcome.SUCCESS)
        );
        LoopClock clock = clockAt(0.0);
        scoring.onCapture = () -> attempt.update(clock);

        attempt.start(clock);
        tick(attempt, clock, 0.0);

        assertEquals(1, scoring.captureCount);
        assertEquals(1, aim.startCount);
        assertEquals(0, aim.updateCount);
        assertEquals(0, scoring.requestCount);
        assertFalse(attempt.isComplete());
    }

    @Test
    public void reentrantUpdateFromShotRequestDoesNotRepeatActionOrShotWaitStart() {
        FakeScoring scoring = new FakeScoring();
        ControlledTask aim = ControlledTask.finishingWith(TaskOutcome.SUCCESS);
        ControlledTask shotWait = ControlledTask.finishingAfterUpdates(TaskOutcome.SUCCESS, 10);
        PhoenixScoringAttemptTask attempt = new PhoenixScoringAttemptTask(
                scoring,
                ControlledTask.finishingWith(TaskOutcome.SUCCESS),
                aim,
                shotWait
        );
        LoopClock clock = clockAt(0.0);

        attempt.start(clock);
        tick(attempt, clock, 0.0);
        scoring.onRequest = () -> attempt.update(clock);
        tick(attempt, clock, 0.1);

        assertEquals(1, scoring.captureCount);
        assertEquals(1, aim.startCount);
        assertEquals(1, scoring.requestCount);
        assertEquals(1, shotWait.startCount);
        assertEquals(0, shotWait.updateCount);
        assertFalse(attempt.isComplete());
    }

    @Test
    public void targetTimeoutStartsAtItsOwnBoundaryAndSkipsEveryDependentAction() {
        FakeScoring scoring = new FakeScoring();
        ControlledTask aim = ControlledTask.finishingWith(TaskOutcome.SUCCESS);
        FakeTargeting targeting = new FakeTargeting(false, aim);
        PhoenixProfile.AutoConfig auto = new PhoenixProfile.AutoConfig();
        auto.waitForTargetSec = 0.5;
        Task attempt = buildFactoryAttempt(scoring, targeting, auto);
        LoopClock clock = clockAt(0.0);

        // A large interval before start must not be charged to the target wait.
        clock.update(100.0);
        attempt.start(clock);
        tick(attempt, clock, 100.0);
        tick(attempt, clock, 100.49);
        assertFalse(attempt.isComplete());

        tick(attempt, clock, 100.5);

        assertEquals(TaskOutcome.TIMEOUT, attempt.getOutcome());
        assertEquals(0, scoring.captureCount);
        assertEquals(0, scoring.requestCount);
        assertEquals(0, scoring.cancelTransientCount);
        assertEquals(0, aim.startCount);
    }

    @Test
    public void aimTimeoutRemainsTimeoutAndNeverRequestsShot() {
        FakeScoring scoring = new FakeScoring();
        ControlledTask aim = ControlledTask.finishingWith(TaskOutcome.TIMEOUT);
        Task attempt = buildFactoryAttempt(
                scoring,
                new FakeTargeting(true, aim),
                new PhoenixProfile.AutoConfig()
        );
        LoopClock clock = clockAt(2.0);

        attempt.start(clock);
        tick(attempt, clock, 2.0);
        tick(attempt, clock, 2.1);

        assertEquals(TaskOutcome.TIMEOUT, attempt.getOutcome());
        assertEquals(1, scoring.captureCount);
        assertEquals(0, scoring.requestCount);
        assertEquals(0, scoring.cancelTransientCount);
    }

    @Test
    public void shotDrainTimeoutRemainsTimeoutAndCancelsOwnedTransientShot() {
        FakeScoring scoring = new FakeScoring();
        PhoenixProfile.AutoConfig auto = new PhoenixProfile.AutoConfig();
        auto.waitForShotCompleteSec = 0.5;
        Task attempt = buildFactoryAttempt(
                scoring,
                new FakeTargeting(true, ControlledTask.finishingWith(TaskOutcome.SUCCESS)),
                auto
        );
        LoopClock clock = clockAt(5.0);

        attempt.start(clock);
        tick(attempt, clock, 5.0);
        tick(attempt, clock, 5.1); // shot wait starts here
        tick(attempt, clock, 5.59);
        assertFalse(attempt.isComplete());

        tick(attempt, clock, 5.61);

        assertEquals(TaskOutcome.TIMEOUT, attempt.getOutcome());
        assertEquals(1, scoring.requestCount);
        assertEquals(1, scoring.cancelTransientCount);
        assertFalse(scoring.pendingShots);
    }

    @Test
    public void directCancellationAfterShotRequestCleansOnceWithoutTouchingHeldIntents() {
        FakeScoring scoring = new FakeScoring();
        Task attempt = buildFactoryAttempt(
                scoring,
                new FakeTargeting(true, ControlledTask.finishingWith(TaskOutcome.SUCCESS)),
                new PhoenixProfile.AutoConfig()
        );
        LoopClock clock = clockAt(3.0);

        attempt.start(clock);
        tick(attempt, clock, 3.0);
        tick(attempt, clock, 3.1);
        assertEquals(1, scoring.requestCount);

        attempt.cancel();
        attempt.cancel();
        tick(attempt, clock, 3.2);

        assertEquals(TaskOutcome.CANCELLED, attempt.getOutcome());
        assertEquals(1, scoring.cancelTransientCount);
        assertFalse(scoring.pendingShots);
        assertNoHeldIntentWrites(scoring);
    }

    @Test
    public void directCancellationWhileWaitingForTargetCancelsOnlyThatPhase() {
        FakeScoring scoring = new FakeScoring();
        ControlledTask target = ControlledTask.finishingAfterUpdates(TaskOutcome.SUCCESS, 10);
        ControlledTask aim = ControlledTask.finishingWith(TaskOutcome.SUCCESS);
        PhoenixScoringAttemptTask attempt = new PhoenixScoringAttemptTask(
                scoring,
                target,
                aim,
                ControlledTask.finishingWith(TaskOutcome.SUCCESS)
        );

        attempt.start(clockAt(0.0));
        attempt.cancel();
        attempt.cancel();

        assertEquals(TaskOutcome.CANCELLED, attempt.getOutcome());
        assertEquals(1, target.cancelCount);
        assertEquals(0, aim.startCount);
        assertEquals(0, scoring.captureCount);
        assertEquals(0, scoring.requestCount);
        assertEquals(0, scoring.cancelTransientCount);
        assertNoHeldIntentWrites(scoring);
    }

    @Test
    public void directCancellationWhileAimingCancelsOnlyAimAndSkipsShot() {
        FakeScoring scoring = new FakeScoring();
        ControlledTask aim = ControlledTask.finishingAfterUpdates(TaskOutcome.SUCCESS, 10);
        PhoenixScoringAttemptTask attempt = new PhoenixScoringAttemptTask(
                scoring,
                ControlledTask.finishingWith(TaskOutcome.SUCCESS),
                aim,
                ControlledTask.finishingWith(TaskOutcome.SUCCESS)
        );
        LoopClock clock = clockAt(0.0);

        attempt.start(clock);
        tick(attempt, clock, 0.0);
        attempt.cancel();

        assertEquals(TaskOutcome.CANCELLED, attempt.getOutcome());
        assertEquals(1, scoring.captureCount);
        assertEquals(1, aim.cancelCount);
        assertEquals(0, scoring.requestCount);
        assertEquals(0, scoring.cancelTransientCount);
        assertNoHeldIntentWrites(scoring);
    }

    @Test
    public void unknownTargetOutcomeRemainsUnknownAndSkipsAim() {
        FakeScoring scoring = new FakeScoring();
        ControlledTask target = ControlledTask.finishingWith(TaskOutcome.UNKNOWN);
        ControlledTask aim = ControlledTask.finishingWith(TaskOutcome.SUCCESS);
        PhoenixScoringAttemptTask attempt = new PhoenixScoringAttemptTask(
                scoring,
                target,
                aim,
                ControlledTask.finishingWith(TaskOutcome.SUCCESS)
        );
        LoopClock clock = clockAt(0.0);

        attempt.start(clock);
        tick(attempt, clock, 0.0);

        assertEquals(TaskOutcome.UNKNOWN, attempt.getOutcome());
        assertEquals(0, scoring.captureCount);
        assertEquals(0, aim.startCount);
        assertEquals(0, scoring.requestCount);
    }

    @Test
    public void cancelledAimOutcomeRemainsCancelledAndSkipsShot() {
        FakeScoring scoring = new FakeScoring();
        ControlledTask aim = ControlledTask.finishingWith(TaskOutcome.CANCELLED);
        PhoenixScoringAttemptTask attempt = new PhoenixScoringAttemptTask(
                scoring,
                ControlledTask.finishingWith(TaskOutcome.SUCCESS),
                aim,
                ControlledTask.finishingWith(TaskOutcome.SUCCESS)
        );
        LoopClock clock = clockAt(0.0);

        attempt.start(clock);
        tick(attempt, clock, 0.0);
        tick(attempt, clock, 0.1);

        assertEquals(TaskOutcome.CANCELLED, attempt.getOutcome());
        assertEquals(1, scoring.captureCount);
        assertEquals(0, scoring.requestCount);
        assertEquals(0, scoring.cancelTransientCount);
    }

    @Test
    public void unknownShotWaitOutcomeRemainsUnknownAndCleansTransientShot() {
        FakeScoring scoring = new FakeScoring();
        PhoenixScoringAttemptTask attempt = new PhoenixScoringAttemptTask(
                scoring,
                ControlledTask.finishingWith(TaskOutcome.SUCCESS),
                ControlledTask.finishingWith(TaskOutcome.SUCCESS),
                ControlledTask.finishingWith(TaskOutcome.UNKNOWN)
        );
        LoopClock clock = clockAt(0.0);

        attempt.start(clock);
        tick(attempt, clock, 0.0);
        tick(attempt, clock, 0.1);
        tick(attempt, clock, 0.2);

        assertEquals(TaskOutcome.UNKNOWN, attempt.getOutcome());
        assertEquals(1, scoring.requestCount);
        assertEquals(1, scoring.cancelTransientCount);
    }

    @Test
    public void lifecycleGuardsAreActiveOnlyAndSingleUse() {
        FakeScoring scoring = new FakeScoring();
        PhoenixScoringAttemptTask attempt = successfulDirectAttempt(scoring);
        LoopClock clock = clockAt(0.0);

        attempt.cancel();
        assertFalse(attempt.isComplete());
        assertEquals(0, scoring.cancelTransientCount);

        try {
            attempt.update(clock);
            fail("Expected update-before-start rejection");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("before start"));
            assertTrue(expected.getMessage().contains("PhoenixAutoTasks.aimAndShootOne"));
        }

        attempt.start(clock);
        tick(attempt, clock, 0.0);
        tick(attempt, clock, 0.1);
        tick(attempt, clock, 0.2);
        assertEquals(TaskOutcome.SUCCESS, attempt.getOutcome());

        try {
            attempt.start(clock);
            fail("Expected second-start rejection");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("single-use"));
            assertTrue(expected.getMessage().contains("aimAndShootOne"));
        }
        assertEquals(1, scoring.requestCount);
    }

    @Test
    public void duplicatePhaseIdentityIsRejectedBeforeStart() {
        FakeScoring scoring = new FakeScoring();
        ControlledTask shared = ControlledTask.finishingWith(TaskOutcome.SUCCESS);

        try {
            new PhoenixScoringAttemptTask(
                    scoring,
                    shared,
                    shared,
                    ControlledTask.finishingWith(TaskOutcome.SUCCESS)
            );
            fail("Expected duplicate phase rejection");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("waitForTargetTask"));
            assertTrue(expected.getMessage().contains("aimTask"));
            assertTrue(expected.getMessage().contains("fresh Task factory"));
        }
        assertEquals(0, shared.startCount);
    }

    @Test
    public void cancellationContinuesCleanupAndSuppressesLaterFailure() {
        FakeScoring scoring = new FakeScoring();
        RuntimeException childFailure = new IllegalStateException("child cancel failed");
        RuntimeException cleanupFailure = new IllegalArgumentException("cleanup failed");
        ControlledTask shotWait = ControlledTask.finishingAfterUpdates(
                TaskOutcome.SUCCESS,
                10
        );
        shotWait.cancelFailure = childFailure;
        scoring.cancelFailure = cleanupFailure;
        PhoenixScoringAttemptTask attempt = new PhoenixScoringAttemptTask(
                scoring,
                ControlledTask.finishingWith(TaskOutcome.SUCCESS),
                ControlledTask.finishingWith(TaskOutcome.SUCCESS),
                shotWait
        );
        LoopClock clock = clockAt(0.0);

        attempt.start(clock);
        tick(attempt, clock, 0.0);
        tick(attempt, clock, 0.1);

        try {
            attempt.cancel();
            fail("Expected the first cleanup failure");
        } catch (RuntimeException actual) {
            assertSame(childFailure, actual);
            assertEquals(1, actual.getSuppressed().length);
            assertSame(cleanupFailure, actual.getSuppressed()[0]);
        }

        assertEquals(TaskOutcome.CANCELLED, attempt.getOutcome());
        assertEquals(1, shotWait.cancelCount);
        assertEquals(1, scoring.cancelTransientCount);
    }

    @Test
    public void cancellationDoesNotSelfSuppressOneSharedCleanupFailure() {
        FakeScoring scoring = new FakeScoring();
        RuntimeException sharedFailure = new IllegalStateException("shared cleanup failure");
        ControlledTask shotWait = ControlledTask.finishingAfterUpdates(
                TaskOutcome.SUCCESS,
                10
        );
        shotWait.cancelFailure = sharedFailure;
        scoring.cancelFailure = sharedFailure;
        PhoenixScoringAttemptTask attempt = new PhoenixScoringAttemptTask(
                scoring,
                ControlledTask.finishingWith(TaskOutcome.SUCCESS),
                ControlledTask.finishingWith(TaskOutcome.SUCCESS),
                shotWait
        );
        LoopClock clock = clockAt(0.0);

        attempt.start(clock);
        tick(attempt, clock, 0.0);
        tick(attempt, clock, 0.1);

        try {
            attempt.cancel();
            fail("Expected the shared cleanup failure");
        } catch (RuntimeException actual) {
            assertSame(sharedFailure, actual);
            assertEquals(0, actual.getSuppressed().length);
        }

        assertEquals(TaskOutcome.CANCELLED, attempt.getOutcome());
        assertEquals(1, scoring.cancelTransientCount);
    }

    @Test
    public void runnerFailureDuringShotRequestCleansPartiallyRequestedShot() {
        FakeScoring scoring = new FakeScoring();
        RuntimeException requestFailure = new IllegalStateException("shot request failed");
        scoring.requestFailure = requestFailure;
        PhoenixScoringAttemptTask attempt = successfulDirectAttempt(scoring);
        TaskRunner runner = new TaskRunner();
        LoopClock clock = clockAt(0.0);
        runner.enqueue(attempt);

        tick(runner, clock, 0.0);
        try {
            tick(runner, clock, 0.1);
            fail("Expected request failure");
        } catch (RuntimeException actual) {
            assertSame(requestFailure, actual);
        }

        assertTrue(runner.isIdle());
        assertEquals(TaskOutcome.CANCELLED, attempt.getOutcome());
        assertEquals(1, scoring.requestCount);
        assertEquals(1, scoring.cancelTransientCount);
        assertFalse(scoring.pendingShots);
        assertNoHeldIntentWrites(scoring);
    }

    @Test
    public void runnerFailureStartingShotWaitCancelsChildAndCleansRequestedShot() {
        FakeScoring scoring = new FakeScoring();
        RuntimeException startFailure = new IllegalArgumentException("shot wait start failed");
        StartFailureTask shotWait = new StartFailureTask(startFailure);
        PhoenixScoringAttemptTask attempt = new PhoenixScoringAttemptTask(
                scoring,
                ControlledTask.finishingWith(TaskOutcome.SUCCESS),
                ControlledTask.finishingWith(TaskOutcome.SUCCESS),
                shotWait
        );
        TaskRunner runner = new TaskRunner();
        LoopClock clock = clockAt(0.0);
        runner.enqueue(attempt);

        tick(runner, clock, 0.0);
        try {
            tick(runner, clock, 0.1);
            fail("Expected shot-wait start failure");
        } catch (RuntimeException actual) {
            assertSame(startFailure, actual);
        }

        assertTrue(runner.isIdle());
        assertEquals(TaskOutcome.CANCELLED, attempt.getOutcome());
        assertEquals(1, shotWait.startCount);
        assertEquals(1, shotWait.cancelCount);
        assertEquals(1, scoring.requestCount);
        assertEquals(1, scoring.cancelTransientCount);
        assertFalse(scoring.pendingShots);
        assertNoHeldIntentWrites(scoring);
    }

    @Test
    public void completedChildMustReportUsableTerminalOutcome() {
        FakeScoring scoring = new FakeScoring();
        PhoenixScoringAttemptTask attempt = new PhoenixScoringAttemptTask(
                scoring,
                new InvalidCompletedOutcomeTask(TaskOutcome.NOT_DONE),
                ControlledTask.finishingWith(TaskOutcome.SUCCESS),
                ControlledTask.finishingWith(TaskOutcome.SUCCESS)
        );
        LoopClock clock = clockAt(0.0);

        attempt.start(clock);
        try {
            tick(attempt, clock, 0.0);
            fail("Expected invalid terminal outcome rejection");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("WAIT_FOR_TARGET"));
            assertTrue(expected.getMessage().contains("NOT_DONE"));
            assertTrue(expected.getMessage().contains("must report"));
        }
        assertEquals(0, scoring.captureCount);
        assertEquals(0, scoring.requestCount);
    }

    @Test
    public void completedChildNullOutcomeIsRejectedWithPhaseContext() {
        FakeScoring scoring = new FakeScoring();
        PhoenixScoringAttemptTask attempt = new PhoenixScoringAttemptTask(
                scoring,
                new InvalidCompletedOutcomeTask(null),
                ControlledTask.finishingWith(TaskOutcome.SUCCESS),
                ControlledTask.finishingWith(TaskOutcome.SUCCESS)
        );
        LoopClock clock = clockAt(0.0);

        attempt.start(clock);
        try {
            tick(attempt, clock, 0.0);
            fail("Expected null terminal outcome rejection");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("WAIT_FOR_TARGET"));
            assertTrue(expected.getMessage().contains("null"));
            assertTrue(expected.getMessage().contains("must report"));
        }
        assertEquals(0, scoring.captureCount);
        assertEquals(0, scoring.requestCount);
    }

    private static PhoenixScoringAttemptTask successfulDirectAttempt(FakeScoring scoring) {
        return new PhoenixScoringAttemptTask(
                scoring,
                ControlledTask.finishingWith(TaskOutcome.SUCCESS),
                ControlledTask.finishingWith(TaskOutcome.SUCCESS),
                ControlledTask.finishingWith(TaskOutcome.SUCCESS)
        );
    }

    private static Task buildFactoryAttempt(FakeScoring scoring,
                                            FakeTargeting targeting,
                                            PhoenixProfile.AutoConfig auto) {
        return PhoenixAutoTasks.aimAndShootOne(
                new PhoenixCapabilities(scoring, targeting),
                NOOP_DRIVE,
                auto
        );
    }

    private static LoopClock clockAt(double nowSec) {
        LoopClock clock = new LoopClock();
        clock.reset(nowSec);
        return clock;
    }

    private static void tick(Task task, LoopClock clock, double nowSec) {
        clock.update(nowSec);
        task.update(clock);
    }

    private static void tick(TaskRunner runner, LoopClock clock, double nowSec) {
        clock.update(nowSec);
        runner.update(clock);
    }

    private static void assertNoHeldIntentWrites(FakeScoring scoring) {
        assertEquals(0, scoring.intakeSetCount);
        assertEquals(0, scoring.flywheelSetCount);
        assertEquals(0, scoring.shootingSetCount);
        assertEquals(0, scoring.ejectSetCount);
    }

    private static ScoringTargeting.Status targetingStatus(boolean hasSelection) {
        TagSelectionResult selection = hasSelection
                ? new TagSelectionResult(
                        false,
                        -1,
                        null,
                        true,
                        20,
                        false,
                        false,
                        null,
                        Collections.singleton(20),
                        "test",
                        "selected for test",
                        0.0
                )
                : TagSelectionResult.none(Collections.<Integer>emptySet());
        return new ScoringTargeting.Status(
                false,
                false,
                false,
                false,
                0.0,
                0.0,
                selection,
                null,
                "test target",
                0.0,
                0.0,
                false,
                Double.NaN,
                null,
                null
        );
    }

    private static final class FakeTargeting implements PhoenixCapabilities.Targeting {
        private final boolean hasSelection;
        private final Task aimTask;

        FakeTargeting(boolean hasSelection, Task aimTask) {
            this.hasSelection = hasSelection;
            this.aimTask = aimTask;
        }

        @Override
        public ScoringTargeting.Status status(LoopClock clock) {
            return targetingStatus(hasSelection);
        }

        @Override
        public Task aimTask(DriveCommandSink driveSink, DriveGuidanceTask.Config cfg) {
            return aimTask;
        }
    }

    private static final class FakeScoring implements PhoenixCapabilities.Scoring {
        int captureCount;
        int requestCount;
        int cancelTransientCount;
        int intakeSetCount;
        int flywheelSetCount;
        int shootingSetCount;
        int ejectSetCount;
        boolean pendingShots;
        RuntimeException requestFailure;
        RuntimeException cancelFailure;
        Runnable onCapture;
        Runnable onRequest;

        @Override
        public void setIntakeEnabled(boolean enabled) {
            intakeSetCount++;
        }

        @Override
        public void setFlywheelEnabled(boolean enabled) {
            flywheelSetCount++;
        }

        @Override
        public void setShootingEnabled(boolean enabled) {
            shootingSetCount++;
        }

        @Override
        public void setEjectEnabled(boolean enabled) {
            ejectSetCount++;
        }

        @Override
        public void requestSingleShot() {
            requestCount++;
            pendingShots = true;
            Runnable callback = onRequest;
            onRequest = null;
            if (callback != null) {
                callback.run();
            }
            if (requestFailure != null) {
                throw requestFailure;
            }
        }

        @Override
        public void requestShots(int shotCount) {
            requestCount += shotCount;
            pendingShots = shotCount > 0;
        }

        @Override
        public void cancelTransientActions() {
            cancelTransientCount++;
            pendingShots = false;
            if (cancelFailure != null) {
                throw cancelFailure;
            }
        }

        @Override
        public void setSelectedVelocityNative(double velocityNative) {
            // Not used by the scoring attempt.
        }

        @Override
        public void adjustSelectedVelocityNative(double deltaNative) {
            // Not used by the scoring attempt.
        }

        @Override
        public void captureSuggestedShotVelocity() {
            captureCount++;
            Runnable callback = onCapture;
            onCapture = null;
            if (callback != null) {
                callback.run();
            }
        }

        @Override
        public boolean hasPendingShots() {
            return pendingShots;
        }

        @Override
        public ScoringPath.Status status() {
            return null;
        }
    }

    private static final class ControlledTask implements Task {
        private final TaskOutcome terminalOutcome;
        private final int updatesToComplete;

        int startCount;
        int updateCount;
        int cancelCount;
        RuntimeException cancelFailure;
        boolean started;
        boolean complete;
        TaskOutcome actualOutcome = TaskOutcome.NOT_DONE;

        static ControlledTask finishingWith(TaskOutcome outcome) {
            return finishingAfterUpdates(outcome, 1);
        }

        static ControlledTask finishingAfterUpdates(TaskOutcome outcome, int updates) {
            return new ControlledTask(outcome, updates);
        }

        private ControlledTask(TaskOutcome terminalOutcome, int updatesToComplete) {
            this.terminalOutcome = terminalOutcome;
            this.updatesToComplete = updatesToComplete;
        }

        @Override
        public void start(LoopClock clock) {
            startCount++;
            if (startCount > 1) {
                throw new IllegalStateException("ControlledTask is single-use");
            }
            started = true;
        }

        @Override
        public void update(LoopClock clock) {
            if (!started) {
                throw new IllegalStateException("ControlledTask update before start");
            }
            if (complete) {
                return;
            }
            updateCount++;
            if (updateCount >= updatesToComplete) {
                complete = true;
                actualOutcome = terminalOutcome;
            }
        }

        @Override
        public void cancel() {
            if (!started || complete) {
                return;
            }
            complete = true;
            actualOutcome = TaskOutcome.CANCELLED;
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
            return complete ? actualOutcome : TaskOutcome.NOT_DONE;
        }
    }

    /** Test child that is terminal as soon as start returns and must never receive an update. */
    private static final class CompleteInStartTask implements Task {
        private final TaskOutcome outcome;

        int startCount;
        int updateCount;
        private boolean complete;

        CompleteInStartTask(TaskOutcome outcome) {
            this.outcome = outcome;
        }

        @Override
        public void start(LoopClock clock) {
            startCount++;
            complete = true;
        }

        @Override
        public void update(LoopClock clock) {
            updateCount++;
            throw new AssertionError("A complete-in-start Task must not be updated");
        }

        @Override
        public boolean isComplete() {
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            return complete ? outcome : TaskOutcome.NOT_DONE;
        }
    }

    /** Test child that acquires its active lifecycle and then fails from start. */
    private static final class StartFailureTask implements Task {
        private final RuntimeException startFailure;

        int startCount;
        int cancelCount;
        private boolean started;
        private boolean complete;

        StartFailureTask(RuntimeException startFailure) {
            this.startFailure = startFailure;
        }

        @Override
        public void start(LoopClock clock) {
            startCount++;
            started = true;
            throw startFailure;
        }

        @Override
        public void update(LoopClock clock) {
            throw new AssertionError("A Task whose start failed must not be updated");
        }

        @Override
        public void cancel() {
            if (!started || complete) {
                return;
            }
            complete = true;
            cancelCount++;
        }

        @Override
        public boolean isComplete() {
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            return complete ? TaskOutcome.CANCELLED : TaskOutcome.NOT_DONE;
        }
    }

    private static final class InvalidCompletedOutcomeTask implements Task {
        private final TaskOutcome outcome;
        private boolean complete;

        InvalidCompletedOutcomeTask(TaskOutcome outcome) {
            this.outcome = outcome;
        }

        @Override
        public void start(LoopClock clock) {
            // No external effect.
        }

        @Override
        public void update(LoopClock clock) {
            complete = true;
        }

        @Override
        public boolean isComplete() {
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            return outcome;
        }
    }
}
