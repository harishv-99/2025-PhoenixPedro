package edu.ftcsushi.fw.drive.guidance;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.sensing.observation.OccupancyObservation;
import edu.ftcsushi.fw.sensing.observation.TargetObservation2d;
import edu.ftcsushi.fw.sensing.observation.TargetObservations2d;
import edu.ftcsushi.fw.sensing.observation.TargetSelectionPolicies;
import edu.ftcsushi.fw.sensing.observation.TargetSelectionResult;
import edu.ftcsushi.fw.sensing.observation.TargetSelectionSource;
import edu.ftcsushi.fw.sensing.observation.TargetSelections;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.task.Tasks;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Scripted camera/sensor evidence with the real selector, approach, controller and Task lifecycle. */
public final class GuidedApproachTest {
    @Test public void finalNeedsTwoPostSettleCapturesAndANewerOccupancyTransition() {
        Fixture f = new Fixture();
        Task task = f.startAt(7, 0);
        assertEquals(GuidedApproach.Phase.VERIFY, f.owner.status().phase);
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        assertZero(f.drive());
        f.step(0.10, 7, 0); // At the settle boundary is deliberately not after it.
        task.update(f.clock());
        assertEquals(0, f.owner.status().verifiedCaptureCount);
        f.step(0.01, 7, 0);
        task.update(f.clock());
        assertEquals(1, f.owner.status().verifiedCaptureCount);
        assertTrue(f.intake.isEmpty());
        f.step(0.01, 7, 0);
        TargetObservations2d second = f.frame;
        task.update(f.clock());
        assertEquals(GuidedApproach.Phase.FINAL_INTAKE, f.owner.status().phase);
        assertSame(second, f.owner.status().verificationFrame);
        assertSame(second.timestamp(), f.owner.status().verificationFrame.timestamp());
        assertEquals(2, f.owner.status().verifiedCaptureCount);
        assertEquals(Collections.singletonList(true), f.intake);
        assertTrue(f.owner.status().commandedIntake);
        assertEquals(0.15, f.drive().axial, 1e-12);
        f.step(0.02); // Occluded image does not end a verified final maneuver.
        task.update(f.clock());
        assertFalse(task.isComplete());
        assertEquals(0.15, f.drive().axial, 1e-12);
        f.step(0.02);
        f.feedback = OccupancyObservation.observed(true, f.clock().nowTimestamp());
        task.update(f.clock());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(TaskOutcome.SUCCESS, f.owner.status().outcome);
        assertFalse(f.owner.status().commandedIntake);
        assertSame(second, f.owner.status().verificationFrame);
        assertEquals(Boolean.FALSE, f.intake.get(f.intake.size() - 1));
        assertZero(f.drive());
    }

    @Test public void guidesUsingLiveRankingAndOnlyCurrentCycleCommands() {
        Fixture f = new Fixture();
        Task task = f.startAt(20, 3);
        assertEquals(GuidedApproach.Phase.GUIDE, f.owner.status().phase);
        assertTrue(f.drive().axial > 0);
        assertTrue(f.drive().omega > 0);
        int reads = f.selectionReads;
        task.update(f.clock());
        task.update(f.clock());
        assertEquals(reads, f.selectionReads);
        f.step(0.02, 15, -4);
        assertZero(f.drive()); // No stale command while the Task has not advanced this cycle.
        task.update(f.clock());
        assertTrue(f.drive().omega < 0);
        assertEquals(-4, f.owner.status().selection.observation().leftInches, 0);
    }

    @Test public void sameTimeCapturesNeverCountEvenWhenReconstructed() {
        Fixture f = new Fixture();
        Task task = f.startAt(7, 0);
        f.step(0.11, 7, 0);
        task.update(f.clock());
        LoopTimestamp stamp = f.frame.timestamp();
        f.step(0.02, 7, 0);
        f.frame = frame(stamp, 7, 0); // Different object and list, same capture time.
        task.update(f.clock());
        assertEquals(1, f.owner.status().verifiedCaptureCount);
        assertTrue(f.intake.isEmpty());
        f.step(0.02, 7, 0);
        task.update(f.clock());
        assertEquals(GuidedApproach.Phase.FINAL_INTAKE, f.owner.status().phase);
    }

    @Test public void contradictorySameTimeReconstructionFailsInsteadOfHidingBehindDeduplication() {
        Fixture f = new Fixture();
        Task task = f.startAt(7, 0);
        f.step(0.11, 7, 0);
        task.update(f.clock());
        LoopTimestamp stamp = f.frame.timestamp();
        f.step(0.02, 7, 0);
        f.frame = frame(stamp, 7, 0, 7, 0);
        task.update(f.clock());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(f.intake.isEmpty());
    }

    @Test public void olderCameraCaptureCannotFollowANewerCapture() {
        Fixture f = new Fixture();
        Task task = f.startAt(20, 0);
        TargetObservations2d original = f.frame;
        f.step(0.02, 15, 0);
        task.update(f.clock());
        f.step(0.02, 15, 0);
        f.frame = original;
        task.update(f.clock());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(f.owner.status().reason.contains("regressed"));
        assertTrue(f.intake.isEmpty());
    }

    @Test public void lossStalenessAndEmptyFramesNeverAuthorizeFinalIntake() {
        for (int phase = 0; phase < 2; phase++) {
            for (int loss = 0; loss < 3; loss++) {
                Fixture f = new Fixture();
                Task task = f.startAt(phase == 0 ? 20 : 7, 0);
                TargetObservations2d prior = f.frame;
                f.step(loss == 2 ? 0.31 : 0.02);
                if (loss == 0) f.frame = TargetObservations2d.unavailable("camera blocked");
                if (loss == 2) f.frame = prior;
                task.update(f.clock());
                assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
                assertTrue(f.intake.isEmpty());
            }
        }
    }

    @Test public void fullFrameAmbiguityAndIncompleteCandidatesFailEvenDuringSettling() {
        for (boolean duringSettle : new boolean[]{true, false}) {
            for (int invalid = 0; invalid < 3; invalid++) {
                Fixture f = new Fixture();
                Task task = f.startAt(7, 0);
                f.step(duringSettle ? 0.02 : 0.11, 7, 0, 7, 0);
                if (invalid == 1) {
                    LoopTimestamp stamp = f.clock().nowTimestamp();
                    f.frame = TargetObservations2d.fromFrame(stamp, Arrays.asList(
                            TargetObservation2d.ofRobotRelativePosition(7, 0, Double.NaN, stamp),
                            TargetObservation2d.ofRobotRelativeBearing(0.8, Double.NaN, stamp)));
                } else if (invalid == 2) f.frame = frame(f.clock().nowTimestamp(), 8, 0);
                task.update(f.clock());
                assertEquals("invalid=" + invalid, TaskOutcome.CANCELLED, task.getOutcome());
                assertTrue(f.intake.isEmpty());
            }
        }
    }

    @Test public void verificationCannotSubstituteAnUnselectedCloseCandidate() {
        Fixture f = new Fixture();
        Task task = f.startAt(7, 0);
        f.step(0.11, 7, 0, 10, 0);
        f.override = TargetSelectionResult.selected(f.frame, f.frame.observations().get(1),
                0.30, 0, "deliberately selected the farther target");
        task.update(f.clock());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(f.intake.isEmpty());
    }

    @Test public void firstVerificationCaptureMustStillMeetItsOriginalAgeAtHandoff() {
        Fixture f = new Fixture();
        Task task = f.startAt(7, 0);
        f.step(0.11, 7, 0);
        f.override = TargetSelectionResult.selected(f.frame, f.frame.observations().get(0),
                0.02, 0, "stricter first capture age");
        task.update(f.clock());
        assertEquals(1, f.owner.status().verifiedCaptureCount);
        f.step(0.03, 7, 0);
        f.override = TargetSelectionResult.selected(f.frame, f.frame.observations().get(0),
                1.0, 0, "second capture must not relax the first age");
        task.update(f.clock());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(f.owner.status().reason.contains("first verification capture expired"));
        assertTrue(f.intake.isEmpty());
    }

    @Test public void emptyOccupancyIsRequiredInAdmissionGuideAndVerify() {
        for (int phase = 0; phase < 3; phase++) {
            Fixture f = new Fixture();
            Task task;
            if (phase == 0) {
                f.capture(7, 0);
                f.feedback = OccupancyObservation.observed(true, f.clock().nowTimestamp());
                task = f.owner.createPickupTask(clock -> true);
                task.start(f.clock());
            } else {
                task = f.startAt(phase == 1 ? 20 : 7, 0);
                f.step(0.02, phase == 1 ? 15 : 7, 0);
                f.feedback = OccupancyObservation.observed(true, f.clock().nowTimestamp());
                task.update(f.clock());
            }
            assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
            assertTrue(f.intake.isEmpty());
        }
    }

    @Test public void missingStaleRegressedAndSameTimeChangedFeedbackNeverCapture() {
        for (int invalid = 0; invalid < 4; invalid++) {
            Fixture f = new Fixture();
            Task task = f.enterFinal();
            LoopTimestamp last = f.feedback.timestamp;
            f.step(invalid == 1 ? 0.11 : 0.02);
            if (invalid == 0) f.feedback = OccupancyObservation.unavailable();
            if (invalid == 1) f.feedback = OccupancyObservation.observed(false, last);
            if (invalid == 2) f.feedback = OccupancyObservation.observed(false,
                    f.clock().timestampSecondsAgo(0.04));
            if (invalid == 3) f.feedback = OccupancyObservation.observed(true, last);
            task.update(f.clock());
            assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
            assertEquals(Boolean.FALSE, f.intake.get(f.intake.size() - 1));
        }
    }

    @Test public void captureAtFinalEntryTimestampCannotClaimSuccess() {
        Fixture f = new Fixture();
        Task task = f.enterFinal();
        LoopTimestamp entry = f.clock().nowTimestamp();
        f.step(0.02);
        f.feedback = OccupancyObservation.observed(true, entry);
        task.update(f.clock());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
    }

    @Test public void finalTimeoutIsNotSuccessEvenWithOcclusionOrANewOccupiedSample() {
        for (boolean occupied : new boolean[]{false, true}) {
            Fixture f = new Fixture();
            Task task = f.enterFinal();
            f.step(0.51);
            f.feedback = OccupancyObservation.observed(occupied, f.clock().nowTimestamp());
            task.update(f.clock());
            assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
            assertEquals(Boolean.FALSE, f.intake.get(f.intake.size() - 1));
        }
    }

    @Test public void wholeAndVerificationDeadlinesWinBeforeNewEvidence() {
        Fixture guide = new Fixture();
        Task task = guide.startAt(20, 0);
        guide.step(3.01, 7, 0);
        task.update(guide.clock());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertTrue(guide.intake.isEmpty());
        Fixture verification = new Fixture();
        Task checking = verification.startAt(7, 0);
        verification.step(0.61, 7, 0);
        checking.update(verification.clock());
        assertEquals(TaskOutcome.TIMEOUT, checking.getOutcome());
        assertTrue(verification.intake.isEmpty());
    }

    @Test public void phaseTimingDoesNotConsumePrestartDtAndPositiveCommandSurvivesItsEntryCycle() {
        Fixture f = new Fixture();
        f.time.nextCycle(50);
        Task task = f.startAt(7, 0);
        task.update(f.clock());
        assertEquals(GuidedApproach.Phase.VERIFY, f.owner.status().phase);
        f.step(0.11, 7, 0);
        task.update(f.clock());
        f.step(0.01, 7, 0);
        task.update(f.clock());
        task.update(f.clock());
        assertEquals(Collections.singletonList(true), f.intake);
        assertEquals(0.15, f.drive().axial, 1e-12);
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
    }

    @Test public void translatedYawedToolUsesItsAxisForFinalTranslation() {
        Pose2d tool = new Pose2d(1, 2, Math.PI / 2);
        Fixture f = new Fixture(tool);
        Task task = f.startAt(1, 7);
        assertEquals(GuidedApproach.Phase.VERIFY, f.owner.status().phase);
        f.step(0.11, 1, 7);
        task.update(f.clock());
        f.step(0.01, 1, 7);
        task.update(f.clock());
        assertEquals(GuidedApproach.Phase.FINAL_INTAKE, f.owner.status().phase);
        assertEquals(0, f.drive().axial, 1e-12);
        assertEquals(0.15, f.drive().lateral, 1e-12);
        assertEquals(0, f.drive().omega, 0);
    }

    @Test public void extremeFiniteToolHeadingUsesTheSameTransformAsGuidance() {
        double heading = 1e16;
        Pose2d tool = new Pose2d(1, 2, heading);
        Pose2d target = tool.then(new Pose2d(5, 0, 0));
        Fixture f = new Fixture(tool);
        Task task = f.startAt(target.xInches, target.yInches);
        assertEquals(GuidedApproach.Phase.VERIFY, f.owner.status().phase);
        f.step(0.11, target.xInches, target.yInches);
        task.update(f.clock());
        f.step(0.01, target.xInches, target.yInches);
        task.update(f.clock());
        assertEquals(GuidedApproach.Phase.FINAL_INTAKE, f.owner.status().phase);
        assertEquals(0.15 * Math.cos(heading), f.drive().axial, 1e-12);
        assertEquals(0.15 * Math.sin(heading), f.drive().lateral, 1e-12);
    }

    @Test public void cancellationIsSingleUseActiveOnlyAndDoesNotReleaseUnclaimedIntake() {
        Fixture f = new Fixture();
        f.capture(20, 0);
        Task task = f.owner.createPickupTask(clock -> true);
        task.cancel();
        assertFalse(task.isComplete());
        assertThrows(IllegalStateException.class, () -> task.update(f.clock()));
        task.start(f.clock());
        assertThrows(IllegalStateException.class, () -> task.start(f.clock()));
        task.cancel();
        task.cancel();
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(f.intake.isEmpty());
        assertZero(f.drive());
        assertNotSame(task, f.owner.createPickupTask(clock -> true));
    }

    @Test public void cancellationDuringFinalWithdrawsDriveAndReleasesRequestOnce() {
        Fixture f = new Fixture();
        Task task = f.enterFinal();
        assertTrue(f.drive().axial > 0);
        task.cancel();
        assertZero(f.drive());
        task.cancel();
        f.owner.stop();
        f.owner.stop();
        assertEquals(Arrays.asList(true, false), f.intake);
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
    }

    @Test public void permissionLossPrecedesOccupiedFeedbackAndCannotAffectAnotherAttempt() {
        Fixture f = new Fixture();
        Task task = f.enterFinal();
        f.permitted = false;
        f.step(0.02);
        f.feedback = OccupancyObservation.observed(true, f.clock().nowTimestamp());
        task.update(f.clock());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(f.owner.status().reason.contains("permission"));
    }

    @Test public void stopAndClockResetInvalidateNonzeroIntentWithoutSourceAdvancement() {
        Fixture stopped = new Fixture();
        Task task = stopped.startAt(20, 0);
        stopped.owner.stop();
        assertZero(stopped.drive());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(GuidedApproach.Phase.STOPPED, stopped.owner.status().phase);
        Fixture reset = new Fixture();
        Task resetTask = reset.startAt(20, 0);
        reset.clock().reset(0);
        assertZero(reset.drive());
        resetTask.update(reset.clock());
        assertEquals(TaskOutcome.CANCELLED, resetTask.getOutcome());
        assertTrue(reset.intake.isEmpty());
    }

    @Test public void foreignClockEvidenceFailsExceptionallyAndNeverContinuesASequence() {
        Fixture f = new Fixture();
        f.capture(7, 0);
        f.feedback = OccupancyObservation.observed(false, new ManualLoopClock().clock().nowTimestamp());
        int[] next = {0};
        Task child = f.owner.createPickupTask(clock -> true);
        Task root = Tasks.sequenceOnCompletion(child, Tasks.runOnce(() -> next[0]++));
        RuntimeException failure = assertThrows(IllegalArgumentException.class, () -> root.start(f.clock()));
        assertSame(failure, assertThrows(RuntimeException.class, child::getOutcome));
        assertEquals(0, next[0]);
        assertTrue(f.owner.status().hasFailure);
        assertZero(f.drive());
    }

    @Test public void callbackCancellationCannotResumeAdmissionSelectionOrSensorWork() {
        for (int site = 0; site < 3; site++) {
            Fixture f = new Fixture();
            f.capture(7, 0);
            if (site == 0) f.permissionHook = f.owner::cancelPickup;
            if (site == 1) f.selectionHook = f.owner::cancelPickup;
            if (site == 2) f.occupancyHook = f.owner::cancelPickup;
            Task task = f.owner.createPickupTask(clock -> {
                f.permissionHook.run();
                return true;
            });
            task.start(f.clock());
            assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
            assertTrue(f.intake.isEmpty());
            assertZero(f.drive());
        }
    }

    @Test public void enablingSetterMayCancelOrStopButNeverRestoreTheFinalCommand() {
        for (boolean stop : new boolean[]{false, true}) {
            Fixture f = new Fixture();
            f.intakeHook = enabled -> {
                if (enabled) {
                    if (stop) f.owner.stop();
                    else f.owner.cancelPickup();
                }
            };
            Task task = f.enterFinal();
            assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
            assertEquals(TaskOutcome.CANCELLED, f.owner.status().outcome);
            assertEquals(Arrays.asList(true, false), f.intake);
            assertZero(f.drive());
        }
    }

    @Test public void setterFailureRetainsPrimaryAndOneCleanupFailureWithoutRepeatingEffects() {
        Fixture f = new Fixture();
        RuntimeException primary = new IllegalStateException("enable failed");
        RuntimeException cleanup = new IllegalStateException("release failed");
        f.intakeHook = enabled -> { throw enabled ? primary : cleanup; };
        Task task = f.startAt(7, 0);
        f.step(0.11, 7, 0);
        task.update(f.clock());
        f.step(0.01, 7, 0);
        assertSame(primary, assertThrows(RuntimeException.class, () -> task.update(f.clock())));
        assertSame(primary, assertThrows(RuntimeException.class, task::getOutcome));
        assertSame(primary, assertThrows(RuntimeException.class, () -> task.update(f.clock())));
        task.cancel();
        f.owner.stop();
        assertEquals(Arrays.asList(true, false), f.intake);
        assertArrayEquals(new Throwable[]{cleanup}, primary.getSuppressed());
        assertTrue(f.owner.status().hasFailure);
        assertEquals(TaskOutcome.NOT_DONE, f.owner.status().outcome);
        assertZero(f.drive());
    }

    @Test public void cleanupHidesOutcomeAndCaughtPendingInspectionStillFailsClosed() {
        Fixture f = new Fixture();
        Task task = f.enterFinal();
        f.intakeHook = enabled -> {
            if (!enabled) {
                assertEquals(TaskOutcome.NOT_DONE, f.owner.status().outcome);
                assertZero(f.drive());
                assertThrows(IllegalStateException.class, task::getOutcome); // Deliberately swallowed.
            }
        };
        RuntimeException failure = assertThrows(IllegalStateException.class, task::cancel);
        assertSame(failure, assertThrows(RuntimeException.class, task::getOutcome));
        assertTrue(f.owner.status().hasFailure);
        assertZero(f.drive());
        assertEquals(Arrays.asList(true, false), f.intake);
    }

    @Test public void rejectedConcurrentStartDuringCleanupCannotTouchIntakeOrReplaceStatus() {
        Fixture f = new Fixture();
        Task task = f.enterFinal();
        Task replacement = f.owner.createPickupTask(clock -> true);
        f.intakeHook = enabled -> { if (!enabled) replacement.start(f.clock()); };
        task.cancel();
        assertEquals(TaskOutcome.CANCELLED, replacement.getOutcome());
        assertEquals(Arrays.asList(true, false), f.intake);
        assertEquals(TaskOutcome.CANCELLED, f.owner.status().outcome);
    }

    @Test public void sourceExceptionsLatchZeroAndTaskFailureCannotReleaseRecovery() {
        Fixture f = new Fixture();
        Task task = f.startAt(20, 0);
        RuntimeException failure = new IllegalStateException("selected source failed");
        f.selectionHook = () -> { throw failure; };
        f.step(0.02, 15, 0);
        assertSame(failure, assertThrows(RuntimeException.class, () -> task.update(f.clock())));
        assertSame(failure, assertThrows(RuntimeException.class, task::getOutcome));
        assertZero(f.drive());
        assertTrue(f.owner.status().hasFailure);
        assertTrue(f.intake.isEmpty());
    }

    @Test public void driveSourceClockFailureImmediatelyReleasesAnActiveIntake() {
        Fixture f = new Fixture();
        Task task = f.enterFinal();
        RuntimeException failure = assertThrows(IllegalArgumentException.class,
                () -> f.owner.driveSource().get(new ManualLoopClock().clock()));
        assertSame(failure, assertThrows(RuntimeException.class, task::getOutcome));
        assertEquals(Arrays.asList(true, false), f.intake);
        assertZero(f.drive());
        assertTrue(f.owner.status().hasFailure);
    }

    @Test public void idleSamplingIsCachedAndBorrowedSourcesAreNeverReset() {
        Fixture f = new Fixture();
        f.idle = new DriveSignal(0.2, -0.1, 0.3);
        assertSame(f.idle, f.drive());
        assertSame(f.idle, f.drive());
        assertEquals(1, f.idleReads);
        f.owner.driveSource().reset();
        f.owner.stop();
        assertZero(f.drive());
        assertEquals(0, f.selectionReads);
        assertEquals(0, f.occupancyReads);
        assertTrue(f.intake.isEmpty());
    }

    @Test public void retainedStagesConstructIndependentOwnersWithoutAnySampling() {
        Fixture f = new Fixture();
        GuidedApproach.LimitStep stage = f.configure(new Pose2d(2, 0, 0));
        GuidedApproach first = stage.withinSec(3);
        GuidedApproach second = stage.withinSec(4);
        assertNotSame(first, second);
        assertEquals(0, f.selectionReads);
        assertEquals(0, f.occupancyReads);
        first.stop();
        f.capture(20, 0);
        Task task = second.createPickupTask(clock -> true);
        task.start(f.clock());
        assertFalse(task.isComplete());
        assertTrue(second.driveSource().get(f.clock()).axial > 0);
    }

    @Test public void invalidConfigurationIsRejectedAtTheAnswerWithoutSamplingInputs() {
        Fixture f = new Fixture();
        for (double bad : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -1}) {
            assertThrows(IllegalArgumentException.class, () -> GuidedApproach.cameraOnly(f.selected)
                    .throughTool(Pose2d.zero(), bad));
            assertThrows(IllegalArgumentException.class, () -> f.verification()
                    .verifyWithZeroCommand(bad, 0.1, 0.1, 0.6));
            assertThrows(IllegalArgumentException.class, () -> f.verification()
                    .verifyWithZeroCommand(0.1, bad, 0.1, 0.6));
            assertThrows(IllegalArgumentException.class, () -> f.verification()
                    .verifyWithZeroCommand(0.1, 0.1, bad, 0.6));
        }
        assertThrows(IllegalArgumentException.class, () -> f.verification()
                .verifyWithZeroCommand(0.1, 5, 0.1, 0.6));
        assertThrows(IllegalArgumentException.class, () -> f.verification()
                .verifyWithZeroCommand(0.1, 0.1, Math.PI / 2, 0.6));
        assertThrows(IllegalArgumentException.class, () -> f.verification()
                .verifyWithZeroCommand(0.1, 0.1, 0.1, 0.1));
        assertThrows(IllegalArgumentException.class, () -> f.verification()
                .verifyWithZeroCommand(0.1, 0.1, 0.1, 0.6).finalIntake(value -> {}, 1.01, 0.5));
        assertThrows(IllegalArgumentException.class, () -> f.verification()
                .verifyWithZeroCommand(0.1, 0.1, 0.1, 0.6).finalIntake(value -> {}, 0, 0.5));
        assertThrows(IllegalArgumentException.class, () -> f.configure(Pose2d.zero()).withinSec(0.4));
        assertEquals(0, f.selectionReads);
        assertEquals(0, f.occupancyReads);
        assertTrue(f.intake.isEmpty());
    }

    /** Synthetic external-world fixture: no commands are integrated into pretend physical motion. */
    private static final class Fixture {
        final ManualLoopClock time = new ManualLoopClock();
        final List<Boolean> intake = new ArrayList<>();
        TargetObservations2d frame = TargetObservations2d.unavailable("not captured");
        OccupancyObservation feedback = OccupancyObservation.unavailable();
        TargetSelectionResult override;
        Runnable selectionHook = () -> {};
        Runnable occupancyHook = () -> {};
        Runnable permissionHook = () -> {};
        Consumer<Boolean> intakeHook = value -> {};
        boolean permitted = true;
        int selectionReads;
        int occupancyReads;
        int idleReads;
        DriveSignal idle = DriveSignal.zero();
        final TargetSelectionSource realSelection = TargetSelections.fromVisibleObjects(
                Source.of(clock -> frame)).freshWithinSec(0.30).choose(TargetSelectionPolicies.nearestToRobot());
        final TargetSelectionSource selected = new TargetSelectionSource() {
            @Override public TargetSelectionResult get(LoopClock clock) {
                selectionReads++;
                selectionHook.run();
                return override != null ? override : realSelection.get(clock);
            }
            @Override public void reset() { throw new AssertionError("borrowed selector reset"); }
        };
        final GuidedApproach owner;

        Fixture() { this(new Pose2d(2, 0, 0)); }
        Fixture(Pose2d tool) { owner = configure(tool).withinSec(3.0); }
        LoopClock clock() { return time.clock(); }
        DriveSignal drive() { return owner.driveSource().get(clock()); }

        GuidedApproach.VerificationStep verification() {
            return GuidedApproach.cameraOnly(selected).throughTool(new Pose2d(2, 0, 0), 5)
                    .driveTuning(DriveGuidancePlan.Tuning.defaults().withMaxTranslateCmd(0.2)
                            .withMaxOmegaCmd(0.2).withMinOmegaCmd(0).withAimDeadbandRad(0));
        }

        GuidedApproach.LimitStep configure(Pose2d tool) {
            return GuidedApproach.cameraOnly(selected).throughTool(tool, 5)
                    .driveTuning(DriveGuidancePlan.Tuning.defaults().withMaxTranslateCmd(0.2)
                            .withMaxOmegaCmd(0.2).withMinOmegaCmd(0).withAimDeadbandRad(0))
                    .verifyWithZeroCommand(0.10, 0.20, 0.10, 0.60)
                    .finalIntake(value -> { intake.add(value); intakeHook.accept(value); }, 0.15, 0.50)
                    .captureFeedback(new Source<OccupancyObservation>() {
                        @Override public OccupancyObservation get(LoopClock clock) {
                            occupancyReads++;
                            occupancyHook.run();
                            return feedback;
                        }
                        @Override public void reset() { throw new AssertionError("borrowed occupancy reset"); }
                    }, 0.10)
                    .idleFrom(clock -> { idleReads++; return idle; });
        }

        void capture(double... coordinates) {
            frame = frame(clock().nowTimestamp(), coordinates);
            feedback = OccupancyObservation.observed(false, clock().nowTimestamp());
        }

        void step(double sec, double... coordinates) {
            time.nextCycle(sec);
            override = null;
            capture(coordinates);
        }

        Task startAt(double... coordinates) {
            capture(coordinates);
            Task task = owner.createPickupTask(clock -> { permissionHook.run(); return permitted; });
            task.start(clock());
            return task;
        }

        Task enterFinal() {
            Task task = startAt(7, 0);
            step(0.11, 7, 0);
            task.update(clock());
            step(0.01, 7, 0);
            task.update(clock());
            return task;
        }
    }

    private static TargetObservations2d frame(LoopTimestamp timestamp, double... coordinates) {
        List<TargetObservation2d> observations = new ArrayList<>();
        for (int index = 0; index < coordinates.length; index += 2) {
            observations.add(TargetObservation2d.ofRobotRelativePosition(coordinates[index],
                    coordinates[index + 1], Double.NaN, timestamp));
        }
        return TargetObservations2d.fromFrame(timestamp, observations);
    }

    private static void assertZero(DriveSignal signal) {
        assertEquals(0, signal.axial, 0);
        assertEquals(0, signal.lateral, 0);
        assertEquals(0, signal.omega, 0);
    }
}
