package edu.ftcsushi.fw.drive.guidance;

import java.util.Collections;
import java.util.function.Consumer;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.sensing.observation.OccupancyObservation;
import edu.ftcsushi.fw.sensing.observation.TargetObservation2d;
import edu.ftcsushi.fw.sensing.observation.TargetObservations2d;
import edu.ftcsushi.fw.sensing.observation.TargetSelectionPolicies;
import edu.ftcsushi.fw.sensing.observation.TargetSelectionSource;
import edu.ftcsushi.fw.sensing.observation.TargetSelections;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Independent software boundary checks; authored frames are not a camera or chassis simulation. */
public final class CameraOnlyPickupBoundaryTest {
    @Test public void verificationNeedsCapturesStrictlyAfterSettlingNotJustNewLoopReads() {
        Rig rig = new Rig();
        Task task = rig.start();
        assertZero(rig.drive());

        rig.tick(task, 0.125); // Exactly the settling boundary: not a qualifying capture.
        assertFalse(rig.collecting);
        rig.tick(task, 0.125); // First strictly later capture.
        assertFalse(rig.collecting);

        rig.time.nextCycle(0.0); // A different cycle at the same timestamp is not another capture.
        rig.publish(false);
        task.update(rig.clock());
        assertFalse(rig.collecting);
        rig.tick(task, 0.125); // Second strictly later capture permits final command.
        assertTrue(rig.collecting);
        assertTrue(rig.drive().axial > 0);
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
    }

    @Test public void finalCommandUsesToolDirectionAndDoesNotNeedAnotherCameraRead() {
        Rig rig = new Rig(new Pose2d(6, 1, Math.PI / 2));
        Task task = rig.start();
        rig.enterFinal(task);
        int readsAtHandoff = rig.cameraReads;
        rig.frame = TargetObservations2d.unavailable("intake now blocks the camera");
        rig.time.nextCycle(0.125);
        rig.occupancy = OccupancyObservation.observed(false, rig.clock().nowTimestamp());
        task.update(rig.clock());
        DriveSignal command = rig.drive();
        assertEquals(0, command.axial, 1e-12);
        assertEquals(0.20, command.lateral, 1e-12);
        assertEquals(0, command.omega, 0);
        assertEquals(readsAtHandoff, rig.cameraReads);
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
    }

    @Test public void finalDeadlineBeatsSimultaneousNewOccupiedEvidence() {
        Rig rig = new Rig();
        Task task = rig.start();
        rig.enterFinal(task);
        rig.time.nextCycle(0.5); // Exact configured final duration.
        rig.occupancy = OccupancyObservation.observed(true, rig.clock().nowTimestamp());
        task.update(rig.clock());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertFalse(rig.collecting);
        assertEquals(1, rig.enables);
        assertEquals(1, rig.disables);
    }

    @Test public void rejectingConcurrentStartCannotWithdrawTheFirstAttemptsCommand() {
        Rig rig = new Rig();
        rig.extraForwardInches = 10;
        rig.publish(false);
        Task first = rig.start();
        DriveSignal before = rig.drive();
        assertTrue(before.axial > 0);
        Task second = rig.pickup.createPickupTask(BooleanSource.of(() -> true));
        second.start(rig.clock());
        assertEquals(TaskOutcome.CANCELLED, second.getOutcome());
        assertFalse(first.isComplete());
        assertSignal(before, rig.drive());
        rig.tick(first, 0.125);
        assertTrue(rig.drive().axial > 0);
        first.cancel();
        assertEquals(TaskOutcome.CANCELLED, first.getOutcome());
        assertEquals(0, rig.disables); // This attempt never acquired an intake request.
    }

    @Test public void cachedDriveCannotSurviveSkippedUpdateOrSameCycleCancellation() {
        Rig rig = new Rig();
        rig.extraForwardInches = 10;
        rig.publish(false);
        Task task = rig.start();
        assertTrue(rig.drive().axial > 0);

        rig.time.nextCycle(0.125);
        assertZero(rig.drive()); // Merely reading drive never advances the attempt.
        rig.publish(false);
        task.update(rig.clock());
        assertTrue(rig.drive().axial > 0);
        task.cancel();
        assertSignal(Rig.MANUAL, rig.drive()); // Same cycle must not replay active intent.
        assertEquals(0, rig.disables);
    }

    @Test public void stopDuringIdleSamplingCannotReturnTheIdleCallbacksLaterCommand() {
        Rig rig = new Rig();
        GuidedApproach[] owner = new GuidedApproach[1];
        owner[0] = rig.build(clock -> {
            owner[0].stop();
            return new DriveSignal(1, 1, 1);
        }, rig::setCollecting);
        assertZero(owner[0].driveSource().get(rig.clock()));
        assertZero(owner[0].driveSource().get(rig.clock()));
        assertEquals(0, rig.disables);
    }

    @Test public void freshPostFinalOccupancySucceedsWithoutClaimingContinuedVisibility() {
        Rig rig = new Rig();
        Task task = rig.start();
        rig.enterFinal(task);
        LoopTimestamp verified = rig.pickup.status().verificationFrame.timestamp();
        rig.time.nextCycle(0.125);
        rig.frame = TargetObservations2d.unavailable("hidden");
        rig.occupancy = OccupancyObservation.observed(true, rig.clock().nowTimestamp());
        task.update(rig.clock());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertSame(verified, rig.pickup.status().verificationFrame.timestamp());
        assertFalse(rig.collecting);
        assertEquals(1, rig.disables);
        assertSignal(Rig.MANUAL, rig.drive());
    }

    @Test public void cleanupFailureCannotReleaseSuccessOrRetryOnTheFailedOwner() {
        Rig rig = new Rig();
        RuntimeException cleanupFailure = new IllegalStateException("test intake release failed");
        rig.pickup = rig.build(clock -> Rig.MANUAL, enabled -> {
            rig.setCollecting(enabled);
            if (!enabled) throw cleanupFailure;
        });
        Task task = rig.start();
        rig.enterFinal(task);
        rig.time.nextCycle(0.125);
        rig.occupancy = OccupancyObservation.observed(true, rig.clock().nowTimestamp());
        assertSame(cleanupFailure, expectFailure(() -> task.update(rig.clock())));
        assertSame(cleanupFailure, expectFailure(task::getOutcome));
        assertTrue(rig.pickup.status().hasFailure);
        assertZero(rig.drive());
        expectFailure(() -> rig.pickup.createPickupTask(BooleanSource.of(() -> true)).start(rig.clock()));
        assertEquals(1, rig.enables);
        assertEquals(1, rig.disables);
    }

    private static RuntimeException expectFailure(Runnable work) {
        try {
            work.run();
            fail("Expected a runtime failure");
            throw new AssertionError();
        } catch (RuntimeException expected) {
            return expected;
        }
    }

    private static void assertZero(DriveSignal signal) {
        assertEquals(0, signal.axial, 0);
        assertEquals(0, signal.lateral, 0);
        assertEquals(0, signal.omega, 0);
    }

    private static void assertSignal(DriveSignal expected, DriveSignal actual) {
        assertEquals(expected.axial, actual.axial, 0);
        assertEquals(expected.lateral, actual.lateral, 0);
        assertEquals(expected.omega, actual.omega, 0);
    }

    /** One real selector/owner/clock with only camera, occupancy, and requested outputs substituted. */
    private static final class Rig {
        static final DriveSignal MANUAL = new DriveSignal(-0.1, 0.05, -0.05);
        final ManualLoopClock time = new ManualLoopClock();
        final Pose2d robotToTool;
        final TargetSelectionSource selected;
        TargetObservations2d frame;
        OccupancyObservation occupancy;
        GuidedApproach pickup;
        double extraForwardInches;
        boolean collecting;
        int enables;
        int disables;
        int cameraReads;

        Rig() { this(new Pose2d(6, 1, 0)); }

        Rig(Pose2d robotToTool) {
            this.robotToTool = robotToTool;
            publish(false);
            selected = TargetSelections.fromVisibleObjects(Source.of(clock -> {
                cameraReads++;
                return frame;
            })).freshWithinSec(1).choose(TargetSelectionPolicies.nearestToRobot());
            pickup = build(clock -> MANUAL, this::setCollecting);
        }

        GuidedApproach build(DriveSource idle, Consumer<Boolean> intake) {
            return GuidedApproach.cameraOnly(selected).throughTool(robotToTool, 2)
                    .driveTuning(DriveGuidancePlan.Tuning.defaults())
                    .verifyWithZeroCommand(0.125, 0.25, 0.1, 1)
                    .finalIntake(intake, 0.2, 0.5)
                    .captureFeedback(Source.of(clock -> occupancy), 1)
                    .idleFrom(idle).withinSec(4);
        }

        LoopClock clock() { return time.clock(); }
        DriveSignal drive() { return pickup.driveSource().get(clock()); }

        Task start() {
            Task task = pickup.createPickupTask(BooleanSource.of(() -> true));
            task.start(clock());
            task.update(clock());
            return task;
        }

        void tick(Task task, double elapsed) {
            time.nextCycle(elapsed);
            publish(false);
            task.update(clock());
        }

        void enterFinal(Task task) {
            for (int i = 0; i < 5 && !collecting; i++) tick(task, 0.125);
            assertTrue("Fresh distinct close frames should authorize final intake", collecting);
        }

        void publish(boolean occupied) {
            LoopTimestamp capture = clock().nowTimestamp();
            TargetObservation2d ball = TargetObservation2d.ofRobotRelativePosition(
                    robotToTool.xInches + 2 * Math.cos(robotToTool.headingRad) + extraForwardInches,
                    robotToTool.yInches + 2 * Math.sin(robotToTool.headingRad), Double.NaN, capture);
            frame = TargetObservations2d.fromFrame(capture, Collections.singletonList(ball));
            occupancy = OccupancyObservation.observed(occupied, capture);
        }

        void setCollecting(boolean enabled) {
            collecting = enabled;
            if (enabled) enables++; else disables++;
        }
    }
}
