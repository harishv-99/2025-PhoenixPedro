package edu.ftcphoenix.fw.drive.guidance;

import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies single-use lifecycle enforcement at the autonomous-guidance task boundary. */
public final class DriveGuidanceTaskSingleUseTest {

    @Test
    public void secondStartWhileActiveFailsBeforeStoppingDriveAgain() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask task = new DriveGuidanceTask(
                "autoAlign", drive, unavailablePlan(manualClock.clock().nowSec()), config());

        task.start(manualClock.clock());
        assertEquals(1, drive.stopCount);

        IllegalStateException failure = expectSecondStartFailure(task, manualClock.clock());

        assertActionable(failure, "autoAlign", "DriveGuidancePlan.task");
        assertEquals(1, drive.stopCount);
        assertEquals(0, drive.driveCount);
        assertFalse(task.isComplete());
    }

    @Test
    public void secondStartAfterTerminalCancelHasNoRepeatedDriveSideEffect() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask task = new DriveGuidanceTask(
                "cancelledAlign", drive, unavailablePlan(manualClock.clock().nowSec()), config());

        task.start(manualClock.clock());
        task.cancel();
        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(2, drive.stopCount);

        IllegalStateException failure = expectSecondStartFailure(task, manualClock.clock());

        assertActionable(failure, "cancelledAlign", "Supplier<Task>");
        assertEquals(2, drive.stopCount);
        assertEquals(0, drive.driveCount);
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
    }

    @Test
    public void freshPlanTasksCanStartIndependently() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidancePlan plan = unavailablePlan(manualClock.clock().nowSec());
        Task first = plan.task(drive, config());
        Task second = plan.task(drive, config());

        first.start(manualClock.clock());
        first.cancel();
        second.start(manualClock.clock());

        assertEquals(3, drive.stopCount);
        assertFalse(second.isComplete());
    }

    @Test
    public void cancelBeforeFirstStartKeepsExistingGuidanceBehavior() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask task = new DriveGuidanceTask(
                "cancelBeforeStart", drive, unavailablePlan(manualClock.clock().nowSec()), config());

        task.cancel();
        assertTrue(task.isComplete());
        assertEquals(1, drive.stopCount);

        task.start(manualClock.clock());

        assertEquals(2, drive.stopCount);
        assertFalse(task.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
    }

    private static IllegalStateException expectSecondStartFailure(Task task, LoopClock clock) {
        try {
            task.start(clock);
            fail("Expected a second start to fail");
            return null;
        } catch (IllegalStateException expected) {
            return expected;
        }
    }

    private static void assertActionable(IllegalStateException failure,
                                         String taskName,
                                         String freshTaskHint) {
        assertTrue(failure.getMessage().contains(taskName));
        assertTrue(failure.getMessage().contains("single-use"));
        assertTrue(failure.getMessage().contains(freshTaskHint));
    }

    private static DriveGuidancePlan unavailablePlan(double nowSec) {
        return DriveGuidance.plan()
                .translateTo()
                    .fieldPointInches(12.0, 0.0)
                .solveWith()
                    .localizationOnlyWithDefaults(new NoPoseEstimator(nowSec))
                .build();
    }

    private static DriveGuidanceTask.Config config() {
        DriveGuidanceTask.Config cfg = new DriveGuidanceTask.Config();
        cfg.timeoutSec = 5.0;
        cfg.maxNoGuidanceSec = 1.0;
        return cfg;
    }

    private static final class NoPoseEstimator implements AbsolutePoseEstimator {
        private final PoseEstimate estimate;

        NoPoseEstimator(double nowSec) {
            estimate = new PoseEstimate(Pose3d.zero(), false, 0.0, 0.0, nowSec);
        }

        @Override
        public void update(LoopClock clock) {
            // Immutable no-pose test snapshot.
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }
    }

    private static final class RecordingDriveSink implements DriveCommandSink {
        private int driveCount;
        private int stopCount;

        @Override
        public void drive(DriveSignal signal) {
            driveCount++;
        }

        @Override
        public void stop() {
            stopCount++;
        }
    }
}
