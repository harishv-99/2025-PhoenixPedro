package edu.ftcphoenix.fw.drive.guidance;

import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.task.TaskRunner;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Verifies that guidance-loss timing begins at the actual no-command boundary. */
public final class DriveGuidanceTaskTimingTest {

    @Test
    public void noGuidanceTimeoutDoesNotConsumeDtFromBeforeTaskStart() {
        ManualLoopClock manualClock = new ManualLoopClock();
        manualClock.nextCycle(1.0);

        MutablePoseEstimator estimator = new MutablePoseEstimator(manualClock.clock().nowTimestamp());
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask task = new DriveGuidanceTask(drive, planFor(estimator), config(0.10));
        TaskRunner runner = new TaskRunner();
        runner.enqueue(task);

        runner.update(manualClock.clock());

        assertFalse(task.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        assertEquals(0, drive.driveCount);

        manualClock.nextCycle(0.08);
        runner.update(manualClock.clock());
        assertFalse(task.isComplete());

        manualClock.nextCycle(0.03);
        runner.update(manualClock.clock());
        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
    }

    @Test
    public void usableCommandResetsConsecutiveNoGuidanceInterval() {
        ManualLoopClock manualClock = new ManualLoopClock();
        MutablePoseEstimator estimator = new MutablePoseEstimator(manualClock.clock().nowTimestamp());
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask task = new DriveGuidanceTask(drive, planFor(estimator), config(0.10));
        TaskRunner runner = new TaskRunner();
        runner.enqueue(task);

        runner.update(manualClock.clock());
        manualClock.nextCycle(0.08);
        runner.update(manualClock.clock());
        assertFalse(task.isComplete());

        manualClock.nextCycle(0.01);
        estimator.setAvailable(manualClock.clock().nowTimestamp());
        runner.update(manualClock.clock());
        assertFalse(task.isComplete());
        assertTrue(drive.driveCount > 0);

        manualClock.nextCycle(0.01);
        estimator.setUnavailable(manualClock.clock().nowTimestamp());
        runner.update(manualClock.clock());

        manualClock.nextCycle(0.08);
        runner.update(manualClock.clock());
        assertFalse(task.isComplete());

        manualClock.nextCycle(0.03);
        runner.update(manualClock.clock());
        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
    }

    private static DriveGuidancePlan planFor(AbsolutePoseEstimator estimator) {
        return DriveGuidance.plan()
                .translateTo()
                    .fieldPointInches(12.0, 0.0)
                .solveWith()
                    .localizationOnlyWithDefaults(estimator)
                .build();
    }

    private static DriveGuidanceTask.Config config(double maxNoGuidanceSec) {
        DriveGuidanceTask.Config cfg = new DriveGuidanceTask.Config();
        cfg.timeoutSec = 5.0;
        cfg.maxNoGuidanceSec = maxNoGuidanceSec;
        return cfg;
    }

    private static final class MutablePoseEstimator implements AbsolutePoseEstimator {
        private PoseEstimate estimate;

        MutablePoseEstimator(LoopTimestamp timestamp) {
            setUnavailable(timestamp);
        }

        void setAvailable(LoopTimestamp timestamp) {
            estimate = new PoseEstimate(Pose3d.zero(), true, 1.0, timestamp);
        }

        void setUnavailable(LoopTimestamp timestamp) {
            estimate = PoseEstimate.noPose(timestamp);
        }

        @Override
        public void update(LoopClock clock) {
            // The test controls the current snapshot directly.
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }
    }

    private static final class RecordingDriveSink implements DriveCommandSink {
        private int driveCount;

        @Override
        public void drive(DriveSignal signal) {
            driveCount++;
        }

        @Override
        public void stop() {
            // Stopping is expected while guidance is unavailable and on timeout.
        }
    }
}
