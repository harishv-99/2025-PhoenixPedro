package edu.ftcsushi.fw.drive.guidance;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.task.TaskRunner;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
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

    @Test
    public void sameCycleAndRecursiveUpdatesDoNotRepeatDriveOrGuidanceEffects() {
        ManualLoopClock clock = new ManualLoopClock();
        MutablePoseEstimator estimator = new MutablePoseEstimator(clock.clock().nowTimestamp());
        estimator.setAvailable(clock.clock().nowTimestamp());
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask task = new DriveGuidanceTask(drive, planFor(estimator), config(1.0));
        drive.onUpdate = () -> task.update(clock.clock());
        task.start(clock.clock());
        task.update(clock.clock());
        task.update(clock.clock());
        assertEquals(1, drive.updateCount);
        assertEquals(1, drive.driveCount);
        assertEquals(1, drive.stopCount);
        clock.nextCycle(0.02);
        estimator.setAvailable(clock.clock().nowTimestamp());
        task.update(clock.clock());
        assertEquals(2, drive.updateCount);
        assertEquals(2, drive.driveCount);
    }

    @Test
    public void noGuidanceDeadlineIsStrictAndItsTerminalLoopStopsOnlyOnce() {
        ManualLoopClock clock = new ManualLoopClock();
        MutablePoseEstimator estimator = new MutablePoseEstimator(clock.clock().nowTimestamp());
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask task = new DriveGuidanceTask(drive, planFor(estimator), config(0.125));
        task.start(clock.clock());
        clock.nextCycle(0.125);
        task.update(clock.clock());
        assertFalse(task.isComplete());
        assertEquals(2, drive.stopCount);
        clock.nextCycle(0.001);
        task.update(clock.clock());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertEquals(3, drive.stopCount);
        task.cancel();
        task.update(clock.clock());
        assertEquals(3, drive.stopCount);
    }

    @Test
    public void hardDeadlineKeepsItsStrictBoundaryAndStopsBeforeAnotherCommand() {
        ManualLoopClock clock = new ManualLoopClock();
        MutablePoseEstimator estimator = new MutablePoseEstimator(clock.clock().nowTimestamp());
        estimator.setAvailable(clock.clock().nowTimestamp());
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask.Config cfg = config(1.0);
        cfg.timeoutSec = 0.125;
        DriveGuidanceTask task = new DriveGuidanceTask(drive, planFor(estimator), cfg);
        task.start(clock.clock());
        clock.nextCycle(0.125);
        estimator.setAvailable(clock.clock().nowTimestamp());
        task.update(clock.clock());
        assertFalse(task.isComplete());
        assertEquals(1, drive.driveCount);
        clock.nextCycle(0.001);
        task.update(clock.clock());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertEquals(2, drive.updateCount);
        assertEquals(1, drive.driveCount);
        assertEquals(2, drive.stopCount);
    }

    @Test
    public void reentrantStopCancellationWinsNoGuidanceTimeoutWithoutRecursiveStop() {
        ManualLoopClock clock = new ManualLoopClock();
        MutablePoseEstimator estimator = new MutablePoseEstimator(clock.clock().nowTimestamp());
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask task = new DriveGuidanceTask(drive, planFor(estimator), config(0.1));
        task.start(clock.clock());
        drive.onStop = task::cancel;
        clock.nextCycle(0.11);
        task.update(clock.clock());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(2, drive.stopCount);
    }

    @Test
    public void reentrantCancellationDuringStartStopDoesNotStopTwice() {
        ManualLoopClock clock = new ManualLoopClock();
        MutablePoseEstimator estimator = new MutablePoseEstimator(clock.clock().nowTimestamp());
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask task = new DriveGuidanceTask(drive, planFor(estimator), config(1.0));
        drive.onStop = task::cancel;
        task.start(clock.clock());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(1, drive.stopCount);
    }

    @Test
    public void directUpdateFailureRetainsPrimaryAndSuppressesTerminalStopFailure() {
        ManualLoopClock clock = new ManualLoopClock();
        MutablePoseEstimator estimator = new MutablePoseEstimator(clock.clock().nowTimestamp());
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask task = new DriveGuidanceTask(drive, planFor(estimator), config(1.0));
        task.start(clock.clock());
        RuntimeException primary = new IllegalStateException("drive update failed");
        RuntimeException secondary = new IllegalStateException("drive stop failed");
        drive.onUpdate = () -> { throw primary; };
        drive.onStop = () -> { throw secondary; };
        assertSame(primary, assertThrows(RuntimeException.class, () -> task.update(clock.clock())));
        assertTrue(task.isComplete());
        assertSame(primary, assertThrows(RuntimeException.class, task::getOutcome));
        assertSame(primary, assertThrows(RuntimeException.class, () -> task.update(clock.clock())));
        assertEquals(1, primary.getSuppressed().length);
        assertSame(secondary, primary.getSuppressed()[0]);
        task.cancel();
        assertEquals(2, drive.stopCount);
    }

    @Test
    public void failedNoGuidanceStopIsNotRetriedOrReportedAsTimeout() {
        ManualLoopClock clock = new ManualLoopClock();
        MutablePoseEstimator estimator = new MutablePoseEstimator(clock.clock().nowTimestamp());
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask task = new DriveGuidanceTask(drive, planFor(estimator), config(0.1));
        task.start(clock.clock());
        RuntimeException primary = new IllegalStateException("unavailable stop failed");
        drive.onStop = () -> { throw primary; };
        clock.nextCycle(0.11);
        assertSame(primary, assertThrows(RuntimeException.class, () -> task.update(clock.clock())));
        assertSame(primary, assertThrows(RuntimeException.class, task::getOutcome));
        assertEquals(0, primary.getSuppressed().length);
        task.cancel();
        assertEquals(2, drive.stopCount);
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
        private int updateCount;
        private int stopCount;
        private Runnable onUpdate;
        private Runnable onStop;

        @Override
        public void update(LoopClock clock) {
            updateCount++;
            if (onUpdate != null) {
                onUpdate.run();
            }
        }

        @Override
        public void drive(DriveSignal signal) {
            driveCount++;
        }

        @Override
        public void stop() {
            stopCount++;
            if (onStop != null) {
                onStop.run();
            }
        }
    }
}
