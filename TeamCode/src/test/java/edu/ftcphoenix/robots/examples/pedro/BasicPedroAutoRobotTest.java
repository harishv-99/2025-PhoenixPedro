package edu.ftcphoenix.robots.examples.pedro;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTargetStatus;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the reference composition root and the disabled FTC lifecycle host. */
public final class BasicPedroAutoRobotTest {

    @Test
    public void startQueuesOnlyAndFirstLoopUsesRequiredOwnershipOrder() {
        List<String> events = new ArrayList<String>();
        RecordingPlant plant = new RecordingPlant(events);
        RecordingTask task = new RecordingTask(events);
        BasicPedroAutoRobot robot = new BasicPedroAutoRobot(
                new RecordingLocalization(events),
                new RecordingDrive(events),
                () -> events.add("startPose"),
                new BasicPedroAutoMechanism(plant, 0.7),
                task
        );

        robot.start(10.0);
        assertEquals(Arrays.asList("startPose"), events);
        assertFalse(robot.isAutoIdle());

        robot.update(10.25);

        assertEquals(
                Arrays.asList(
                        "startPose",
                        "localization(cycle=1,dt=0.25)",
                        "drive.update(cycle=1)",
                        "task.start(cycle=1)",
                        "task.update(cycle=1)",
                        "plant.update(cycle=1)"
                ),
                events
        );
    }

    @Test
    public void stopIsExactOnceAndAttemptsEveryCleanupAfterFailures() {
        List<String> events = new ArrayList<String>();
        RuntimeException taskFailure = new RuntimeException("task cleanup failed");
        RuntimeException mechanismFailure = new RuntimeException("mechanism stop failed");
        RuntimeException driveFailure = new RuntimeException("drive stop failed");
        RecordingPlant plant = new RecordingPlant(events);
        plant.stopFailure = mechanismFailure;
        RecordingTask task = new RecordingTask(events);
        task.cancelFailure = taskFailure;
        RecordingDrive drive = new RecordingDrive(events);
        drive.stopFailure = driveFailure;
        BasicPedroAutoRobot robot = new BasicPedroAutoRobot(
                new RecordingLocalization(events),
                drive,
                () -> events.add("startPose"),
                new BasicPedroAutoMechanism(plant, 0.7),
                task
        );
        robot.start(0.0);
        robot.update(0.02);
        events.clear();

        try {
            robot.stop();
            fail("expected cleanup failure");
        } catch (RuntimeException failure) {
            assertSame(taskFailure, failure);
            assertEquals(2, failure.getSuppressed().length);
            assertSame(mechanismFailure, failure.getSuppressed()[0]);
            assertSame(driveFailure, failure.getSuppressed()[1]);
        }

        assertEquals(
                Arrays.asList("task.cancel", "plant.stop", "drive.stop"),
                events
        );
        assertEquals(0.0, plant.target.get(), 0.0);
        assertTrue(robot.isAutoIdle());
        assertTrue(robot.isStopped());

        robot.stop();
        assertEquals(3, events.size());
    }

    /** Creates the same small fake-backed root for the separate FTC host lifecycle test. */
    public static BasicPedroAutoRobot newRecordingRobot(List<String> events) {
        RecordingPlant plant = new RecordingPlant(events);
        return new BasicPedroAutoRobot(
                new RecordingLocalization(events),
                new RecordingDrive(events),
                () -> events.add("startPose"),
                new BasicPedroAutoMechanism(plant, 0.7),
                new RecordingTask(events)
        );
    }

    private static final class RecordingLocalization implements AbsolutePoseEstimator {
        private final List<String> events;

        RecordingLocalization(List<String> events) {
            this.events = events;
        }

        @Override
        public void update(LoopClock clock) {
            events.add(String.format(
                    "localization(cycle=%d,dt=%.2f)",
                    clock.cycle(),
                    clock.dtSec()
            ));
        }

        @Override
        public PoseEstimate getEstimate() {
            return PoseEstimate.noPose(0.0);
        }
    }

    private static final class RecordingDrive implements DriveCommandSink {
        private final List<String> events;
        RuntimeException stopFailure;

        RecordingDrive(List<String> events) {
            this.events = events;
        }

        @Override
        public void update(LoopClock clock) {
            events.add("drive.update(cycle=" + clock.cycle() + ")");
        }

        @Override
        public void drive(DriveSignal signal) {
            events.add("drive.command");
        }

        @Override
        public void stop() {
            events.add("drive.stop");
            if (stopFailure != null) {
                throw stopFailure;
            }
        }
    }

    private static final class RecordingPlant implements Plant {
        private final List<String> events;
        final ScalarTarget target = ScalarTarget.held(0.0);
        RuntimeException stopFailure;

        RecordingPlant(List<String> events) {
            this.events = events;
        }

        @Override
        public void update(LoopClock clock) {
            events.add("plant.update(cycle=" + clock.cycle() + ")");
        }

        @Override
        public double getRequestedTarget() {
            return target.get();
        }

        @Override
        public double getAppliedTarget() {
            return target.get();
        }

        @Override
        public PlantTargetStatus getTargetStatus() {
            return PlantTargetStatus.ACCEPTED;
        }

        @Override
        public boolean hasWritableTarget() {
            return true;
        }

        @Override
        public ScalarTarget writableTarget() {
            return target;
        }

        @Override
        public void stop() {
            events.add("plant.stop");
            if (stopFailure != null) {
                throw stopFailure;
            }
        }
    }

    private static final class RecordingTask implements Task {
        private final List<String> events;
        private boolean started;
        private boolean cancelled;
        RuntimeException cancelFailure;

        RecordingTask(List<String> events) {
            this.events = events;
        }

        @Override
        public void start(LoopClock clock) {
            started = true;
            events.add("task.start(cycle=" + clock.cycle() + ")");
        }

        @Override
        public void update(LoopClock clock) {
            events.add("task.update(cycle=" + clock.cycle() + ")");
        }

        @Override
        public void cancel() {
            if (!started || cancelled) {
                return;
            }
            cancelled = true;
            events.add("task.cancel");
            if (cancelFailure != null) {
                throw cancelFailure;
            }
        }

        @Override
        public boolean isComplete() {
            return cancelled;
        }

        @Override
        public TaskOutcome getOutcome() {
            return cancelled ? TaskOutcome.CANCELLED : TaskOutcome.NOT_DONE;
        }
    }
}
