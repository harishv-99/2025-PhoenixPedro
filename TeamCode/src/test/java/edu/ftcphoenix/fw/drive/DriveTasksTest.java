package edu.ftcphoenix.fw.drive;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.task.TaskRunner;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the exclusive ownership and lifecycle contract exposed by {@link DriveTasks}. */
public final class DriveTasksTest {

    @Test
    public void positiveStartUpdatesThenDrivesAndDeduplicatesTheStartCycle() {
        ManualLoopClock manualClock = new ManualLoopClock();
        manualClock.nextCycle(2.0);
        RecordingDriveSink sink = new RecordingDriveSink();
        DriveSignal command = new DriveSignal(0.4, -0.1, 0.2);
        Task task = DriveTasks.driveExclusivelyForSeconds(sink, command, 0.5);

        task.start(manualClock.clock());

        assertFalse(task.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        assertEquals(Arrays.asList("update", "drive"), sink.events);
        assertEquals(1, sink.updateCount);
        assertEquals(1, sink.driveCount);
        assertEquals(0, sink.stopCount);
        assertSame(command, sink.lastCommand);

        task.update(manualClock.clock());

        assertEquals(Arrays.asList("update", "drive"), sink.events);
        assertEquals(1, sink.updateCount);
        assertEquals(1, sink.driveCount);
    }

    @Test
    public void eachLaterUnexpiredCycleRefreshesExactlyOnceInUpdateThenDriveOrder() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingDriveSink sink = new RecordingDriveSink();
        DriveSignal command = new DriveSignal(0.3, 0.2, -0.1);
        Task task = DriveTasks.driveExclusivelyForSeconds(sink, command, 1.0);
        task.start(manualClock.clock());

        manualClock.nextCycle(0.1);
        task.update(manualClock.clock());
        task.update(manualClock.clock());

        manualClock.nextCycle(0.1);
        task.update(manualClock.clock());

        assertEquals(
                Arrays.asList("update", "drive", "update", "drive", "update", "drive"),
                sink.events);
        assertEquals(3, sink.updateCount);
        assertEquals(3, sink.driveCount);
        assertEquals(0, sink.stopCount);
        assertSame(command, sink.lastCommand);
        assertFalse(task.isComplete());
    }

    @Test
    public void exactAndOvershootExpiryStopWithoutAnotherUpdateOrDrive() {
        assertExpiryStopsWithoutAnotherCommand(0.25, 0.25);
        assertExpiryStopsWithoutAnotherCommand(0.25, 0.40);
    }

    @Test
    public void positiveDurationShorterThanOneLoopIsPublishedBeforeItExpires() {
        ManualLoopClock manualClock = new ManualLoopClock();
        manualClock.nextCycle(3.0);
        RecordingDriveSink sink = new RecordingDriveSink();
        DriveSignal command = new DriveSignal(0.2, 0.0, 0.0);
        Task task = DriveTasks.driveExclusivelyForSeconds(sink, command, 0.02);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(task);

        runner.update(manualClock.clock());

        assertFalse(task.isComplete());
        assertEquals(Arrays.asList("update", "drive"), sink.events);
        assertSame(command, sink.lastCommand);

        manualClock.nextCycle(0.10);
        runner.update(manualClock.clock());

        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(Arrays.asList("update", "drive", "stop"), sink.events);

        manualClock.nextCycle(0.01);
        runner.update(manualClock.clock());
        assertTrue(runner.isIdle());
        assertEquals(Arrays.asList("update", "drive", "stop"), sink.events);
    }

    @Test
    public void lateSequenceChildPublishesFromItsOwnStartBoundary() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingDriveSink sink = new RecordingDriveSink();
        DriveSignal command = new DriveSignal(0.25, 0.0, 0.0);
        Task drive = DriveTasks.driveExclusivelyForSeconds(sink, command, 0.02);
        Task sequence = Tasks.sequence(Tasks.waitForSeconds(0.10), drive);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(sequence);

        runner.update(manualClock.clock());
        assertTrue(sink.events.isEmpty());

        manualClock.nextCycle(0.10);
        runner.update(manualClock.clock());

        assertFalse(sequence.isComplete());
        assertFalse(drive.isComplete());
        assertEquals(Arrays.asList("update", "drive"), sink.events);
        assertSame(command, sink.lastCommand);

        manualClock.nextCycle(0.10);
        runner.update(manualClock.clock());

        assertTrue(sequence.isComplete());
        assertEquals(TaskOutcome.SUCCESS, drive.getOutcome());
        assertEquals(Arrays.asList("update", "drive", "stop"), sink.events);
    }

    @Test
    public void zeroDurationPublishesNoMotionAndStopsExactlyOnce() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingDriveSink sink = new RecordingDriveSink();
        Task task = DriveTasks.driveExclusivelyForSeconds(
                sink, new DriveSignal(0.5, 0.0, 0.0), 0.0);

        task.start(manualClock.clock());

        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(Arrays.asList("stop"), sink.events);
        assertEquals(0, sink.updateCount);
        assertEquals(0, sink.driveCount);
        assertEquals(1, sink.stopCount);

        task.update(manualClock.clock());
        task.cancel();
        assertEquals(Arrays.asList("stop"), sink.events);
    }

    @Test
    public void throwingZeroDurationStopLeavesSuccessTerminalAndIsNeverRetried() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingDriveSink sink = new RecordingDriveSink();
        sink.throwOnStopNumber = 1;
        Task task = DriveTasks.driveExclusivelyForSeconds(sink, DriveSignal.zero(), 0.0);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(task);

        IllegalStateException failure = expectIllegalState(
                () -> runner.update(manualClock.clock()), "test stop failure");

        assertEquals(0, failure.getSuppressed().length);
        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(Arrays.asList("stop"), sink.events);
        assertEquals(1, sink.stopCount);
        assertTrue(runner.isIdle());

        task.cancel();
        task.update(manualClock.clock());
        assertEquals(1, sink.stopCount);
    }

    @Test
    public void factoryRejectsNullInputsAndInvalidDurationsWithParameterNames() {
        RecordingDriveSink sink = new RecordingDriveSink();
        DriveSignal signal = DriveSignal.zero();

        assertRejected("sink", () ->
                DriveTasks.driveExclusivelyForSeconds(null, signal, 0.1));
        assertRejected("signal", () ->
                DriveTasks.driveExclusivelyForSeconds(sink, null, 0.1));
        assertRejected("durationSec", () ->
                DriveTasks.driveExclusivelyForSeconds(sink, signal, -0.01));
        assertRejected("durationSec", () ->
                DriveTasks.driveExclusivelyForSeconds(sink, signal, Double.NaN));
        assertRejected("durationSec", () ->
                DriveTasks.driveExclusivelyForSeconds(sink, signal, Double.POSITIVE_INFINITY));
        assertRejected("durationSec", () ->
                DriveTasks.driveExclusivelyForSeconds(sink, signal, Double.NEGATIVE_INFINITY));
    }

    @Test
    public void updateBeforeStartFailsWithoutTouchingTheSink() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingDriveSink sink = new RecordingDriveSink();
        Task task = DriveTasks.driveExclusivelyForSeconds(sink, DriveSignal.zero(), 0.2);

        IllegalStateException failure = expectIllegalState(
                () -> task.update(manualClock.clock()), "before start");

        assertTrue(failure.getMessage().contains("TaskRunner"));
        assertTrue(sink.events.isEmpty());
        assertFalse(task.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
    }

    @Test
    public void cancelBeforeStartIsNoOpAndDoesNotPreventTheFirstStart() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingDriveSink sink = new RecordingDriveSink();
        DriveSignal command = new DriveSignal(0.1, 0.2, 0.3);
        Task task = DriveTasks.driveExclusivelyForSeconds(sink, command, 0.2);

        task.cancel();
        task.cancel();

        assertFalse(task.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        assertTrue(sink.events.isEmpty());

        task.start(manualClock.clock());

        assertFalse(task.isComplete());
        assertEquals(Arrays.asList("update", "drive"), sink.events);
        assertSame(command, sink.lastCommand);
    }

    @Test
    public void activeCancellationIsTerminalIdempotentAndStopsExactlyOnce() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingDriveSink sink = new RecordingDriveSink();
        Task task = DriveTasks.driveExclusivelyForSeconds(sink, DriveSignal.zero(), 1.0);
        task.start(manualClock.clock());

        task.cancel();

        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(Arrays.asList("update", "drive", "stop"), sink.events);

        task.cancel();
        manualClock.nextCycle(0.1);
        task.update(manualClock.clock());

        assertEquals(Arrays.asList("update", "drive", "stop"), sink.events);
        assertEquals(1, sink.stopCount);
    }

    @Test
    public void throwingActiveCancellationLeavesCancelledTerminalAndIsNeverRetried() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingDriveSink sink = new RecordingDriveSink();
        Task task = DriveTasks.driveExclusivelyForSeconds(sink, DriveSignal.zero(), 1.0);
        task.start(manualClock.clock());
        sink.throwOnStopNumber = 1;

        expectIllegalState(task::cancel, "test stop failure");

        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(Arrays.asList("update", "drive", "stop"), sink.events);
        assertEquals(1, sink.stopCount);

        task.cancel();
        manualClock.nextCycle(0.1);
        task.update(manualClock.clock());
        assertEquals(1, sink.stopCount);
    }

    @Test
    public void secondStartWhileActiveOrAfterCompletionFailsBeforeAnySideEffect() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingDriveSink activeSink = new RecordingDriveSink();
        Task active = DriveTasks.driveExclusivelyForSeconds(activeSink, DriveSignal.zero(), 1.0);
        active.start(manualClock.clock());

        IllegalStateException activeFailure = expectIllegalState(
                () -> active.start(manualClock.clock()), "single-use");

        assertTrue(activeFailure.getMessage().contains("fresh"));
        assertEquals(Arrays.asList("update", "drive"), activeSink.events);

        RecordingDriveSink completeSink = new RecordingDriveSink();
        Task complete = DriveTasks.driveExclusivelyForSeconds(
                completeSink, DriveSignal.zero(), 0.0);
        complete.start(manualClock.clock());

        expectIllegalState(() -> complete.start(manualClock.clock()), "single-use");

        assertEquals(TaskOutcome.SUCCESS, complete.getOutcome());
        assertEquals(Arrays.asList("stop"), completeSink.events);
    }

    @Test
    public void cancellationReenteredFromSinkUpdateStopsAndPreventsTheNonzeroWrite() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingDriveSink sink = new RecordingDriveSink();
        Task[] taskHolder = new Task[1];
        sink.onUpdate = () -> taskHolder[0].cancel();
        taskHolder[0] = DriveTasks.driveExclusivelyForSeconds(
                sink, new DriveSignal(0.6, 0.0, 0.0), 1.0);

        taskHolder[0].start(manualClock.clock());

        assertTrue(taskHolder[0].isComplete());
        assertEquals(TaskOutcome.CANCELLED, taskHolder[0].getOutcome());
        assertEquals(Arrays.asList("update", "stop"), sink.events);
        assertEquals(0, sink.driveCount);
        assertEquals(1, sink.stopCount);
    }

    @Test
    public void taskUpdateReenteredFromSinkUpdateIsSameCycleNoOp() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingDriveSink sink = new RecordingDriveSink();
        Task[] taskHolder = new Task[1];
        sink.onUpdate = () -> taskHolder[0].update(manualClock.clock());
        DriveSignal command = new DriveSignal(0.3, 0.1, -0.2);
        taskHolder[0] = DriveTasks.driveExclusivelyForSeconds(sink, command, 1.0);

        taskHolder[0].start(manualClock.clock());

        assertFalse(taskHolder[0].isComplete());
        assertEquals(Arrays.asList("update", "drive"), sink.events);
        assertEquals(1, sink.updateCount);
        assertEquals(1, sink.driveCount);
        assertSame(command, sink.lastCommand);

        manualClock.nextCycle(0.1);
        taskHolder[0].update(manualClock.clock());

        assertEquals(Arrays.asList("update", "drive", "update", "drive"), sink.events);
        assertEquals(2, sink.updateCount);
        assertEquals(2, sink.driveCount);
    }

    @Test
    public void sinkUpdateFailureMakesTaskCancelledAndRunnerStopsOnce() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingDriveSink sink = new RecordingDriveSink();
        sink.throwOnUpdateNumber = 1;
        Task task = DriveTasks.driveExclusivelyForSeconds(sink, DriveSignal.zero(), 1.0);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(task);

        IllegalStateException failure = expectIllegalState(
                () -> runner.update(manualClock.clock()), "test update failure");

        assertEquals(0, failure.getSuppressed().length);
        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(Arrays.asList("update", "stop"), sink.events);
        assertEquals(1, sink.stopCount);
        assertTrue(runner.isIdle());
    }

    @Test
    public void sinkDriveFailurePreservesOriginalAndSuppressesThrowingStopCleanup() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingDriveSink sink = new RecordingDriveSink();
        sink.throwOnDriveNumber = 1;
        sink.throwOnStopNumber = 1;
        Task task = DriveTasks.driveExclusivelyForSeconds(sink, DriveSignal.zero(), 1.0);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(task);

        IllegalStateException failure = expectIllegalState(
                () -> runner.update(manualClock.clock()), "test drive failure");

        assertEquals(1, failure.getSuppressed().length);
        assertTrue(failure.getSuppressed()[0].getMessage().contains("test stop failure"));
        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(Arrays.asList("update", "drive", "stop"), sink.events);
        assertEquals(1, sink.stopCount);
        assertTrue(runner.isIdle());

        task.cancel();
        assertEquals(1, sink.stopCount);
    }

    @Test
    public void throwingNormalStopLeavesSuccessTerminalAndIsNeverRetried() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingDriveSink sink = new RecordingDriveSink();
        sink.throwOnStopNumber = 1;
        Task task = DriveTasks.driveExclusivelyForSeconds(sink, DriveSignal.zero(), 0.1);
        TaskRunner runner = new TaskRunner();
        runner.enqueue(task);
        runner.update(manualClock.clock());

        manualClock.nextCycle(0.1);
        IllegalStateException failure = expectIllegalState(
                () -> runner.update(manualClock.clock()), "test stop failure");

        assertEquals(0, failure.getSuppressed().length);
        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(Arrays.asList("update", "drive", "stop"), sink.events);
        assertEquals(1, sink.stopCount);
        assertTrue(runner.isIdle());

        task.cancel();
        task.update(manualClock.clock());
        assertEquals(1, sink.stopCount);
    }

    @Test
    public void competingTeleOpFinalWriterWinsAndDemonstratesWhyOwnershipMustBeExclusive() {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingDriveSink sink = new RecordingDriveSink();
        DriveSignal exclusiveAutoCommand = new DriveSignal(0.4, 0.0, 0.0);
        DriveSignal teleOpFinalCommand = new DriveSignal(0.0, -0.3, 0.1);
        Task task = DriveTasks.driveExclusivelyForSeconds(sink, exclusiveAutoCommand, 1.0);

        task.start(manualClock.clock());
        sink.drive(teleOpFinalCommand);
        task.update(manualClock.clock());

        assertSame(teleOpFinalCommand, sink.lastCommand);
        assertEquals(Arrays.asList("update", "drive", "drive"), sink.events);

        manualClock.nextCycle(0.1);
        task.update(manualClock.clock());
        assertSame(exclusiveAutoCommand, sink.lastCommand);
        sink.drive(teleOpFinalCommand);

        assertSame(teleOpFinalCommand, sink.lastCommand);
        assertEquals(
                Arrays.asList("update", "drive", "drive", "update", "drive", "drive"),
                sink.events);
    }

    private static void assertExpiryStopsWithoutAnotherCommand(double durationSec,
                                                                double elapsedSec) {
        ManualLoopClock manualClock = new ManualLoopClock();
        RecordingDriveSink sink = new RecordingDriveSink();
        Task task = DriveTasks.driveExclusivelyForSeconds(
                sink, new DriveSignal(0.2, 0.1, 0.0), durationSec);
        task.start(manualClock.clock());
        sink.events.clear();

        manualClock.nextCycle(elapsedSec);
        task.update(manualClock.clock());

        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(Arrays.asList("stop"), sink.events);
        assertEquals(1, sink.updateCount);
        assertEquals(1, sink.driveCount);
        assertEquals(1, sink.stopCount);

        task.cancel();
        task.update(manualClock.clock());
        assertEquals(Arrays.asList("stop"), sink.events);
    }

    private static void assertRejected(String expectedParameterName, Runnable factoryCall) {
        try {
            factoryCall.run();
            fail("expected invalid " + expectedParameterName + " to be rejected");
        } catch (RuntimeException expected) {
            assertTrue(
                    "expected message to name " + expectedParameterName + ", got: "
                            + expected.getMessage(),
                    expected.getMessage() != null
                            && expected.getMessage().contains(expectedParameterName));
        }
    }

    private static IllegalStateException expectIllegalState(Runnable call,
                                                             String expectedMessageFragment) {
        try {
            call.run();
            fail("expected IllegalStateException containing: " + expectedMessageFragment);
            return null;
        } catch (IllegalStateException expected) {
            assertTrue(
                    "expected message containing " + expectedMessageFragment + ", got: "
                            + expected.getMessage(),
                    expected.getMessage() != null
                            && expected.getMessage().contains(expectedMessageFragment));
            return expected;
        }
    }

    /** Test sink with explicit callback order, failures, and the last applied command. */
    private static final class RecordingDriveSink implements DriveCommandSink {
        private final List<String> events = new ArrayList<>();
        private int updateCount;
        private int driveCount;
        private int stopCount;
        private int throwOnUpdateNumber;
        private int throwOnDriveNumber;
        private int throwOnStopNumber;
        private Runnable onUpdate;
        private DriveSignal lastCommand;

        @Override
        public void update(LoopClock clock) {
            updateCount++;
            events.add("update");
            if (onUpdate != null) {
                onUpdate.run();
            }
            if (updateCount == throwOnUpdateNumber) {
                throw new IllegalStateException("test update failure");
            }
        }

        @Override
        public void drive(DriveSignal signal) {
            driveCount++;
            events.add("drive");
            lastCommand = signal;
            if (driveCount == throwOnDriveNumber) {
                throw new IllegalStateException("test drive failure");
            }
        }

        @Override
        public void stop() {
            stopCount++;
            events.add("stop");
            lastCommand = null;
            if (stopCount == throwOnStopNumber) {
                throw new IllegalStateException("test stop failure");
            }
        }
    }
}
