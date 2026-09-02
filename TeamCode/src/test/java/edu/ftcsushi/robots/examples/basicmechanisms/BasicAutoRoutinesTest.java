package edu.ftcsushi.robots.examples.basicmechanisms;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;

/** Verifies that the two teaching Autos compose semantic capabilities without hiding outcomes. */
public final class BasicAutoRoutinesTest {

    @Test
    public void guideRunsTheDocumentedCommandGroupsInOrder() {
        RecordingLift lift = new RecordingLift();
        RecordingClaw claw = new RecordingClaw();
        Task root = BasicAutoRoutines.guide(lift, claw);
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(1, lift.homeRequests);
        assertEquals(
                Arrays.asList(
                        BasicLift.Height.HIGH,
                        BasicLift.Height.LOW,
                        BasicLift.Height.STOWED),
                lift.moveHeights);
        assertTrue(lift.startedHeights.isEmpty());
        root.start(time.clock());

        finishSuccessfullyAndStartNext(lift.lastHome, root, time);
        assertEquals(Arrays.asList(BasicLift.Height.HIGH), lift.startedHeights);
        assertTrue(claw.states.isEmpty());

        finishSuccessfullyAndStartNext(lift.moveTask(0), root, time);
        assertEquals(
                Arrays.asList(BasicLift.Height.HIGH, BasicLift.Height.LOW),
                lift.startedHeights);
        assertEquals(Arrays.asList(BasicClaw.State.CLOSED), claw.states);

        finishSuccessfullyAndStartNext(lift.moveTask(1), root, time);
        root.update(time.nextCycle(0.25));
        assertEquals(2, lift.startedHeights.size());
        assertEquals(1, claw.states.size());

        root.update(time.nextCycle(0.26));
        assertEquals(
                Arrays.asList(
                        BasicLift.Height.HIGH,
                        BasicLift.Height.LOW,
                        BasicLift.Height.STOWED),
                lift.startedHeights);
        assertEquals(
                Arrays.asList(BasicClaw.State.CLOSED, BasicClaw.State.OPEN),
                claw.states);

        finishAndUpdate(lift.moveTask(2), TaskOutcome.SUCCESS, root, time, 0.02);
        assertEquals(TaskOutcome.SUCCESS, root.getOutcome());
    }

    @Test
    public void guidePreservesEveryNonSuccessPrerequisiteWithoutLaterMotion() {
        for (TaskOutcome outcome : Arrays.asList(
                TaskOutcome.TIMEOUT,
                TaskOutcome.CANCELLED,
                TaskOutcome.UNKNOWN)) {
            RecordingLift lift = new RecordingLift();
            RecordingClaw claw = new RecordingClaw();
            Task root = BasicAutoRoutines.guide(lift, claw);
            ManualLoopClock time = new ManualLoopClock();
            root.start(time.clock());

            finishAndUpdate(lift.lastHome, outcome, root, time, 0.02);

            assertEquals(outcome, root.getOutcome());
            assertEquals(
                    Arrays.asList(
                            BasicLift.Height.HIGH,
                            BasicLift.Height.LOW,
                            BasicLift.Height.STOWED),
                    lift.moveHeights);
            assertTrue(lift.startedHeights.isEmpty());
            assertTrue(claw.states.isEmpty());
        }
    }

    @Test
    public void guidePreservesAbnormalHighAndLowDeadlineOutcomes() {
        for (TaskOutcome outcome : Arrays.asList(
                TaskOutcome.TIMEOUT,
                TaskOutcome.CANCELLED,
                TaskOutcome.UNKNOWN)) {
            RecordingLift highLift = new RecordingLift();
            RecordingClaw highClaw = new RecordingClaw();
            Task highRoot = BasicAutoRoutines.guide(highLift, highClaw);
            ManualLoopClock highTime = new ManualLoopClock();
            highRoot.start(highTime.clock());
            finishSuccessfullyAndStartNext(highLift.lastHome, highRoot, highTime);

            finishAndUpdate(highLift.moveTask(0), outcome, highRoot, highTime, 0.02);

            assertEquals(outcome, highRoot.getOutcome());
            assertEquals(Arrays.asList(BasicLift.Height.HIGH), highLift.startedHeights);
            assertTrue(highClaw.states.isEmpty());

            RecordingLift lowLift = new RecordingLift();
            RecordingClaw lowClaw = new RecordingClaw();
            Task lowRoot = BasicAutoRoutines.guide(lowLift, lowClaw);
            ManualLoopClock lowTime = new ManualLoopClock();
            lowRoot.start(lowTime.clock());
            finishSuccessfullyAndStartNext(lowLift.lastHome, lowRoot, lowTime);
            finishSuccessfullyAndStartNext(lowLift.moveTask(0), lowRoot, lowTime);

            finishAndUpdate(lowLift.moveTask(1), outcome, lowRoot, lowTime, 0.02);

            assertEquals(outcome, lowRoot.getOutcome());
            assertEquals(
                    Arrays.asList(BasicLift.Height.HIGH, BasicLift.Height.LOW),
                    lowLift.startedHeights);
            assertEquals(Arrays.asList(BasicClaw.State.CLOSED), lowClaw.states);
        }
    }

    @Test
    public void guidePreservesAbnormalFinalStowDeadlineOutcomes() {
        for (TaskOutcome outcome : Arrays.asList(
                TaskOutcome.TIMEOUT,
                TaskOutcome.CANCELLED,
                TaskOutcome.UNKNOWN)) {
            RecordingLift lift = new RecordingLift();
            RecordingClaw claw = new RecordingClaw();
            Task root = BasicAutoRoutines.guide(lift, claw);
            ManualLoopClock time = new ManualLoopClock();
            root.start(time.clock());
            finishSuccessfullyAndStartNext(lift.lastHome, root, time);
            finishSuccessfullyAndStartNext(lift.moveTask(0), root, time);
            finishSuccessfullyAndStartNext(lift.moveTask(1), root, time);
            root.update(time.nextCycle(0.51));

            assertEquals(BasicLift.Height.STOWED, lift.startedHeights.get(2));
            assertEquals(BasicClaw.State.OPEN, claw.states.get(1));
            finishAndUpdate(lift.moveTask(2), outcome, root, time, 0.02);

            assertEquals(outcome, root.getOutcome());
        }
    }

    @Test
    public void directCancellationStopsTheActiveGuidePrerequisiteWithoutLaterMotion() {

        RecordingLift cancelledLift = new RecordingLift();
        RecordingClaw cancelledClaw = new RecordingClaw();
        Task cancelled = BasicAutoRoutines.guide(cancelledLift, cancelledClaw);
        ManualLoopClock cancelledTime = new ManualLoopClock();
        cancelled.start(cancelledTime.clock());
        finishSuccessfullyAndStartNext(cancelledLift.lastHome, cancelled, cancelledTime);

        cancelled.cancel();
        cancelled.cancel();
        assertEquals(TaskOutcome.CANCELLED, cancelled.getOutcome());
        assertEquals(TaskOutcome.CANCELLED, cancelledLift.moveTask(0).getOutcome());
        assertEquals(Arrays.asList(BasicLift.Height.HIGH), cancelledLift.startedHeights);
        assertTrue(cancelledClaw.states.isEmpty());
    }

    @Test
    public void completeAutoCommandsAndStopsItsExclusiveTimedDriveBeforeRelease() {
        RecordingLift lift = new RecordingLift();
        RecordingClaw claw = new RecordingClaw();
        RecordingDriveSink drive = new RecordingDriveSink();
        Task root = BasicRobotAutoRoutines.complete(lift, claw, drive);
        ManualLoopClock time = new ManualLoopClock();

        root.start(time.clock());
        finishSuccessfullyAndStartNext(lift.lastHome, root, time);
        assertEquals(Arrays.asList(BasicLift.Height.HIGH), lift.startedHeights);
        assertEquals(Arrays.asList(BasicClaw.State.CLOSED), claw.states);

        finishSuccessfullyAndStartNext(lift.moveTask(0), root, time);
        assertEquals(Arrays.asList("update", "drive"), drive.events);
        assertEquals(0.20, drive.lastCommand.axial, 0.0);
        assertEquals(0.0, drive.lastCommand.lateral, 0.0);
        assertEquals(0.0, drive.lastCommand.omega, 0.0);
        assertEquals(0, drive.stopCount);

        root.update(time.nextCycle(0.75));
        assertEquals(1, drive.stopCount);
        assertEquals("stop", drive.events.get(drive.events.size() - 1));
        assertEquals(
                Arrays.asList(BasicLift.Height.HIGH, BasicLift.Height.STOWED),
                lift.startedHeights);
        assertEquals(
                Arrays.asList(BasicClaw.State.CLOSED, BasicClaw.State.OPEN),
                claw.states);

        finishAndUpdate(lift.moveTask(1), TaskOutcome.SUCCESS, root, time, 0.02);
        assertEquals(TaskOutcome.SUCCESS, root.getOutcome());
        assertEquals(1, drive.stopCount);
    }

    @Test
    public void stopOwnerZerosOnceEvenWhenTheDriveTaskNeverStarts() {
        RecordingDriveSink drive = new RecordingDriveSink();
        BasicDriveStopOwner owner = new BasicDriveStopOwner(drive);
        ManualLoopClock time = new ManualLoopClock();

        owner.update(time.clock());
        assertTrue(drive.events.isEmpty());

        owner.stop();
        owner.stop();

        assertEquals(Arrays.asList("stop"), drive.events);
        assertEquals(0, drive.driveCount);
        assertEquals(1, drive.stopCount);
    }

    @Test
    public void carryFailureBeforeDrivePreservesOutcomeAndLifecycleStop() {
        for (TaskOutcome outcome : Arrays.asList(
                TaskOutcome.TIMEOUT,
                TaskOutcome.CANCELLED,
                TaskOutcome.UNKNOWN)) {
            RecordingLift lift = new RecordingLift();
            RecordingClaw claw = new RecordingClaw();
            RecordingDriveSink drive = new RecordingDriveSink();
            BasicDriveStopOwner owner = new BasicDriveStopOwner(drive);
            Task root = BasicRobotAutoRoutines.complete(lift, claw, drive);
            ManualLoopClock time = new ManualLoopClock();
            root.start(time.clock());
            finishSuccessfullyAndStartNext(lift.lastHome, root, time);

            finishAndUpdate(lift.moveTask(0), outcome, root, time, 0.02);

            assertEquals(outcome, root.getOutcome());
            assertEquals(0, drive.driveCount);
            assertEquals(0, drive.stopCount);
            assertEquals(Arrays.asList(BasicClaw.State.CLOSED), claw.states);

            // This is the managed STOP path after Auto ended before its drive phase.
            owner.stop();
            owner.stop();
            assertEquals(1, drive.stopCount);
        }
    }

    @Test
    public void completeAutoPreservesAbnormalFinalStowDeadlineOutcomes() {
        for (TaskOutcome outcome : Arrays.asList(
                TaskOutcome.TIMEOUT,
                TaskOutcome.CANCELLED,
                TaskOutcome.UNKNOWN)) {
            RecordingLift lift = new RecordingLift();
            RecordingClaw claw = new RecordingClaw();
            RecordingDriveSink drive = new RecordingDriveSink();
            Task root = BasicRobotAutoRoutines.complete(lift, claw, drive);
            ManualLoopClock time = new ManualLoopClock();
            root.start(time.clock());
            finishSuccessfullyAndStartNext(lift.lastHome, root, time);
            finishSuccessfullyAndStartNext(lift.moveTask(0), root, time);
            root.update(time.nextCycle(0.75));

            assertEquals(BasicLift.Height.STOWED, lift.startedHeights.get(1));
            assertEquals(BasicClaw.State.OPEN, claw.states.get(1));
            finishAndUpdate(lift.moveTask(1), outcome, root, time, 0.02);

            assertEquals(outcome, root.getOutcome());
            assertEquals(1, drive.stopCount);
        }
    }

    @Test
    public void cancellingCompleteAutoStopsActiveDriveAndStartsNoReleasePhase() {
        RecordingLift lift = new RecordingLift();
        RecordingClaw claw = new RecordingClaw();
        RecordingDriveSink drive = new RecordingDriveSink();
        Task root = BasicRobotAutoRoutines.complete(lift, claw, drive);
        ManualLoopClock time = new ManualLoopClock();

        root.start(time.clock());
        finishSuccessfullyAndStartNext(lift.lastHome, root, time);
        finishSuccessfullyAndStartNext(lift.moveTask(0), root, time);
        assertEquals(1, drive.driveCount);

        root.cancel();
        root.cancel();

        assertEquals(TaskOutcome.CANCELLED, root.getOutcome());
        assertEquals(1, drive.stopCount);
        assertEquals(Arrays.asList(BasicLift.Height.HIGH), lift.startedHeights);
        assertEquals(Arrays.asList(BasicClaw.State.CLOSED), claw.states);
    }

    @Test
    public void eachFactoryCallBuildsFreshRootAndHomeTasks() {
        RecordingLift lift = new RecordingLift();
        RecordingClaw claw = new RecordingClaw();
        RecordingDriveSink drive = new RecordingDriveSink();

        Task first = BasicRobotAutoRoutines.complete(lift, claw, drive);
        ControllableTask firstHome = lift.lastHome;
        Task second = BasicRobotAutoRoutines.complete(lift, claw, drive);

        assertNotSame(first, second);
        assertNotSame(firstHome, lift.lastHome);
        assertEquals(2, lift.homeRequests);
    }

    private static void finishAndUpdate(ControllableTask task,
                                        TaskOutcome outcome,
                                        Task root,
                                        ManualLoopClock time,
                                        double dtSec) {
        task.finish(outcome);
        root.update(time.nextCycle(dtSec));
    }

    /** Exact-success sequence handoff starts the next fixed child in the same lifecycle call. */
    private static void finishSuccessfullyAndStartNext(ControllableTask task,
                                                        Task root,
                                                        ManualLoopClock time) {
        finishAndUpdate(task, TaskOutcome.SUCCESS, root, time, 0.02);
    }

    private static final class RecordingLift implements BasicLift {
        private final List<Height> moveHeights = new ArrayList<Height>();
        private final List<Height> startedHeights = new ArrayList<Height>();
        private final List<ControllableTask> moveTasks = new ArrayList<ControllableTask>();
        private int homeRequests;
        private ControllableTask lastHome;

        @Override
        public void setHeight(Height height) {
            throw new AssertionError("Basic Auto should use moveTo(Height)");
        }

        @Override
        public Task moveTo(Height height) {
            moveHeights.add(height);
            ControllableTask move = new ControllableTask(
                    "move-" + height,
                    () -> startedHeights.add(height));
            moveTasks.add(move);
            return move;
        }

        @Override
        public Task home() {
            homeRequests++;
            lastHome = new ControllableTask("home", null);
            return lastHome;
        }

        @Override
        public Status status() {
            return new Status(Height.STOWED, 0.0, 0.0, true, true);
        }

        private ControllableTask moveTask(int index) {
            return moveTasks.get(index);
        }
    }

    private static final class RecordingClaw implements BasicClaw {
        private final List<State> states = new ArrayList<State>();

        @Override
        public void setState(State state) {
            states.add(state);
        }

        @Override
        public Status status() {
            State state = states.isEmpty() ? State.CLOSED : states.get(states.size() - 1);
            return new Status(state, 0.0);
        }
    }

    private static final class RecordingDriveSink implements DriveCommandSink {
        private final List<String> events = new ArrayList<String>();
        private int driveCount;
        private int stopCount;
        private DriveSignal lastCommand;

        @Override
        public void update(LoopClock clock) {
            events.add("update");
        }

        @Override
        public void drive(DriveSignal signal) {
            driveCount++;
            lastCommand = signal;
            events.add("drive");
        }

        @Override
        public void stop() {
            stopCount++;
            lastCommand = null;
            events.add("stop");
        }
    }

    private static final class ControllableTask implements Task {
        private final String name;
        private final Runnable onStart;
        private boolean startAttempted;
        private boolean started;
        private boolean complete;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;

        private ControllableTask(String name, Runnable onStart) {
            this.name = name;
            this.onStart = onStart;
        }

        @Override
        public void start(LoopClock clock) {
            if (startAttempted) {
                throw new IllegalStateException(name + " is single-use");
            }
            startAttempted = true;
            started = true;
            if (onStart != null) {
                onStart.run();
            }
        }

        @Override
        public void update(LoopClock clock) {
            if (!started) {
                throw new IllegalStateException(name + " updated before start");
            }
        }

        @Override
        public void cancel() {
            if (!started || complete) {
                return;
            }
            complete = true;
            outcome = TaskOutcome.CANCELLED;
        }

        @Override
        public boolean isComplete() {
            return complete;
        }

        @Override
        public TaskOutcome getOutcome() {
            return complete ? outcome : TaskOutcome.NOT_DONE;
        }

        @Override
        public String getDebugName() {
            return name;
        }

        private void finish(TaskOutcome terminalOutcome) {
            if (!started || complete) {
                throw new IllegalStateException(name + " cannot finish now");
            }
            complete = true;
            outcome = terminalOutcome;
        }
    }
}
