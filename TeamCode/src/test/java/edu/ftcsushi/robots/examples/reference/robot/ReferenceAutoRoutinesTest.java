package edu.ftcsushi.robots.examples.reference.robot;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.robots.examples.reference.autonomous.ReferenceAutoRoutines;
import edu.ftcsushi.robots.examples.reference.capability.launcher.ReferenceLauncher;
import edu.ftcsushi.robots.examples.reference.capability.lift.ReferenceLift;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/** Verifies Reference Auto sequencing and timeout-preserving cleanup policy. */
public final class ReferenceAutoRoutinesTest {

    @Test
    public void capabilitiesExposeExactlyTheDeclaredModeNeutralFamilies() {
        RecordingLift lift = new RecordingLift();
        RecordingLauncher launcher = new RecordingLauncher();
        ReferenceCapabilities capabilities = new ReferenceCapabilities(lift, launcher);

        assertSame(lift, capabilities.lift());
        assertSame(launcher, capabilities.launcher());
    }

    @Test
    public void routineWaitsForHomeThenLowThenLaunch() {
        RecordingLift lift = new RecordingLift();
        RecordingLauncher launcher = new RecordingLauncher();
        Task root = ReferenceAutoRoutines.homeMoveLowThenLaunch(
                new ReferenceCapabilities(lift, launcher));
        ManualLoopClock time = new ManualLoopClock();

        assertEquals(1, lift.homeRequests);
        assertEquals(0, lift.moveRequests);
        assertEquals(0, launcher.launchRequests);

        root.start(time.clock());
        assertEquals("home.start", lift.events.get(0));
        lift.lastHome.finish(TaskOutcome.SUCCESS);
        root.update(time.nextCycle(0.02));

        assertEquals(1, lift.moveRequests);
        assertEquals(ReferenceLift.Height.LOW, lift.lastMoveHeight);
        assertEquals(0, launcher.launchRequests);
        lift.lastMove.finish(TaskOutcome.SUCCESS);
        root.update(time.nextCycle(0.02));

        assertEquals(1, launcher.launchRequests);
        assertEquals(0, launcher.abortRequests);
        launcher.lastLaunch.finish(TaskOutcome.SUCCESS);
        root.update(time.nextCycle(0.02));

        assertTrue(root.isComplete());
        assertEquals(TaskOutcome.SUCCESS, root.getOutcome());
        assertEquals(
                list("home.start", "home.SUCCESS", "move-LOW.start", "move-LOW.SUCCESS"),
                lift.events);
        assertEquals(list("launch.start", "launch.SUCCESS"), launcher.events);
    }

    @Test
    public void homeTimeoutAbortsWithoutConstructingMoveOrLaunchAndRemainsTimeout() {
        RecordingLift lift = new RecordingLift();
        RecordingLauncher launcher = new RecordingLauncher();
        Task root = ReferenceAutoRoutines.homeMoveLowThenLaunch(
                new ReferenceCapabilities(lift, launcher));
        ManualLoopClock time = new ManualLoopClock();

        root.start(time.clock());
        lift.lastHome.finish(TaskOutcome.TIMEOUT);
        root.update(time.nextCycle(0.02));

        assertTrue(root.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, root.getOutcome());
        assertEquals(0, lift.moveRequests);
        assertEquals(0, launcher.launchRequests);
        assertEquals(1, launcher.abortRequests);
    }

    @Test
    public void lowMoveTimeoutAbortsWithoutConstructingLaunchAndRemainsTimeout() {
        RecordingLift lift = new RecordingLift();
        RecordingLauncher launcher = new RecordingLauncher();
        Task root = ReferenceAutoRoutines.homeMoveLowThenLaunch(
                new ReferenceCapabilities(lift, launcher));
        ManualLoopClock time = new ManualLoopClock();

        root.start(time.clock());
        lift.lastHome.finish(TaskOutcome.SUCCESS);
        root.update(time.nextCycle(0.02));
        lift.lastMove.finish(TaskOutcome.TIMEOUT);
        root.update(time.nextCycle(0.02));

        assertTrue(root.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, root.getOutcome());
        assertEquals(0, launcher.launchRequests);
        assertEquals(1, launcher.abortRequests);
    }

    @Test
    public void launchTimeoutRunsAbortCleanupAndRemainsTimeout() {
        RecordingLift lift = new RecordingLift();
        RecordingLauncher launcher = new RecordingLauncher();
        Task root = ReferenceAutoRoutines.homeMoveLowThenLaunch(
                new ReferenceCapabilities(lift, launcher));
        ManualLoopClock time = new ManualLoopClock();

        root.start(time.clock());
        lift.lastHome.finish(TaskOutcome.SUCCESS);
        root.update(time.nextCycle(0.02));
        lift.lastMove.finish(TaskOutcome.SUCCESS);
        root.update(time.nextCycle(0.02));
        launcher.lastLaunch.finish(TaskOutcome.TIMEOUT);
        root.update(time.nextCycle(0.02));

        assertTrue(root.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, root.getOutcome());
        assertEquals(1, launcher.abortRequests);
    }

    @Test
    public void directCancellationStaysCancelledAndStartsNoFallback() {
        RecordingLift lift = new RecordingLift();
        RecordingLauncher launcher = new RecordingLauncher();
        Task root = ReferenceAutoRoutines.homeMoveLowThenLaunch(
                new ReferenceCapabilities(lift, launcher));
        ManualLoopClock time = new ManualLoopClock();

        root.cancel();
        assertFalse(root.isComplete());
        root.start(time.clock());
        root.cancel();
        root.cancel();

        assertTrue(root.isComplete());
        assertEquals(TaskOutcome.CANCELLED, root.getOutcome());
        assertEquals(TaskOutcome.CANCELLED, lift.lastHome.getOutcome());
        assertEquals(0, lift.moveRequests);
        assertEquals(0, launcher.launchRequests);
        assertEquals(0, launcher.abortRequests);
    }

    @Test
    public void cancellationDuringMoveOrLaunchDoesNotStartCleanupBranches() {
        RecordingLift movingLift = new RecordingLift();
        RecordingLauncher movingLauncher = new RecordingLauncher();
        Task movingRoot = ReferenceAutoRoutines.homeMoveLowThenLaunch(
                new ReferenceCapabilities(movingLift, movingLauncher));
        ManualLoopClock movingTime = new ManualLoopClock();
        movingRoot.start(movingTime.clock());
        movingLift.lastHome.finish(TaskOutcome.SUCCESS);
        movingRoot.update(movingTime.nextCycle(0.02));

        movingRoot.cancel();
        assertEquals(TaskOutcome.CANCELLED, movingRoot.getOutcome());
        assertEquals(TaskOutcome.CANCELLED, movingLift.lastMove.getOutcome());
        assertEquals(0, movingLauncher.launchRequests);
        assertEquals(0, movingLauncher.abortRequests);

        RecordingLift launchingLift = new RecordingLift();
        RecordingLauncher launchingLauncher = new RecordingLauncher();
        Task launchingRoot = ReferenceAutoRoutines.homeMoveLowThenLaunch(
                new ReferenceCapabilities(launchingLift, launchingLauncher));
        ManualLoopClock launchingTime = new ManualLoopClock();
        launchingRoot.start(launchingTime.clock());
        launchingLift.lastHome.finish(TaskOutcome.SUCCESS);
        launchingRoot.update(launchingTime.nextCycle(0.02));
        launchingLift.lastMove.finish(TaskOutcome.SUCCESS);
        launchingRoot.update(launchingTime.nextCycle(0.02));

        launchingRoot.cancel();
        assertEquals(TaskOutcome.CANCELLED, launchingRoot.getOutcome());
        assertEquals(TaskOutcome.CANCELLED, launchingLauncher.lastLaunch.getOutcome());
        assertEquals(0, launchingLauncher.abortRequests);
    }

    @Test
    public void everyRoutineCallBuildsFreshHomeAndRootTasks() {
        RecordingLift lift = new RecordingLift();
        RecordingLauncher launcher = new RecordingLauncher();
        ReferenceCapabilities capabilities = new ReferenceCapabilities(lift, launcher);

        Task first = ReferenceAutoRoutines.homeMoveLowThenLaunch(capabilities);
        ControllableTask firstHome = lift.lastHome;
        Task second = ReferenceAutoRoutines.homeMoveLowThenLaunch(capabilities);

        assertNotSame(first, second);
        assertNotSame(firstHome, lift.lastHome);
        assertEquals(2, lift.homeRequests);
    }

    private static List<String> list(String... values) {
        List<String> result = new ArrayList<String>();
        for (String value : values) result.add(value);
        return result;
    }

    private static final class RecordingLift implements ReferenceLift {
        private final List<String> events = new ArrayList<String>();
        private int homeRequests;
        private int moveRequests;
        private Height lastMoveHeight;
        private ControllableTask lastHome;
        private ControllableTask lastMove;

        @Override
        public void setHeight(Height height) {
            throw new AssertionError("Reference Auto should use moveTo(Height)");
        }

        @Override
        public Task moveTo(Height height) {
            moveRequests++;
            lastMoveHeight = height;
            lastMove = new ControllableTask("move-" + height, events);
            return lastMove;
        }

        @Override
        public Task home() {
            homeRequests++;
            lastHome = new ControllableTask("home", events);
            return lastHome;
        }

        @Override
        public Status status() {
            return new Status(Height.STOWED, 0.0, 0.0, true, true);
        }
    }

    private static final class RecordingLauncher implements ReferenceLauncher {
        private final List<String> events = new ArrayList<String>();
        private int launchRequests;
        private int abortRequests;
        private ControllableTask lastLaunch;

        @Override
        public void setTargetVelocityTicksPerSec(double velocityTicksPerSec) {
            throw new AssertionError("Reference Auto should use launchOne()");
        }

        @Override
        public void abortLaunches() {
            abortRequests++;
            events.add("abort");
        }

        @Override
        public Task launchOne() {
            launchRequests++;
            lastLaunch = new ControllableTask("launch", events);
            return lastLaunch;
        }

        @Override
        public Status status() {
            return new Status(0.0, 0.0, 0.0,
                    false, false, false, false);
        }
    }

    private static final class ControllableTask implements Task {
        private final String name;
        private final List<String> events;
        private boolean startAttempted;
        private boolean started;
        private boolean complete;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;

        private ControllableTask(String name, List<String> events) {
            this.name = name;
            this.events = events;
        }

        @Override
        public void start(LoopClock clock) {
            if (startAttempted) throw new IllegalStateException(name + " started twice");
            startAttempted = true;
            started = true;
            events.add(name + ".start");
        }

        @Override
        public void update(LoopClock clock) {
            if (!started) throw new IllegalStateException(name + " updated before start");
        }

        @Override
        public void cancel() {
            if (!started || complete) return;
            complete = true;
            outcome = TaskOutcome.CANCELLED;
            events.add(name + ".CANCELLED");
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

        private void finish(TaskOutcome result) {
            if (!started || complete) throw new IllegalStateException(name + " cannot finish");
            complete = true;
            outcome = result;
            events.add(name + "." + result);
        }
    }
}
