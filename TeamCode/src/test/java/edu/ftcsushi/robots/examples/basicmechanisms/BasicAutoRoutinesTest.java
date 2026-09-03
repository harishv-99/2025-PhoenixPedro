package edu.ftcsushi.robots.examples.basicmechanisms;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.task.Tasks;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;

/**
 * Supplied maintainer contract evidence for the optional lift-and-claw Auto policy.
 *
 * <p>Students should learn the golden path from {@link BasicAutoSoftwareScenarioTest} first; this
 * broader suite retains controllable abnormal outcomes, cancellation, and freshness coverage.</p>
 */
public final class BasicAutoRoutinesTest {

    @Test
    public void guideStartsOnlyTheWorkWhosePrerequisitesSucceeded() {
        RecordingLift lift = new RecordingLift();
        RecordingClaw claw = new RecordingClaw();
        Task auto = BasicAutoRoutines.guide(lift, claw);
        ManualLoopClock time = new ManualLoopClock();

        // Construction creates fresh child Tasks but must not publish mechanism requests.
        assertEquals(1, lift.homeRequests);
        assertEquals(Arrays.asList(
                BasicLift.Height.HIGH,
                BasicLift.Height.LOW,
                BasicLift.Height.STOWED), lift.createdMoves);
        assertTrue(lift.startedMoves.isEmpty());
        assertTrue(claw.requests.isEmpty());

        // Home is the first prerequisite. Exact success admits the HIGH move.
        auto.start(time.clock());
        finishAndAdvance(lift.home, TaskOutcome.SUCCESS, auto, time, 0.02);
        assertEquals(Arrays.asList(BasicLift.Height.HIGH), lift.startedMoves);

        // HIGH success starts LOW and the immediate CLOSED request together.
        finishAndAdvance(lift.move(0), TaskOutcome.SUCCESS, auto, time, 0.02);
        assertEquals(Arrays.asList(
                BasicLift.Height.HIGH,
                BasicLift.Height.LOW), lift.startedMoves);
        assertEquals(Arrays.asList(BasicClaw.State.CLOSED), claw.requests);

        // LOW feedback gates the hold; the hold's own boundary gates final stow/open.
        finishAndAdvance(lift.move(1), TaskOutcome.SUCCESS, auto, time, 0.02);
        auto.update(time.nextCycle(0.51));
        assertEquals(Arrays.asList(
                BasicLift.Height.HIGH,
                BasicLift.Height.LOW,
                BasicLift.Height.STOWED), lift.startedMoves);
        assertEquals(Arrays.asList(
                BasicClaw.State.CLOSED,
                BasicClaw.State.OPEN), claw.requests);

        finishAndAdvance(lift.move(2), TaskOutcome.SUCCESS, auto, time, 0.02);
        assertEquals(TaskOutcome.SUCCESS, auto.getOutcome());
    }

    @Test
    public void failedPrerequisitePreservesItsOutcomeAndStartsNothingLater() {
        RecordingLift lift = new RecordingLift();
        RecordingClaw claw = new RecordingClaw();
        Task auto = BasicAutoRoutines.guide(lift, claw);
        ManualLoopClock time = new ManualLoopClock();

        auto.start(time.clock());
        finishAndAdvance(lift.home, TaskOutcome.TIMEOUT, auto, time, 0.02);

        assertEquals(TaskOutcome.TIMEOUT, auto.getOutcome());
        assertTrue(lift.startedMoves.isEmpty());
        assertTrue(claw.requests.isEmpty());
    }

    @Test
    public void activeCancellationCancelsTheCurrentMoveWithoutStartingLaterWork() {
        RecordingLift lift = new RecordingLift();
        RecordingClaw claw = new RecordingClaw();
        Task auto = BasicAutoRoutines.guide(lift, claw);
        ManualLoopClock time = new ManualLoopClock();

        auto.start(time.clock());
        finishAndAdvance(lift.home, TaskOutcome.SUCCESS, auto, time, 0.02);
        auto.cancel();
        auto.cancel();

        assertEquals(TaskOutcome.CANCELLED, auto.getOutcome());
        assertEquals(TaskOutcome.CANCELLED, lift.move(0).getOutcome());
        assertEquals(Arrays.asList(BasicLift.Height.HIGH), lift.startedMoves);
        assertTrue(claw.requests.isEmpty());
    }

    @Test
    public void everyRoutineFactoryCallBuildsFreshSingleUseTasks() {
        RecordingLift lift = new RecordingLift();
        RecordingClaw claw = new RecordingClaw();

        Task first = BasicAutoRoutines.guide(lift, claw);
        ControlledTask firstHome = lift.home;
        Task second = BasicAutoRoutines.guide(lift, claw);

        assertNotSame(first, second);
        assertNotSame(firstHome, lift.home);
        assertEquals(2, lift.homeRequests);
    }

    private static void finishAndAdvance(ControlledTask child,
                                         TaskOutcome outcome,
                                         Task root,
                                         ManualLoopClock time,
                                         double dtSec) {
        child.finish(outcome);
        root.update(time.nextCycle(dtSec));
    }

    private static final class RecordingLift implements BasicLift {
        private final List<Height> createdMoves = new ArrayList<Height>();
        private final List<Height> startedMoves = new ArrayList<Height>();
        private final List<ControlledTask> moves = new ArrayList<ControlledTask>();
        private int homeRequests;
        private ControlledTask home;

        @Override
        public void setHeight(Height height) {
            throw new AssertionError("Auto policy should use feedback-aware moveTo(Height)");
        }

        @Override
        public Task moveTo(Height height) {
            Height required = Objects.requireNonNull(height, "height");
            createdMoves.add(required);
            ControlledTask move = new ControlledTask(
                    "move-" + required,
                    () -> startedMoves.add(required));
            moves.add(move);
            return move;
        }

        @Override
        public Task home() {
            homeRequests++;
            home = new ControlledTask("home", null);
            return home;
        }

        @Override
        public Status status() {
            throw new AssertionError("status is outside this policy test");
        }

        private ControlledTask move(int index) {
            return moves.get(index);
        }
    }

    private static final class RecordingClaw implements BasicClaw {
        private final List<State> requests = new ArrayList<State>();

        @Override
        public void setState(State state) {
            requests.add(state);
        }

        @Override
        public Task setStateTask(State state) {
            State required = Objects.requireNonNull(state, "state");
            return Tasks.runOnce(() -> setState(required));
        }

        @Override
        public Status status() {
            throw new AssertionError("status is outside this policy test");
        }
    }

    /** Test-only terminal control; production examples use framework Task factories. */
    private static final class ControlledTask implements Task {
        private final String name;
        private final Runnable onStart;
        private boolean startAttempted;
        private boolean started;
        private boolean complete;
        private TaskOutcome outcome = TaskOutcome.NOT_DONE;

        private ControlledTask(String name, Runnable onStart) {
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
            if (!started || complete || terminalOutcome == TaskOutcome.NOT_DONE) {
                throw new IllegalStateException(name + " cannot finish with " + terminalOutcome);
            }
            complete = true;
            outcome = terminalOutcome;
        }
    }
}
