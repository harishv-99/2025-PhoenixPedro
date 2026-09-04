package edu.ftcsushi.robots.examples.basicmechanisms;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.task.Tasks;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;

/** Beginner-facing evidence for the first lift-only autonomous sequence. */
public final class BasicAutoSoftwareScenarioTest {
    private static final double STEP_SEC = 1.0;

    @Test
    public void successfulEvidenceAdmitsEachLaterLiftMoveInOrder() {
        // ARRANGE: the real routine uses framework-built recording capability Tasks.
        List<String> events = new ArrayList<String>();
        Task auto = BasicAutoRoutines.liftOnly(new RecordingLift(events, null));
        ManualLoopClock time = new ManualLoopClock();

        // START: only the first prerequisite begins.
        auto.start(time.clock());
        assertEquals(Arrays.asList("home"), events);

        // INJECT EVIDENCE: each successful boundary admits exactly the next move.
        auto.update(time.nextCycle(STEP_SEC));
        assertEquals(Arrays.asList("home", "lift HIGH"), events);
        auto.update(time.nextCycle(STEP_SEC));
        assertEquals(Arrays.asList("home", "lift HIGH", "lift STOWED"), events);
        assertFalse(auto.isComplete());

        // ASSERT: final successful evidence becomes the exact root outcome.
        auto.update(time.nextCycle(STEP_SEC));
        assertTrue(auto.isComplete());
        assertEquals(TaskOutcome.SUCCESS, auto.getOutcome());
        // NEXT GATE: verify reference, feedback, and clearance on the isolated lift.
    }

    @Test
    public void timeoutAndActiveCancellationBothSuppressTheLaterMove() {
        // TIMEOUT: HIGH begins after home, then times out instead of admitting STOWED.
        List<String> timedEvents = new ArrayList<String>();
        RecordingLift timedLift = new RecordingLift(timedEvents, BasicLift.Height.HIGH);
        Task timed = BasicAutoRoutines.liftOnly(timedLift);
        ManualLoopClock timeoutTime = new ManualLoopClock();
        timed.start(timeoutTime.clock());
        timed.update(timeoutTime.nextCycle(STEP_SEC));
        timed.update(timeoutTime.nextCycle(STEP_SEC));
        assertEquals(TaskOutcome.TIMEOUT, timed.getOutcome());
        assertEquals(TaskOutcome.TIMEOUT, timedLift.highTask.getOutcome());
        assertEquals(Arrays.asList("home", "lift HIGH"), timedEvents);

        // CANCEL: active HIGH is terminally cancelled; repeated cancellation remains inert.
        List<String> cancelledEvents = new ArrayList<String>();
        RecordingLift cancelledLift = new RecordingLift(cancelledEvents, null);
        Task cancelled = BasicAutoRoutines.liftOnly(cancelledLift);
        assertNotSame(timed, cancelled);
        ManualLoopClock cancelTime = new ManualLoopClock();
        cancelled.start(cancelTime.clock());
        cancelled.update(cancelTime.nextCycle(STEP_SEC));
        cancelled.cancel();
        cancelled.cancel();
        assertEquals(TaskOutcome.CANCELLED, cancelled.getOutcome());
        assertEquals(TaskOutcome.CANCELLED, cancelledLift.highTask.getOutcome());
        assertEquals(Arrays.asList("home", "lift HIGH"), cancelledEvents);
    }

    private static final class RecordingLift implements BasicLift {
        private final List<String> events;
        private final Height timeoutAt;
        private Task highTask;

        private RecordingLift(List<String> events, Height timeoutAt) {
            this.events = events;
            this.timeoutAt = timeoutAt;
        }

        @Override
        public void setHeight(Height height) {
            throw new AssertionError("Auto must use feedback-aware moveTo(Height)");
        }

        @Override
        public Task moveTo(Height height) {
            Task task = recordedStep("lift " + height, height == timeoutAt);
            if (height == Height.HIGH) {
                highTask = task;
            }
            return task;
        }

        @Override
        public Task home() {
            return recordedStep("home", false);
        }

        @Override
        public Status status() {
            throw new AssertionError("Status is outside this routine-policy question");
        }

        private Task recordedStep(String event, boolean timesOut) {
            Task evidence = timesOut
                    ? Tasks.waitUntil(() -> false, STEP_SEC)
                    : Tasks.waitForSeconds(STEP_SEC);
            return Tasks.sequence(
                    Tasks.runOnce(() -> events.add(event)),
                    evidence);
        }
    }
}
