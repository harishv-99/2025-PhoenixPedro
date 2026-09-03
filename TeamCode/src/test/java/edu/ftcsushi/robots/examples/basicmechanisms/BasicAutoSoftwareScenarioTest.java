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
import static org.junit.Assert.assertTrue;

/**
 * Beginner-facing golden path for the first lift-and-claw autonomous routine.
 *
 * <p>The production routine and Sushi Task composition stay real. Tiny recording capabilities
 * replace hardware and use only framework Task factories to supply successful evidence. This test
 * cannot prove mechanism timing, clearance, wiring, direction, or physical STOP behavior.</p>
 */
public final class BasicAutoSoftwareScenarioTest {
    private static final double STEP_SEC = 1.0;

    @Test
    public void successfulStepsAdmitTheNextWorkInAuthoredOrder() {
        // ARRANGE: every fake feedback step records its start, then succeeds one test second later.
        List<String> events = new ArrayList<String>();
        RecordingLift lift = new RecordingLift(events);
        RecordingClaw claw = new RecordingClaw(events);
        Task auto = BasicAutoRoutines.guide(lift, claw);
        ManualLoopClock time = new ManualLoopClock();

        // REQUEST: starting the real routine admits only its first prerequisite: homing.
        auto.start(time.clock());
        assertEquals(Arrays.asList("home"), events);

        // HEARTBEAT: successful home admits HIGH; successful HIGH admits LOW and CLOSED together.
        auto.update(time.nextCycle(STEP_SEC));
        auto.update(time.nextCycle(STEP_SEC));
        assertEquals(
                Arrays.asList("home", "lift HIGH", "lift LOW", "claw CLOSED"),
                events);

        // HEARTBEAT: LOW succeeds, the authored hold elapses, then STOWED and OPEN start together.
        auto.update(time.nextCycle(STEP_SEC));
        auto.update(time.nextCycle(0.50));
        assertEquals(Arrays.asList(
                "home", "lift HIGH", "lift LOW", "claw CLOSED", "lift STOWED", "claw OPEN"),
                events);
        assertFalse(auto.isComplete());

        // ASSERT: final lift feedback completes the deadline and the exact root outcome is SUCCESS.
        auto.update(time.nextCycle(STEP_SEC));
        assertTrue(auto.isComplete());
        assertEquals(TaskOutcome.SUCCESS, auto.getOutcome());
        // NEXT GATE: prove timing and clearance with both isolated mechanisms before a combined run.
    }

    private static final class RecordingLift implements BasicLift {
        private final List<String> events;

        private RecordingLift(List<String> events) {
            this.events = events;
        }

        @Override
        public void setHeight(Height height) {
            throw new AssertionError("Auto must use feedback-aware moveTo(Height)");
        }

        @Override
        public Task moveTo(Height height) {
            return successfulStep("lift " + height);
        }

        @Override
        public Task home() {
            return successfulStep("home");
        }

        @Override
        public Status status() {
            throw new AssertionError("Status is outside this routine-policy question");
        }

        private Task successfulStep(String event) {
            return Tasks.sequence(
                    Tasks.runOnce(() -> events.add(event)),
                    Tasks.waitForSeconds(STEP_SEC));
        }
    }

    private static final class RecordingClaw implements BasicClaw {
        private final List<String> events;

        private RecordingClaw(List<String> events) {
            this.events = events;
        }

        @Override
        public void setState(State state) {
            events.add("claw " + state);
        }

        @Override
        public Task setStateTask(State state) {
            return Tasks.runOnce(() -> setState(state));
        }

        @Override
        public Status status() {
            throw new AssertionError("Status is outside this routine-policy question");
        }
    }
}
