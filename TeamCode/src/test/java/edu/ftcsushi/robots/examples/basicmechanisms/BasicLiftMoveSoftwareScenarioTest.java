package edu.ftcsushi.robots.examples.basicmechanisms;

import org.junit.Test;

import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/**
 * Beginner software proof for one feedback-aware lift move.
 *
 * <p>The test keeps the production mechanism, semantic request, Task, Plant, and managed loop
 * order. It replaces only the motor and switch, then authors their observations explicitly. It
 * proves how the software reacts to those observations; it does not simulate or prove motion.</p>
 */
public final class BasicLiftMoveSoftwareScenarioTest {

    @Test
    public void freshEncoderEvidenceCompletesTheSelectedSemanticMove() {
        Scenario scenario = new Scenario();
        scenario.motor.setCurrentPositionTicks(0);
        scenario.establishReferenceFromAuthoredSwitchEvidence();

        // REQUEST: starting the fresh Task publishes LOW, but does not write hardware itself.
        int targetWritesBeforeRequest = scenario.motor.targetPositionWrites();
        scenario.task = scenario.lift.moveTo(BasicLift.Height.LOW);
        scenario.time.nextCycle(0.02);
        scenario.task.start(scenario.time.clock());
        scenario.task.update(scenario.time.clock());
        assertEquals(BasicLift.Height.LOW, scenario.lift.status().requestedHeight());
        assertEquals(scenario.config.lowHeightIn,
                scenario.lift.status().requestedPositionIn(), 0.0);
        assertFalse(scenario.lift.status().atTarget());
        assertEquals(targetWritesBeforeRequest, scenario.motor.targetPositionWrites());

        // HEARTBEAT: the output owner writes the mapped encoder target and samples old feedback.
        scenario.lift.update(scenario.time.clock());
        int lowTicks = (int) Math.round(
                scenario.config.lowHeightIn * scenario.config.ticksPerIn);
        assertEquals(lowTicks, scenario.motor.targetPositionTicks());
        assertEquals(scenario.config.lowHeightIn,
                scenario.lift.status().appliedPositionIn(), 0.0);
        assertEquals(0.0, scenario.lift.status().measuredPositionIn(), 0.0);
        assertFalse(scenario.task.isComplete());

        // INJECT EVIDENCE: the fake motor moves only because this test supplies a new observation.
        scenario.motor.setCurrentPositionTicks(lowTicks);
        scenario.advance(0.02);
        assertTrue(scenario.lift.status().atTarget());
        assertFalse("Tasks run before outputs, so completion waits for the next cycle",
                scenario.task.isComplete());

        // NEXT TASK PHASE: the Task now sees the fresh cached arrival and reports exact success.
        scenario.advance(0.02);
        assertEquals(TaskOutcome.SUCCESS, scenario.task.getOutcome());

        // PHYSICAL NEXT GATE: only the robot can prove direction, scale, tuning, and safe travel.
    }

    @Test
    public void activeCancellationLeavesTheSelectedPersistentRequest() {
        Scenario scenario = new Scenario();
        scenario.motor.setCurrentPositionTicks(0);
        scenario.establishReferenceFromAuthoredSwitchEvidence();

        // REQUEST + HEARTBEAT: select HIGH through the Task, then apply it through the Plant owner.
        scenario.task = scenario.lift.moveTo(BasicLift.Height.HIGH);
        scenario.task.start(scenario.time.nextCycle(0.02));
        scenario.task.update(scenario.time.clock());
        scenario.lift.update(scenario.time.clock());
        int highTicks = (int) Math.round(
                scenario.config.highHeightIn * scenario.config.ticksPerIn);
        assertEquals(highTicks, scenario.motor.targetPositionTicks());

        // CANCEL: the Task ends, but leaveRequestOnCancel preserves the selected semantic hold.
        int writesBeforeCancel = scenario.motor.targetPositionWrites();
        scenario.task.cancel();
        assertEquals(TaskOutcome.CANCELLED, scenario.task.getOutcome());
        assertEquals(BasicLift.Height.HIGH, scenario.lift.status().requestedHeight());
        assertEquals(scenario.config.highHeightIn,
                scenario.lift.status().requestedPositionIn(), 0.0);
        assertEquals(writesBeforeCancel, scenario.motor.targetPositionWrites());

        // NEXT OUTPUT: the unchanged request still travels through the normal Plant heartbeat.
        scenario.lift.update(scenario.time.nextCycle(0.02));
        assertEquals(highTicks, scenario.motor.targetPositionTicks());
        assertEquals(writesBeforeCancel + 1, scenario.motor.targetPositionWrites());
        assertFalse(scenario.lift.status().atTarget());
    }

    private static final class Scenario {
        private final BasicLiftMechanism.Config config = BasicLiftProfile.current().lift;
        private final FtcTestHardware hardware = new FtcTestHardware();
        private final FtcTestHardware.MotorProbe motor = hardware.addMotor(config.motorName);
        private final FtcTestHardware.DigitalProbe bottomSwitch =
                hardware.addDigitalInput(config.bottomSwitchName);
        private final BasicLiftMechanism lift = new BasicLiftMechanism(hardware, config);
        private final ManualLoopClock time = new ManualLoopClock();
        private Task task;

        private void establishReferenceFromAuthoredSwitchEvidence() {
            bottomSwitch.setHigh(true);
            task = lift.home();
            task.start(time.clock());
            task.update(time.clock());
            lift.update(time.clock());

            bottomSwitch.setHigh(false);
            advance(0.01);
            advance(0.01);
            assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
            assertTrue(lift.status().referenced());
        }

        /** Advances in RobotProgram's managed Task-before-Output order. */
        private void advance(double dtSec) {
            time.nextCycle(dtSec);
            task.update(time.clock());
            lift.update(time.clock());
        }
    }
}
