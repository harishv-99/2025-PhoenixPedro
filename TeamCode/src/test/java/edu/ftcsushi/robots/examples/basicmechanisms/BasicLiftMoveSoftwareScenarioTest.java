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
        assertEquals(targetWritesBeforeRequest, scenario.motor.targetPositionWrites());

        // HEARTBEAT: the output owner writes the mapped encoder target and samples old feedback.
        scenario.lift.update(scenario.time.clock());
        int lowTicks = (int) Math.round(
                scenario.config.lowHeightIn * scenario.config.ticksPerIn);
        assertEquals(lowTicks, scenario.motor.targetPositionTicks());
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

    private static final class Scenario {
        private final BasicLiftMechanism.Config config = BasicLiftMechanism.Config.defaults();
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
