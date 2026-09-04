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
 * Beginner software proof for establishing one position reference.
 *
 * <p>The test authors every bottom-switch observation. It verifies how the production lift reacts
 * to that cue; it does not simulate motor motion or prove the physical switch, polarity,
 * conversion, limits, or homing safety.</p>
 */
public final class BasicLiftSoftwareScenarioTest {

    @Test
    public void injectedSwitchEvidenceEstablishesReferenceAndSelectsStowedHold() {
        Scenario scenario = new Scenario();

        // ARRANGE: HIGH is the explicitly authored "not pressed" state of this active-low switch.
        scenario.motor.setCurrentPositionTicks(0);
        scenario.bottomSwitch.setHigh(true);
        scenario.lift.setHeight(BasicLift.Height.LOW);
        assertEquals(BasicLift.Height.LOW, scenario.lift.status().requestedHeight());
        assertEquals(0, scenario.motor.targetPositionWrites());

        // REQUEST: start a cooperative home Task; it may command search power but cannot invent a hit.
        scenario.task = scenario.lift.home();
        scenario.task.start(scenario.time.clock());
        scenario.task.update(scenario.time.clock());
        scenario.lift.update(scenario.time.clock());
        assertFalse(scenario.task.isComplete());
        assertEquals(scenario.config.homingPower, scenario.motor.power(), 0.0);

        // INJECT EVIDENCE: LOW must remain observed long enough to pass the configured debouncer.
        scenario.bottomSwitch.setHigh(false);
        scenario.advance(0.01);
        assertFalse(scenario.task.isComplete());
        int targetWritesBeforeSuccess = scenario.motor.targetPositionWrites();
        scenario.advance(0.01);
        assertEquals(TaskOutcome.SUCCESS, scenario.task.getOutcome());
        assertTrue(scenario.lift.status().referenced());
        assertEquals(BasicLift.Height.STOWED, scenario.lift.status().requestedHeight());
        assertEquals(scenario.config.stowedHeightIn,
                scenario.lift.status().requestedPositionIn(), 0.0);
        assertEquals((int) Math.round(
                        scenario.config.stowedHeightIn * scenario.config.ticksPerIn),
                scenario.motor.targetPositionTicks());
        assertEquals(targetWritesBeforeSuccess + 1, scenario.motor.targetPositionWrites());

        // PHYSICAL NEXT GATE: only the robot can prove the switch and homing motion are safe.
    }

    @Test
    public void missingSwitchEvidenceTimesOutAndReleasesSearchPower() {
        Scenario scenario = new Scenario();
        scenario.bottomSwitch.setHigh(true); // The authored switch observation never becomes pressed.
        scenario.lift.setHeight(BasicLift.Height.LOW);
        assertEquals(BasicLift.Height.LOW, scenario.lift.status().requestedHeight());
        scenario.task = scenario.lift.home();
        scenario.task.start(scenario.time.clock());
        scenario.task.update(scenario.time.clock());
        scenario.lift.update(scenario.time.clock());

        scenario.advance(scenario.config.homingTimeoutSec);

        assertEquals(TaskOutcome.TIMEOUT, scenario.task.getOutcome());
        assertFalse(scenario.lift.status().referenced());
        assertEquals(BasicLift.Height.LOW, scenario.lift.status().requestedHeight());
        assertEquals(0.0, scenario.motor.power(), 0.0);
        assertEquals(0, scenario.motor.targetPositionWrites());
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

        /** Advance in RobotProgram's managed Task-before-Output order. */
        private void advance(double dtSec) {
            time.nextCycle(dtSec);
            task.update(time.clock());
            lift.update(time.clock());
        }
    }
}
