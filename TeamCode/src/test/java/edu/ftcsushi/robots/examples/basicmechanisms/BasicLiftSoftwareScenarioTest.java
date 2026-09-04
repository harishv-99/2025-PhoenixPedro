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
 * Beginner software proof for a referenced positional mechanism.
 *
 * <p>The test authors every switch and encoder observation. It verifies how the production lift
 * reacts to those facts; it does not simulate motor motion or prove the physical switch, polarity,
 * conversion, limits, or tuning.</p>
 */
public final class BasicLiftSoftwareScenarioTest {

    @Test
    public void injectedSwitchAndEncoderEvidenceDriveHomingAndPositionStatus() {
        Scenario scenario = new Scenario();

        // ARRANGE: HIGH is the explicitly authored "not pressed" state of this active-low switch.
        scenario.motor.setCurrentPositionTicks(0);
        scenario.bottomSwitch.setHigh(true);

        // ARRANGE: begin from a non-STOWED request so success-only post-home policy is observable.
        scenario.lift.setHeight(BasicLift.Height.HIGH);
        assertEquals(BasicLift.Height.HIGH, scenario.lift.status().requestedHeight());

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
        scenario.advance(0.01);
        assertEquals(TaskOutcome.SUCCESS, scenario.task.getOutcome());
        assertTrue(scenario.lift.status().referenced());
        assertEquals(BasicLift.Height.STOWED, scenario.lift.status().requestedHeight());

        // REQUEST + HEARTBEAT: a semantic height becomes the mapped encoder target on output update.
        scenario.lift.setHeight(BasicLift.Height.LOW);
        scenario.time.nextCycle(0.02);
        scenario.lift.update(scenario.time.clock());
        int expectedTargetTicks = (int) Math.round(
                scenario.config.lowHeightIn * scenario.config.ticksPerIn);
        assertEquals(expectedTargetTicks, scenario.motor.targetPositionTicks());

        // INJECT FEEDBACK: the command does not move this probe; the test supplies a measurement.
        int observedTicks = 250;
        scenario.motor.setCurrentPositionTicks(observedTicks);
        scenario.time.nextCycle(0.02);
        scenario.lift.update(scenario.time.clock());
        assertEquals(observedTicks / scenario.config.ticksPerIn,
                scenario.lift.status().measuredPositionIn(), 0.0);

        // PHYSICAL NEXT GATE: only the robot can establish direction, safe limits, and response.
    }

    @Test
    public void missingSwitchEvidenceTimesOutAndReleasesSearchPower() {
        Scenario scenario = new Scenario();
        scenario.bottomSwitch.setHigh(true); // The authored switch observation never becomes pressed.
        scenario.lift.setHeight(BasicLift.Height.HIGH);
        scenario.task = scenario.lift.home();
        scenario.task.start(scenario.time.clock());
        scenario.task.update(scenario.time.clock());
        scenario.lift.update(scenario.time.clock());

        scenario.advance(scenario.config.homingTimeoutSec);

        assertEquals(TaskOutcome.TIMEOUT, scenario.task.getOutcome());
        assertFalse(scenario.lift.status().referenced());
        assertEquals(BasicLift.Height.HIGH, scenario.lift.status().requestedHeight());
        assertEquals(0.0, scenario.motor.power(), 0.0);
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

        /** Advance in RobotProgram's managed Task-before-Output order. */
        private void advance(double dtSec) {
            time.nextCycle(dtSec);
            task.update(time.clock());
            lift.update(time.clock());
        }
    }
}
