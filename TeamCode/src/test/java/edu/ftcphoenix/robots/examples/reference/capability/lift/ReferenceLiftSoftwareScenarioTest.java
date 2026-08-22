package edu.ftcphoenix.robots.examples.reference.capability.lift;

import org.junit.Test;

import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.testing.ManualLoopClock;
import edu.ftcphoenix.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Reactive hardware-free scenarios for active-low homing and explicit device evidence. */
public final class ReferenceLiftSoftwareScenarioTest {

    @Test
    public void activeLowHomingReactsToInjectedSwitchAndEncoderEvidence() {
        Scenario scenario = new Scenario();
        scenario.motor.setCurrentPositionTicks(0);
        scenario.bottomSwitch.setHigh(true); // HIGH means the active-low switch is not pressed.

        scenario.currentTask = scenario.lift.home();
        scenario.currentTask.start(scenario.time.clock());
        scenario.currentTask.update(scenario.time.clock());
        scenario.lift.update(scenario.time.clock());

        assertFalse(scenario.currentTask.isComplete());
        assertEquals(scenario.config.homingPower, scenario.motor.power(), 0.0);

        scenario.bottomSwitch.setHigh(false); // LOW is the explicitly injected pressed fact.
        scenario.advance(0.01);

        assertFalse("the debouncer must observe LOW over time",
                scenario.currentTask.isComplete());
        assertEquals(scenario.config.homingPower, scenario.motor.power(), 0.0);

        scenario.advance(0.01);

        assertEquals(TaskOutcome.SUCCESS, scenario.currentTask.getOutcome());
        assertTrue(scenario.lift.status().referenced);

        int lowTargetTicks = (int) Math.round(
                scenario.config.lowHeightIn * scenario.config.ticksPerIn);
        scenario.lift.setHeight(ReferenceLift.Height.LOW);
        scenario.time.nextCycle(0.02);
        scenario.lift.update(scenario.time.clock());

        assertEquals(lowTargetTicks, scenario.motor.targetPositionTicks());

        int injectedTicks = 250;
        scenario.motor.setCurrentPositionTicks(injectedTicks);
        scenario.time.nextCycle(0.02);
        scenario.lift.update(scenario.time.clock());

        assertEquals(
                injectedTicks / scenario.config.ticksPerIn,
                scenario.lift.status().measuredPositionIn,
                0.0);
    }

    @Test
    public void neverPressedSwitchTimesOutAndReleasesHomingOutput() {
        Scenario scenario = new Scenario();
        scenario.motor.setCurrentPositionTicks(0);
        scenario.bottomSwitch.setHigh(true); // The active-low switch remains unpressed.

        scenario.currentTask = scenario.lift.home();
        scenario.currentTask.start(scenario.time.clock());
        scenario.currentTask.update(scenario.time.clock());
        scenario.lift.update(scenario.time.clock());

        assertEquals(scenario.config.homingPower, scenario.motor.power(), 0.0);

        scenario.advance(scenario.config.homingTimeoutSec);

        assertTrue(scenario.bottomSwitch.high());
        assertEquals(TaskOutcome.TIMEOUT, scenario.currentTask.getOutcome());
        assertFalse(scenario.lift.status().referenced);
        assertEquals(ReferenceLift.Height.STOWED,
                scenario.lift.status().requestedHeight);
        assertEquals(scenario.config.stowedHeightIn,
                scenario.lift.status().requestedPositionIn, 0.0);
        assertEquals(0.0, scenario.motor.power(), 0.0);
    }

    private static final class Scenario {
        private final ReferenceLiftMechanism.Config config;
        private final FtcTestHardware.MotorProbe motor;
        private final FtcTestHardware.DigitalProbe bottomSwitch;
        private final ReferenceLiftMechanism lift;
        private final ManualLoopClock time = new ManualLoopClock();
        private Task currentTask;

        private Scenario() {
            config = ReferenceLiftMechanism.Config.defaults();
            FtcTestHardware hardware = new FtcTestHardware();
            motor = hardware.addMotor(config.motorName);
            bottomSwitch = hardware.addDigitalInput(config.bottomSwitchName);
            lift = new ReferenceLiftMechanism(hardware, config);
        }

        private void advance(double dtSec) {
            time.nextCycle(dtSec);
            currentTask.update(time.clock());
            lift.update(time.clock());
        }
    }
}
