package edu.ftcphoenix.robots.examples.reference.capability.lift;

import org.junit.Test;

import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.testing.ManualLoopClock;
import edu.ftcphoenix.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Hardware-free scenario for active-low homing and explicit encoder/output evidence. */
public final class ReferenceLiftSoftwareScenarioTest {

    @Test
    public void activeLowHomingAndInjectedEncoderTicksRemainExplicit() {
        ReferenceLiftMechanism.Config config = ReferenceLiftMechanism.Config.defaults();
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.MotorProbe motor = hardware.addMotor(config.motorName);
        motor.setCurrentPositionTicks(0);
        FtcTestHardware.DigitalProbe bottomSwitch =
                hardware.addDigitalInput(config.bottomSwitchName);
        bottomSwitch.setHigh(true); // HIGH means the active-low switch is not pressed.

        ReferenceLiftMechanism lift = new ReferenceLiftMechanism(hardware, config);
        ManualLoopClock time = new ManualLoopClock();
        Task home = lift.home();
        home.start(time.clock());

        lift.update(time.clock());
        home.update(time.nextCycle(0.01));
        lift.update(time.clock());
        assertFalse(home.isComplete());
        assertEquals(config.homingPower, motor.power(), 0.0);

        bottomSwitch.setHigh(false); // LOW is the explicitly injected pressed fact.
        home.update(time.nextCycle(0.01));
        lift.update(time.clock());
        assertFalse("the debouncer must observe LOW over time", home.isComplete());

        home.update(time.nextCycle(0.03));
        lift.update(time.clock());
        assertTrue(home.isComplete());
        assertEquals(TaskOutcome.SUCCESS, home.getOutcome());
        assertTrue(lift.status().referenced);

        int lowTargetTicks = (int) Math.round(config.lowHeightIn * config.ticksPerIn);
        lift.setHeight(ReferenceLift.Height.LOW);
        lift.update(time.nextCycle(0.02));
        assertEquals(lowTargetTicks, motor.targetPositionTicks());

        int injectedTicks = 250;
        motor.setCurrentPositionTicks(injectedTicks);
        lift.update(time.nextCycle(0.02));
        assertEquals(
                injectedTicks / config.ticksPerIn,
                lift.status().measuredPositionIn,
                0.0);
    }
}
