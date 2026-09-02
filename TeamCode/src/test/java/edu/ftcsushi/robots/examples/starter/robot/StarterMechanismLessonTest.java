package edu.ftcsushi.robots.examples.starter.robot;

import org.junit.Test;

import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;
import edu.ftcsushi.robots.examples.starter.capability.intake.StarterIntake;
import edu.ftcsushi.robots.examples.starter.capability.intake.StarterIntakeMechanism;

import static org.junit.Assert.assertEquals;

/** First hardware-free lesson using the production intake mechanism and its real Plant. */
public final class StarterMechanismLessonTest {

    @Test
    public void semanticRequestIsAppliedByTheNextOutputHeartbeat() {
        StarterIntakeMechanism.Config config = StarterIntakeMechanism.Config.defaults();
        config.collectPower = 0.37;
        config.ejectPower = -0.22;

        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.MotorProbe motor = hardware.addMotor(config.motorName);
        StarterIntakeMechanism intake = new StarterIntakeMechanism(hardware, config);
        ManualLoopClock time = new ManualLoopClock();

        intake.setMode(StarterIntake.Mode.COLLECT);
        assertEquals(StarterIntake.Mode.COLLECT, intake.status().mode());
        assertEquals(0.0, intake.status().appliedPower(), 0.0);
        assertEquals(0, motor.powerWrites());

        intake.update(time.clock());
        assertEquals(config.collectPower, intake.status().appliedPower(), 0.0);
        assertEquals(config.collectPower, motor.power(), 0.0);

        intake.setMode(StarterIntake.Mode.EJECT);
        intake.update(time.nextCycle(0.02));
        assertEquals(config.ejectPower, motor.power(), 0.0);

        intake.setMode(StarterIntake.Mode.STOPPED);
        assertEquals(config.ejectPower, intake.status().appliedPower(), 0.0);
        intake.update(time.nextCycle(0.02));
        assertEquals(0.0, intake.status().appliedPower(), 0.0);
        assertEquals(0.0, motor.power(), 0.0);
    }
}
