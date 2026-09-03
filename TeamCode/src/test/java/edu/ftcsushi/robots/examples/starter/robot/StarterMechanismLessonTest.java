package edu.ftcsushi.robots.examples.starter.robot;

import org.junit.Test;

import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;
import edu.ftcsushi.robots.examples.starter.capability.intake.StarterIntake;
import edu.ftcsushi.robots.examples.starter.capability.intake.StarterIntakeMechanism;

import static org.junit.Assert.assertEquals;

/**
 * First hardware-free lesson using the production intake mechanism and its real Plant.
 *
 * <p>The testing idea is causal: issue one semantic request, advance the one normal output
 * heartbeat, and then observe both Sushi status and the fake hardware boundary. A passing test
 * proves software mapping and lifecycle order; it cannot prove motor direction, mechanical
 * safety, or that the real mechanism moved.</p>
 */
public final class StarterMechanismLessonTest {

    @Test
    public void semanticRequestIsAppliedByTheNextOutputHeartbeat() {
        // ARRANGE: production configuration and mechanism with only the FTC motor replaced.
        StarterIntakeMechanism.Config config = StarterIntakeMechanism.Config.defaults();
        config.collectPower = 0.37;
        config.ejectPower = -0.22;

        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.MotorProbe motor = hardware.addMotor(config.motorName);
        StarterIntakeMechanism intake = new StarterIntakeMechanism(hardware, config);
        ManualLoopClock time = new ManualLoopClock();

        // REQUEST: semantic intent changes immediately; hardware has not been updated yet.
        intake.setMode(StarterIntake.Mode.COLLECT);
        assertEquals(StarterIntake.Mode.COLLECT, intake.status().mode());
        assertEquals(0.0, intake.status().appliedPower(), 0.0);
        assertEquals(0, motor.powerWrites());

        // HEARTBEAT: the same production update path maps the request and writes the motor.
        intake.update(time.clock());
        assertEquals(config.collectPower, intake.status().appliedPower(), 0.0);
        assertEquals(config.collectPower, motor.power(), 0.0);

        // REPEAT: another semantic request is realized on the next cycle, not by hidden timing.
        intake.setMode(StarterIntake.Mode.EJECT);
        intake.update(time.nextCycle(0.02));
        assertEquals(config.ejectPower, motor.power(), 0.0);

        // SAFE REQUEST: STOPPED is also staged until the next normal output heartbeat.
        intake.setMode(StarterIntake.Mode.STOPPED);
        assertEquals(config.ejectPower, intake.status().appliedPower(), 0.0);
        intake.update(time.nextCycle(0.02));
        assertEquals(0.0, intake.status().appliedPower(), 0.0);
        assertEquals(0.0, motor.power(), 0.0);

        // PHYSICAL NEXT GATE: verify direction and safe motion on blocks before enabling a robot.
    }
}
