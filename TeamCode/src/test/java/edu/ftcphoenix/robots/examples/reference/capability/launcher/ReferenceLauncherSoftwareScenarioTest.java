package edu.ftcphoenix.robots.examples.reference.capability.launcher;

import org.junit.Test;

import edu.ftcphoenix.fw.testing.ManualLoopClock;
import edu.ftcphoenix.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Hardware-free scenario with explicit per-wheel measurements and no simulated flywheel physics. */
public final class ReferenceLauncherSoftwareScenarioTest {

    @Test
    public void readinessUsesTwoInjectedMeasurementsRatherThanTheCommandOrTheirAverage() {
        ReferenceLauncherMechanism.Config config = ReferenceLauncherMechanism.Config.defaults();
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.MotorProbe left = hardware.addMotor(config.leftFlywheelName);
        FtcTestHardware.MotorProbe right = hardware.addMotor(config.rightFlywheelName);
        left.setMeasuredVelocityTicksPerSec(0.0);
        right.setMeasuredVelocityTicksPerSec(0.0);
        hardware.addCrServo(config.transferName);
        hardware.addServo(config.releaseServoName);
        hardware.addDigitalInput(config.objectSensorName);

        ReferenceLauncherMechanism launcher =
                new ReferenceLauncherMechanism(hardware, config);
        ManualLoopClock time = new ManualLoopClock();
        double targetTicksPerSec = 1000.0;

        launcher.setTargetVelocityTicksPerSec(targetTicksPerSec);
        launcher.update(time.clock());
        assertEquals(targetTicksPerSec, left.commandedVelocityTicksPerSec(), 0.0);
        assertEquals(targetTicksPerSec, right.commandedVelocityTicksPerSec(), 0.0);
        assertEquals(0.0, launcher.status().leftMeasuredVelocityTicksPerSec, 0.0);
        assertFalse("a recorded command is not measured feedback", launcher.status().ready);

        left.setMeasuredVelocityTicksPerSec(1125.0);
        right.setMeasuredVelocityTicksPerSec(875.0);
        launcher.update(time.nextCycle(0.02));
        assertFalse("opposite errors must not become ready by averaging", launcher.status().ready);

        left.setMeasuredVelocityTicksPerSec(1100.0);
        right.setMeasuredVelocityTicksPerSec(900.0);
        launcher.update(time.nextCycle(0.02));
        assertTrue(launcher.status().leftAtTarget);
        assertTrue(launcher.status().rightAtTarget);
        assertTrue(launcher.status().ready);
    }
}
