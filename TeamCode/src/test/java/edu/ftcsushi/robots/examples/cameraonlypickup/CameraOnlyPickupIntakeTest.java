package edu.ftcsushi.robots.examples.cameraonlypickup;

import org.junit.Test;

import edu.ftcsushi.fw.sensing.observation.OccupancyObservation;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.*;

/** Real mechanism/Plant construction; device probes do not simulate intake or sensor physics. */
public final class CameraOnlyPickupIntakeTest {
    @Test public void motorRequestsDoNotCreateOccupancyAndSensorReadsShareOriginalTime() {
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.MotorProbe motor = hardware.addMotor("intakeMotor");
        FtcTestHardware.DigitalProbe input = hardware.addDigitalInput("intakeOccupied");
        CameraOnlyPickupIntake.Config config = CameraOnlyPickupIntake.Config.defaults();
        CameraOnlyPickupIntake intake = new CameraOnlyPickupIntake(hardware, config);
        ManualLoopClock time = new ManualLoopClock();
        input.setHigh(true); // Independent empty observation: not derived from motor power.
        intake.setCollecting(true);
        intake.update(time.clock());
        assertEquals(0.20, motor.power(), 0);
        OccupancyObservation empty = intake.occupancy().get(time.clock());
        assertTrue(empty.available);
        assertFalse(empty.occupied);
        input.setHigh(false);
        assertSame(empty, intake.occupancy().get(time.clock()));
        assertEquals(1, input.stateReadCalls());
        time.nextCycle(0.02);
        OccupancyObservation occupied = intake.occupancy().get(time.clock());
        assertTrue(occupied.occupied);
        assertNotSame(empty.timestamp, occupied.timestamp);
        config.collectPower = 0.9; // Active owner retained its original snapshot.
        intake.setCollecting(true);
        intake.update(time.clock());
        assertEquals(0.20, motor.power(), 0);
        intake.stop();
        assertEquals(0, motor.power(), 0);
        assertFalse(intake.occupancy().get(time.clock()).available);
        intake.setCollecting(true);
        intake.update(time.nextCycle(0.02));
        intake.stop();
        assertEquals(0, motor.power(), 0);
    }

    @Test public void invalidConfigFailsBeforeLookingUpDevices() {
        CameraOnlyPickupIntake.Config config = CameraOnlyPickupIntake.Config.defaults();
        config.collectPower = Double.NaN;
        FtcTestHardware hardware = new FtcTestHardware();
        assertThrows(IllegalArgumentException.class, () -> new CameraOnlyPickupIntake(hardware, config));
        assertEquals(0, hardware.lookupCalls());
        config.collectPower = 0.20;
        config.occupancySwitchName = " ";
        assertThrows(IllegalArgumentException.class, () -> new CameraOnlyPickupIntake(hardware, config));
        assertEquals(0, hardware.lookupCalls());
    }
}
