package edu.ftcsushi.robots.examples.firstdrive;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.drive.source.GamepadDriveSource;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;

/** First software proof for the same robot-centric stick mapping used by First Drive. */
public final class FirstDriveSoftwareScenarioTest {

    @Test
    public void stickMeaningsBecomeOneRobotCentricDriveSignal() {
        // ARRANGE: keep Sushi's real input adapters; replace only the driver's physical gamepad.
        Gamepad gamepad = new Gamepad();
        GamepadDevice driver = new GamepadDevice(gamepad);
        GamepadDriveSource.Config config = GamepadDriveSource.Config.defaults();
        config.deadband = 0.0;
        config.translateExpo = 1.0;
        config.rotateExpo = 1.0;
        DriveSource drive = new GamepadDriveSource(
                driver.leftX(), driver.leftY(), driver.rightX(), config);
        ManualLoopClock time = new ManualLoopClock();

        // BEFORE HEARTBEAT: centered sticks produce the one shared zero signal.
        DriveSignal centered = drive.get(time.clock());
        assertEquals(0.0, centered.axial, 0.0);
        assertEquals(0.0, centered.lateral, 0.0);
        assertEquals(0.0, centered.omega, 0.0);

        // REQUEST: author the FTC stick readings that a driver would physically supply.
        gamepad.left_stick_y = -0.50f; // FTC up is negative; GamepadDevice makes up positive.
        gamepad.left_stick_x = 0.25f;  // FTC right becomes Sushi's negative-left command.
        gamepad.right_stick_x = -0.40f; // FTC left becomes Sushi's positive-CCW command.

        // HEARTBEAT + ASSERT: one normal sample exposes the robot-centric command.
        DriveSignal signal = drive.get(time.nextCycle(0.02));
        assertEquals(0.50, signal.axial, 1e-6);
        assertEquals(-0.25, signal.lateral, 1e-6);
        assertEquals(0.40, signal.omega, 1e-6);

        // NEXT GATE: wheels-up hardware testing must verify motor names, directions, and motion.
    }
}
