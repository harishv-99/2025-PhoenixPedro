package edu.ftcsushi.robots.examples.firstdrive;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.junit.Test;

import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;
import edu.ftcsushi.fw.testing.ftc.FtcTestTelemetry;

import static org.junit.Assert.assertEquals;

/** Software proof of the maintained First Drive controls, wiring, output caps, and stop path. */
public final class FirstDriveSoftwareScenarioTest {

    @Test
    public void productionControlsPreserveRobotFrameAxisSigns() {
        // ARRANGE: construct the exact production controls from a centered software gamepad.
        Gamepad gamepad = new Gamepad();
        FirstDriveTeleOp.FirstDriveControls controls =
                new FirstDriveTeleOp.FirstDriveControls(new GamepadDevice(gamepad));
        DriveSource drive = controls.driveSource();
        ManualLoopClock time = new ManualLoopClock();

        // ASSERT: the construction-time neutral reading produces no drive request.
        assertSignal(drive.get(time.clock()), 0.0, 0.0, 0.0);

        // REQUEST + ASSERT: FTC stick up becomes positive robot-forward axial intent.
        gamepad.left_stick_y = -1.0f;
        assertSignal(drive.get(time.nextCycle(0.02)), 1.0, 0.0, 0.0);

        // REQUEST + ASSERT: FTC stick left becomes positive robot-left lateral intent.
        gamepad.left_stick_y = 0.0f;
        gamepad.left_stick_x = -1.0f;
        assertSignal(drive.get(time.nextCycle(0.02)), 0.0, 1.0, 0.0);

        // REQUEST + ASSERT: FTC stick left-turn becomes positive counter-clockwise intent.
        gamepad.left_stick_x = 0.0f;
        gamepad.right_stick_x = -1.0f;
        assertSignal(drive.get(time.nextCycle(0.02)), 0.0, 0.0, 1.0);
        // NEXT GATE: motor commands and terminal stop still need the managed-host scenario.
    }

    @Test
    public void managedFirstDriveCapsWheelWritesAndStops() {
        // ARRANGE: keep the complete production config/OpMode; replace FTC devices at the boundary.
        FtcDrives.MecanumConfig config = FirstDriveTeleOp.firstRunDriveConfig();
        FtcTestHardware hardware = hardwareFor(config);
        Gamepad gamepad = new Gamepad();
        FirstDriveTeleOp mode = configuredMode(hardware, gamepad);

        // START: let the managed host own initialization, heartbeat, and cleanup.
        mode.init();
        mode.start();

        // REQUEST + HEARTBEAT + ASSERT: each isolated axis reaches the capped wheel mixer.
        gamepad.left_stick_y = -1.0f;
        mode.loop();
        assertWheelPowers(hardware, config, 0.25, 0.25, 0.25, 0.25);

        gamepad.left_stick_y = 0.0f;
        gamepad.left_stick_x = -1.0f;
        mode.loop();
        assertWheelPowers(hardware, config, -0.25, 0.25, 0.25, -0.25);

        gamepad.left_stick_x = 0.0f;
        gamepad.right_stick_x = -1.0f;
        mode.loop();
        assertWheelPowers(hardware, config, -0.20, 0.20, -0.20, 0.20);

        // STOP + ASSERT: one terminal cleanup writes four zeros and repeated stop remains inert.
        mode.stop();
        assertWheelPowers(hardware, config, 0.0, 0.0, 0.0, 0.0);

        int writesAfterStop = hardware.totalMotorPowerWrites();
        mode.stop();
        mode.loop();
        assertEquals(writesAfterStop, hardware.totalMotorPowerWrites());
        // NEXT GATE: only a supported wheels-up run can prove physical directions and braking.
    }

    private static FtcTestHardware hardwareFor(FtcDrives.MecanumConfig config) {
        FtcTestHardware hardware = new FtcTestHardware();
        hardware.addMotor(config.wiring.frontLeftName);
        hardware.addMotor(config.wiring.frontRightName);
        hardware.addMotor(config.wiring.backLeftName);
        hardware.addMotor(config.wiring.backRightName);
        return hardware;
    }

    private static FirstDriveTeleOp configuredMode(FtcTestHardware hardware, Gamepad gamepad) {
        FirstDriveTeleOp mode = new FirstDriveTeleOp();
        mode.hardwareMap = hardware;
        mode.telemetry = FtcTestTelemetry.silent();
        mode.gamepad1 = gamepad;
        mode.gamepad2 = new Gamepad();
        mode.resetRuntime();
        return mode;
    }

    private static void assertSignal(
            DriveSignal signal, double axial, double lateral, double omega) {
        assertEquals(axial, signal.axial, 1e-9);
        assertEquals(lateral, signal.lateral, 1e-9);
        assertEquals(omega, signal.omega, 1e-9);
    }

    private static void assertWheelPowers(
            FtcTestHardware hardware, FtcDrives.MecanumConfig config,
            double frontLeft, double frontRight, double backLeft, double backRight) {
        assertEquals(frontLeft, hardware.motor(config.wiring.frontLeftName).power(), 1e-9);
        assertEquals(frontRight, hardware.motor(config.wiring.frontRightName).power(), 1e-9);
        assertEquals(backLeft, hardware.motor(config.wiring.backLeftName).power(), 1e-9);
        assertEquals(backRight, hardware.motor(config.wiring.backRightName).power(), 1e-9);
    }

}
