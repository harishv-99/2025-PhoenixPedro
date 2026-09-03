package edu.ftcsushi.robots.examples.firstdrive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.drive.source.GamepadDriveSource;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

@TeleOp(name = "FW First Drive", group = "FW Examples")
@Disabled
public final class FirstDriveTeleOp extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        FirstDriveControls controls = new FirstDriveControls(new GamepadDevice(gamepad1));
        FtcDrives.MecanumConfig drive = FtcDrives.MecanumConfig.defaults();

        drive.wiring.frontLeftName = "frontLeftMotor";
        drive.wiring.frontRightName = "frontRightMotor";
        drive.wiring.backLeftName = "backLeftMotor";
        drive.wiring.backRightName = "backRightMotor";
        drive.wiring.frontLeftDirection = Direction.FORWARD;
        drive.wiring.frontRightDirection = Direction.REVERSE;
        drive.wiring.backLeftDirection = Direction.FORWARD;
        drive.wiring.backRightDirection = Direction.REVERSE;
        drive.enableZeroPowerBrake = true; // Review BRAKE versus FLOAT for this drivetrain.
        drive.drivebase.maxAxial = 0.25;
        drive.drivebase.maxLateral = 0.25;
        drive.drivebase.maxOmega = 0.20;

        program.drive(controls.driveSource(), FtcDrives.mecanum(hardwareMap, drive));
    }

    private static final class FirstDriveControls {
        private final DriveSource driveSource;

        FirstDriveControls(GamepadDevice driver) {
            driveSource = new GamepadDriveSource(
                    driver.leftX(),   // Left/right translation.
                    driver.leftY(),   // Forward/back translation.
                    driver.rightX(),  // Clockwise/counter-clockwise turn.
                    GamepadDriveSource.Config.defaults());
        }

        DriveSource driveSource() {
            return driveSource;
        }
    }
}
