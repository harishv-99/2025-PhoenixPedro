package edu.ftcsushi.robots.examples.firstdrive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Objects;

import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.drive.source.GamepadDriveSource;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

/**
 * Maintained first-run mecanum lesson from one FTC gamepad to four managed motor outputs.
 *
 * <p>The OpMode stays disabled until its complete motor configuration has been reviewed on the
 * target robot. {@link FtcRobotOpMode} owns the loop heartbeat and managed stop path.</p>
 */
@TeleOp(name = "FW First Drive", group = "FW Examples")
@Disabled
public final class FirstDriveTeleOp extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        FirstDriveControls controls = new FirstDriveControls(
                new GamepadDevice(gamepad1));
        FtcDrives.MecanumConfig drive = firstRunDriveConfig();

        program.drive(controls.driveSource(), FtcDrives.mecanum(hardwareMap, drive));
    }

    /** Returns a fresh, complete mecanum configuration for the cautious first hardware run. */
    static FtcDrives.MecanumConfig firstRunDriveConfig() {
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
        return drive;
    }

    /** Controls-owned mapping from one adapted driver gamepad to one stable drive source. */
    static final class FirstDriveControls {
        private final DriveSource driveSource;

        /** Maps the selected driver axes into Sushi's robot-centric drive convention. */
        FirstDriveControls(GamepadDevice driver) {
            GamepadDevice requiredDriver = Objects.requireNonNull(driver, "driver");
            driveSource = new GamepadDriveSource(
                    requiredDriver.leftX(),
                    requiredDriver.leftY(),
                    requiredDriver.rightX(),
                    GamepadDriveSource.Config.defaults());
        }

        /** Returns the stable drive source sampled by the managed program. */
        DriveSource driveSource() {
            return driveSource;
        }
    }
}
