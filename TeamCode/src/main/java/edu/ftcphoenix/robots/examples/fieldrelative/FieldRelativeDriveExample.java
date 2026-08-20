package edu.ftcphoenix.robots.examples.fieldrelative;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.FtcRobotOpMode;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.ftc.input.GamepadDevice;
import edu.ftcphoenix.fw.ftc.localization.FtcImuHeadingEstimator;

/** Managed example of explicit station-relative TeleOp drive using the Hub IMU. */
@TeleOp(name = "FW Example: Field-relative drive", group = "FW Examples")
@Disabled
public final class FieldRelativeDriveExample extends FtcRobotOpMode {
    @Override
    protected void configure(RobotProgram program) {
        FieldRelativeExampleProfile profile = FieldRelativeExampleProfile.current();
        if (!profile.allowDriveMotion) {
            throw new IllegalStateException(
                    "FieldRelativeExampleProfile.allowDriveMotion must be true only after reviewing "
                            + "motor wiring, Hub orientation, station headings, low-power motion, and STOP."
            );
        }

        GamepadDevice driver = new GamepadDevice(gamepad1);
        FieldRelativeExamplePrestart prestart = program.prestart(
                new FieldRelativeExamplePrestart(profile.stations, gamepad1)
        );
        FtcImuHeadingEstimator heading = program.service(new FtcImuHeadingEstimator(
                hardwareMap,
                profile.imu,
                prestart::frozenInitialRobotFieldHeadingRad
        ));
        FieldRelativeExampleControls controls = new FieldRelativeExampleControls(
                driver,
                heading,
                prestart,
                profile.manualDrive
        );
        DriveSource drive = controls.drive();
        program.drive(drive, FtcDrives.mecanum(hardwareMap, profile.drive));
        program.presenter((clock, telemetry) -> {
            prestart.present(telemetry);
            telemetry.addLine("Field-relative translation uses the frozen station up direction.");
        });
    }
}
