package edu.ftcphoenix.robots.examples.fieldrelative.robot;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.ftc.input.GamepadDevice;
import edu.ftcphoenix.fw.ftc.localization.FtcImuHeadingEstimator;

/** Composition root for the managed field-relative drive lesson. */
public final class FieldRelativeExampleRobot {
    private final HardwareMap hardwareMap;

    /** Retains the FTC registry used by the owner-local construction paths. */
    public FieldRelativeExampleRobot(HardwareMap hardwareMap) {
        this.hardwareMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
    }

    /** Declares station selection, heading service, controls, drive, and presentation. */
    public void declareTeleOp(RobotProgram program,
                              FieldRelativeExampleProfile profile,
                              Gamepad gamepad1) {
        Objects.requireNonNull(program, "program");
        FieldRelativeExampleProfile activeProfile = Objects.requireNonNull(profile, "profile");
        Gamepad requiredGamepad = Objects.requireNonNull(gamepad1, "gamepad1");
        if (!activeProfile.allowDriveMotion) {
            throw new IllegalStateException(
                    "FieldRelativeExampleProfile.allowDriveMotion must be true only after reviewing "
                            + "motor wiring, Hub orientation, station headings, low-power motion, and STOP."
            );
        }

        FieldRelativeExamplePrestart prestart = program.prestart(
                new FieldRelativeExamplePrestart(activeProfile.stations, requiredGamepad));
        FtcImuHeadingEstimator heading = program.service(new FtcImuHeadingEstimator(
                hardwareMap,
                activeProfile.imu,
                prestart::frozenInitialRobotFieldHeadingRad));
        FieldRelativeExampleControls controls = new FieldRelativeExampleControls(
                new GamepadDevice(requiredGamepad),
                heading,
                prestart,
                activeProfile.manualDrive);
        DriveSource drive = controls.drive();
        program.drive(drive, FtcDrives.mecanum(hardwareMap, activeProfile.drive));
        program.presenter((clock, telemetry) -> {
            prestart.present(telemetry);
            telemetry.addLine("Field-relative translation uses the frozen station up direction.");
        });
    }
}
