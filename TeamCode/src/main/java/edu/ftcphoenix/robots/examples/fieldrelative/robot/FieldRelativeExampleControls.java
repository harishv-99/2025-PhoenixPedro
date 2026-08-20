package edu.ftcphoenix.robots.examples.fieldrelative.robot;

import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.ftc.input.GamepadDevice;
import edu.ftcphoenix.fw.localization.HeadingEstimator;

/** Owns the field-relative example's driver meanings. */
final class FieldRelativeExampleControls {
    private final DriveSource drive;

    FieldRelativeExampleControls(GamepadDevice driver,
                                 HeadingEstimator heading,
                                 FieldRelativeExamplePrestart prestart,
                                 GamepadDriveSource.Config config) {
        drive = new GamepadDriveSource(
                driver.leftX(),
                driver.leftY(),
                driver.rightX(),
                config
        ).fieldRelativeTo(heading, prestart::frozenControlUpFieldHeadingRad);
    }

    DriveSource drive() {
        return drive;
    }
}
