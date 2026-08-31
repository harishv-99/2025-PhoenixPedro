package edu.ftcsushi.robots.examples.fieldrelative.robot;

import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.drive.source.GamepadDriveSource;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.localization.HeadingEstimator;

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
