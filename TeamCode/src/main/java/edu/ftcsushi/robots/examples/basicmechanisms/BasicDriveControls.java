package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.drive.source.GamepadDriveSource;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;

/** Owns the stable robot-centric drive source shared by the focused and complete TeleOps. */
final class BasicDriveControls {

    private final DriveSource driveSource;

    BasicDriveControls(GamepadDevice driver) {
        GamepadDevice requiredDriver = Objects.requireNonNull(driver, "driver");
        driveSource = new GamepadDriveSource(
                requiredDriver.leftX(),
                requiredDriver.leftY(),
                requiredDriver.rightX(),
                GamepadDriveSource.Config.defaults());
    }

    /** Returns the stable robot-centric source used by the managed TeleOp drive phase. */
    DriveSource driveSource() {
        return driveSource;
    }
}
