package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.ftc.FtcDrives;

/** Package-local cross-capability checks used only by composition roots that own both resources. */
final class BasicHardwareOwnership {

    private BasicHardwareOwnership() {
        // Static validation utility.
    }

    /** Rejects a lift motor key that would also be owned by the direct drivetrain. */
    static void requireDistinctDriveAndLiftMotors(FtcDrives.MecanumConfig drive,
                                                  BasicLiftMechanism.Config lift,
                                                  String mode) {
        FtcDrives.MecanumConfig requiredDrive = Objects.requireNonNull(drive, "drive config");
        BasicLiftMechanism.Config requiredLift = Objects.requireNonNull(lift, "lift config");
        if (requiredDrive.wiring == null) {
            return; // FtcDrives reports the missing nested config before hardware lookup.
        }

        String liftName = requiredLift.motorName;
        requireDistinct(
                mode, liftName, "BasicLiftProfile.lift.motorName",
                requiredDrive.wiring.frontLeftName,
                "BasicDriveProfile.drive.wiring.frontLeftName");
        requireDistinct(
                mode, liftName, "BasicLiftProfile.lift.motorName",
                requiredDrive.wiring.frontRightName,
                "BasicDriveProfile.drive.wiring.frontRightName");
        requireDistinct(
                mode, liftName, "BasicLiftProfile.lift.motorName",
                requiredDrive.wiring.backLeftName,
                "BasicDriveProfile.drive.wiring.backLeftName");
        requireDistinct(
                mode, liftName, "BasicLiftProfile.lift.motorName",
                requiredDrive.wiring.backRightName,
                "BasicDriveProfile.drive.wiring.backRightName");
    }

    private static void requireDistinct(String mode,
                                        String firstName,
                                        String firstPath,
                                        String secondName,
                                        String secondPath) {
        if (isBlank(firstName) || isBlank(secondName)) {
            return;
        }
        String firstKey = firstName.trim();
        String secondKey = secondName.trim();
        if (firstKey.equals(secondKey)) {
            throw new IllegalStateException(
                    mode + " motor ownership collision: " + firstPath + " and " + secondPath
                            + " both resolve to FTC hardware key \"" + firstKey
                            + "\". Configure distinct motor names.");
        }
    }

    private static boolean isBlank(String value) {
        return value == null || value.trim().isEmpty();
    }
}
