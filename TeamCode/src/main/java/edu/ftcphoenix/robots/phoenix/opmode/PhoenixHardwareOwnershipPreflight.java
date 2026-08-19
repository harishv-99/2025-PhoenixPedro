package edu.ftcphoenix.robots.phoenix.opmode;

import java.util.Objects;

import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;
import edu.ftcphoenix.robots.phoenix.scoring.PhoenixScoring;

/** Owns ordinary Phoenix modes' cross-owner DC-motor exclusivity policy. */
final class PhoenixHardwareOwnershipPreflight {

    private PhoenixHardwareOwnershipPreflight() {
        // Package policy utility.
    }

    /** Reject scoring-versus-drive motor aliases before either ordinary mode crosses into hardware. */
    static void requireDistinctMotorOwners(PhoenixProfile profile) {
        PhoenixProfile source = Objects.requireNonNull(profile, "profile");
        PhoenixScoring.Config scoring = source.scoring;
        FtcDrives.MecanumWiringConfig wiring = source.drive == null
                ? null
                : source.drive.wiring;
        if (scoring == null || wiring == null) {
            return;
        }

        requireDistinctDriveNames(
                scoring.nameMotorIntake,
                "PhoenixProfile.scoring.nameMotorIntake",
                wiring
        );
        requireDistinctDriveNames(
                scoring.nameMotorShooterWheel,
                "PhoenixProfile.scoring.nameMotorShooterWheel",
                wiring
        );
    }

    private static void requireDistinctDriveNames(
            String scoringName,
            String scoringPath,
            FtcDrives.MecanumWiringConfig wiring
    ) {
        requireDistinct(
                scoringName,
                scoringPath,
                wiring.frontLeftName,
                "PhoenixProfile.drive.wiring.frontLeftName"
        );
        requireDistinct(
                scoringName,
                scoringPath,
                wiring.frontRightName,
                "PhoenixProfile.drive.wiring.frontRightName"
        );
        requireDistinct(
                scoringName,
                scoringPath,
                wiring.backLeftName,
                "PhoenixProfile.drive.wiring.backLeftName"
        );
        requireDistinct(
                scoringName,
                scoringPath,
                wiring.backRightName,
                "PhoenixProfile.drive.wiring.backRightName"
        );
    }

    private static void requireDistinct(
            String scoringName,
            String scoringPath,
            String driveName,
            String drivePath
    ) {
        if (isBlank(scoringName) || isBlank(driveName)) {
            return;
        }
        String scoringKey = scoringName.trim();
        String driveKey = driveName.trim();
        if (!scoringKey.equals(driveKey)) {
            return;
        }
        throw new IllegalStateException(
                "Phoenix motor ownership collision: " + scoringPath + " authored as \""
                        + scoringName + "\" and " + drivePath + " authored as \"" + driveName
                        + "\" both resolve to FTC hardware key \"" + scoringKey
                        + "\". Configure distinct motor names."
        );
    }

    private static boolean isBlank(String value) {
        return value == null || value.trim().isEmpty();
    }
}
