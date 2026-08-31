package edu.ftcsushi.robots.phoenix.opmode;

import org.junit.Test;

import edu.ftcsushi.robots.phoenix.PhoenixProfile;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Locks Phoenix's ordinary-mode cross-owner FTC motor identity policy. */
public final class PhoenixHardwareOwnershipPreflightTest {

    private static final String[] DRIVE_PATHS = {
            "frontLeftName",
            "frontRightName",
            "backLeftName",
            "backRightName"
    };

    @Test
    public void everyScoringDrivePairRejectsExactAndTrimEquivalentIdentity() {
        for (int scoringIndex = 0; scoringIndex < 2; scoringIndex++) {
            for (int driveIndex = 0; driveIndex < DRIVE_PATHS.length; driveIndex++) {
                assertCollision(scoringIndex, driveIndex, "shared", "shared");
                assertCollision(scoringIndex, driveIndex, "  shared", "shared  ");
            }
        }
    }

    @Test
    public void identityIsCaseSensitiveAndMalformedOwnerFactsKeepOwnerPrecedence() {
        PhoenixProfile profile = PhoenixProfile.current();
        profile.scoring.nameMotorIntake = "Shared";
        profile.drive.wiring.frontLeftName = "shared";
        PhoenixHardwareOwnershipPreflight.requireDistinctMotorOwners(profile);

        profile.scoring.nameMotorIntake = "   ";
        profile.drive.wiring.frontLeftName = "   ";
        PhoenixHardwareOwnershipPreflight.requireDistinctMotorOwners(profile);

        profile.scoring.nameMotorIntake = null;
        profile.drive.wiring.frontLeftName = null;
        PhoenixHardwareOwnershipPreflight.requireDistinctMotorOwners(profile);

        profile.scoring = null;
        PhoenixHardwareOwnershipPreflight.requireDistinctMotorOwners(profile);
        profile = PhoenixProfile.current();
        profile.drive = null;
        PhoenixHardwareOwnershipPreflight.requireDistinctMotorOwners(profile);
        profile = PhoenixProfile.current();
        profile.drive.wiring = null;
        PhoenixHardwareOwnershipPreflight.requireDistinctMotorOwners(profile);
    }

    private static void assertCollision(
            int scoringIndex,
            int driveIndex,
            String scoringName,
            String driveName
    ) {
        PhoenixProfile profile = PhoenixProfile.current();
        if (scoringIndex == 0) {
            profile.scoring.nameMotorIntake = scoringName;
        } else {
            profile.scoring.nameMotorShooterWheel = scoringName;
        }
        setDriveName(profile, driveIndex, driveName);

        try {
            PhoenixHardwareOwnershipPreflight.requireDistinctMotorOwners(profile);
            fail("expected scoring/drive collision");
        } catch (IllegalStateException expected) {
            String scoringField = scoringIndex == 0
                    ? "nameMotorIntake"
                    : "nameMotorShooterWheel";
            assertTrue(expected.getMessage(), expected.getMessage().contains(
                    "PhoenixProfile.scoring." + scoringField
            ));
            assertTrue(expected.getMessage(), expected.getMessage().contains(
                    "PhoenixProfile.drive.wiring." + DRIVE_PATHS[driveIndex]
            ));
            assertTrue(expected.getMessage(), expected.getMessage().contains(
                    "authored as \"" + scoringName + "\""
            ));
            assertTrue(expected.getMessage(), expected.getMessage().contains(
                    "authored as \"" + driveName + "\""
            ));
            assertTrue(expected.getMessage(), expected.getMessage().contains(
                    "FTC hardware key \"shared\""
            ));
        }
    }

    private static void setDriveName(PhoenixProfile profile, int index, String value) {
        switch (index) {
            case 0:
                profile.drive.wiring.frontLeftName = value;
                return;
            case 1:
                profile.drive.wiring.frontRightName = value;
                return;
            case 2:
                profile.drive.wiring.backLeftName = value;
                return;
            case 3:
                profile.drive.wiring.backRightName = value;
                return;
            default:
                throw new AssertionError("Unexpected drive index " + index);
        }
    }
}
