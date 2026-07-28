package edu.ftcphoenix.robots.examples.starter;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.ftc.FtcDrives;

/**
 * Data-only configuration for the modern starter robot.
 *
 * <p>The checked-in profile is deliberately incomplete. Fill in {@link #current()} with values
 * verified for the adopting robot, then set {@link #hardwareConfigurationReviewed} to {@code true}.
 * The composition root rejects an incomplete profile before looking up hardware.</p>
 */
public final class StarterProfile {

    private StarterProfile() {
        // Use current() so the checked-in safety blockers stay visible in one place.
    }

    /** Records review of every physical value required by the selected mode. */
    public boolean hardwareConfigurationReviewed;

    /** Complete direct-mecanum configuration used by TeleOp. */
    public FtcDrives.MecanumConfig drive = FtcDrives.MecanumConfig.defaults();

    /** Intake hardware and normalized power configuration shared by TeleOp and Auto. */
    public IntakeConfig intake = new IntakeConfig();

    /**
     * Returns the one checked-in starter profile.
     *
     * <p>Edit this method when adopting the example. Blank names, null directions, non-finite
     * powers, and the false review acknowledgement are intentional safety blockers.</p>
     */
    public static StarterProfile current() {
        StarterProfile profile = new StarterProfile();
        profile.drive.wiring.frontLeftName = "";
        profile.drive.wiring.frontRightName = "";
        profile.drive.wiring.backLeftName = "";
        profile.drive.wiring.backRightName = "";
        profile.drive.wiring.frontLeftDirection = null;
        profile.drive.wiring.frontRightDirection = null;
        profile.drive.wiring.backLeftDirection = null;
        profile.drive.wiring.backRightDirection = null;
        profile.intake.motorName = "";
        profile.intake.direction = null;
        profile.intake.collectPower = Double.NaN;
        profile.intake.ejectPower = Double.NaN;
        profile.hardwareConfigurationReviewed = false;
        return profile;
    }

    /** Returns an independent raw snapshot for the robot's mode-specific validation. */
    StarterProfile copy() {
        StarterProfile copy = new StarterProfile();
        copy.hardwareConfigurationReviewed = hardwareConfigurationReviewed;
        copy.drive = copyDrive(drive);
        copy.intake = intake == null ? null : intake.copy();
        return copy;
    }

    /** Adds the drive requirements used only by TeleOp to the shared intake requirements. */
    void requireReadyForTeleOp() {
        List<String> issues = sharedIssues();
        addDriveIssues(issues);
        throwIfIncomplete("TeleOp", issues);
    }

    /** Auto in this starter owns only the shared intake mechanism. */
    void requireReadyForAuto() {
        List<String> issues = sharedIssues();
        throwIfIncomplete("Auto", issues);
    }

    private List<String> sharedIssues() {
        List<String> issues = new ArrayList<>();
        if (!hardwareConfigurationReviewed) {
            issues.add("hardwareConfigurationReviewed must be true after physical review");
        }
        if (intake == null) {
            issues.add("intake config is required");
            return issues;
        }
        addNameIssue(issues, "intake.motorName", intake.motorName);
        if (intake.direction == null) {
            issues.add("intake.direction is required");
        }
        addPowerIssue(issues, "intake.collectPower", intake.collectPower);
        addPowerIssue(issues, "intake.ejectPower", intake.ejectPower);
        if (Double.isFinite(intake.collectPower)
                && Double.isFinite(intake.ejectPower)
                && intake.collectPower == intake.ejectPower) {
            issues.add("intake collectPower and ejectPower must be different");
        }
        return issues;
    }

    private void addDriveIssues(List<String> issues) {
        if (drive == null) {
            issues.add("drive config is required");
            return;
        }
        if (drive.wiring == null) {
            issues.add("drive.wiring is required");
        } else {
            FtcDrives.MecanumWiringConfig wiring = drive.wiring;
            String[] labels = {
                    "drive.frontLeftName",
                    "drive.frontRightName",
                    "drive.backLeftName",
                    "drive.backRightName"
            };
            String[] names = {
                    wiring.frontLeftName,
                    wiring.frontRightName,
                    wiring.backLeftName,
                    wiring.backRightName
            };
            Direction[] directions = {
                    wiring.frontLeftDirection,
                    wiring.frontRightDirection,
                    wiring.backLeftDirection,
                    wiring.backRightDirection
            };

            Set<String> claimedNames = new HashSet<>();
            if (intake != null && !isBlank(intake.motorName)) {
                claimedNames.add(intake.motorName.trim());
            }
            for (int index = 0; index < names.length; index++) {
                addNameIssue(issues, labels[index], names[index]);
                if (!isBlank(names[index]) && !claimedNames.add(names[index].trim())) {
                    issues.add(labels[index] + " duplicates another starter motor name");
                }
                if (directions[index] == null) {
                    issues.add(labels[index].replace("Name", "Direction") + " is required");
                }
            }
        }
        if (drive.drivebase == null) {
            issues.add("drive.drivebase config is required");
        } else {
            addScaleIssue(issues, "drive.drivebase.maxAxial", drive.drivebase.maxAxial);
            addScaleIssue(issues, "drive.drivebase.maxLateral", drive.drivebase.maxLateral);
            addScaleIssue(issues, "drive.drivebase.maxOmega", drive.drivebase.maxOmega);
        }
    }

    private static void addNameIssue(List<String> issues, String label, String name) {
        if (isBlank(name)) {
            issues.add(label + " must be configured");
        }
    }

    private static void addPowerIssue(List<String> issues, String label, double value) {
        if (!Double.isFinite(value) || value < -1.0 || value > 1.0 || value == 0.0) {
            issues.add(label + " must be finite, nonzero, and in [-1.0, 1.0]");
        }
    }

    private static void addScaleIssue(List<String> issues, String label, double value) {
        if (!Double.isFinite(value) || value < 0.0 || value > 1.0) {
            issues.add(label + " must be finite and in [0.0, 1.0]");
        }
    }

    private static boolean isBlank(String value) {
        return value == null || value.trim().isEmpty();
    }

    private static void throwIfIncomplete(String mode, List<String> issues) {
        if (!issues.isEmpty()) {
            throw new IllegalStateException(
                    "StarterProfile is not ready for " + mode + ": "
                            + issues
                            + ". Edit StarterProfile.current() with values verified on your robot.");
        }
    }

    private static FtcDrives.MecanumConfig copyDrive(FtcDrives.MecanumConfig source) {
        if (source == null) {
            return null;
        }
        FtcDrives.MecanumConfig copy = FtcDrives.MecanumConfig.defaults();
        copy.wiring = source.wiring == null ? null : source.wiring.copy();
        copy.enableZeroPowerBrake = source.enableZeroPowerBrake;
        if (source.drivebase == null) {
            copy.drivebase = null;
        } else {
            copy.drivebase = MecanumDrivebase.Config.defaults();
            copy.drivebase.maxAxial = source.drivebase.maxAxial;
            copy.drivebase.maxLateral = source.drivebase.maxLateral;
            copy.drivebase.maxOmega = source.drivebase.maxOmega;
        }
        return copy;
    }

    /** Mutable data for the starter intake; the owning robot keeps a defensive copy. */
    public static final class IntakeConfig {
        private IntakeConfig() {
            // Defaults are the checked-in safety blockers above.
        }

        /** FTC Robot Configuration name for the intake motor. */
        public String motorName = "";

        /** Logical direction that makes the configured collect/eject signs correct. */
        public Direction direction;

        /** Normalized power used while collecting. */
        public double collectPower = Double.NaN;

        /** Normalized power used while ejecting. */
        public double ejectPower = Double.NaN;

        /** Returns an independent copy. */
        IntakeConfig copy() {
            IntakeConfig copy = new IntakeConfig();
            copy.motorName = motorName;
            copy.direction = direction;
            copy.collectPower = collectPower;
            copy.ejectPower = ejectPower;
            return copy;
        }
    }
}
