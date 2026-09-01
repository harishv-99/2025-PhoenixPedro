package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.ftc.FtcDrives;

/** Data-only drivetrain answers and the explicit motion permission used by each drive lesson. */
public final class BasicDriveProfile {

    /**
     * Complete direct-mecanum wiring, direction, BRAKE/FLOAT, and normalized mixer-scale
     * configuration. The configured {@code maxAxial}, {@code maxLateral}, and {@code maxOmega}
     * values are dimensionless multipliers in {@code [0, 1]} applied to robot-centric requests.
     */
    public FtcDrives.MecanumConfig drive;

    /**
     * Whether supervised drivetrain motion is permitted after names, directions, scaling, and
     * physical STOP behavior have been reviewed. {@link #current()} always returns {@code false}.
     */
    public boolean allowDriveMotion;

    private BasicDriveProfile() {
        // Start from current() so every required drivetrain answer is populated together.
    }

    /** Returns a fresh conservative software baseline, not reviewed physical robot facts. */
    public static BasicDriveProfile current() {
        BasicDriveProfile profile = new BasicDriveProfile();
        profile.drive = FtcDrives.MecanumConfig.defaults();
        profile.drive.wiring.frontLeftName = "frontLeftMotor";
        profile.drive.wiring.frontRightName = "frontRightMotor";
        profile.drive.wiring.backLeftName = "backLeftMotor";
        profile.drive.wiring.backRightName = "backRightMotor";
        profile.drive.wiring.frontLeftDirection = Direction.FORWARD;
        profile.drive.wiring.frontRightDirection = Direction.REVERSE;
        profile.drive.wiring.backLeftDirection = Direction.FORWARD;
        profile.drive.wiring.backRightDirection = Direction.REVERSE;
        profile.drive.enableZeroPowerBrake = true;
        profile.drive.drivebase.maxAxial = 0.25;
        profile.drive.drivebase.maxLateral = 0.25;
        profile.drive.drivebase.maxOmega = 0.20;
        profile.allowDriveMotion = false;
        return profile;
    }

    /** Rejects unchecked drivetrain motion before a host performs any hardware lookup. */
    static void requireMotionAllowed(BasicDriveProfile profile, String mode) {
        BasicDriveProfile p = Objects.requireNonNull(profile, "driveProfile");
        if (!p.allowDriveMotion) {
            throw new IllegalStateException(
                    "BasicDriveProfile.allowDriveMotion must be true before " + mode
                            + " may construct a motion-capable drivetrain. Review motor names, "
                            + "directions, scales, and small supervised motion first; then verify "
                            + "physical STOP.");
        }
    }
}
