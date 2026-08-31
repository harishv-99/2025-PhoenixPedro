package edu.ftcsushi.robots.examples.starter.robot;

import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.robots.examples.starter.capability.intake.StarterIntakeMechanism;

/**
 * Data-only configuration for the modern starter robot.
 *
 * <p>{@link #current()} returns a complete, software-valid example rather than claiming facts
 * about an adopting robot. Review every active hardware answer on the physical robot before
 * setting that capability's {@code allow...Motion} permission to {@code true}.</p>
 */
public final class StarterProfile {

    /** Intake wiring and action powers shared by TeleOp and Auto. */
    public StarterIntakeMechanism.Config intake;

    /** Explicit permission for the reviewed program to move the intake. */
    public boolean allowIntakeMotion;

    /** Complete direct-mecanum configuration used only by TeleOp. */
    public FtcDrives.MecanumConfig drive;

    /** Explicit permission for the reviewed program to move the drivetrain. */
    public boolean allowDriveMotion;

    private StarterProfile() {
        // Use current() so all student-reviewed answers remain together.
    }

    /**
     * Returns a fresh, complete starter profile with conservative software example values.
     *
     * <p>Replace and physically review every name, direction, power sign, brake choice, and drive
     * scale for the adopting robot. Keep a capability's motion permission false until that review
     * is complete; setting it true only enables the subsequent supervised motion and STOP checks
     * and does not prove physical safety.</p>
     */
    public static StarterProfile current() {
        StarterProfile profile = new StarterProfile();

        profile.intake = StarterIntakeMechanism.Config.defaults();
        profile.intake.motorName = "intakeMotor";
        profile.intake.direction = Direction.FORWARD;
        profile.intake.collectPower = 0.20;
        profile.intake.ejectPower = -0.20;
        profile.allowIntakeMotion = false;

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
}
