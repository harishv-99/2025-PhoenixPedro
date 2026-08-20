package edu.ftcphoenix.robots.examples.reference.robot;

import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.robots.examples.reference.capability.launcher.ReferenceLauncherMechanism;
import edu.ftcphoenix.robots.examples.reference.capability.lift.ReferenceLiftMechanism;

/** Data-only configuration for the season-independent reference robot. */
public final class ReferenceProfile {
    public FtcDrives.MecanumConfig drive;
    public ReferenceLiftMechanism.Config lift;
    public ReferenceLauncherMechanism.Config launcher;
    public boolean allowDriveMotion;
    public boolean allowLiftMotion;
    public boolean allowLauncherMotion;

    private ReferenceProfile() {
    }

    /** Returns complete example values whose physical answers and permissions remain unreviewed. */
    public static ReferenceProfile current() {
        ReferenceProfile profile = new ReferenceProfile();
        profile.drive = FtcDrives.MecanumConfig.defaults();
        profile.drive.wiring.frontLeftName = "frontLeftMotor";
        profile.drive.wiring.frontRightName = "frontRightMotor";
        profile.drive.wiring.backLeftName = "backLeftMotor";
        profile.drive.wiring.backRightName = "backRightMotor";
        profile.drive.wiring.frontLeftDirection = Direction.FORWARD;
        profile.drive.wiring.frontRightDirection = Direction.REVERSE;
        profile.drive.wiring.backLeftDirection = Direction.FORWARD;
        profile.drive.wiring.backRightDirection = Direction.REVERSE;
        profile.drive.drivebase.maxAxial = 0.25;
        profile.drive.drivebase.maxLateral = 0.25;
        profile.drive.drivebase.maxOmega = 0.20;
        profile.lift = ReferenceLiftMechanism.Config.defaults();
        profile.launcher = ReferenceLauncherMechanism.Config.defaults();
        profile.allowDriveMotion = false;
        profile.allowLiftMotion = false;
        profile.allowLauncherMotion = false;
        return profile;
    }
}
