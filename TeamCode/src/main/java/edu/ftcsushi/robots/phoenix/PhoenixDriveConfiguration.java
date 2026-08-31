package edu.ftcsushi.robots.phoenix;

import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.ftc.FtcDrives;

/** Checked-in direct-drive construction recipe for the current Phoenix robot. */
final class PhoenixDriveConfiguration {

    private PhoenixDriveConfiguration() {
    }

    /**
     * Returns a fresh direct-drive configuration containing Phoenix's reviewed current answers.
     *
     * <p>These names, directions, and brake behavior are physical claims for Phoenix. A successful
     * software build does not prove the robot is wired or configured safely.</p>
     *
     * @return fresh mutable direct-drive configuration for Phoenix
     */
    static FtcDrives.MecanumConfig current() {
        FtcDrives.MecanumConfig config = FtcDrives.MecanumConfig.defaults();
        config.wiring.frontLeftName = "frontLeftMotor";
        config.wiring.frontLeftDirection = Direction.FORWARD;
        config.wiring.frontRightName = "frontRightMotor";
        config.wiring.frontRightDirection = Direction.FORWARD;
        config.wiring.backLeftName = "backLeftMotor";
        config.wiring.backLeftDirection = Direction.FORWARD;
        config.wiring.backRightName = "backRightMotor";
        config.wiring.backRightDirection = Direction.FORWARD;
        config.enableZeroPowerBrake = true;
        return config;
    }
}
