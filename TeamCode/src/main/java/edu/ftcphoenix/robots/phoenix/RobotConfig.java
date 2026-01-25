package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.localization.PinpointPoseEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;

/**
 * Centralized robot configuration for the Phoenix example robot.
 *
 * <p>This file is intentionally "boring": it stores hardware map names,
 * directions, and calibration constants in one place so the rest of the robot
 * code can stay clean and robot-centric.</p>
 *
 * <p>Rule of thumb: if you find yourself repeating a device name string or a
 * mount/offset constant in multiple files, it probably belongs here.</p>
 */
public class RobotConfig {

    /**
     * Drivetrain hardware mapping (motor names + directions).
     */
    public static class DriveTrain {
        public static final String nameMotorFrontLeft = "frontLeftMotor";
        public static final Direction directionMotorFrontLeft = Direction.REVERSE;

        public static final String nameMotorFrontRight = "frontRightMotor";
        public static final Direction directionMotorFrontRight = Direction.REVERSE;

        public static final String nameMotorBackLeft = "backLeftMotor";
        public static final Direction directionMotorBackLeft = Direction.FORWARD;

        public static final String nameMotorBackRight = "backRightMotor";
        public static final Direction directionMotorBackRight = Direction.REVERSE;

        /**
         * If true, set drivetrain motors to BRAKE when commanded power is 0.
         *
         * <p>This helps the robot resist small bumps while stopped (useful for aiming/shooting).
         * If you prefer smoother coasting stops, set this to false.</p>
         */
        public static final boolean zeroPowerBrake = true;


        /**
         * Convenience wiring bundle for framework helpers/testers that want to instantiate a mecanum drive.
         */
        public static FtcDrives.MecanumWiringConfig mecanumWiring() {
            FtcDrives.MecanumWiringConfig w = FtcDrives.MecanumWiringConfig.defaults();
            w.frontLeftName = nameMotorFrontLeft;
            w.frontLeftDirection = directionMotorFrontLeft;
            w.frontRightName = nameMotorFrontRight;
            w.frontRightDirection = directionMotorFrontRight;
            w.backLeftName = nameMotorBackLeft;
            w.backLeftDirection = directionMotorBackLeft;
            w.backRightName = nameMotorBackRight;
            w.backRightDirection = directionMotorBackRight;
            return w;

        }
    }

    /**
     * Shooter hardware mapping + basic tuning constants.
     *
     * <p>Units for shooter velocity are the motor's native velocity units
     * (e.g., ticks/sec), because Phoenix plants operate in native units by design.</p>
     */
    public static class Shooter {
        public static final String nameServoPusher = "pusher";
        public static final Direction directionServoPusher = Direction.FORWARD;

        public static final String nameCrServoTransferLeft = "transferLeft";
        public static final Direction directionServoTransferLeft = Direction.REVERSE;

        public static final String nameCrServoTransferRight = "transferRight";
        public static final Direction directionServoTransferRight = Direction.FORWARD;

        public static final String nameMotorShooterLeft = "shooterLeft";
        public static final Direction directionMotorShooterLeft = Direction.FORWARD;

        public static final String nameMotorShooterRight = "shooterRight";
        public static final Direction directionMotorShooterRight = Direction.FORWARD;

        public static final double velocityMin = 1500;
        public static final double velocityMax = 1900;
        public static final double velocityIncrement = 25;

        public static final double targetPusherBack = 0.2;
        public static final double targetPusherFront = 1.0;
    }


    /**
     * Vision-related configuration for the Phoenix robot.
     *
     * <p>These values are consumed by robot code (for example {@code PhoenixRobot}) and by
     * robot-specific testers. Keeping them here avoids hard-coding camera names and mount
     * numbers in many places.</p>
     *
     * <p><b>Camera mount calibration:</b> use the framework tester <i>Calib: Camera Mount</i>
     * to solve {@link #cameraMount}. Paste the printed {@link CameraMountConfig#of(double, double, double, double, double, double)}
     * values here.</p>
     */
    public static class Vision {

        /**
         * FTC Robot Configuration name of the webcam used for AprilTag vision.
         */
        public static final String nameWebcam = "Webcam 1";

        /**
         * Robot→camera extrinsics (Phoenix axes: +X forward, +Y left, +Z up).
         *
         * <p>Defaults to identity (all zeros) so code compiles out-of-the-box. For accurate
         * localization/aiming, calibrate and update this value.</p>
         */
        public static final CameraMountConfig cameraMount = CameraMountConfig.ofDegrees(
                5.095,
                -2.969,
                5.816,
                0.5,
                -18.2,
                2.9);
    }

    /**
     * Localization-related configuration.
     *
     * <p>Phase 1: Pinpoint + AprilTag fusion requires the Pinpoint device name and offsets.
     * Offsets use the Pinpoint convention (matching the goBILDA docs):</p>
     * <ul>
     *   <li><b>forwardPodOffsetLeftInches</b>: how far LEFT of your tracking point the forward (X) pod is (+left, -right)</li>
     *   <li><b>strafePodOffsetForwardInches</b>: how far FORWARD of your tracking point the strafe (Y) pod is (+forward, -back)</li>
     * </ul>
     */
    public static class Localization {

        /**
         * goBILDA Pinpoint configuration.
         *
         * <p>Distances are in inches to match Phoenix's pose types.</p>
         */
        public static PinpointPoseEstimator.Config pinpoint =
                PinpointPoseEstimator.Config.defaults()
                        .withHardwareMapName("odo")
                        .withOffsets(0.0, 0.0)
                        .withForwardPodDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                        .withStrafePodDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Advanced options (uncomment if needed):
        static {
            // pinpoint.withEncoderPods(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            // pinpoint.withCustomEncoderResolutionTicksPerInch(null);
            // pinpoint.withYawScalar(null);
            // pinpoint.withResetOnInit(true);
            // pinpoint.withResetWaitMs(300);
            // pinpoint.withQuality(0.75);
        }
    }

    /**
     * Calibration status and guidance.
     *
     * <p>Many Phoenix testers can optionally use other subsystems to improve results (example:
     * Pinpoint pod offset calibration can use AprilTag assistance). To keep that knowledge
     * out of scattered tester classes, we centralize the "what's calibrated" flags here so
     * menus/testers can choose sensible defaults and display helpful warnings.</p>
     *
     * <p>Some flags are auto-detectable (camera mount != identity), while others need an explicit
     * acknowledgement after you run a sanity-check style tester.</p>
     */
    public static final class Calibration {

        private Calibration() {
        }

        /**
         * True once {@link Vision#cameraMount} has been solved and pasted from the
         * <i>Calib: Camera Mount</i> tester output.
         */
        public static boolean cameraMountCalibrated() {
            return Vision.cameraMount != null && !Vision.cameraMount.equals(CameraMountConfig.identity());
        }

        /**
         * Set to true after running the <i>Robot Calib: Pinpoint Axis Check</i> tester and confirming
         * the forward/strafe directions are correct.
         */
        public static final boolean pinpointAxesVerified = true;

        /**
         * Set to true after you run the <i>Robot Calib: Pinpoint Pod Offsets</i> calibrator and copy
         * the recommended offsets into {@link Localization#pinpoint}.
         */
        public static final boolean pinpointPodOffsetsCalibrated = false;

        /**
         * Returns true if Pinpoint pod offsets look non-default (best-effort heuristic).
         * Prefer {@link #pinpointPodOffsetsCalibrated} for an explicit signal.
         */
        public static boolean pinpointPodOffsetsNonDefault() {
            double fx = Localization.pinpoint.forwardPodOffsetLeftInches;
            double sy = Localization.pinpoint.strafePodOffsetForwardInches;
            return Math.abs(fx) > 1e-6 || Math.abs(sy) > 1e-6;
        }

        /**
         * Convenience: whether it's reasonable to enable AprilTag assistance where supported.
         */
        public static boolean canUseAprilTagAssist() {
            return cameraMountCalibrated();
        }
    }

    /**
     * Auto-aim targeting configuration.
     *
     * <p>Offsets in this section are expressed in the <b>tag frame</b> (inches):</p>
     * <ul>
     *   <li><b>+forward</b> is +X: straight <em>out of the tag</em> (away from the tag face)</li>
     *   <li><b>+left</b> is +Y: left when looking in +forward</li>
     * </ul>
     *
     * <p>That means you can define a point like “the corner of the basket opening” as a
     * tag-relative offset, and Phoenix will automatically rotate it using the tag’s heading.
     * This is exactly what we want for auto-aim from weird angles.</p>
     */
    public static final class AutoAim {

        private AutoAim() {
        }

        // Tag IDs for your scoring targets.
        // (These match the IDs used in PhoenixRobot.SCORING_TAG_IDS today.)
        public static final int BLUE_TARGET_TAG_ID = 20;
        public static final int RED_TARGET_TAG_ID = 24;

        /**
         * Where to aim relative to the BLUE target tag.
         *
         * <p>Set this to the point you want the robot to face (e.g. a basket corner) instead of
         * the AprilTag center. Start with (0,0) and measure/tune from there.</p>
         */
        public static final AimOffset BLUE_AIM_OFFSET = new AimOffset(
                0.0,   // forward (in)
                0.0    // left (in)
        );

        /**
         * Where to aim relative to the RED target tag.
         *
         * <p>If you only ever score on one side, it is totally fine for this to match the blue
         * offset or stay at (0,0).</p>
         */
        public static final AimOffset RED_AIM_OFFSET = new AimOffset(
                0.0,   // forward (in)
                0.0    // left (in)
        );

        /**
         * Default aim offset when we see an unexpected tag ID.
         *
         * <p>(0,0) means “fall back to tag center”.</p>
         */
        public static final AimOffset DEFAULT_AIM_OFFSET = new AimOffset(0.0, 0.0);

        /**
         * Returns the configured tag-relative aim offset for the given tag ID.
         */
        public static AimOffset aimOffsetForTag(int tagId) {
            if (tagId == BLUE_TARGET_TAG_ID) {
                return BLUE_AIM_OFFSET;
            }
            if (tagId == RED_TARGET_TAG_ID) {
                return RED_AIM_OFFSET;
            }
            return DEFAULT_AIM_OFFSET;
        }

        /**
         * Simple (forward,left) pair in inches.
         */
        public static final class AimOffset {
            public final double forwardInches;
            public final double leftInches;

            public AimOffset(double forwardInches, double leftInches) {
                this.forwardInches = forwardInches;
                this.leftInches = leftInches;
            }
        }
    }
}
