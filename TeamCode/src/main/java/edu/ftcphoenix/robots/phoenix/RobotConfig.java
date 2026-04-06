package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.FtcFieldRegions;
import edu.ftcphoenix.fw.ftc.localization.PinpointPoseEstimator;
import edu.ftcphoenix.fw.localization.apriltag.TagOnlyPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.OdometryTagEkfPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.OdometryTagFusionPoseEstimator;
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
        public static final Direction directionMotorFrontLeft = Direction.FORWARD;

        public static final String nameMotorFrontRight = "frontRightMotor";
        public static final Direction directionMotorFrontRight = Direction.FORWARD;

        public static final String nameMotorBackLeft = "backLeftMotor";
        public static final Direction directionMotorBackLeft = Direction.FORWARD;

        public static final String nameMotorBackRight = "backRightMotor";
        public static final Direction directionMotorBackRight = Direction.FORWARD;

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

        // ------------------------------------------------------------------
        // Hardware mapping (match your FTC Robot Configuration)
        // ------------------------------------------------------------------

        /**
         * Intake wheels motor (pulls balls into the robot).
         */
        public static final String nameMotorIntake = "intakeMotor";
        public static final Direction directionMotorIntake = Direction.FORWARD;

        /**
         * Intake-to-storage transfer (continuous rotation servo).
         */
        public static final String nameCrServoIntakeTransfer = "intakeTransfer";
        public static final Direction directionCrServoIntakeTransfer = Direction.REVERSE;

        /**
         * Shooter transfer (two continuous rotation servos).
         */
        public static final String nameCrServoShooterTransferLeft = "shooterTransferLeft";
        public static final Direction directionCrServoShooterTransferLeft = Direction.REVERSE;

        public static final String nameCrServoShooterTransferRight = "shooterTransferRight";
        public static final Direction directionCrServoShooterTransferRight = Direction.FORWARD;

        // ------------------------------------------------------------------
        // Servo mismatch hack
        // ------------------------------------------------------------------

        /**
         * Scale applied to the LEFT shooter-transfer CR servo.
         *
         * <p>We currently have mismatched servo models: one "speed" servo and one "super speed" servo.
         * The super speed servo should usually get <b>less</b> power so both sides move balls at a similar speed.
         *
         * <p>This is implemented using {@link edu.ftcphoenix.fw.actuation.Actuators} multi-CR-servo scaling.
         */
        public static final double shooterTransferLeftScale = 0.65;

        /**
         * Optional additive bias for the LEFT shooter-transfer servo (usually 0).
         */
        public static final double shooterTransferLeftBias = 0.0;

        // ------------------------------------------------------------------
        // Shooter wheel (single motor, velocity control)
        // ------------------------------------------------------------------

        public static final String nameMotorShooterWheel = "shooterMotor";
        public static final Direction directionMotorShooterWheel = Direction.FORWARD;

        public static final double velocityMin = 700;
        public static final double velocityMax = 2000;
        public static final double velocityIncrement = 25;

        /** Tolerance for {@code Plant.atSetpoint()} in native velocity units. */
        public static final double velocityToleranceNative = 50;

        // ------------------------------------------------------------------
        // Optional: tune the motor controller's built-in velocity PIDF
        // ------------------------------------------------------------------

        /**
         * If true, apply the PIDF coefficients below to the shooter wheel motor.
         *
         * <p>Why this exists:
         * <ul>
         *   <li>The rapid-fire ready gate keeps timing consistent.</li>
         *   <li>Better PIDF tuning reduces dip/overshoot so you can shoot faster.</li>
         * </ul>
         *
         * <p>Default is false so we do not accidentally overwrite a working tune.
         */
        public static final boolean applyFlywheelVelocityPIDF = false;

        /** Velocity loop P gain (native motor controller). */
        public static final double flywheelVelKp = 0.0;
        /** Velocity loop I gain (native motor controller). */
        public static final double flywheelVelKi = 0.0;
        /** Velocity loop D gain (native motor controller). */
        public static final double flywheelVelKd = 0.0;
        /** Velocity loop F gain / feedforward (native motor controller). */
        public static final double flywheelVelKf = 0.0;

        /**
         * Shooter-ready tolerance BELOW the target speed.
         *
         * <p>This is the "underspeed" allowance. Leaving this a bit larger helps avoid getting stuck
         * when the wheel is recovering from a shot.</p>
         */
        public static final double velocityToleranceBelowNative = 50;

        /**
         * Shooter-ready tolerance ABOVE the target speed.
         *
         * <p>This is the "overspeed" allowance. Keeping this smaller than
         * {@link #velocityToleranceBelowNative} helps avoid feeding a ball while the wheel is still
         * overshooting upward (which can make shots go long).</p>
         */
        public static final double velocityToleranceAboveNative = 50;

        /**
         * Feed-to-flywheel contact lead time (seconds).
         *
         * <p>When you start the feed motors/servos, the ball does <b>not</b> instantly touch the
         * flywheel. There is a short mechanical delay as the ball travels through the transfer
         * path and reaches the wheel.
         *
         * <p>We use this value to decide whether the flywheel will be at the right speed
         * <em>when the ball contacts</em>, not necessarily at the exact instant the feed command
         * turns on. This is the key to consistent rapid-fire:
         * <ul>
         *   <li>If the 2nd ball tends to <b>overshoot</b>, this value is usually too small.
         *       Increase it to start feeding a little earlier.</li>
         *   <li>If the 2nd ball tends to <b>undershoot</b>, this value is usually too large.
         *       Decrease it to start feeding a little later.</li>
         * </ul>
         */
        public static final double readyPredictLeadSec = 0.10;

        /**
         * Debounce time for the shooter "ready" latch.
         *
         * <p>Smaller = faster bursts, but more sensitive to sensor noise.</p>
         */
        // Note: with prediction, the "ready" window can be brief while the wheel is
        // accelerating quickly. A short debounce (1–3 loops) catches that window.
        public static final double readyStableSec = 0.03;

        // ------------------------------------------------------------------
        // TeleOp behavior tuning
        // ------------------------------------------------------------------

        /**
         * Base power while intaking.
         */
        public static final double intakeMotorPower = 1.0;
        public static final double intakeTransferPower = 1.0;

        /**
         * While intaking, spin the shooter transfer <b>backwards</b> to keep balls from reaching the wheel.
         */
        public static final double intakeShooterTransferHoldBackPower = 0.5;

        /**
         * Base power while ejecting/unjamming (reverse direction).
         */
        public static final double ejectMotorPower = 1.0;
        public static final double ejectTransferPower = 1.0;
        public static final double ejectShooterTransferPower = 1.0;

        // ------------------------------------------------------------------
        // Shooting (one-ball pulses)
        // ------------------------------------------------------------------

        /** Output from the feed queue while a "shoot one" pulse is active. */
        public static final double shootFeedPower = 1.0;

        /** Time to run the feed path to advance approximately one ball. */
        public static final double shootFeedPulseSec = 0.22;

        /**
         * Optional pause between shots (helps flywheel recover for accuracy).
         */
        public static final double shootFeedCooldownSec = 0.06;

        /**
         * Keep the flywheel spinning briefly after the last shot request for faster follow-ups.
         */
        public static final double flywheelKeepAliveSec = 0.75;

        // ------------------------------------------------------------------
        // Feed distribution (macro output scaling per stage)
        // ------------------------------------------------------------------

        /**
         * Scale applied to feed-queue output for the intake motor during shooting pulses.
         */
        public static final double feedScaleIntakeMotor = 1.0;

        /**
         * Scale applied to feed-queue output for the intake transfer servo during shooting pulses.
         */
        public static final double feedScaleIntakeTransfer = 1.0;

        /** Scale applied to feed-queue output for the shooter transfer during shooting pulses. */
        public static final double feedScaleShooterTransfer = 1.0;
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
                9.97,
                -1.80,
                13.68,
                1.9,
                -18.2,
                -1.7);
    }

    /**
     * Localization-related configuration.
     *
     * <p>Phoenix supports both the default lightweight Pinpoint + AprilTag fusion path and an
     * optional EKF-style estimator. Both require the Pinpoint device name and offsets.
     * Offsets use the Pinpoint convention (matching the goBILDA docs):</p>
     * <ul>
     *   <li><b>forwardPodOffsetLeftInches</b>: how far LEFT of your tracking point the forward (X) pod is (+left, -right)</li>
     *   <li><b>strafePodOffsetForwardInches</b>: how far FORWARD of your tracking point the strafe (Y) pod is (+forward, -back)</li>
     * </ul>
     */
    public static class Localization {

        /** Selects which optional odometry+AprilTag global localizer Phoenix should instantiate. */
        public enum GlobalEstimatorMode {
            /** Simpler complementary fusion with latency-compensated replay. Default for Phoenix. */
            FUSION,
            /** Optional EKF-style estimator with covariance-aware vision updates. */
            EKF
        }

        /**
         * Phoenix currently keeps the lightweight fusion estimator as the default because it is
         * easier to tune and debug. Switch this to {@link GlobalEstimatorMode#EKF} only after the
         * camera mount, Pinpoint directions, and pod offsets are calibrated and field-validated.
         */
        public static GlobalEstimatorMode globalEstimatorMode = GlobalEstimatorMode.FUSION;

        /**
         * goBILDA Pinpoint configuration.
         *
         * <p>Distances are in inches to match Phoenix's pose types.</p>
         */
        public static PinpointPoseEstimator.Config pinpoint =
                PinpointPoseEstimator.Config.defaults()
                        .withHardwareMapName("pinPoint")
                        .withOffsets(0.0, 0.0)
                        .withForwardPodDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                        .withStrafePodDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /**
         * AprilTag-only global pose solve used as the vision lane for localization fusion.
         *
         * <p>The camera mount itself still lives in {@link Vision#cameraMount}; robot code should
         * copy this config and attach the calibrated mount at the call site. The Phoenix AprilTag
         * localization testers also consume this config so practice tools match production tuning.</p>
         */
        public static TagOnlyPoseEstimator.Config aprilTags =
                TagOnlyPoseEstimator.Config.defaults();

        /**
         * Lightweight complementary fusion tuning for Pinpoint odometry corrected by AprilTag
         * global pose measurements. This remains Phoenix's default global estimator.
         */
        public static OdometryTagFusionPoseEstimator.Config pinpointAprilTagFusion =
                OdometryTagFusionPoseEstimator.Config.defaults();

        /**
         * Optional EKF-style global estimator tuned for the same Pinpoint+AprilTag pipeline.
         *
         * <p><b>Calibration requirements matter more here than with the simpler fusion localizer.</b>
         * Before enabling {@link GlobalEstimatorMode#EKF}, make sure all of the following are true:</p>
         * <ul>
         *   <li>{@link Vision#cameraMount} has been solved and pasted from the camera-mount calibrator.</li>
         *   <li>{@link #pinpoint} uses the correct Pinpoint axis directions.</li>
         *   <li>{@link #pinpoint} pod offsets have been measured instead of left at default 0/0.</li>
         *   <li>The FTC fixed-tag layout policy matches the game pieces being treated as landmarks.</li>
         * </ul>
         *
         * <p>The EKF can produce smoother and more principled covariance-aware estimates, but poor
         * calibration will make it confidently wrong. Keep the fusion estimator as a comparison
         * baseline while tuning.</p>
         */
        public static OdometryTagEkfPoseEstimator.Config pinpointAprilTagEkf =
                OdometryTagEkfPoseEstimator.Config.defaults();

        // Advanced options / default tuning:
        static {
            // pinpoint.withEncoderPods(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            // pinpoint.withCustomEncoderResolutionTicksPerInch(null);
            // pinpoint.withYawScalar(null);
            // pinpoint.withResetOnInit(true);
            // pinpoint.withResetWaitMs(300);
            // pinpoint.withQuality(0.75);

            // The framework's FtcGameTagLayout.currentGameFieldFixed() now owns the official
            // current-game fixed-tag policy, so robot code no longer carries season-specific
            // exclusion lists here.

            // AprilTag-only solve: use all visible fixed tags, weight closer / more centered tags,
            // and allow the FTC SDK robotPose when it agrees with Phoenix's explicit geometry.
            aprilTags.maxAbsBearingRad = 0.0;
            aprilTags.preferObservationFieldPose = true;
            aprilTags.observationFieldPoseMaxDeltaInches = 8.0;
            aprilTags.observationFieldPoseMaxDeltaHeadingRad = Math.toRadians(12.0);
            aprilTags.rangeSoftnessInches = 36.0;
            aprilTags.minObservationWeight = 0.05;
            aprilTags.outlierPositionGateInches = 18.0;
            aprilTags.outlierHeadingGateRad = Math.toRadians(25.0);
            aprilTags.plausibleFieldRegion = FtcFieldRegions.fullField();
            aprilTags.maxOutsidePlausibleFieldRegionInches = 3.0;

            // Fusion: trust Pinpoint for smooth motion, but let fresh AprilTag solves gently pull
            // the robot back toward the field truth when they are plausible. Keep enough odometry
            // history for latency compensation so accepted AprilTag frames can be corrected at their
            // measurement timestamp and replayed forward to "now".
            pinpointAprilTagFusion.maxVisionAgeSec = 0.35;
            pinpointAprilTagFusion.minVisionQuality = 0.10;
            pinpointAprilTagFusion.visionPositionGain = 0.25;
            pinpointAprilTagFusion.visionHeadingGain = 0.35;
            pinpointAprilTagFusion.maxVisionPositionJumpIn = 24.0;
            pinpointAprilTagFusion.maxVisionHeadingJumpRad = Math.toRadians(60.0);
            pinpointAprilTagFusion.enableLatencyCompensation = true;
            pinpointAprilTagFusion.odomHistorySec = 1.0;

            // Optional EKF: use the same freshness gates as the fusion estimator, but keep explicit
            // covariance knobs so Phoenix can compare the two implementations on real hardware.
            pinpointAprilTagEkf.maxVisionAgeSec = 0.35;
            pinpointAprilTagEkf.minVisionQuality = 0.10;
            pinpointAprilTagEkf.maxVisionPositionInnovationIn = 24.0;
            pinpointAprilTagEkf.maxVisionHeadingInnovationRad = Math.toRadians(60.0);
            pinpointAprilTagEkf.maxVisionMahalanobisSq = 14.0;
            pinpointAprilTagEkf.enableLatencyCompensation = true;
            pinpointAprilTagEkf.odomHistorySec = 1.0;
            pinpointAprilTagEkf.initialPositionStdIn = 6.0;
            pinpointAprilTagEkf.initialHeadingStdRad = Math.toRadians(12.0);
            pinpointAprilTagEkf.manualPosePositionStdIn = 3.0;
            pinpointAprilTagEkf.manualPoseHeadingStdRad = Math.toRadians(6.0);
            pinpointAprilTagEkf.odomProcessPositionStdFloorIn = 0.20;
            pinpointAprilTagEkf.odomProcessPositionStdPerIn = 0.03;
            pinpointAprilTagEkf.odomProcessPositionStdPerRad = 0.55;
            pinpointAprilTagEkf.odomProcessHeadingStdFloorRad = Math.toRadians(0.35);
            pinpointAprilTagEkf.odomProcessHeadingStdPerIn = Math.toRadians(0.06);
            pinpointAprilTagEkf.odomProcessHeadingStdPerRad = 0.06;
            pinpointAprilTagEkf.visionPositionStdFloorIn = 1.75;
            pinpointAprilTagEkf.visionPositionStdScaleIn = 6.0;
            pinpointAprilTagEkf.visionHeadingStdFloorRad = Math.toRadians(3.0);
            pinpointAprilTagEkf.visionHeadingStdScaleRad = Math.toRadians(10.0);
            pinpointAprilTagEkf.qualityPositionStdScaleIn = 18.0;
            pinpointAprilTagEkf.qualityHeadingStdScaleRad = Math.toRadians(30.0);
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

        // ------------------------------------------------------------------
        // TeleOp auto-aim tuning
        // ------------------------------------------------------------------

        /**
         * How tight the robot must be aligned (in degrees) to be considered "aimed".
         *
         * <p>This is also used as the DriveGuidance aim deadband: the assist will output
         * 0 turn command once the error is within this angle.</p>
         */
        public static final double AIM_TOLERANCE_DEG = 0.25;

        /**
         * P gain for auto-aim omega (unitless DriveSignal command per radian of error).
         */
        public static final double AIM_KP = 1.5;

        /**
         * Max turn command for auto-aim (0..1-ish DriveSignal units).
         */
        public static final double AIM_MAX_OMEGA_CMD = 0.80;

        /**
         * Ready-to-shoot tolerance (degrees).
         *
         * <p>This is used only for the <b>"aim ready"</b> gate that decides whether we are
         * allowed to feed a ball.
         *
         * <p>It is intentionally allowed to be a bit looser than {@link #AIM_TOLERANCE_DEG} so we
         * don't get stuck waiting for a perfect last fraction of a degree. Auto-aim will still
         * try to converge inside {@link #AIM_TOLERANCE_DEG}, but we consider it "good enough"
         * to shoot once we're within this value.
         */
        // Default to slightly looser than the aim controller.
        // If you ever find yourself "practically aimed" but the gate won't go true, bump this
        // up a bit (example: 0.75–1.0). If you want higher accuracy, set it equal to
        // AIM_TOLERANCE_DEG.
        public static final double AIM_READY_TOLERANCE_DEG = AIM_TOLERANCE_DEG * 2.0;

        /**
         * Debounce time for the aim-ready gate (seconds).
         */
        public static final double AIM_READY_DEBOUNCE_SEC = 0.05;

        /**
         * Minimum turn command for auto-aim when outside the deadband.
         *
         * <p>Helps overcome drivetrain stiction so the robot doesn't "stall" while still slightly off.
         */
        public static final double AIM_MIN_OMEGA_CMD = 0.05;

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

            /**
             * Creates an aim offset in tag-local forward/left inches.
             */
            public AimOffset(double forwardInches, double leftInches) {
                this.forwardInches = forwardInches;
                this.leftInches = leftInches;
            }
        }
    }
}
