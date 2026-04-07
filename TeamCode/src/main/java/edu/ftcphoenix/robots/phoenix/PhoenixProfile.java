package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.math.InterpolatingTable1D;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.FtcFieldRegions;
import edu.ftcphoenix.fw.ftc.localization.PinpointPoseEstimator;
import edu.ftcphoenix.fw.localization.apriltag.TagOnlyPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.OdometryTagEkfPoseEstimator;
import edu.ftcphoenix.fw.localization.fusion.OdometryTagFusionPoseEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;

/**
 * Phoenix robot profile.
 *
 * <p>This replaces the old static {@code RobotConfig} bag with one explicit profile object that
 * the robot container can own, copy, and pass into collaborators. The profile still keeps the
 * familiar robot-side editing experience: device names and tuning values live in one place, but the
 * rest of the code now depends on an instance instead of global statics.</p>
 */
public final class PhoenixProfile {

    private static final PhoenixProfile CURRENT = defaults();

    /**
     * Drivetrain hardware + drivebase tuning.
     */
    public DriveTrainConfig driveTrain = new DriveTrainConfig();

    /**
     * Shooter hardware + scoring-path tuning.
     */
    public ShooterConfig shooter = new ShooterConfig();

    /**
     * Vision hardware and camera extrinsics.
     */
    public VisionConfig vision = new VisionConfig();

    /**
     * Localization configs shared by TeleOp and testers.
     */
    public LocalizationConfig localization = new LocalizationConfig();

    /**
     * Human-acknowledged calibration checkpoints.
     */
    public CalibrationConfig calibration = new CalibrationConfig();

    /**
     * Auto-aim / selected-tag tuning.
     */
    public AutoAimConfig autoAim = new AutoAimConfig();

    /**
     * Creates a Phoenix profile initialized with the default checked-in values.
     */
    public PhoenixProfile() {
    }

    /**
     * Returns the current checked-in Phoenix profile.
     *
     * <p>Robot code should usually pass {@link #copy()} into long-lived owners so runtime code does
     * not accidentally depend on mutable global state.</p>
     *
     * @return shared checked-in Phoenix profile instance
     */
    public static PhoenixProfile current() {
        return CURRENT;
    }

    /**
     * Creates a new profile instance populated with Phoenix defaults.
     *
     * @return new mutable profile instance with default Phoenix settings
     */
    public static PhoenixProfile defaults() {
        return new PhoenixProfile();
    }

    /**
     * Creates a deep copy of this profile.
     *
     * @return copy whose nested config objects can be edited independently of the original
     */
    public PhoenixProfile copy() {
        PhoenixProfile copy = new PhoenixProfile();
        copy.driveTrain = this.driveTrain.copy();
        copy.shooter = this.shooter.copy();
        copy.vision = this.vision.copy();
        copy.localization = this.localization.copy();
        copy.calibration = this.calibration.copy();
        copy.autoAim = this.autoAim.copy();
        return copy;
    }

    /**
     * Drivetrain hardware mapping and drivebase configuration.
     */
    public static final class DriveTrainConfig {
        public String nameMotorFrontLeft = "frontLeftMotor";
        public Direction directionMotorFrontLeft = Direction.FORWARD;

        public String nameMotorFrontRight = "frontRightMotor";
        public Direction directionMotorFrontRight = Direction.FORWARD;

        public String nameMotorBackLeft = "backLeftMotor";
        public Direction directionMotorBackLeft = Direction.FORWARD;

        public String nameMotorBackRight = "backRightMotor";
        public Direction directionMotorBackRight = Direction.FORWARD;

        /**
         * If true, set drivetrain motors to BRAKE when commanded power is 0.
         */
        public boolean zeroPowerBrake = true;

        /**
         * Open-loop drivebase tuning.
         */
        public MecanumDrivebase.Config drivebase = MecanumDrivebase.Config.defaults();

        /**
         * Creates a drivetrain config initialized with Phoenix defaults.
         */
        public DriveTrainConfig() {
        }

        /**
         * Builds the mecanum hardware-wiring config expected by the framework helpers.
         *
         * @return wiring config containing the configured motor names and directions
         */
        public FtcDrives.MecanumWiringConfig mecanumWiring() {
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

        /**
         * Creates a deep copy of this drivetrain config.
         *
         * @return copied drivetrain config
         */
        public DriveTrainConfig copy() {
            DriveTrainConfig c = new DriveTrainConfig();
            c.nameMotorFrontLeft = this.nameMotorFrontLeft;
            c.directionMotorFrontLeft = this.directionMotorFrontLeft;
            c.nameMotorFrontRight = this.nameMotorFrontRight;
            c.directionMotorFrontRight = this.directionMotorFrontRight;
            c.nameMotorBackLeft = this.nameMotorBackLeft;
            c.directionMotorBackLeft = this.directionMotorBackLeft;
            c.nameMotorBackRight = this.nameMotorBackRight;
            c.directionMotorBackRight = this.directionMotorBackRight;
            c.zeroPowerBrake = this.zeroPowerBrake;
            c.drivebase = this.drivebase.copy();
            return c;
        }
    }

    /**
     * Shooter hardware mapping, feed tuning, and flywheel calibration values.
     */
    public static final class ShooterConfig {
        public String nameMotorIntake = "intakeMotor";
        public Direction directionMotorIntake = Direction.FORWARD;

        public String nameCrServoIntakeTransfer = "intakeTransfer";
        public Direction directionCrServoIntakeTransfer = Direction.REVERSE;

        public String nameCrServoShooterTransferLeft = "shooterTransferLeft";
        public Direction directionCrServoShooterTransferLeft = Direction.REVERSE;

        public String nameCrServoShooterTransferRight = "shooterTransferRight";
        public Direction directionCrServoShooterTransferRight = Direction.FORWARD;

        public double shooterTransferLeftScale = 0.65;
        public double shooterTransferLeftBias = 0.0;

        public String nameMotorShooterWheel = "shooterMotor";
        public Direction directionMotorShooterWheel = Direction.FORWARD;

        public double velocityMin = 700;
        public double velocityMax = 2000;
        public double velocityIncrement = 25;
        public double velocityToleranceNative = 50;

        public boolean applyFlywheelVelocityPIDF = false;
        public double flywheelVelKp = 0.0;
        public double flywheelVelKi = 0.0;
        public double flywheelVelKd = 0.0;
        public double flywheelVelKf = 0.0;

        public double velocityToleranceBelowNative = 50;
        public double velocityToleranceAboveNative = 50;
        public double readyPredictLeadSec = 0.10;
        public double readyStableSec = 0.03;

        public double intakeMotorPower = 1.0;
        public double intakeTransferPower = 1.0;
        public double intakeShooterTransferHoldBackPower = 0.5;

        public double ejectMotorPower = 1.0;
        public double ejectTransferPower = 1.0;
        public double ejectShooterTransferPower = 1.0;

        public double shootFeedPower = 1.0;
        public double shootFeedPulseSec = 0.22;
        public double shootFeedCooldownSec = 0.06;
        public double flywheelKeepAliveSec = 0.75;

        public double feedScaleIntakeMotor = 1.0;
        public double feedScaleIntakeTransfer = 1.0;
        public double feedScaleShooterTransfer = 1.0;

        /**
         * Distance (in) -> flywheel velocity (native units).
         */
        public InterpolatingTable1D velocityTable = InterpolatingTable1D.ofSortedPairs(
                28.06, 1505.6,
                36.52, 1427.4,
                42.3, 1424.35,
                50.3, 1450,
                56.5, 1461,
                62.9, 1538,
                65.8, 1535.7,
                70, 1530,
                74.2, 1575,
                79.5, 1600,
                83.4, 1625,
                93.6, 1700,
                96.6, 1700,
                103.2, 1775,
                104.7, 1800,
                109.2, 1800,
                112, 1818,
                115, 1825,
                120, 1850,
                130, 1875
        );

        /**
         * Creates a shooter config initialized with Phoenix defaults.
         */
        public ShooterConfig() {
        }

        /**
         * Creates a deep copy of this shooter config.
         *
         * @return copied shooter config
         */
        public ShooterConfig copy() {
            ShooterConfig c = new ShooterConfig();
            c.nameMotorIntake = this.nameMotorIntake;
            c.directionMotorIntake = this.directionMotorIntake;
            c.nameCrServoIntakeTransfer = this.nameCrServoIntakeTransfer;
            c.directionCrServoIntakeTransfer = this.directionCrServoIntakeTransfer;
            c.nameCrServoShooterTransferLeft = this.nameCrServoShooterTransferLeft;
            c.directionCrServoShooterTransferLeft = this.directionCrServoShooterTransferLeft;
            c.nameCrServoShooterTransferRight = this.nameCrServoShooterTransferRight;
            c.directionCrServoShooterTransferRight = this.directionCrServoShooterTransferRight;
            c.shooterTransferLeftScale = this.shooterTransferLeftScale;
            c.shooterTransferLeftBias = this.shooterTransferLeftBias;
            c.nameMotorShooterWheel = this.nameMotorShooterWheel;
            c.directionMotorShooterWheel = this.directionMotorShooterWheel;
            c.velocityMin = this.velocityMin;
            c.velocityMax = this.velocityMax;
            c.velocityIncrement = this.velocityIncrement;
            c.velocityToleranceNative = this.velocityToleranceNative;
            c.applyFlywheelVelocityPIDF = this.applyFlywheelVelocityPIDF;
            c.flywheelVelKp = this.flywheelVelKp;
            c.flywheelVelKi = this.flywheelVelKi;
            c.flywheelVelKd = this.flywheelVelKd;
            c.flywheelVelKf = this.flywheelVelKf;
            c.velocityToleranceBelowNative = this.velocityToleranceBelowNative;
            c.velocityToleranceAboveNative = this.velocityToleranceAboveNative;
            c.readyPredictLeadSec = this.readyPredictLeadSec;
            c.readyStableSec = this.readyStableSec;
            c.intakeMotorPower = this.intakeMotorPower;
            c.intakeTransferPower = this.intakeTransferPower;
            c.intakeShooterTransferHoldBackPower = this.intakeShooterTransferHoldBackPower;
            c.ejectMotorPower = this.ejectMotorPower;
            c.ejectTransferPower = this.ejectTransferPower;
            c.ejectShooterTransferPower = this.ejectShooterTransferPower;
            c.shootFeedPower = this.shootFeedPower;
            c.shootFeedPulseSec = this.shootFeedPulseSec;
            c.shootFeedCooldownSec = this.shootFeedCooldownSec;
            c.flywheelKeepAliveSec = this.flywheelKeepAliveSec;
            c.feedScaleIntakeMotor = this.feedScaleIntakeMotor;
            c.feedScaleIntakeTransfer = this.feedScaleIntakeTransfer;
            c.feedScaleShooterTransfer = this.feedScaleShooterTransfer;
            c.velocityTable = this.velocityTable;
            return c;
        }
    }

    /**
     * Vision-related profile values.
     */
    public static final class VisionConfig {
        public String nameWebcam = "Webcam 1";

        public CameraMountConfig cameraMount = CameraMountConfig.ofDegrees(
                9.97,
                -1.80,
                13.68,
                1.9,
                -18.2,
                -1.7
        );

        /**
         * Creates a vision config initialized with Phoenix defaults.
         */
        public VisionConfig() {
        }

        /**
         * Creates a copy of this vision config.
         *
         * @return copied vision config
         */
        public VisionConfig copy() {
            VisionConfig c = new VisionConfig();
            c.nameWebcam = this.nameWebcam;
            c.cameraMount = this.cameraMount;
            return c;
        }
    }

    /**
     * Localization-related configs shared by the robot and testers.
     */
    public static final class LocalizationConfig {
        /**
         * Selects which global pose estimator Phoenix should use at runtime.
         */
        public enum GlobalEstimatorMode {
            FUSION,
            EKF
        }

        public GlobalEstimatorMode globalEstimatorMode = GlobalEstimatorMode.FUSION;

        public PinpointPoseEstimator.Config pinpoint =
                PinpointPoseEstimator.Config.defaults()
                        .withHardwareMapName("pinPoint")
                        .withOffsets(0.0, 0.0)
                        .withForwardPodDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                        .withStrafePodDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

        public TagOnlyPoseEstimator.Config aprilTags = TagOnlyPoseEstimator.Config.defaults();
        public OdometryTagFusionPoseEstimator.Config pinpointAprilTagFusion =
                OdometryTagFusionPoseEstimator.Config.defaults();
        public OdometryTagEkfPoseEstimator.Config pinpointAprilTagEkf =
                OdometryTagEkfPoseEstimator.Config.defaults();

        /**
         * Creates a localization config initialized with Phoenix defaults.
         */
        public LocalizationConfig() {
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

            pinpointAprilTagFusion.maxVisionAgeSec = 0.35;
            pinpointAprilTagFusion.minVisionQuality = 0.10;
            pinpointAprilTagFusion.visionPositionGain = 0.25;
            pinpointAprilTagFusion.visionHeadingGain = 0.35;
            pinpointAprilTagFusion.maxVisionPositionJumpIn = 24.0;
            pinpointAprilTagFusion.maxVisionHeadingJumpRad = Math.toRadians(60.0);
            pinpointAprilTagFusion.enableLatencyCompensation = true;
            pinpointAprilTagFusion.odomHistorySec = 1.0;

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

        /**
         * Creates a deep copy of this localization config.
         *
         * @return copied localization config
         */
        public LocalizationConfig copy() {
            LocalizationConfig c = new LocalizationConfig();
            c.globalEstimatorMode = this.globalEstimatorMode;
            c.pinpoint = this.pinpoint.copy();
            c.aprilTags = this.aprilTags.copy();
            c.pinpointAprilTagFusion = this.pinpointAprilTagFusion.copy();
            c.pinpointAprilTagEkf = this.pinpointAprilTagEkf.copy();
            return c;
        }
    }

    /**
     * Human-acknowledged calibration steps that cannot be inferred automatically.
     */
    public static final class CalibrationConfig {
        public boolean pinpointAxesVerified = true;
        public boolean pinpointPodOffsetsCalibrated = false;

        /**
         * Creates a calibration config initialized with Phoenix defaults.
         */
        public CalibrationConfig() {
        }

        /**
         * Creates a copy of this calibration config.
         *
         * @return copied calibration config
         */
        public CalibrationConfig copy() {
            CalibrationConfig c = new CalibrationConfig();
            c.pinpointAxesVerified = this.pinpointAxesVerified;
            c.pinpointPodOffsetsCalibrated = this.pinpointPodOffsetsCalibrated;
            return c;
        }
    }

    /**
     * Auto-aim / selected-tag profile values.
     */
    public static final class AutoAimConfig {
        public int blueTargetTagId = 20;
        public int redTargetTagId = 24;

        public double aimToleranceDeg = 0.25;
        public double aimKp = 1.5;
        public double aimMaxOmegaCmd = 0.80;
        public double aimReadyToleranceDeg = aimToleranceDeg * 2.0;
        public double aimReadyDebounceSec = 0.05;
        public double aimMinOmegaCmd = 0.05;

        public double selectionMaxAgeSec = 0.50;
        public double selectionReacquireSec = 0.20;

        public double shootBraceEnterMagnitude = 0.06;
        public double shootBraceExitMagnitude = 0.10;
        public double shootBraceTranslateKp = 0.08;
        public double shootBraceMaxTranslateCmd = 0.35;

        public AimOffset blueAimOffset = new AimOffset(0.0, 0.0);
        public AimOffset redAimOffset = new AimOffset(0.0, 0.0);
        public AimOffset defaultAimOffset = new AimOffset(0.0, 0.0);

        /**
         * Creates an auto-aim config initialized with Phoenix defaults.
         */
        public AutoAimConfig() {
        }

        /**
         * Returns the set of fixed tag ids Phoenix will consider for scoring alignment.
         *
         * @return unmodifiable set of scoring tag ids
         */
        public Set<Integer> scoringTagIds() {
            LinkedHashSet<Integer> ids = new LinkedHashSet<Integer>();
            ids.add(blueTargetTagId);
            ids.add(redTargetTagId);
            return Collections.unmodifiableSet(ids);
        }

        /**
         * Returns the configured per-tag aim offsets keyed by tag id.
         *
         * @return unmodifiable map from tag id to tag-local aim offset
         */
        public Map<Integer, AimOffset> aimOffsetsByTag() {
            LinkedHashMap<Integer, AimOffset> offsets = new LinkedHashMap<Integer, AimOffset>();
            offsets.put(blueTargetTagId, blueAimOffset);
            offsets.put(redTargetTagId, redAimOffset);
            return Collections.unmodifiableMap(offsets);
        }

        /**
         * Returns the configured aim offset for a specific scoring tag.
         *
         * @param tagId AprilTag id to query
         * @return configured tag-local aim offset, or {@link #defaultAimOffset} when the tag id does
         * not match one of the explicit scoring targets
         */
        public AimOffset aimOffsetForTag(int tagId) {
            if (tagId == blueTargetTagId) {
                return blueAimOffset;
            }
            if (tagId == redTargetTagId) {
                return redAimOffset;
            }
            return defaultAimOffset;
        }

        /**
         * Creates a deep copy of this auto-aim config.
         *
         * @return copied auto-aim config
         */
        public AutoAimConfig copy() {
            AutoAimConfig c = new AutoAimConfig();
            c.blueTargetTagId = this.blueTargetTagId;
            c.redTargetTagId = this.redTargetTagId;
            c.aimToleranceDeg = this.aimToleranceDeg;
            c.aimKp = this.aimKp;
            c.aimMaxOmegaCmd = this.aimMaxOmegaCmd;
            c.aimReadyToleranceDeg = this.aimReadyToleranceDeg;
            c.aimReadyDebounceSec = this.aimReadyDebounceSec;
            c.aimMinOmegaCmd = this.aimMinOmegaCmd;
            c.selectionMaxAgeSec = this.selectionMaxAgeSec;
            c.selectionReacquireSec = this.selectionReacquireSec;
            c.shootBraceEnterMagnitude = this.shootBraceEnterMagnitude;
            c.shootBraceExitMagnitude = this.shootBraceExitMagnitude;
            c.shootBraceTranslateKp = this.shootBraceTranslateKp;
            c.shootBraceMaxTranslateCmd = this.shootBraceMaxTranslateCmd;
            c.blueAimOffset = this.blueAimOffset.copy();
            c.redAimOffset = this.redAimOffset.copy();
            c.defaultAimOffset = this.defaultAimOffset.copy();
            return c;
        }

        /**
         * Tag-local forward/left offset in inches.
         */
        public static final class AimOffset {
            public double forwardInches;
            public double leftInches;

            /**
             * Creates a tag-local aim offset.
             *
             * @param forwardInches forward offset from the tag frame, in inches
             * @param leftInches    left offset from the tag frame, in inches
             */
            public AimOffset(double forwardInches, double leftInches) {
                this.forwardInches = forwardInches;
                this.leftInches = leftInches;
            }

            /**
             * Creates a copy of this aim offset.
             *
             * @return copied aim offset
             */
            public AimOffset copy() {
                return new AimOffset(forwardInches, leftInches);
            }
        }
    }
}
