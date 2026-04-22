package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.math.InterpolatingTable1D;
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.ftc.FtcFieldRegions;
import edu.ftcphoenix.fw.ftc.FtcGameTagLayout;
import edu.ftcphoenix.fw.ftc.drive.FtcMecanumDriveLane;
import edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcphoenix.fw.ftc.vision.FtcLimelightAprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.FtcWebcamAprilTagVisionLane;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;

/**
 * Phoenix robot profile.
 *
 * <p>
 * Phoenix treats stable framework lanes and a few robot-owned lane-selection wrappers as first-class
 * config owners while keeping robot-specific strategy and operator policy in the robot layer. The
 * result is a profile whose top-level sections
 * mirror the architectural roles used in code:
 * </p>
 * <ul>
 *   <li>framework-owned lanes: drive and localization, plus a robot-owned AprilTag backend wrapper</li>
 *   <li>shared field facts: fixed AprilTag layout for the current game</li>
 *   <li>robot-owned controls: TeleOp stick shaping and slow-mode tuning</li>
 *   <li>robot-owned drive assists: scoring-related drive overlays and brace tuning</li>
 *   <li>robot-owned mechanisms and strategy: scoring path, targeting, Auto timing, and calibration acknowledgements</li>
 * </ul>
 */
public final class PhoenixProfile {

    private static final PhoenixProfile CURRENT = defaults();

    /**
     * Stable drivetrain hardware/lifecycle configuration owned by the framework drive lane.
     */
    public FtcMecanumDriveLane.Config drive = defaultDriveConfig();

    /**
     * Phoenix-owned AprilTag backend-selection config.
     */
    public VisionConfig vision = defaultVisionConfig();

    /**
     * Stable localization-strategy configuration owned by the framework localization lane.
     */
    public FtcOdometryAprilTagLocalizationLane.Config localization = defaultLocalizationConfig();

    /**
     * Shared field facts consumed by localization, targeting, and calibration tools.
     */
    public FieldConfig field = new FieldConfig();

    /**
     * TeleOp control-layer tuning owned by Phoenix's robot-specific controls object.
     */
    public TeleOpControlsConfig controls = new TeleOpControlsConfig();

    /**
     * Robot-specific drive-assist tuning layered on top of the stable framework drive lane.
     */
    public DriveAssistConfig driveAssist = new DriveAssistConfig();

    /**
     * Scoring-path hardware and policy tuning.
     */
    public ScoringPathConfig scoring = new ScoringPathConfig();

    /**
     * Human-acknowledged calibration checkpoints.
     */
    public CalibrationConfig calibration = new CalibrationConfig();

    /**
     * Auto-aim / selected-tag tuning.
     */
    public AutoAimConfig autoAim = new AutoAimConfig();

    /**
     * Autonomous route/aim/wait timing tuned by Phoenix Auto strategy code.
     */
    public AutoConfig auto = new AutoConfig();

    /**
     * Creates a Phoenix profile initialized with the checked-in defaults.
     */
    public PhoenixProfile() {
    }

    /**
     * Returns the current checked-in Phoenix profile.
     *
     * <p>
     * Robot code should usually pass {@link #copy()} into long-lived owners so runtime logic does
     * not accidentally depend on mutable global state.
     * </p>
     *
     * @return shared checked-in Phoenix profile instance
     */
    public static PhoenixProfile current() {
        return CURRENT;
    }

    /**
     * Creates a new profile instance populated with Phoenix defaults.
     *
     * @return new mutable profile instance with Phoenix's checked-in defaults
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
        copy.drive = this.drive.copy();
        copy.vision = this.vision.copy();
        copy.localization = this.localization.copy();
        copy.field = this.field.copy();
        copy.controls = this.controls.copy();
        copy.driveAssist = this.driveAssist.copy();
        copy.scoring = this.scoring.copy();
        copy.calibration = this.calibration.copy();
        copy.autoAim = this.autoAim.copy();
        copy.auto = this.auto.copy();
        return copy;
    }

    /**
     * Phoenix-owned AprilTag backend-selection config.
     *
     * <p>
     * The framework owns concrete FTC-boundary lane implementations such as
     * {@link FtcWebcamAprilTagVisionLane} and {@link FtcLimelightAprilTagVisionLane}. Phoenix owns
     * the higher-level choice of which backend to instantiate. That keeps backend selection in the
     * robot layer while the rest of Phoenix consumes only the backend-neutral lane seam.
     * </p>
     */
    public static final class VisionConfig {

        /**
         * Supported Phoenix AprilTag backend selections.
         */
        public enum Backend {
            /**
             * Use the standard FTC webcam-backed AprilTag lane.
             */
            WEBCAM,
            /**
             * Use the FTC Limelight-backed AprilTag lane.
             */
            LIMELIGHT
        }

        /**
         * Which AprilTag backend Phoenix should instantiate.
         */
        public Backend backend = Backend.WEBCAM;

        /**
         * Webcam-backed lane configuration used when {@link #backend} is {@link Backend#WEBCAM}.
         */
        public FtcWebcamAprilTagVisionLane.Config webcam = defaultWebcamVisionConfig();

        /**
         * Limelight-backed lane configuration used when {@link #backend} is {@link Backend#LIMELIGHT}.
         */
        public FtcLimelightAprilTagVisionLane.Config limelight = defaultLimelightVisionConfig();

        /**
         * Creates a vision config initialized with Phoenix defaults.
         */
        public VisionConfig() {
        }

        /**
         * Creates a deep copy of this vision config.
         *
         * @return copied backend-selection config
         */
        public VisionConfig copy() {
            VisionConfig c = new VisionConfig();
            c.backend = this.backend;
            c.webcam = this.webcam.copy();
            c.limelight = this.limelight.copy();
            return c;
        }

        /**
         * Returns the camera mount configured for the active backend choice.
         *
         * @return camera extrinsics for whichever backend Phoenix is currently configured to use
         */
        public CameraMountConfig activeCameraMount() {
            return backend == Backend.LIMELIGHT ? limelight.cameraMount : webcam.cameraMount;
        }

        /**
         * Returns the preferred hardware/device name for the active backend choice.
         *
         * @return backend-specific hardware/device identifier
         */
        public String activeDeviceName() {
            return backend == Backend.LIMELIGHT ? limelight.hardwareName : webcam.webcamName;
        }
    }

    /**
     * Shared field facts for Phoenix.
     *
     * <p>
     * Field facts are intentionally separate from both the vision lane and the localization lane.
     * The camera rig is a physical sensor concern. Localization is a pose-estimation concern.
     * Field facts answer a different question entirely: which fixed landmarks and regions describe
     * the current game field? Keeping them separate makes it obvious where future overrides or
     * practice-field substitutions belong.
     * </p>
     */
    public static final class FieldConfig {

        /**
         * Fixed AprilTag layout trusted as field landmarks for localization and targeting.
         */
        public TagLayout fixedAprilTagLayout = FtcGameTagLayout.currentGameFieldFixed();

        /**
         * Creates a field-facts config initialized with Phoenix defaults.
         */
        public FieldConfig() {
        }

        /**
         * Creates a copy of this field-facts config.
         *
         * @return copied field-facts config
         */
        public FieldConfig copy() {
            FieldConfig c = new FieldConfig();
            c.fixedAprilTagLayout = this.fixedAprilTagLayout;
            return c;
        }
    }

    /**
     * Phoenix TeleOp control-layer configuration.
     *
     * <p>
     * This config intentionally holds only control-layer tuning. Physical button identities and
     * control semantics still live in {@code PhoenixTeleOpControls}, because those choices are part
     * of the robot's control policy rather than generic framework configuration.
     * </p>
     */
    public static final class TeleOpControlsConfig {

        /**
         * Drive-stick shaping and slow-mode tuning for the manual drive source.
         */
        public DriveControlsConfig drive = new DriveControlsConfig();

        /**
         * Step size used when the operator nudges the selected flywheel velocity target.
         *
         * <p>
         * This lives in the controls config because it is part of the control-layer experience, not
         * part of the shooter hardware definition.
         * </p>
         */
        public double selectedVelocityStepNative = 25;

        /**
         * Creates a TeleOp controls config initialized with Phoenix defaults.
         */
        public TeleOpControlsConfig() {
        }

        /**
         * Creates a deep copy of this TeleOp controls config.
         *
         * @return copied controls config
         */
        public TeleOpControlsConfig copy() {
            TeleOpControlsConfig c = new TeleOpControlsConfig();
            c.drive = this.drive.copy();
            c.selectedVelocityStepNative = this.selectedVelocityStepNative;
            return c;
        }

        /**
         * Drive-control tuning owned by the robot controls layer.
         */
        public static final class DriveControlsConfig {

            /**
             * Stick shaping and scaling for the base manual drive source before any slow-mode wrapper.
             */
            public GamepadDriveSource.Config manualDrive = GamepadDriveSource.Config.defaults();

            /**
             * Translation scale applied while the Phoenix slow-mode button is held.
             */
            public double slowTranslateScale = 0.35;

            /**
             * Rotation scale applied while the Phoenix slow-mode button is held.
             */
            public double slowOmegaScale = 0.20;

            /**
             * Creates a drive-controls config initialized with Phoenix defaults.
             */
            public DriveControlsConfig() {
            }

            /**
             * Creates a deep copy of this drive-controls config.
             *
             * @return copied drive-controls config
             */
            public DriveControlsConfig copy() {
                DriveControlsConfig c = new DriveControlsConfig();
                c.manualDrive = this.manualDrive.copy();
                c.slowTranslateScale = this.slowTranslateScale;
                c.slowOmegaScale = this.slowOmegaScale;
                return c;
            }
        }
    }


    /**
     * Robot-specific drive-assist tuning layered on top of the framework drive lane.
     *
     * <p>
     * Phoenix distinguishes between:
     * </p>
     * <ul>
     *   <li>controls config: how the driver requests motion</li>
     *   <li>drive-assist config: how robot policy reshapes that motion using localization,
     *       scoring state, and overlays</li>
     * </ul>
     *
     * <p>
     * Keeping these values out of {@link AutoAimConfig} makes the ownership boundary explicit.
     * Shoot-brace is not target-selection policy. It is a robot-specific drive-assist policy that
     * uses scoring state and manual-drive idleness to decide when translation should be held.
     * </p>
     */
    public static final class DriveAssistConfig {

        /**
         * Shoot-brace tuning for translation hold while actively shooting.
         */
        public ShootBraceConfig shootBrace = new ShootBraceConfig();

        /**
         * Creates a drive-assist config initialized with Phoenix defaults.
         */
        public DriveAssistConfig() {
        }

        /**
         * Creates a deep copy of this drive-assist config.
         *
         * @return copied drive-assist config
         */
        public DriveAssistConfig copy() {
            DriveAssistConfig c = new DriveAssistConfig();
            c.shootBrace = this.shootBrace.copy();
            return c;
        }

        /**
         * Translation-hold tuning for the Phoenix shoot-brace drive assist.
         */
        public static final class ShootBraceConfig {

            /**
             * Translation-stick magnitude at or below which shoot-brace may latch on.
             */
            public double enterTranslateMagnitude = 0.06;

            /**
             * Translation-stick magnitude at or above which shoot-brace must drop back out.
             */
            public double exitTranslateMagnitude = 0.10;

            /**
             * Translation proportional gain used by the pose-lock overlay while shoot-brace is active.
             */
            public double translateKp = 0.08;

            /**
             * Maximum translation command magnitude contributed by the shoot-brace overlay.
             */
            public double maxTranslateCmd = 0.35;

            /**
             * Creates a shoot-brace config initialized with Phoenix defaults.
             */
            public ShootBraceConfig() {
            }

            /**
             * Creates a deep copy of this shoot-brace config.
             *
             * @return copied shoot-brace config
             */
            public ShootBraceConfig copy() {
                ShootBraceConfig c = new ShootBraceConfig();
                c.enterTranslateMagnitude = this.enterTranslateMagnitude;
                c.exitTranslateMagnitude = this.exitTranslateMagnitude;
                c.translateKp = this.translateKp;
                c.maxTranslateCmd = this.maxTranslateCmd;
                return c;
            }
        }
    }


    private static FtcMecanumDriveLane.Config defaultDriveConfig() {
        FtcMecanumDriveLane.Config cfg = FtcMecanumDriveLane.Config.defaults();
        cfg.wiring.frontLeftName = "frontLeftMotor";
        cfg.wiring.frontLeftDirection = Direction.FORWARD;
        cfg.wiring.frontRightName = "frontRightMotor";
        cfg.wiring.frontRightDirection = Direction.FORWARD;
        cfg.wiring.backLeftName = "backLeftMotor";
        cfg.wiring.backLeftDirection = Direction.FORWARD;
        cfg.wiring.backRightName = "backRightMotor";
        cfg.wiring.backRightDirection = Direction.FORWARD;
        cfg.zeroPowerBrake = true;
        return cfg;
    }

    private static VisionConfig defaultVisionConfig() {
        VisionConfig cfg = new VisionConfig();
        cfg.backend = VisionConfig.Backend.WEBCAM;
        cfg.webcam = defaultWebcamVisionConfig();
        cfg.limelight = defaultLimelightVisionConfig();
        return cfg;
    }

    private static FtcWebcamAprilTagVisionLane.Config defaultWebcamVisionConfig() {
        FtcWebcamAprilTagVisionLane.Config cfg = FtcWebcamAprilTagVisionLane.Config.defaults();
        cfg.webcamName = "Webcam 1";
        cfg.cameraMount = CameraMountConfig.ofDegrees(
                9.97,
                -1.80,
                13.68,
                1.9,
                -18.2,
                -1.7
        );
        return cfg;
    }

    private static FtcLimelightAprilTagVisionLane.Config defaultLimelightVisionConfig() {
        FtcLimelightAprilTagVisionLane.Config cfg = FtcLimelightAprilTagVisionLane.Config.defaults();
        cfg.hardwareName = "limelight";
        cfg.pipelineIndex = 0;
        cfg.pollRateHz = 100;
        cfg.cameraMount = CameraMountConfig.ofDegrees(
                9.97,
                -1.80,
                13.68,
                1.9,
                -18.2,
                -1.7
        );
        return cfg;
    }

    private static FtcOdometryAprilTagLocalizationLane.Config defaultLocalizationConfig() {
        FtcOdometryAprilTagLocalizationLane.Config cfg = FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.correctedEstimatorMode = FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.FUSION;
        cfg.correctionSource.mode = FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.APRILTAG_POSE;

        cfg.predictor = cfg.predictor
                .withHardwareMapName("pinPoint")
                .withOffsets(0.0, 0.0)
                .withForwardPodDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                .withStrafePodDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

        cfg.aprilTags.maxDetectionAgeSec = 0.50;
        cfg.aprilTags.fieldPoseSolver.maxAbsBearingRad = 0.0;
        cfg.aprilTags.fieldPoseSolver.preferObservationFieldPose = true;
        cfg.aprilTags.fieldPoseSolver.observationFieldPoseMaxDeltaInches = 8.0;
        cfg.aprilTags.fieldPoseSolver.observationFieldPoseMaxDeltaHeadingRad = Math.toRadians(12.0);
        cfg.aprilTags.fieldPoseSolver.rangeSoftnessInches = 36.0;
        cfg.aprilTags.fieldPoseSolver.minObservationWeight = 0.05;
        cfg.aprilTags.fieldPoseSolver.outlierPositionGateInches = 18.0;
        cfg.aprilTags.fieldPoseSolver.outlierHeadingGateRad = Math.toRadians(25.0);
        cfg.aprilTags.fieldPoseSolver.plausibleFieldRegion = FtcFieldRegions.fullField();
        cfg.aprilTags.fieldPoseSolver.maxOutsidePlausibleFieldRegionInches = 3.0;

        cfg.correctionSource.limelightFieldPose.mode = edu.ftcphoenix.fw.ftc.localization.LimelightFieldPoseEstimator.Config.Mode.BOTPOSE;
        cfg.correctionSource.limelightFieldPose.maxResultAgeSec = 0.25;
        cfg.correctionSource.limelightFieldPose.minVisibleTags = 1;
        cfg.correctionSource.limelightFieldPose.singleTagQuality = 0.55;
        cfg.correctionSource.limelightFieldPose.multiTagQuality = 0.85;
        cfg.correctionSource.limelightFieldPose.degradeWhenMoving = true;
        cfg.correctionSource.limelightFieldPose.translationSpeedForZeroQualityInPerSec = 72.0;
        cfg.correctionSource.limelightFieldPose.yawRateForZeroQualityRadPerSec = Math.toRadians(360.0);
        cfg.correctionSource.limelightFieldPose.rejectWhenMovingTooFast = false;
        cfg.correctionSource.limelightFieldPose.maxTranslationSpeedInPerSec = 120.0;
        cfg.correctionSource.limelightFieldPose.maxYawRateRadPerSec = Math.toRadians(720.0);

        cfg.correctionFusion.maxCorrectionAgeSec = 0.35;
        cfg.correctionFusion.minCorrectionQuality = 0.10;
        cfg.correctionFusion.correctionPositionGain = 0.25;
        cfg.correctionFusion.correctionHeadingGain = 0.35;
        cfg.correctionFusion.maxCorrectionPositionJumpIn = 24.0;
        cfg.correctionFusion.maxCorrectionHeadingJumpRad = Math.toRadians(60.0);
        cfg.correctionFusion.enableLatencyCompensation = true;
        cfg.correctionFusion.predictorHistorySec = 1.0;

        cfg.correctionEkf.maxCorrectionAgeSec = 0.35;
        cfg.correctionEkf.minCorrectionQuality = 0.10;
        cfg.correctionEkf.maxCorrectionPositionInnovationIn = 24.0;
        cfg.correctionEkf.maxCorrectionHeadingInnovationRad = Math.toRadians(60.0);
        cfg.correctionEkf.maxCorrectionMahalanobisSq = 14.0;
        cfg.correctionEkf.enableLatencyCompensation = true;
        cfg.correctionEkf.predictorHistorySec = 1.0;
        cfg.correctionEkf.initialPositionStdIn = 6.0;
        cfg.correctionEkf.initialHeadingStdRad = Math.toRadians(12.0);
        cfg.correctionEkf.manualPosePositionStdIn = 3.0;
        cfg.correctionEkf.manualPoseHeadingStdRad = Math.toRadians(6.0);
        cfg.correctionEkf.predictorProcessPositionStdFloorIn = 0.20;
        cfg.correctionEkf.predictorProcessPositionStdPerIn = 0.03;
        cfg.correctionEkf.predictorProcessPositionStdPerRad = 0.55;
        cfg.correctionEkf.predictorProcessHeadingStdFloorRad = Math.toRadians(0.35);
        cfg.correctionEkf.predictorProcessHeadingStdPerIn = Math.toRadians(0.06);
        cfg.correctionEkf.predictorProcessHeadingStdPerRad = 0.06;
        cfg.correctionEkf.correctionPositionStdFloorIn = 1.75;
        cfg.correctionEkf.correctionPositionStdScaleIn = 6.0;
        cfg.correctionEkf.correctionHeadingStdFloorRad = Math.toRadians(3.0);
        cfg.correctionEkf.correctionHeadingStdScaleRad = Math.toRadians(10.0);
        cfg.correctionEkf.projectedCorrectionPositionStdPerSec = 18.0;
        cfg.correctionEkf.projectedCorrectionHeadingStdPerSec = Math.toRadians(30.0);

        return cfg;
    }

    /**
     * Scoring-path hardware mapping, feed tuning, and flywheel calibration values.
     */
    public static final class ScoringPathConfig {
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
        public double feedScaleIntakeMotor = 1.0;
        public double feedScaleIntakeTransfer = 1.0;
        public double feedScaleShooterTransfer = 1.0;


        /**
         * Creates a scoring-path config initialized with Phoenix defaults.
         */
        public ScoringPathConfig() {
        }

        /**
         * Creates a deep copy of this scoring-path config.
         *
         * @return copied scoring-path config
         */
        public ScoringPathConfig copy() {
            ScoringPathConfig c = new ScoringPathConfig();
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
            c.feedScaleIntakeMotor = this.feedScaleIntakeMotor;
            c.feedScaleIntakeTransfer = this.feedScaleIntakeTransfer;
            c.feedScaleShooterTransfer = this.feedScaleShooterTransfer;
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
     * Autonomous route-following, aim-task, and wait timing.
     *
     * <p>These values are consumed by Phoenix Auto plans and routine factories. Keeping them in a
     * profile section avoids hardcoded timing literals in OpModes while still leaving route geometry
     * and game strategy in robot-specific Auto code.</p>
     */
    public static final class AutoConfig {
        /**
         * Maximum time allowed for a single route segment before timing out.
         */
        public double routeTimeoutSec = 4.0;

        /**
         * Heading tolerance used by autonomous aim tasks, in degrees.
         */
        public double aimHeadingToleranceDeg = 2.0;

        /**
         * Overall timeout for autonomous aim tasks.
         */
        public double aimTimeoutSec = 1.75;

        /**
         * Maximum time an aim task may run without usable guidance before timing out.
         */
        public double aimMaxNoGuidanceSec = 0.75;

        /**
         * How long Auto waits for a scoring target selection before skipping aim/shoot work.
         */
        public double waitForTargetSec = 0.75;

        /**
         * How long Auto waits for a requested shot to drain from the scoring queue.
         */
        public double waitForShotCompleteSec = 2.5;

        /**
         * Scoring target id Auto should keep when a red-alliance spec narrows the targeting catalog.
         *
         * <p>The id stays in the profile rather than {@code PhoenixAutoProfiles} so field-specific
         * target choices can be tuned in one checked-in robot config location.</p>
         */
        public int redAllianceScoringTagId = 24;

        /**
         * Scoring target id Auto should keep when a blue-alliance spec narrows the targeting catalog.
         *
         * <p>The id stays in the profile rather than {@code PhoenixAutoProfiles} so field-specific
         * target choices can be tuned in one checked-in robot config location.</p>
         */
        public int blueAllianceScoringTagId = 20;

        /**
         * Distance used by the checked-in Pedro integration placeholder path.
         */
        public double pedroIntegrationTestDistanceIn = 12.0;

        /**
         * Creates an Auto config initialized with Phoenix defaults.
         */
        public AutoConfig() {
        }

        /**
         * Creates a copy of this Auto config.
         *
         * @return copied Auto timing config
         */
        public AutoConfig copy() {
            AutoConfig c = new AutoConfig();
            c.routeTimeoutSec = this.routeTimeoutSec;
            c.aimHeadingToleranceDeg = this.aimHeadingToleranceDeg;
            c.aimTimeoutSec = this.aimTimeoutSec;
            c.aimMaxNoGuidanceSec = this.aimMaxNoGuidanceSec;
            c.waitForTargetSec = this.waitForTargetSec;
            c.waitForShotCompleteSec = this.waitForShotCompleteSec;
            c.redAllianceScoringTagId = this.redAllianceScoringTagId;
            c.blueAllianceScoringTagId = this.blueAllianceScoringTagId;
            c.pedroIntegrationTestDistanceIn = this.pedroIntegrationTestDistanceIn;
            return c;
        }
    }

    /**
     * Auto-aim / selected-tag profile values.
     *
     * <p>
     * This section owns targeting and shot-selection policy only. Robot-specific drive-assist tuning
     * such as shoot-brace lives in {@link DriveAssistConfig} so the config boundary matches the
     * runtime ownership boundary.
     * </p>
     */
    public static final class AutoAimConfig {
        /**
         * Catalog of scoring targets keyed by tag id. Insertion order is preserved for menus,
         * telemetry, and deterministic profile diffs.
         */
        public LinkedHashMap<Integer, ScoringTarget> scoringTargets = defaultScoringTargets();

        public double aimToleranceDeg = 0.25;
        public double aimKp = 1.5;
        public double aimMaxOmegaCmd = 0.80;
        public double aimReadyToleranceDeg = aimToleranceDeg * 2.0;
        public double aimReadyDebounceSec = 0.05;
        public double aimMinOmegaCmd = 0.05;

        public double selectionMaxAgeSec = 0.50;
        public double selectionReacquireSec = 0.20;

        /**
         * Default tag-local offset used when an unknown tag id is queried.
         */
        public AimOffset defaultAimOffset = new AimOffset(0.0, 0.0);

        /**
         * Distance (in) -> recommended flywheel velocity (native units) for range-based shot capture.
         */
        public InterpolatingTable1D shotVelocityTable = InterpolatingTable1D.ofSortedPairs(
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
            return Collections.unmodifiableSet(new LinkedHashSet<Integer>(scoringTargets.keySet()));
        }

        /**
         * Returns the configured per-tag target catalog keyed by tag id.
         *
         * @return unmodifiable map of scoring-target definitions keyed by AprilTag id
         */
        public Map<Integer, ScoringTarget> scoringTargetsById() {
            return Collections.unmodifiableMap(scoringTargets);
        }

        /**
         * Returns the configured aim offsets keyed by target tag id.
         *
         * @return unmodifiable map from tag id to tag-local aim offset
         */
        public Map<Integer, AimOffset> aimOffsetsByTag() {
            LinkedHashMap<Integer, AimOffset> offsets = new LinkedHashMap<Integer, AimOffset>();
            for (Map.Entry<Integer, ScoringTarget> entry : scoringTargets.entrySet()) {
                offsets.put(entry.getKey(), entry.getValue().aimOffset);
            }
            return Collections.unmodifiableMap(offsets);
        }

        /**
         * Returns the configured target profile for a specific scoring tag.
         *
         * @param tagId AprilTag id to query
         * @return configured target profile, or a synthetic default profile when the tag id does not
         * match one of the explicit scoring targets
         */
        public ScoringTarget targetProfileForTag(int tagId) {
            ScoringTarget explicit = scoringTargets.get(tagId);
            return explicit != null ? explicit : defaultTargetProfile(tagId);
        }

        /**
         * Returns a synthetic fallback profile for an unknown scoring tag id.
         *
         * @param tagId tag id to stamp into the fallback profile; use a negative value when no tag is selected
         * @return fallback target profile using the default aim offset
         */
        public ScoringTarget defaultTargetProfile(int tagId) {
            return new ScoringTarget(tagId, tagId >= 0 ? ("Tag " + tagId) : "No target", defaultAimOffset.copy());
        }

        /**
         * Creates a deep copy of this auto-aim config.
         *
         * @return copied auto-aim config
         */
        public AutoAimConfig copy() {
            AutoAimConfig c = new AutoAimConfig();
            c.scoringTargets = new LinkedHashMap<Integer, ScoringTarget>();
            for (Map.Entry<Integer, ScoringTarget> entry : this.scoringTargets.entrySet()) {
                c.scoringTargets.put(entry.getKey(), entry.getValue().copy());
            }
            c.aimToleranceDeg = this.aimToleranceDeg;
            c.aimKp = this.aimKp;
            c.aimMaxOmegaCmd = this.aimMaxOmegaCmd;
            c.aimReadyToleranceDeg = this.aimReadyToleranceDeg;
            c.aimReadyDebounceSec = this.aimReadyDebounceSec;
            c.aimMinOmegaCmd = this.aimMinOmegaCmd;
            c.selectionMaxAgeSec = this.selectionMaxAgeSec;
            c.selectionReacquireSec = this.selectionReacquireSec;
            c.defaultAimOffset = this.defaultAimOffset.copy();
            c.shotVelocityTable = this.shotVelocityTable;
            return c;
        }

        /**
         * Returns Phoenix's default scoring-target catalog.
         *
         * @return mutable linked hash map containing the checked-in scoring targets
         */
        public static LinkedHashMap<Integer, ScoringTarget> defaultScoringTargets() {
            LinkedHashMap<Integer, ScoringTarget> targets = new LinkedHashMap<Integer, ScoringTarget>();
            targets.put(20, new ScoringTarget(20, "Blue scoring target", new AimOffset(0.0, 0.0)));
            targets.put(24, new ScoringTarget(24, "Red scoring target", new AimOffset(0.0, 0.0)));
            return targets;
        }

        /**
         * Target-catalog entry describing one scoreable fixed tag.
         */
        public static final class ScoringTarget {
            public int tagId;
            public String label;
            public AimOffset aimOffset;

            /**
             * Creates a scoring-target profile.
             *
             * @param tagId     AprilTag id that identifies the scoring target
             * @param label     human-readable label for telemetry and documentation
             * @param aimOffset tag-local point offset used for auto-aim geometry
             */
            public ScoringTarget(int tagId, String label, AimOffset aimOffset) {
                this.tagId = tagId;
                this.label = label;
                this.aimOffset = aimOffset != null ? aimOffset : new AimOffset(0.0, 0.0);
            }

            /**
             * Creates a deep copy of this scoring-target profile.
             *
             * @return copied scoring-target profile
             */
            public ScoringTarget copy() {
                return new ScoringTarget(tagId, label, aimOffset.copy());
            }
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
