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
 *   <li>robot-owned mechanisms and strategy: shooter, targeting, and calibration acknowledgements</li>
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
     *
     * <p>The checked-in backend is a webcam-backed framework lane today, but Phoenix keeps the
     * wrapper robot-owned so future smart-camera backends can slot in without reshaping the rest
     * of the robot container.</p>
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

    /** TeleOp control-layer tuning owned by Phoenix's robot-specific controls object. */
    public TeleOpControlsConfig controls = new TeleOpControlsConfig();

    /**
     * Robot-specific drive-assist tuning layered on top of the stable framework drive lane.
     */
    public DriveAssistConfig driveAssist = new DriveAssistConfig();

    /** Shooter hardware + scoring-path tuning. */
    public ShooterConfig shooter = new ShooterConfig();

    /** Human-acknowledged calibration checkpoints. */
    public CalibrationConfig calibration = new CalibrationConfig();

    /** Auto-aim / selected-tag tuning. */
    public AutoAimConfig autoAim = new AutoAimConfig();

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
        copy.shooter = this.shooter.copy();
        copy.calibration = this.calibration.copy();
        copy.autoAim = this.autoAim.copy();
        return copy;
    }


    /**
     * Phoenix-owned AprilTag backend-selection config.
     *
     * <p>
     * The framework owns concrete FTC-boundary lane implementations such as
     * {@link FtcWebcamAprilTagVisionLane}. Phoenix owns the higher-level choice of which backend
     * to instantiate. That keeps backend selection in the robot layer while the rest of Phoenix
     * consumes only the backend-neutral lane seam.
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
             * Reserved for a later stage of Limelight integration.
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
         * Future smart-camera mount/config slot reserved for later Limelight integration.
         *
         * <p>
         * Stage 1 does not yet instantiate a Limelight lane, but keeping this config slot in the
         * robot profile now avoids another profile reshape when that backend lands.
         * </p>
         */
        public LimelightConfig limelight = LimelightConfig.defaults();

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

        /**
         * Placeholder config for a future Limelight-backed AprilTag lane.
         */
        public static final class LimelightConfig {

            /**
             * Preferred FTC hardware-map name for the Limelight device.
             */
            public String hardwareName = "limelight";

            /**
             * Camera extrinsics expressed in the robot frame.
             */
            public CameraMountConfig cameraMount = CameraMountConfig.identity();

            private LimelightConfig() {
                // Defaults assigned in field initializers.
            }

            /**
             * Creates a config populated with Phoenix defaults.
             *
             * @return new mutable config instance
             */
            public static LimelightConfig defaults() {
                return new LimelightConfig();
            }

            /**
             * Creates a deep copy of this config.
             *
             * @return copied config whose fields can be edited independently
             */
            public LimelightConfig copy() {
                LimelightConfig c = new LimelightConfig();
                c.hardwareName = this.hardwareName;
                c.cameraMount = this.cameraMount;
                return c;
            }
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

        /** Drive-stick shaping and slow-mode tuning for the manual drive source. */
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
        cfg.limelight = VisionConfig.LimelightConfig.defaults();
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

    private static FtcOdometryAprilTagLocalizationLane.Config defaultLocalizationConfig() {
        FtcOdometryAprilTagLocalizationLane.Config cfg = FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.globalEstimatorMode = FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.FUSION;

        cfg.odometry = cfg.odometry
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

        cfg.odometryTagFusion.maxVisionAgeSec = 0.35;
        cfg.odometryTagFusion.minVisionQuality = 0.10;
        cfg.odometryTagFusion.visionPositionGain = 0.25;
        cfg.odometryTagFusion.visionHeadingGain = 0.35;
        cfg.odometryTagFusion.maxVisionPositionJumpIn = 24.0;
        cfg.odometryTagFusion.maxVisionHeadingJumpRad = Math.toRadians(60.0);
        cfg.odometryTagFusion.enableLatencyCompensation = true;
        cfg.odometryTagFusion.odomHistorySec = 1.0;

        cfg.odometryTagEkf.maxVisionAgeSec = 0.35;
        cfg.odometryTagEkf.minVisionQuality = 0.10;
        cfg.odometryTagEkf.maxVisionPositionInnovationIn = 24.0;
        cfg.odometryTagEkf.maxVisionHeadingInnovationRad = Math.toRadians(60.0);
        cfg.odometryTagEkf.maxVisionMahalanobisSq = 14.0;
        cfg.odometryTagEkf.enableLatencyCompensation = true;
        cfg.odometryTagEkf.odomHistorySec = 1.0;
        cfg.odometryTagEkf.initialPositionStdIn = 6.0;
        cfg.odometryTagEkf.initialHeadingStdRad = Math.toRadians(12.0);
        cfg.odometryTagEkf.manualPosePositionStdIn = 3.0;
        cfg.odometryTagEkf.manualPoseHeadingStdRad = Math.toRadians(6.0);
        cfg.odometryTagEkf.odomProcessPositionStdFloorIn = 0.20;
        cfg.odometryTagEkf.odomProcessPositionStdPerIn = 0.03;
        cfg.odometryTagEkf.odomProcessPositionStdPerRad = 0.55;
        cfg.odometryTagEkf.odomProcessHeadingStdFloorRad = Math.toRadians(0.35);
        cfg.odometryTagEkf.odomProcessHeadingStdPerIn = Math.toRadians(0.06);
        cfg.odometryTagEkf.odomProcessHeadingStdPerRad = 0.06;
        cfg.odometryTagEkf.visionPositionStdFloorIn = 1.75;
        cfg.odometryTagEkf.visionPositionStdScaleIn = 6.0;
        cfg.odometryTagEkf.visionHeadingStdFloorRad = Math.toRadians(3.0);
        cfg.odometryTagEkf.visionHeadingStdScaleRad = Math.toRadians(10.0);
        cfg.odometryTagEkf.qualityPositionStdScaleIn = 18.0;
        cfg.odometryTagEkf.qualityHeadingStdScaleRad = Math.toRadians(30.0);

        return cfg;
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

        /** Default tag-local offset used when an unknown tag id is queried. */
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
         *         match one of the explicit scoring targets
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
         * Builds the default range-to-velocity model for the targeting layer.
         *
         * @return new interpolating shot-velocity model backed by {@link #shotVelocityTable}
         */
        public ShotVelocityModel shotVelocityModel() {
            return new InterpolatingShotVelocityModel(shotVelocityTable);
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
             * @param tagId AprilTag id that identifies the scoring target
             * @param label human-readable label for telemetry and documentation
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
             * @param leftInches left offset from the tag frame, in inches
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
