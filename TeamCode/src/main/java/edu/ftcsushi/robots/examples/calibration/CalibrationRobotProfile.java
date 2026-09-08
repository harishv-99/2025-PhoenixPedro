package edu.ftcsushi.robots.examples.calibration;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.field.TagLayout;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcGameTagLayout;
import edu.ftcsushi.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcsushi.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane.AprilTagLocalizationConfig;
import edu.ftcsushi.fw.ftc.localization.PinpointOdometryPredictor;
import edu.ftcsushi.fw.ftc.vision.FtcLimelightVisionLane;
import edu.ftcsushi.fw.ftc.vision.FtcWebcamVisionLane;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;

/**
 * Canonical data-only facts for the independent calibration teaching robot.
 *
 * <p>Replace these illustrative values with the adopting robot's reviewed facts in its existing
 * configuration home; do not maintain a second calibration-only copy beside match settings.
 * Nothing here opens hardware. Each recipe returns a fresh mutable Config, and optional camera
 * and drive Configs are built only when their recipe is called. Numeric defaults, acknowledgements,
 * and successful construction are not measurements or physical safety evidence.</p>
 *
 * <p>After accepting a result, edit the canonical facts, rebuild, and create a fresh tester. Neither
 * a running tester nor a previously built suite reloads changes to this authoring draft.</p>
 */
public final class CalibrationRobotProfile {
    /** The one installed camera backend used by optional vision checks. */
    public enum CameraBackend { NONE, WEBCAM, LIMELIGHT }

    /** NONE keeps the default suite independent of camera configuration. */
    public CameraBackend cameraBackend = CameraBackend.NONE;
    /** Explicit review of wiring, commands, clearance, controls, and the physical stop plan. */
    public boolean poweredMotionReviewed = false;
    /** Human acceptance for this selected camera and mount; reconsider after either changes. */
    public boolean cameraMountAccepted = false;
    /** Human acknowledgement after the rebuilt configured axes pass, not a tester-written flag. */
    public boolean pinpointAxesVerified = false;
    /** Human acknowledgement after the rebuilt configured offsets pass. */
    public boolean pinpointOffsetsVerified = false;

    /** Exact FTC Pinpoint hardware name. */
    public String pinpointHardwareName = "odo";
    /** Forward pod's left offset from robot origin, in inches; zero is a placeholder. */
    public double forwardPodOffsetLeftInches = 0.0;
    /** Strafe pod's forward offset from robot origin, in inches; zero is a placeholder. */
    public double strafePodOffsetForwardInches = 0.0;
    /** Installed encoder resolution; verify that the named pod preset matches the hardware. */
    public PinpointOdometryPredictor.EncoderResolution encoderResolution =
            PinpointOdometryPredictor.EncoderResolution.forGoBildaPod(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    /** Configured forward-pod sign; the axis tester recommends keeping or changing this value. */
    public GoBildaPinpointDriver.EncoderDirection forwardPodDirection =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    /** Configured strafe-pod sign. */
    public GoBildaPinpointDriver.EncoderDirection strafePodDirection =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    /** Null retains factory yaw calibration; a numeric override must be positive. */
    public Double yawScalar = null;
    /** Authored odometry quality in [0, 1], not measured accuracy. */
    public double odometryQuality = 0.75;

    /** Configured webcam name, used only by the WEBCAM branch. */
    public String webcamName = "Webcam 1";
    /** Configured Limelight name, used only by the LIMELIGHT branch. */
    public String limelightName = "limelight";
    /** Robot-to-selected-camera geometry; identity is an unaccepted teaching placeholder. */
    public CameraMountConfig cameraMount = CameraMountConfig.identity();
    /** Shared maximum AprilTag detection age in seconds. */
    public double maxDetectionAgeSec = 0.35;
    /** Shared multi-tag positional outlier gate, in inches. */
    public double tagOutlierPositionGateInches = 18.0;
    /** Shared multi-tag heading outlier gate, in radians. */
    public double tagOutlierHeadingGateRad = Math.toRadians(25.0);
    /** AprilTag pipeline that must also be configured on the installed Limelight. */
    public int limelightPipelineIndex = 0;
    /** Requested Limelight polling rate, in hertz. */
    public int limelightPollRateHz = 100;
    /** Limelight transport-readiness result age in seconds, separate from solver acceptance. */
    public double limelightMaxResultAgeSec = 0.25;

    /** Exact configured front-left drive motor name. */
    public String frontLeftMotorName = "frontLeftMotor";
    /** Exact configured front-right drive motor name. */
    public String frontRightMotorName = "frontRightMotor";
    /** Exact configured back-left drive motor name. */
    public String backLeftMotorName = "backLeftMotor";
    /** Exact configured back-right drive motor name. */
    public String backRightMotorName = "backRightMotor";
    /** Reviewed front-left logical direction; this initial value is illustrative. */
    public Direction frontLeftMotorDirection = Direction.FORWARD;
    /** Reviewed front-right logical direction; this initial value is illustrative. */
    public Direction frontRightMotorDirection = Direction.REVERSE;
    /** Reviewed back-left logical direction; this initial value is illustrative. */
    public Direction backLeftMotorDirection = Direction.FORWARD;
    /** Reviewed back-right logical direction; this initial value is illustrative. */
    public Direction backRightMotorDirection = Direction.REVERSE;
    /** Reviewed zero-power braking policy. */
    public boolean enableZeroPowerBrake = true;
    /** Normalized right-stick turn scale during a powered manual sample. */
    public double manualOmegaScale = 0.20;
    /** Normalized automatic-turn command magnitude, not measured angular speed. */
    public double autoOmegaCmd = 0.20;
    /** Automatic sample target in radians; positive is counterclockwise. */
    public double targetTurnRad = Math.PI;
    /** Cooperative automatic-phase timeout in seconds; not a hardware watchdog. */
    public double automaticPhaseTimeoutSec = 10.0;
    /** Normalized left-stick translation scale during powered recentering. */
    public double recenterTranslationScale = 0.20;

    private CalibrationRobotProfile() { }

    /** Returns a fresh draft of this example's checked-in canonical facts. */
    public static CalibrationRobotProfile current() {
        return new CalibrationRobotProfile();
    }

    /** Returns the same canonical Pinpoint facts for a fresh diagnostic or adopting robot owner. */
    public PinpointOdometryPredictor.Config pinpoint() {
        PinpointOdometryPredictor.Config cfg = PinpointOdometryPredictor.Config.defaults();
        cfg.hardwareMapName = pinpointHardwareName;
        cfg.forwardPodOffsetLeftInches = forwardPodOffsetLeftInches;
        cfg.strafePodOffsetForwardInches = strafePodOffsetForwardInches;
        cfg.encoderResolution = encoderResolution;
        cfg.forwardPodDirection = forwardPodDirection;
        cfg.strafePodDirection = strafePodDirection;
        cfg.yawScalar = yawScalar;
        cfg.quality = odometryQuality;
        return cfg;
    }

    /**
     * Returns trusted current-game fixed-tag facts, not a camera or localization owner.
     * For a custom field, replace this recipe and the webcam detector library together.
     */
    public TagLayout fixedTagLayout() {
        return FtcGameTagLayout.currentGameFieldFixed();
    }

    /** Returns fresh mount-free age and solver policy shared by every configured tag consumer. */
    public AprilTagLocalizationConfig aprilTags() {
        AprilTagLocalizationConfig cfg = AprilTagLocalizationConfig.defaults();
        cfg.maxDetectionAgeSec = maxDetectionAgeSec;
        cfg.fieldPoseSolver.outlierPositionGateInches = tagOutlierPositionGateInches;
        cfg.fieldPoseSolver.outlierHeadingGateRad = tagOutlierHeadingGateRad;
        return cfg;
    }

    /** Returns a tag-enabled webcam draft; the maintained software resolution is 640 by 480. */
    public FtcWebcamVisionLane.Config webcam() {
        FtcWebcamVisionLane.Config cfg = FtcWebcamVisionLane.Config.defaults();
        cfg.webcamName = webcamName;
        cfg.cameraMount = cameraMount;
        cfg.aprilTags = FtcWebcamVisionLane.AprilTagConfig.defaults();
        return cfg;
    }

    /** Returns a tag-enabled Limelight draft with one explicit AprilTag pipeline. */
    public FtcLimelightVisionLane.Config limelight() {
        FtcLimelightVisionLane.Config cfg = FtcLimelightVisionLane.Config.defaults();
        cfg.hardwareName = limelightName;
        cfg.cameraMount = cameraMount;
        cfg.pipelineIndex = limelightPipelineIndex;
        cfg.pollRateHz = limelightPollRateHz;
        cfg.maxResultAgeSec = limelightMaxResultAgeSec;
        cfg.aprilTags = FtcLimelightVisionLane.AprilTagConfig.defaults();
        cfg.aprilTags.pipelineIndex = limelightPipelineIndex;
        return cfg;
    }

    /**
     * Returns fresh ordinary Pinpoint plus raw-AprilTag fusion policy, never a live graph.
     * The shared camera owner supplies the mount separately; no second mount is authored here.
     */
    public FtcOdometryAprilTagLocalizationLane.Config localization() {
        FtcOdometryAprilTagLocalizationLane.Config cfg =
                FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.predictor = pinpoint();
        cfg.estimation.aprilTags = aprilTags();
        cfg.estimation.correctedEstimatorMode =
                FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.FUSION;
        cfg.estimation.correctionSource.mode =
                FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.APRILTAG_POSE;
        return cfg;
    }

    /**
     * Returns a fresh complete drive draft, without authorizing or acquiring a powered owner.
     * Mixer scales remain 1.0; the separate tester command scales are the visible motion limits.
     */
    public FtcDrives.MecanumConfig mecanum() {
        FtcDrives.MecanumConfig cfg = FtcDrives.MecanumConfig.defaults();
        cfg.wiring.frontLeftName = frontLeftMotorName;
        cfg.wiring.frontRightName = frontRightMotorName;
        cfg.wiring.backLeftName = backLeftMotorName;
        cfg.wiring.backRightName = backRightMotorName;
        cfg.wiring.frontLeftDirection = frontLeftMotorDirection;
        cfg.wiring.frontRightDirection = frontRightMotorDirection;
        cfg.wiring.backLeftDirection = backLeftMotorDirection;
        cfg.wiring.backRightDirection = backRightMotorDirection;
        cfg.enableZeroPowerBrake = enableZeroPowerBrake;
        return cfg;
    }

    /** Raw registration snapshot: copies facts without constructing or validating dormant branches. */
    CalibrationRobotProfile copy() {
        CalibrationRobotProfile copy = new CalibrationRobotProfile();
        copy.cameraBackend = cameraBackend;
        copy.poweredMotionReviewed = poweredMotionReviewed;
        copy.cameraMountAccepted = cameraMountAccepted;
        copy.pinpointAxesVerified = pinpointAxesVerified;
        copy.pinpointOffsetsVerified = pinpointOffsetsVerified;
        copy.pinpointHardwareName = pinpointHardwareName;
        copy.forwardPodOffsetLeftInches = forwardPodOffsetLeftInches;
        copy.strafePodOffsetForwardInches = strafePodOffsetForwardInches;
        copy.encoderResolution = encoderResolution;
        copy.forwardPodDirection = forwardPodDirection;
        copy.strafePodDirection = strafePodDirection;
        copy.yawScalar = yawScalar;
        copy.odometryQuality = odometryQuality;
        copy.webcamName = webcamName;
        copy.limelightName = limelightName;
        copy.cameraMount = cameraMount;
        copy.maxDetectionAgeSec = maxDetectionAgeSec;
        copy.tagOutlierPositionGateInches = tagOutlierPositionGateInches;
        copy.tagOutlierHeadingGateRad = tagOutlierHeadingGateRad;
        copy.limelightPipelineIndex = limelightPipelineIndex;
        copy.limelightPollRateHz = limelightPollRateHz;
        copy.limelightMaxResultAgeSec = limelightMaxResultAgeSec;
        copy.frontLeftMotorName = frontLeftMotorName;
        copy.frontRightMotorName = frontRightMotorName;
        copy.backLeftMotorName = backLeftMotorName;
        copy.backRightMotorName = backRightMotorName;
        copy.frontLeftMotorDirection = frontLeftMotorDirection;
        copy.frontRightMotorDirection = frontRightMotorDirection;
        copy.backLeftMotorDirection = backLeftMotorDirection;
        copy.backRightMotorDirection = backRightMotorDirection;
        copy.enableZeroPowerBrake = enableZeroPowerBrake;
        copy.manualOmegaScale = manualOmegaScale;
        copy.autoOmegaCmd = autoOmegaCmd;
        copy.targetTurnRad = targetTurnRad;
        copy.automaticPhaseTimeoutSec = automaticPhaseTimeoutSec;
        copy.recenterTranslationScale = recenterTranslationScale;
        return copy;
    }
}
