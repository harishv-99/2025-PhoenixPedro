package edu.ftcphoenix.robots.phoenix.tester;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.function.Function;

import edu.ftcphoenix.fw.ftc.localization.PinpointPoseEstimator;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLaneFactories;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLaneFactory;
import edu.ftcphoenix.fw.ftc.vision.FtcLimelightAprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.FtcWebcamAprilTagVisionLane;
import edu.ftcphoenix.fw.tools.tester.TeleOpTester;
import edu.ftcphoenix.fw.tools.tester.TesterSuite;
import edu.ftcphoenix.fw.tools.tester.calibration.CalibrationChecks;
import edu.ftcphoenix.fw.tools.tester.calibration.CalibrationStatus;
import edu.ftcphoenix.fw.tools.tester.calibration.CameraMountCalibrator;
import edu.ftcphoenix.fw.tools.tester.calibration.PinpointAxisDirectionTester;
import edu.ftcphoenix.fw.tools.tester.calibration.PinpointPodOffsetCalibrator;
import edu.ftcphoenix.fw.tools.tester.localization.AprilTagLocalizationTester;
import edu.ftcphoenix.fw.tools.tester.localization.PinpointAprilTagFusionLocalizationTester;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;

/**
 * Central home for Phoenix robot-specific tester wiring.
 *
 * <p>This class adapts Phoenix's checked-in profile into concrete framework testers while keeping
 * the higher-level tester menu robot-owned. All AprilTag-facing testers are wired through the active
 * Phoenix vision backend so the same calibration and localization flows work for either a webcam or
 * a Limelight-backed rig.</p>
 */
public final class PhoenixRobotTesters {

    private PhoenixRobotTesters() {
    }

    private static PhoenixProfile profile() {
        return PhoenixProfile.current();
    }

    private static String activeVisionBackendLabel(PhoenixProfile p) {
        return p.vision.backend == PhoenixProfile.VisionConfig.Backend.LIMELIGHT
                ? "Limelight"
                : "Webcam";
    }

    private static Class<? extends HardwareDevice> activeVisionDeviceType(PhoenixProfile p) {
        return p.vision.backend == PhoenixProfile.VisionConfig.Backend.LIMELIGHT
                ? Limelight3A.class
                : WebcamName.class;
    }

    private static String activeVisionPickerTitle(PhoenixProfile p) {
        return p.vision.backend == PhoenixProfile.VisionConfig.Backend.LIMELIGHT
                ? "Select Limelight"
                : "Select Camera";
    }

    private static Function<String, AprilTagVisionLaneFactory> activeVisionLaneFactoryBuilder(PhoenixProfile p) {
        switch (p.vision.backend) {
            case WEBCAM:
                return hardwareName -> {
                    FtcWebcamAprilTagVisionLane.Config cfg = p.vision.webcam.copy();
                    cfg.webcamName = hardwareName;
                    return AprilTagVisionLaneFactories.webcam(cfg);
                };
            case LIMELIGHT:
                return hardwareName -> {
                    FtcLimelightAprilTagVisionLane.Config cfg = p.vision.limelight.copy();
                    cfg.hardwareName = hardwareName;
                    return AprilTagVisionLaneFactories.limelight(cfg);
                };
            default:
                throw new IllegalArgumentException("Unsupported vision backend: " + p.vision.backend);
        }
    }

    /**
     * Registers the Phoenix-specific tester groups in the supplied suite.
     *
     * @param suite suite that should receive the Phoenix tester entries; ignored when {@code null}
     */
    public static void register(TesterSuite suite) {
        if (suite == null) return;

        suite.add(
                "Guide: Phoenix Calibration Walkthrough",
                "Recommended order for bringing Phoenix from fresh wiring to validated localization.",
                PhoenixCalibrationWalkthrough::createSuite
        );

        suite.add(
                "Phoenix: Calibration & Localization",
                "Phoenix-configured camera, Pinpoint, and localization tools for the active vision backend.",
                PhoenixRobotTesters::createCalibrationAndLocalizationSuite
        );

        suite.add(
                "Phoenix: Hardware Bring-up",
                "Robot-specific hardware sanity checks.",
                PhoenixRobotTesters::createHardwareSuite
        );
    }

    /**
     * Builds the Phoenix hardware bring-up submenu.
     *
     * @return tester suite containing robot-specific hardware sanity checks
     */
    public static TesterSuite createHardwareSuite() {
        TesterSuite suite = new TesterSuite()
                .setTitle("Phoenix Hardware Bring-up")
                .setHelp("Robot-specific hardware sanity checks.")
                .setMaxVisibleItems(8);

        suite.add(
                "HW: Drivetrain Motor Direction",
                "Hold X / Y / A / B to run FL / FR / BL / BR. Each should drive the robot forward.",
                PhoenixRobotTesters::drivetrainMotorDirection
        );

        return suite;
    }

    /**
     * Builds the Phoenix calibration and localization submenu.
     *
     * @return tester suite containing Phoenix-configured calibration and localization tools
     */
    public static TesterSuite createCalibrationAndLocalizationSuite() {
        PhoenixProfile p = profile();
        String backend = activeVisionBackendLabel(p);

        TesterSuite suite = new TesterSuite()
                .setTitle("Phoenix Calibration & Localization")
                .setHelp("Phoenix-configured bring-up and localization tools for the active vision backend.")
                .setMaxVisibleItems(8);

        CalibrationStatus mount = cameraMountStatus();
        CalibrationStatus axes = pinpointAxesStatus();
        CalibrationStatus offsets = pinpointOffsetsStatus();

        suite.add(
                "Calib: Camera Mount (Robot)",
                "Uses Phoenix's active " + backend + " AprilTag backend. Status: " + mount.summaryOrEmpty(),
                PhoenixRobotTesters::cameraMountCalibrator
        );

        suite.add(
                "Loc: AprilTag Localization (Robot)",
                "Uses Phoenix vision/localization defaults on the active " + backend + " backend. Status: " + mount.summaryOrEmpty(),
                PhoenixRobotTesters::aprilTagLocalization
        );

        suite.add(
                "Calib: Pinpoint Axis Check (Robot)",
                "Verify +X forward, +Y left, heading CCW+. Status: " + axes.summaryOrEmpty(),
                PhoenixRobotTesters::pinpointAxisCheck
        );

        suite.add(
                "Calib: Pinpoint Pod Offsets (Robot)",
                "Estimate pod offsets with Phoenix Pinpoint + drive config. AprilTag assist follows the active " + backend + " backend. Status: " + offsets.summaryOrEmpty(),
                PhoenixRobotTesters::pinpointPodOffsets
        );

        suite.add(
                "Loc: Pinpoint + AprilTag Fusion (Robot)",
                "Default Phoenix global localizer on the active " + backend + " backend. Status: " + globalLocalizationStatus().summaryOrEmpty(),
                PhoenixRobotTesters::pinpointAprilTagFusion
        );

        suite.add(
                "Loc: Pinpoint + AprilTag EKF (Robot)",
                "Optional comparison localizer after the default fusion path looks good. Status: "
                        + globalLocalizationStatus().summaryOrEmpty(),
                PhoenixRobotTesters::pinpointAprilTagEkf
        );

        return suite;
    }

    /**
     * Creates the Phoenix camera-mount calibration tester.
     *
     * @return tester configured to solve Phoenix's active camera mount pose
     */
    public static TeleOpTester cameraMountCalibrator() {
        PhoenixProfile p = profile();
        return new CameraMountCalibrator(
                p.vision.activeDeviceName(),
                activeVisionDeviceType(p),
                activeVisionPickerTitle(p),
                activeVisionLaneFactoryBuilder(p),
                null,
                0.35
        );
    }

    /**
     * Creates the Phoenix AprilTag-only localization tester.
     *
     * @return tester configured with Phoenix's active vision backend, field facts, and AprilTag-localizer defaults
     */
    public static TeleOpTester aprilTagLocalization() {
        PhoenixProfile p = profile();
        return new AprilTagLocalizationTester(
                p.vision.activeDeviceName(),
                activeVisionDeviceType(p),
                activeVisionPickerTitle(p),
                activeVisionLaneFactoryBuilder(p),
                p.field.fixedAprilTagLayout,
                p.localization.aprilTags.toTagOnlyPoseEstimatorConfig(p.vision.activeCameraMount()),
                p.localization.aprilTags.maxDetectionAgeSec
        );
    }

    /**
     * Creates the Pinpoint axis-direction tester using the current Phoenix profile.
     *
     * @return tester that verifies forward/left/heading sign conventions for Pinpoint
     */
    public static TeleOpTester pinpointAxisCheck() {
        PinpointAxisDirectionTester.Config cfg = PinpointAxisDirectionTester.Config.defaults();
        cfg.pinpoint = profile().localization.odometry.copy();
        return new PinpointAxisDirectionTester(cfg);
    }

    /**
     * Creates the Pinpoint pod-offset calibration tester using the current Phoenix profile.
     *
     * @return tester configured for Phoenix drivetrain wiring, odometry config, and optional
     *         AprilTag assist through Phoenix's active vision backend when the shared camera mount is trustworthy
     */
    public static TeleOpTester pinpointPodOffsets() {
        PhoenixProfile p = profile();
        PinpointPodOffsetCalibrator.Config cfg = PinpointPodOffsetCalibrator.Config.defaults();
        cfg.pinpoint = p.localization.odometry.copy();
        cfg.mecanumWiring = p.drive.wiring.copy();
        cfg.targetTurnRad = Math.PI;
        cfg.enableAprilTagAssist = CalibrationChecks.canUseAprilTagAssist(p.vision.activeCameraMount());
        cfg.autoComputeAfterAutoSample = true;
        cfg.preferredVisionDeviceName = p.vision.activeDeviceName();
        cfg.visionDeviceType = activeVisionDeviceType(p);
        cfg.visionPickerTitle = activeVisionPickerTitle(p);
        cfg.visionLaneFactoryBuilder = activeVisionLaneFactoryBuilder(p);
        cfg.cameraMount = p.vision.activeCameraMount();
        return new PinpointPodOffsetCalibrator(cfg);
    }

    /**
     * Creates the default Phoenix global-localization tester based on odometry and AprilTag fusion.
     *
     * @return tester configured with Phoenix's default fusion estimator settings
     */
    public static TeleOpTester pinpointAprilTagFusion() {
        PhoenixProfile p = profile();
        PinpointPoseEstimator.Config cfg = p.localization.odometry.copy();
        return new PinpointAprilTagFusionLocalizationTester(
                p.vision.activeDeviceName(),
                activeVisionDeviceType(p),
                activeVisionPickerTitle(p),
                activeVisionLaneFactoryBuilder(p),
                cfg,
                p.localization.odometryTagFusion.copy(),
                p.field.fixedAprilTagLayout,
                p.localization.aprilTags.toTagOnlyPoseEstimatorConfig(p.vision.activeCameraMount())
        );
    }

    /**
     * Creates the optional Phoenix EKF-based global-localization tester.
     *
     * @return tester configured to compare the optional EKF path against the default fusion path
     */
    public static TeleOpTester pinpointAprilTagEkf() {
        PhoenixProfile p = profile();
        PinpointPoseEstimator.Config cfg = p.localization.odometry.copy();
        return PinpointAprilTagFusionLocalizationTester.ekf(
                p.vision.activeDeviceName(),
                activeVisionDeviceType(p),
                activeVisionPickerTitle(p),
                activeVisionLaneFactoryBuilder(p),
                cfg,
                p.localization.odometryTagEkf.validatedCopy(
                        "PhoenixProfile.current().localization.odometryTagEkf"
                ),
                p.field.fixedAprilTagLayout,
                p.localization.aprilTags.toTagOnlyPoseEstimatorConfig(p.vision.activeCameraMount())
        );
    }

    /**
     * Creates the Phoenix drivetrain motor-direction tester.
     *
     * @return tester that runs one drivetrain motor at a time to verify direction wiring
     */
    public static TeleOpTester drivetrainMotorDirection() {
        return new DrivetrainMotorDirectionTester();
    }

    /**
     * Computes the current camera-mount calibration status.
     *
     * @return status indicating whether the configured Phoenix camera mount looks solved
     */
    public static CalibrationStatus cameraMountStatus() {
        return CalibrationChecks.cameraMount(profile().vision.activeCameraMount());
    }

    /**
     * Computes the current Pinpoint-axis verification status.
     *
     * @return status indicating whether Phoenix has acknowledged correct Pinpoint axis directions
     */
    public static CalibrationStatus pinpointAxesStatus() {
        return CalibrationChecks.pinpointAxes(profile().calibration.pinpointAxesVerified);
    }

    /**
     * Computes the current Pinpoint-offset calibration status.
     *
     * @return status indicating whether Phoenix has calibrated and acknowledged Pinpoint pod offsets
     */
    public static CalibrationStatus pinpointOffsetsStatus() {
        PhoenixProfile p = profile();
        return CalibrationChecks.pinpointOffsets(
                p.localization.odometry,
                p.calibration.pinpointPodOffsetsCalibrated
        );
    }

    /**
     * Computes whether the prerequisites for validating the global localizer are satisfied.
     *
     * @return complete status when camera mount, Pinpoint axes, and Pinpoint offsets all look ready
     */
    public static CalibrationStatus globalLocalizationStatus() {
        CalibrationStatus mount = cameraMountStatus();
        CalibrationStatus axes = pinpointAxesStatus();
        CalibrationStatus offsets = pinpointOffsetsStatus();

        if (mount.complete && axes.complete && offsets.complete) {
            return CalibrationStatus.complete("camera mount, Pinpoint axes, and Pinpoint offsets all look ready");
        }

        return CalibrationStatus.incomplete("finish prerequisites: " + prerequisiteSummary());
    }

    /**
     * Summarizes the prerequisite calibration statuses in one compact string.
     *
     * @return string of the form {@code mount=..., axes=..., offsets=...}
     */
    public static String prerequisiteSummary() {
        return "mount=" + cameraMountStatus().menuTag()
                + ", axes=" + pinpointAxesStatus().menuTag()
                + ", offsets=" + pinpointOffsetsStatus().menuTag();
    }
}
