package edu.ftcphoenix.robots.phoenix.tester;

import edu.ftcphoenix.fw.ftc.localization.PinpointPoseEstimator;
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
 */
public final class PhoenixRobotTesters {

    private PhoenixRobotTesters() {
    }

    private static PhoenixProfile profile() {
        return PhoenixProfile.current();
    }

    private static PhoenixProfile webcamProfile() {
        PhoenixProfile p = profile();
        if (p.vision.backend != PhoenixProfile.VisionConfig.Backend.WEBCAM) {
            throw new IllegalStateException(
                    "Phoenix's Stage 1 tester flow is webcam-backed. "
                            + "Set PhoenixProfile.vision.backend = WEBCAM or wait for a later Limelight stage."
            );
        }
        return p;
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
                "Phoenix-configured camera, Pinpoint, and localization tools.",
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
        TesterSuite suite = new TesterSuite()
                .setTitle("Phoenix Calibration & Localization")
                .setHelp("Phoenix-configured bring-up and localization tools.")
                .setMaxVisibleItems(8);

        CalibrationStatus mount = cameraMountStatus();
        CalibrationStatus axes = pinpointAxesStatus();
        CalibrationStatus offsets = pinpointOffsetsStatus();

        suite.add(
                "Calib: Camera Mount (Robot)",
                "Uses Phoenix's preferred camera. Status: " + mount.summaryOrEmpty(),
                PhoenixRobotTesters::cameraMountCalibrator
        );

        suite.add(
                "Loc: AprilTag Localization (Robot)",
                "Uses Phoenix vision/localization defaults. Status: " + mount.summaryOrEmpty(),
                PhoenixRobotTesters::aprilTagLocalization
        );

        suite.add(
                "Calib: Pinpoint Axis Check (Robot)",
                "Verify +X forward, +Y left, heading CCW+. Status: " + axes.summaryOrEmpty(),
                PhoenixRobotTesters::pinpointAxisCheck
        );

        suite.add(
                "Calib: Pinpoint Pod Offsets (Robot)",
                "Estimate pod offsets with Phoenix Pinpoint + drive config. Status: " + offsets.summaryOrEmpty(),
                PhoenixRobotTesters::pinpointPodOffsets
        );

        suite.add(
                "Loc: Pinpoint + AprilTag Fusion (Robot)",
                "Default Phoenix global localizer. Status: " + globalLocalizationStatus().summaryOrEmpty(),
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
     * @return tester configured to solve Phoenix's webcam mount pose
     */
    public static TeleOpTester cameraMountCalibrator() {
        return new CameraMountCalibrator(webcamProfile().vision.webcam.webcamName);
    }

    /**
     * Creates the Phoenix AprilTag-only localization tester.
     *
     * @return tester configured with Phoenix's vision lane, field facts, and AprilTag-localizer defaults
     */
    public static TeleOpTester aprilTagLocalization() {
        PhoenixProfile p = webcamProfile();
        return new AprilTagLocalizationTester(
                p.vision.webcam.webcamName,
                p.vision.webcam.cameraMount,
                p.field.fixedAprilTagLayout,
                null,
                p.localization.aprilTags.toTagOnlyPoseEstimatorConfig(p.vision.webcam.cameraMount),
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
     *         AprilTag assist when the shared webcam mount is already trustworthy
     */
    public static TeleOpTester pinpointPodOffsets() {
        PhoenixProfile p = webcamProfile();
        PinpointPodOffsetCalibrator.Config cfg = PinpointPodOffsetCalibrator.Config.defaults();
        cfg.pinpoint = p.localization.odometry.copy();
        cfg.mecanumWiring = p.drive.wiring.copy();
        cfg.targetTurnRad = Math.PI;
        cfg.enableAprilTagAssist = CalibrationChecks.canUseAprilTagAssist(p.vision.webcam.cameraMount);
        cfg.autoComputeAfterAutoSample = true;
        cfg.preferredCameraName = p.vision.webcam.webcamName;
        cfg.cameraMount = p.vision.webcam.cameraMount;
        return new PinpointPodOffsetCalibrator(cfg);
    }

    /**
     * Creates the default Phoenix global-localization tester based on odometry and AprilTag fusion.
     *
     * @return tester configured with Phoenix's default fusion estimator settings
     */
    public static TeleOpTester pinpointAprilTagFusion() {
        PhoenixProfile p = webcamProfile();
        PinpointPoseEstimator.Config cfg = p.localization.odometry.copy();
        return new PinpointAprilTagFusionLocalizationTester(
                p.vision.webcam.webcamName,
                p.vision.webcam.cameraMount,
                cfg,
                p.localization.odometryTagFusion.copy(),
                p.field.fixedAprilTagLayout,
                null,
                p.localization.aprilTags.toTagOnlyPoseEstimatorConfig(p.vision.webcam.cameraMount)
        );
    }

    /**
     * Creates the optional Phoenix EKF-based global-localization tester.
     *
     * @return tester configured to compare the optional EKF path against the default fusion path
     */
    public static TeleOpTester pinpointAprilTagEkf() {
        PhoenixProfile p = webcamProfile();
        PinpointPoseEstimator.Config cfg = p.localization.odometry.copy();
        return PinpointAprilTagFusionLocalizationTester.ekf(
                p.vision.webcam.webcamName,
                p.vision.webcam.cameraMount,
                cfg,
                p.localization.odometryTagEkf.validatedCopy(
                        "PhoenixProfile.current().localization.odometryTagEkf"
                ),
                p.field.fixedAprilTagLayout,
                null,
                p.localization.aprilTags.toTagOnlyPoseEstimatorConfig(p.vision.webcam.cameraMount)
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
        return CalibrationChecks.cameraMount(webcamProfile().vision.webcam.cameraMount);
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
