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
import edu.ftcphoenix.robots.phoenix.RobotConfig;

/**
 * Central home for Phoenix robot-specific tester wiring.
 *
 * <p>The goal is to keep the robot-specific layer thin: the framework owns the tester logic, while
 * this class only supplies Phoenix's config objects, preferred camera names, and explicit
 * calibration-status flags.</p>
 */
public final class PhoenixRobotTesters {

    private PhoenixRobotTesters() {
    }

    /**
     * Adds Phoenix tester groups to the supplied top-level suite.
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
     * Creates the Phoenix robot-specific hardware suite.
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
     * Creates the Phoenix robot-specific calibration/localization suite.
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
     * Framework tester with Phoenix's preferred camera pre-selected.
     */
    public static TeleOpTester cameraMountCalibrator() {
        return new CameraMountCalibrator(RobotConfig.Vision.nameWebcam);
    }

    /**
     * Framework tester configured with Phoenix vision defaults.
     */
    public static TeleOpTester aprilTagLocalization() {
        return new AprilTagLocalizationTester(
                RobotConfig.Vision.nameWebcam,
                RobotConfig.Vision.cameraMount,
                null,
                null,
                RobotConfig.Localization.aprilTags.copy(),
                RobotConfig.Localization.aprilTags.maxDetectionAgeSec
        );
    }

    /**
     * Framework tester configured with Phoenix Pinpoint defaults.
     */
    public static TeleOpTester pinpointAxisCheck() {
        PinpointAxisDirectionTester.Config cfg = PinpointAxisDirectionTester.Config.defaults();
        cfg.pinpoint = RobotConfig.Localization.pinpoint;
        return new PinpointAxisDirectionTester(cfg);
    }

    /**
     * Framework tester configured with Phoenix Pinpoint, drivetrain, and optional AprilTag assist.
     */
    public static TeleOpTester pinpointPodOffsets() {
        PinpointPodOffsetCalibrator.Config cfg = PinpointPodOffsetCalibrator.Config.defaults();
        cfg.pinpoint = RobotConfig.Localization.pinpoint;
        cfg.mecanumWiring = RobotConfig.DriveTrain.mecanumWiring();

        // IMPORTANT: don't use a full 360° turn here.
        // After a full turn, the ideal drift is ~0 so the solve becomes ill-conditioned and very
        // sensitive to noise. ~180° provides a much stronger signal.
        cfg.targetTurnRad = Math.PI;

        cfg.enableAprilTagAssist = CalibrationChecks.canUseAprilTagAssist(RobotConfig.Vision.cameraMount);
        cfg.autoComputeAfterAutoSample = true;
        cfg.preferredCameraName = RobotConfig.Vision.nameWebcam;
        cfg.cameraMount = RobotConfig.Vision.cameraMount;

        return new PinpointPodOffsetCalibrator(cfg);
    }

    /**
     * Phoenix's default lightweight fusion localizer tester.
     */
    public static TeleOpTester pinpointAprilTagFusion() {
        PinpointPoseEstimator.Config cfg = RobotConfig.Localization.pinpoint;

        return new PinpointAprilTagFusionLocalizationTester(
                RobotConfig.Vision.nameWebcam,
                RobotConfig.Vision.cameraMount,
                cfg,
                RobotConfig.Localization.pinpointAprilTagFusion.copy(),
                null,
                null,
                RobotConfig.Localization.aprilTags.copy()
        );
    }

    /**
     * Phoenix's optional EKF-style localizer tester.
     */
    public static TeleOpTester pinpointAprilTagEkf() {
        PinpointPoseEstimator.Config cfg = RobotConfig.Localization.pinpoint;
        return PinpointAprilTagFusionLocalizationTester.ekf(
                RobotConfig.Vision.nameWebcam,
                RobotConfig.Vision.cameraMount,
                cfg,
                RobotConfig.Localization.pinpointAprilTagEkf.validatedCopy(
                        "RobotConfig.Localization.pinpointAprilTagEkf"
                ),
                null,
                null,
                RobotConfig.Localization.aprilTags.copy()
        );
    }

    /**
     * Phoenix-specific drivetrain direction sanity-check tester.
     */
    public static TeleOpTester drivetrainMotorDirection() {
        return new DrivetrainMotorDirectionTester();
    }

    /**
     * Camera-mount status derived from the current Phoenix config.
     */
    public static CalibrationStatus cameraMountStatus() {
        return CalibrationChecks.cameraMount(RobotConfig.Vision.cameraMount);
    }

    /**
     * Pinpoint-axis status derived from the current Phoenix config.
     */
    public static CalibrationStatus pinpointAxesStatus() {
        return CalibrationChecks.pinpointAxes(RobotConfig.Calibration.pinpointAxesVerified);
    }

    /**
     * Pinpoint pod-offset status derived from the current Phoenix config.
     */
    public static CalibrationStatus pinpointOffsetsStatus() {
        return CalibrationChecks.pinpointOffsets(
                RobotConfig.Localization.pinpoint,
                RobotConfig.Calibration.pinpointPodOffsetsCalibrated
        );
    }

    /**
     * Readiness for running the global-localization comparison testers.
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
     * Short prerequisite summary for localizer-comparison testers.
     */
    public static String prerequisiteSummary() {
        return "mount=" + cameraMountStatus().menuTag()
                + ", axes=" + pinpointAxesStatus().menuTag()
                + ", offsets=" + pinpointOffsetsStatus().menuTag();
    }
}
