package edu.ftcsushi.robots.phoenix.tester;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.function.Function;

import edu.ftcsushi.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane;
import edu.ftcsushi.fw.ftc.vision.AprilTagVisionLaneFactories;
import edu.ftcsushi.fw.ftc.vision.AprilTagVisionLaneFactory;
import edu.ftcsushi.fw.ftc.vision.FtcLimelightAprilTagVisionLane;
import edu.ftcsushi.fw.ftc.vision.FtcWebcamAprilTagVisionLane;
import edu.ftcsushi.fw.tools.tester.TeleOpTester;
import edu.ftcsushi.fw.tools.tester.TesterSuite;
import edu.ftcsushi.fw.tools.tester.calibration.CalibrationChecks;
import edu.ftcsushi.fw.tools.tester.calibration.CalibrationStatus;
import edu.ftcsushi.fw.tools.tester.calibration.CameraMountCalibrator;
import edu.ftcsushi.fw.tools.tester.calibration.PinpointAxisDirectionTester;
import edu.ftcsushi.fw.tools.tester.calibration.PinpointPodOffsetCalibrator;
import edu.ftcsushi.fw.tools.tester.localization.AprilTagLocalizationTester;
import edu.ftcsushi.fw.tools.tester.localization.PinpointAprilTagCorrectedLocalizationTester;
import edu.ftcsushi.robots.phoenix.PhoenixProfile;
import edu.ftcsushi.robots.phoenix.PhoenixVisionFactory;

/**
 * Central home for Phoenix robot-specific tester wiring.
 *
 * <p>This class adapts Phoenix's checked-in profile into concrete framework testers while keeping
 * the higher-level tester menu robot-owned. All AprilTag-facing testers are wired through the active
 * Phoenix vision backend so the same calibration and localization flows work for either a webcam or
 * a Limelight-backed rig.</p>
 *
 * <p>Each factory maps the current profile into one fresh tool-owned Config and supplies backend
 * behavior separately through an {@link AprilTagVisionLaneFactory} builder. The builder captures the
 * selected backend template immediately instead of rereading the broad mutable profile on a later
 * picker retry; a borrowed custom SDK tag library must therefore remain stable for the tester's
 * complete lifetime.</p>
 */
public final class PhoenixRobotTesters {

    private PhoenixRobotTesters() {
    }

    private static PhoenixProfile profile() {
        return PhoenixProfile.current();
    }

    private static String activeVisionBackendLabel(PhoenixProfile p) {
        return p.vision.backend == PhoenixVisionFactory.Backend.LIMELIGHT
                ? "Limelight"
                : "Webcam";
    }

    private static Class<? extends HardwareDevice> activeVisionDeviceType(PhoenixProfile p) {
        return p.vision.backend == PhoenixVisionFactory.Backend.LIMELIGHT
                ? Limelight3A.class
                : WebcamName.class;
    }

    private static String activeVisionPickerTitle(PhoenixProfile p) {
        return p.vision.backend == PhoenixVisionFactory.Backend.LIMELIGHT
                ? "Select Limelight"
                : "Select Camera";
    }

    private static Function<String, AprilTagVisionLaneFactory> activeVisionLaneFactoryBuilder(PhoenixProfile p) {
        switch (p.vision.backend) {
            case WEBCAM: {
                final FtcWebcamAprilTagVisionLane.Config template = p.vision.webcam.copy();
                return hardwareName -> {
                    FtcWebcamAprilTagVisionLane.Config cfg = template.copy();
                    cfg.webcamName = hardwareName;
                    return AprilTagVisionLaneFactories.webcam(cfg);
                };
            }
            case LIMELIGHT: {
                final FtcLimelightAprilTagVisionLane.Config template = p.vision.limelight.copy();
                return hardwareName -> {
                    FtcLimelightAprilTagVisionLane.Config cfg = template.copy();
                    cfg.hardwareName = hardwareName;
                    return AprilTagVisionLaneFactories.limelight(cfg);
                };
            }
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
                "Recommended Phoenix sequence, beginning with canonical actuator bring-up.",
                PhoenixCalibrationWalkthrough::createSuite
        );

        suite.add(
                "Phoenix: Calibration & Localization",
                "Phoenix-configured camera, Pinpoint, and localization tools for the active vision backend.",
                PhoenixRobotTesters::createCalibrationAndLocalizationSuite
        );

        suite.add(
                "Phoenix: Configured Hardware Verification",
                "Robot-specific integration checks after canonical actuator bring-up.",
                PhoenixRobotTesters::createConfiguredHardwareVerificationSuite
        );
    }

    /**
     * Builds the Phoenix configured-hardware verification submenu.
     *
     * @return tester suite containing robot-specific configuration checks
     */
    public static TesterSuite createConfiguredHardwareVerificationSuite() {
        TesterSuite suite = new TesterSuite()
                .setTitle("Phoenix Configured Hardware Verification")
                .setHelp("Verify Phoenix configuration after raw actuator facts are established.")
                .setMaxVisibleItems(8);

        suite.add(
                "HW: Configured Drivetrain Verification",
                "After actuator bring-up, verify Phoenix's configured FL / FR / BL / BR one raised wheel at a time.",
                PhoenixRobotTesters::configuredDrivetrainVerification
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

        CalibrationStatus mount = cameraMountStatus(p);
        CalibrationStatus axes = pinpointAxesStatus(p);
        CalibrationStatus offsets = pinpointOffsetsStatus(p);
        CalibrationStatus global = globalLocalizationStatus(p);

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
                "Loc: Pinpoint + Field Corrections (Robot)",
                "Default Phoenix corrected-global localizer on the active " + backend + " backend. Status: " + global.summaryOrEmpty(),
                PhoenixRobotTesters::pinpointAprilTagFusion
        );

        suite.add(
                "Loc: Pinpoint + Field Corrections EKF (Robot)",
                "Optional comparison corrected localizer after the default fusion path looks good. Status: "
                        + global.summaryOrEmpty(),
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
        CameraMountCalibrator.Config cfg = CameraMountCalibrator.Config.defaults();
        cfg.preferredVisionDeviceName = p.vision.activeDeviceName();
        cfg.visionDeviceType = activeVisionDeviceType(p);
        cfg.visionPickerTitle = activeVisionPickerTitle(p);
        cfg.fixedTagLayout = p.fixedAprilTagLayout;
        return new CameraMountCalibrator(cfg, activeVisionLaneFactoryBuilder(p));
    }

    /**
     * Creates the Phoenix AprilTag-only localization tester.
     *
     * @return tester configured with Phoenix's active vision backend, field facts, and AprilTag-localizer defaults
     */
    public static TeleOpTester aprilTagLocalization() {
        PhoenixProfile p = profile();
        AprilTagLocalizationTester.Config cfg = AprilTagLocalizationTester.Config.defaults();
        cfg.preferredVisionDeviceName = p.vision.activeDeviceName();
        cfg.visionDeviceType = activeVisionDeviceType(p);
        cfg.visionPickerTitle = activeVisionPickerTitle(p);
        cfg.fixedTagLayout = p.fixedAprilTagLayout;
        cfg.aprilTags = p.localization.estimation.aprilTags.copy();
        return new AprilTagLocalizationTester(cfg, activeVisionLaneFactoryBuilder(p));
    }

    /**
     * Creates the Pinpoint axis-direction tester using the current Phoenix profile.
     *
     * @return tester that verifies forward/left/heading sign conventions for Pinpoint
     */
    public static TeleOpTester pinpointAxisCheck() {
        PinpointAxisDirectionTester.Config cfg = PinpointAxisDirectionTester.Config.defaults();
        cfg.pinpoint = profile().localization.predictor.copy();
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
        cfg.pinpoint = p.localization.predictor.copy();
        cfg.mecanum = p.drive.copy();
        cfg.targetTurnRad = Math.PI;
        cfg.autoComputeAfterAutoSample = true;
        cfg.preferredVisionDeviceName = p.vision.activeDeviceName();
        cfg.visionDeviceType = activeVisionDeviceType(p);
        cfg.visionPickerTitle = activeVisionPickerTitle(p);
        cfg.fixedTagLayout = p.fixedAprilTagLayout;
        cfg.aprilTags = p.localization.estimation.aprilTags.copy();
        Function<String, AprilTagVisionLaneFactory> visionFactoryBuilder =
                CalibrationChecks.canUseAprilTagAssist(p.vision.activeCameraMount())
                        ? activeVisionLaneFactoryBuilder(p)
                        : null;
        return new PinpointPodOffsetCalibrator(cfg, visionFactoryBuilder);
    }

    /**
     * Creates the default Phoenix corrected-global-localization tester based on Pinpoint prediction plus the configured correction source.
     *
     * @return tester configured with Phoenix's default fusion estimator settings
     */
    public static TeleOpTester pinpointAprilTagFusion() {
        PhoenixProfile p = profile();
        FtcOdometryAprilTagLocalizationLane.Config cfg = p.localization.copy();
        return configuredLocalizationTester(
                p,
                cfg,
                FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.FUSION
        );
    }

    /**
     * Creates the optional Phoenix covariance-aware corrected-global-localization tester.
     *
     * @return tester configured to compare the optional EKF path against the default fusion path
     */
    public static TeleOpTester pinpointAprilTagEkf() {
        PhoenixProfile p = profile();
        FtcOdometryAprilTagLocalizationLane.Config cfg = p.localization.copy();
        return configuredLocalizationTester(
                p,
                cfg,
                FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.EKF
        );
    }

    private static TeleOpTester configuredLocalizationTester(PhoenixProfile p,
                                                             FtcOdometryAprilTagLocalizationLane.Config localizationCfg,
                                                             FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode estimatorMode) {
        FtcOdometryAprilTagLocalizationLane.Config cfg = localizationCfg != null
                ? localizationCfg.copy()
                : FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.estimation.correctedEstimatorMode = estimatorMode != null
                ? estimatorMode
                : FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.FUSION;

        PinpointAprilTagCorrectedLocalizationTester.Config toolCfg =
                PinpointAprilTagCorrectedLocalizationTester.Config.defaults();
        toolCfg.preferredVisionDeviceName = p.vision.activeDeviceName();
        toolCfg.visionDeviceType = activeVisionDeviceType(p);
        toolCfg.visionPickerTitle = activeVisionPickerTitle(p);
        toolCfg.fixedTagLayout = p.fixedAprilTagLayout;
        toolCfg.localization = cfg;
        return new PinpointAprilTagCorrectedLocalizationTester(
                toolCfg,
                activeVisionLaneFactoryBuilder(p)
        );
    }

    /**
     * Creates the Phoenix configured-drivetrain verification tester.
     *
     * @return tester that verifies the profile's drivetrain names and directions one raised wheel
     *         at a time
     */
    static TeleOpTester configuredDrivetrainVerification() {
        return new ConfiguredDrivetrainVerificationTester(profile().drive);
    }

    /**
     * Computes the current camera-mount calibration status.
     *
     * @return status indicating whether the configured Phoenix camera mount looks solved
     */
    public static CalibrationStatus cameraMountStatus() {
        return cameraMountStatus(profile());
    }

    /**
     * Computes the current Pinpoint-axis verification status.
     *
     * @return status indicating whether Phoenix has acknowledged correct Pinpoint axis directions
     */
    public static CalibrationStatus pinpointAxesStatus() {
        return pinpointAxesStatus(profile());
    }

    /**
     * Computes the current Pinpoint-offset calibration status.
     *
     * @return status indicating whether Phoenix has calibrated and acknowledged Pinpoint pod offsets
     */
    public static CalibrationStatus pinpointOffsetsStatus() {
        return pinpointOffsetsStatus(profile());
    }

    /**
     * Computes whether the prerequisites for validating the global localizer are satisfied.
     *
     * @return complete status when camera mount, Pinpoint axes, and Pinpoint offsets all look ready
     */
    public static CalibrationStatus globalLocalizationStatus() {
        return globalLocalizationStatus(profile());
    }

    private static CalibrationStatus cameraMountStatus(PhoenixProfile p) {
        return CalibrationChecks.cameraMount(p.vision.activeCameraMount());
    }

    private static CalibrationStatus pinpointAxesStatus(PhoenixProfile p) {
        return CalibrationChecks.pinpointAxes(p.calibration.pinpointAxesVerified);
    }

    private static CalibrationStatus pinpointOffsetsStatus(PhoenixProfile p) {
        return CalibrationChecks.pinpointOffsets(
                p.localization.predictor,
                p.calibration.pinpointPodOffsetsCalibrated
        );
    }

    private static CalibrationStatus globalLocalizationStatus(PhoenixProfile p) {
        CalibrationStatus mount = cameraMountStatus(p);
        CalibrationStatus axes = pinpointAxesStatus(p);
        CalibrationStatus offsets = pinpointOffsetsStatus(p);

        if (mount.complete && axes.complete && offsets.complete) {
            return CalibrationStatus.complete("camera mount, Pinpoint axes, and Pinpoint offsets all look ready");
        }

        return CalibrationStatus.incomplete(
                "finish prerequisites: " + prerequisiteSummary(mount, axes, offsets)
        );
    }

    /**
     * Summarizes the prerequisite calibration statuses in one compact string.
     *
     * @return string of the form {@code mount=..., axes=..., offsets=...}
     */
    public static String prerequisiteSummary() {
        PhoenixProfile p = profile();
        return prerequisiteSummary(
                cameraMountStatus(p),
                pinpointAxesStatus(p),
                pinpointOffsetsStatus(p)
        );
    }

    private static String prerequisiteSummary(
            CalibrationStatus mount,
            CalibrationStatus axes,
            CalibrationStatus offsets
    ) {
        return "mount=" + mount.menuTag()
                + ", axes=" + axes.menuTag()
                + ", offsets=" + offsets.menuTag();
    }
}
