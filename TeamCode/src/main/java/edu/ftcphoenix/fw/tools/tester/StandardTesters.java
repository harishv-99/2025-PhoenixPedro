package edu.ftcphoenix.fw.tools.tester;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.function.Function;

import edu.ftcphoenix.fw.ftc.localization.PinpointOdometryPredictor;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLaneFactories;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLaneFactory;
import edu.ftcphoenix.fw.ftc.vision.FtcLimelightAprilTagVisionLane;
import edu.ftcphoenix.fw.localization.fusion.OdometryCorrectionFusionEstimator;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.tools.tester.calibration.CameraMountCalibrator;
import edu.ftcphoenix.fw.tools.tester.calibration.PinpointAxisDirectionTester;
import edu.ftcphoenix.fw.tools.tester.calibration.PinpointPodOffsetCalibrator;
import edu.ftcphoenix.fw.tools.tester.hardware.DcMotorPositionTester;
import edu.ftcphoenix.fw.tools.tester.hardware.DcMotorPowerTester;
import edu.ftcphoenix.fw.tools.tester.hardware.DcMotorVelocityTester;
import edu.ftcphoenix.fw.tools.tester.hardware.NormalizedColorSensorTester;
import edu.ftcphoenix.fw.tools.tester.localization.AprilTagLocalizationTester;
import edu.ftcphoenix.fw.tools.tester.localization.PinpointAprilTagFusionLocalizationTester;

/**
 * Registers the standard Phoenix framework testers.
 *
 * <p>This class intentionally supports two slightly different roles:</p>
 * <ul>
 *   <li><b>Embedded framework groups</b> via {@link #register(TesterSuite)}. Robot projects use this
 *       path when they already provide their own configured walkthroughs and robot-specific tester
 *       categories (for example Phoenix).</li>
 *   <li><b>Framework-only home suite</b> via {@link #createSuite()}. This is the no-robot-glue
 *       entrypoint used by both framework-owned {@code FW: Testers (...)} console OpModes, so
 *       teams that copy the framework with its pinned Panels integration get a meaningful tester
 *       tree immediately.</li>
 * </ul>
 *
 * <p>Both paths expose the same ordinary actuator entry: one device-first bring-up wizard. Typed
 * motor diagnostics with genuinely different evidence remain under an explicitly advanced menu;
 * they are not parallel beginner recipes.</p>
 */
public final class StandardTesters {

    private StandardTesters() {
    }

    /**
     * Adds the shared framework tester groups to an existing robot-owned top-level suite.
     *
     * <p>This is the embed-safe registration path. It adds the generic framework-owned categories
     * without assuming anything about robot-specific wrappers layered above them.</p>
     */
    public static void register(TesterSuite suite) {
        if (suite == null) return;

        suite.add(
                "HW: Actuator Bring-up",
                "Select one configured motor or servo by name to establish direction and optional safe endpoints.",
                StandardTesters::createActuatorBringUp
        );

        suite.add(
                "Framework: Calibration & Localization",
                "Framework-owned camera-mount, AprilTag, and localization tools for webcam and Limelight backends.",
                StandardTesters::createCalibrationAndLocalizationSuite
        );

        suite.add(
                "Advanced: Hardware Diagnostics",
                "Specialized motor and sensor diagnostics after ordinary actuator bring-up.",
                StandardTesters::createAdvancedHardwareDiagnosticsSuite
        );
    }

    /**
     * Creates the standalone framework tester home.
     *
     * <p>This is the entrypoint that makes the framework useful even when a team has copied only the
     * {@code fw} portion of Phoenix and has not built a robot-specific tester menu yet.</p>
     */
    public static TesterSuite createSuite() {
        TesterSuite suite = new TesterSuite()
                .setTitle("Framework Tester Home")
                .setHelp("Start with actuator bring-up. Dpad: select | A: enter | BACK: back");

        suite.add(
                "HW: Actuator Bring-up",
                "Select one configured motor or servo by name to establish direction and optional safe endpoints.",
                StandardTesters::createActuatorBringUp
        );

        suite.add(
                "Framework: Calibration & Localization",
                "Framework-owned camera, AprilTag, Limelight, and generic Pinpoint tools.",
                StandardTesters::createStandaloneCalibrationAndLocalizationSuite
        );

        suite.add(
                "Advanced: Hardware Diagnostics",
                "Specialized motor and sensor diagnostics after ordinary actuator bring-up.",
                StandardTesters::createAdvancedHardwareDiagnosticsSuite
        );

        return suite;
    }

    /**
     * Creates the one ordinary actuator bring-up workflow.
     *
     * <p>The declared return type intentionally hides the package-private implementation. Robot
     * projects should embed all standard entries through {@link #register(TesterSuite)}; this
     * factory is the canonical reuse hook for a direct single-tool host or an ordered
     * robot-specific walkthrough. Use {@link #register(TesterSuite)} for the complete standard
     * tester home.</p>
     *
     * @return inactive device-first actuator bring-up tester
     */
    public static TeleOpTester createActuatorBringUp() {
        return new ActuatorBringUpTester();
    }

    /**
     * Framework camera-mount and AprilTag tools that embed cleanly under a robot-specific tester tree.
     */
    public static TesterSuite createCalibrationAndLocalizationSuite() {
        TesterSuite suite = new TesterSuite()
                .setTitle("Framework Calibration & Localization")
                .setHelp("Camera mount and AprilTag bring-up tools for webcam and Limelight backends.")
                .setMaxVisibleItems(10);

        suite.add(
                "Calib: Camera Mount (Webcam)",
                "Solve robotToCameraPose using a webcam-backed AprilTag lane and the fixed field layout.",
                StandardTesters::createGenericWebcamCameraMountTester
        );

        suite.add(
                "Loc: AprilTag Localization (Webcam)",
                "Verify AprilTag detections and the field pose solve using a webcam-backed lane.",
                StandardTesters::createGenericWebcamAprilTagLocalizationTester
        );

        suite.add(
                "Calib: Camera Mount (Limelight)",
                "Solve robotToCameraPose using Limelight AprilTag fiducials and the fixed field layout.",
                StandardTesters::createGenericLimelightCameraMountTester
        );

        suite.add(
                "Loc: AprilTag Localization (Limelight)",
                "Verify AprilTag detections and the field pose solve using a Limelight-backed lane.",
                StandardTesters::createGenericLimelightAprilTagLocalizationTester
        );

        return suite;
    }

    /**
     * Standalone framework calibration/localization tools.
     *
     * <p>This suite includes the generic Pinpoint tools that are still usable without a robot
     * profile. The framework-only menu selects the Pinpoint device name at runtime before opening
     * those testers.</p>
     */
    public static TesterSuite createStandaloneCalibrationAndLocalizationSuite() {
        TesterSuite suite = new TesterSuite()
                .setTitle("Framework Calibration & Localization")
                .setHelp("Camera, AprilTag, Limelight, and generic Pinpoint bring-up tools.")
                .setMaxVisibleItems(10);

        suite.add(
                "Calib: Camera Mount (Webcam)",
                "Solve robotToCameraPose using a webcam-backed AprilTag lane and the fixed field layout.",
                StandardTesters::createGenericWebcamCameraMountTester
        );

        suite.add(
                "Loc: AprilTag Localization (Webcam)",
                "Verify AprilTag detections and the field pose solve using a webcam-backed lane.",
                StandardTesters::createGenericWebcamAprilTagLocalizationTester
        );

        suite.add(
                "Calib: Camera Mount (Limelight)",
                "Solve robotToCameraPose using Limelight AprilTag fiducials and the fixed field layout.",
                StandardTesters::createGenericLimelightCameraMountTester
        );

        suite.add(
                "Loc: AprilTag Localization (Limelight)",
                "Verify AprilTag detections and the field pose solve using a Limelight-backed lane.",
                StandardTesters::createGenericLimelightAprilTagLocalizationTester
        );

        suite.add(
                "Calib: Pinpoint Axis Check",
                "Choose the Pinpoint device name at runtime, then verify +X forward, +Y left, heading CCW+.",
                StandardTesters::createGenericPinpointAxisCheckTester
        );

        suite.add(
                "Calib: Pinpoint Pod Offsets",
                "Choose the Pinpoint device name at runtime. Manual by-hand samples work without robot drive wiring.",
                StandardTesters::createGenericPinpointPodOffsetTester
        );

        suite.add(
                "Loc: Pinpoint + Field Corrections (Webcam)",
                "Choose the Pinpoint device name at runtime. The tester still provides a webcam picker and uses raw AprilTag correction with an identity camera mount until calibrated.",
                StandardTesters::createGenericPinpointAprilTagFusionTesterWebcam
        );

        suite.add(
                "Loc: Pinpoint + Field Corrections (Limelight)",
                "Choose the Pinpoint device name at runtime. The tester provides a Limelight picker and uses raw AprilTag correction with an identity camera mount until calibrated.",
                StandardTesters::createGenericPinpointAprilTagFusionTesterLimelight
        );

        return suite;
    }

    /** Specialized diagnostics that provide evidence beyond ordinary actuator bring-up. */
    private static TesterSuite createAdvancedHardwareDiagnosticsSuite() {
        TesterSuite suite = new TesterSuite()
                .setTitle("Advanced Hardware Diagnostics")
                .setHelp("Use after direction and safe endpoints are established in actuator bring-up.")
                .setMaxVisibleItems(8);

        suite.add(
                "Advanced: Motor Power & Encoder Evidence",
                "Open-loop motor power plus optional direct-vs-derived encoder velocity logging.",
                DcMotorPowerTester::new
        );

        suite.add(
                "Advanced: Motor Position",
                "OpMode START-locked RUN_TO_POSITION diagnostic with stepped target and power.",
                DcMotorPositionTester::new
        );

        suite.add(
                "Advanced: Motor Velocity",
                "OpMode START-locked DcMotorEx setVelocity diagnostic with stepped target.",
                DcMotorVelocityTester::new
        );

        suite.add(
                "Advanced: Normalized Color Sensor",
                "NormalizedColorSensor bring-up (ratios + alpha/chroma + HSV, optional raw RGBA detail, live gain tuning).",
                NormalizedColorSensorTester::new
        );

        return suite;
    }

    private static TeleOpTester createGenericWebcamCameraMountTester() {
        return new CameraMountCalibrator();
    }

    private static TeleOpTester createGenericLimelightCameraMountTester() {
        return new CameraMountCalibrator(
                null,
                Limelight3A.class,
                "Select Limelight",
                limelightLaneFactoryBuilder(),
                null,
                0.35
        );
    }

    private static TeleOpTester createGenericWebcamAprilTagLocalizationTester() {
        return new AprilTagLocalizationTester();
    }

    private static TeleOpTester createGenericLimelightAprilTagLocalizationTester() {
        return new AprilTagLocalizationTester(
                null,
                Limelight3A.class,
                "Select Limelight",
                limelightLaneFactoryBuilder(),
                null,
                null,
                0.35
        );
    }

    private static TeleOpTester createGenericPinpointAxisCheckTester() {
        return new HardwareSelectingTester(
                "Pinpoint Axis Check",
                GoBildaPinpointDriver.class,
                "Select Pinpoint",
                "Dpad: highlight | A: choose | X: refresh",
                PinpointOdometryPredictor.Config.defaults().hardwareMapName,
                hardwareName -> {
                    PinpointAxisDirectionTester.Config cfg = PinpointAxisDirectionTester.Config.defaults();
                    cfg.pinpoint = PinpointOdometryPredictor.Config.defaults().withHardwareMapName(hardwareName);
                    return new PinpointAxisDirectionTester(cfg);
                }
        );
    }

    private static TeleOpTester createGenericPinpointPodOffsetTester() {
        return new HardwareSelectingTester(
                "Pinpoint Pod Offset Calibrator",
                GoBildaPinpointDriver.class,
                "Select Pinpoint",
                "Dpad: highlight | A: choose | X: refresh",
                PinpointOdometryPredictor.Config.defaults().hardwareMapName,
                hardwareName -> {
                    PinpointPodOffsetCalibrator.Config cfg = PinpointPodOffsetCalibrator.Config.defaults();
                    cfg.pinpoint = PinpointOdometryPredictor.Config.defaults().withHardwareMapName(hardwareName);
                    return new PinpointPodOffsetCalibrator(cfg);
                }
        );
    }

    private static TeleOpTester createGenericPinpointAprilTagFusionTesterWebcam() {
        return new HardwareSelectingTester(
                "Pinpoint + Field Corrections (Webcam)",
                GoBildaPinpointDriver.class,
                "Select Pinpoint",
                "Dpad: highlight | A: choose | X: refresh",
                PinpointOdometryPredictor.Config.defaults().hardwareMapName,
                hardwareName -> new PinpointAprilTagFusionLocalizationTester(
                        null,
                        null,
                        PinpointOdometryPredictor.Config.defaults().withHardwareMapName(hardwareName)
                )
        );
    }

    private static TeleOpTester createGenericPinpointAprilTagFusionTesterLimelight() {
        return new HardwareSelectingTester(
                "Pinpoint + Field Corrections (Limelight)",
                GoBildaPinpointDriver.class,
                "Select Pinpoint",
                "Dpad: highlight | A: choose | X: refresh",
                PinpointOdometryPredictor.Config.defaults().hardwareMapName,
                hardwareName -> new PinpointAprilTagFusionLocalizationTester(
                        null,
                        Limelight3A.class,
                        "Select Limelight",
                        limelightLaneFactoryBuilder(),
                        buildLimelightFusionLocalizationConfig(hardwareName),
                        null
                )
        );
    }

    private static edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane.Config buildLimelightFusionLocalizationConfig(String hardwareName) {
        edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane.Config cfg =
                edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.predictor = PinpointOdometryPredictor.Config.defaults().withHardwareMapName(hardwareName);
        cfg.correctedEstimatorMode = edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.FUSION;
        cfg.correctionSource.mode = edu.ftcphoenix.fw.ftc.localization.FtcOdometryAprilTagLocalizationLane.CorrectionSourceMode.APRILTAG_POSE;
        cfg.correctionFusion = OdometryCorrectionFusionEstimator.Config.defaults();
        return cfg;
    }

    private static Function<String, AprilTagVisionLaneFactory> limelightLaneFactoryBuilder() {
        return hardwareName -> {
            FtcLimelightAprilTagVisionLane.Config cfg = FtcLimelightAprilTagVisionLane.Config.defaults();
            cfg.hardwareName = hardwareName;
            cfg.cameraMount = CameraMountConfig.identity();
            return AprilTagVisionLaneFactories.limelight(cfg);
        };
    }
}
