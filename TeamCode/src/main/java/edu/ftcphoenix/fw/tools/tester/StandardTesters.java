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
import edu.ftcphoenix.fw.tools.tester.calibration.CalibrationWalkthroughBuilder;
import edu.ftcphoenix.fw.tools.tester.calibration.CameraMountCalibrator;
import edu.ftcphoenix.fw.tools.tester.calibration.PinpointAxisDirectionTester;
import edu.ftcphoenix.fw.tools.tester.calibration.PinpointPodOffsetCalibrator;
import edu.ftcphoenix.fw.tools.tester.hardware.CrServoPowerTester;
import edu.ftcphoenix.fw.tools.tester.hardware.DcMotorPositionTester;
import edu.ftcphoenix.fw.tools.tester.hardware.DcMotorPowerTester;
import edu.ftcphoenix.fw.tools.tester.hardware.DcMotorVelocityTester;
import edu.ftcphoenix.fw.tools.tester.hardware.NormalizedColorSensorTester;
import edu.ftcphoenix.fw.tools.tester.hardware.ServoPositionTester;
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
 *       entrypoint used by the framework-owned {@code FW: Testers} OpMode, so teams that copy only
 *       {@code fw} still get a meaningful tester tree immediately.</li>
 * </ul>
 *
 * <p>The standalone home suite goes a bit farther than the embedded registration path. It includes
 * a generic bring-up walkthrough plus default-config Pinpoint/global-localization tools that are
 * useful when no robot project has supplied better-configured wrappers yet. The embedded path stays
 * conservative about robot-specific duplication, but now exposes both standard AprilTag backends:
 * webcam and Limelight.</p>
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
                "Framework: Calibration & Localization",
                "Framework-owned camera-mount, AprilTag, and localization tools for webcam and Limelight backends.",
                StandardTesters::createCalibrationAndLocalizationSuite
        );

        suite.add(
                "Framework: Hardware Testers",
                "Generic DcMotor / Servo / CRServo / color-sensor bring-up tools.",
                StandardTesters::createHardwareSuite
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
                .setHelp("Guide first for a fresh robot. Dpad: select | A: enter | BACK: back");

        suite.add(
                "Guide: Framework Bring-up",
                "Recommended order when you only have the framework tester tree available.",
                StandardTesters::createWalkthrough
        );

        suite.add(
                "Framework: Calibration & Localization",
                "Framework-owned camera, AprilTag, Limelight, and generic Pinpoint tools.",
                StandardTesters::createStandaloneCalibrationAndLocalizationSuite
        );

        suite.add(
                "Framework: Hardware Testers",
                "Generic DcMotor / Servo / CRServo / color-sensor bring-up tools.",
                StandardTesters::createHardwareSuite
        );

        return suite;
    }

    /**
     * Creates a simple framework-owned bring-up guide for teams that only copied {@code fw}.
     *
     * <p>These steps are intentionally untracked because the framework, by itself, does not own a
     * robot profile where calibration completion can be persisted. Robot projects should still create
     * their own richer walkthroughs with explicit status checks.</p>
     */
    public static TesterSuite createWalkthrough() {
        CalibrationWalkthroughBuilder guide = new CalibrationWalkthroughBuilder("Framework Bring-up Guide")
                .setHelp("Run steps in order. A: enter step | BACK: go back")
                .setMaxVisibleItems(10);

        guide.addStep(
                "HW: DcMotor Power",
                "Select each drivetrain motor and verify direction one at a time.",
                DcMotorPowerTester::new
        );

        guide.addStep(
                "Calib: Camera Mount (Webcam)",
                "Use when your AprilTag backend is a standard FTC webcam. Solve robot->camera and paste the printed CameraMountConfig.ofDegrees(...).",
                StandardTesters::createGenericWebcamCameraMountTester
        );

        guide.addStep(
                "Calib: Camera Mount (Limelight)",
                "Use when your AprilTag backend is a Limelight. Solve robot->camera from Limelight fiducials and paste the printed CameraMountConfig.ofDegrees(...).",
                StandardTesters::createGenericLimelightCameraMountTester
        );

        guide.addStep(
                "Loc: AprilTag Localization (Webcam)",
                "Sanity-check tag detection and the AprilTag-only field pose solve using a webcam-backed lane.",
                StandardTesters::createGenericWebcamAprilTagLocalizationTester
        );

        guide.addStep(
                "Loc: AprilTag Localization (Limelight)",
                "Sanity-check the same AprilTag-only field pose solve using Limelight fiducials.",
                StandardTesters::createGenericLimelightAprilTagLocalizationTester
        );

        guide.addStep(
                "Calib: Pinpoint Axis Check",
                "Pick the Pinpoint device name if needed, then verify +X forward, +Y left, heading CCW+.",
                StandardTesters::createGenericPinpointAxisCheckTester
        );

        guide.addStep(
                "Calib: Pinpoint Pod Offsets",
                "Pick the Pinpoint device name if needed. Manual by-hand samples work without drive wiring; robot-owned wrappers can also supply AprilTag assist.",
                StandardTesters::createGenericPinpointPodOffsetTester
        );

        guide.addStep(
                "Loc: Pinpoint + Field Corrections (Webcam)",
                "Pick the Pinpoint device name if needed. The tester opens a webcam-backed AprilTag lane and uses raw AprilTag correction with an identity camera mount until you calibrate it.",
                StandardTesters::createGenericPinpointAprilTagFusionTesterWebcam
        );

        guide.addStep(
                "Loc: Pinpoint + Field Corrections (Limelight)",
                "Pick the Pinpoint device name if needed. The tester opens a Limelight-backed AprilTag lane and uses raw AprilTag correction with an identity camera mount until you calibrate it.",
                StandardTesters::createGenericPinpointAprilTagFusionTesterLimelight
        );

        return guide.build();
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

    /**
     * Framework hardware bring-up tools.
     */
    public static TesterSuite createHardwareSuite() {
        TesterSuite suite = new TesterSuite()
                .setTitle("Framework Hardware Testers")
                .setHelp("Generic hardware sanity-check tools.")
                .setMaxVisibleItems(8);

        suite.add(
                "HW: DcMotor Power",
                "Open-loop motor power test (enable, invert, step, stick override).",
                DcMotorPowerTester::new
        );

        suite.add(
                "HW: DcMotor Position",
                "RUN_TO_POSITION encoder target test (target + power + stick nudge).",
                DcMotorPositionTester::new
        );

        suite.add(
                "HW: DcMotor Velocity",
                "DcMotorEx velocity closed-loop test (setVelocity + target + stick nudge).",
                DcMotorVelocityTester::new
        );

        suite.add(
                "HW: CRServo Power",
                "Continuous rotation servo power test (enable, invert, step, stick override).",
                CrServoPowerTester::new
        );

        suite.add(
                "HW: Servo Position",
                "Standard servo position test (enable=hold, invert, step, stick override).",
                ServoPositionTester::new
        );

        suite.add(
                "HW: Color Sensor (Normalized)",
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
