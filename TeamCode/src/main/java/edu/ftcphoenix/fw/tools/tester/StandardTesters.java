package edu.ftcphoenix.fw.tools.tester;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import edu.ftcphoenix.fw.ftc.localization.PinpointPoseEstimator;
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
 * a generic bring-up walkthrough plus a few default-config Pinpoint/global-localization tools that
 * are useful when no robot project has supplied better-configured wrappers yet. The embedded path is
 * intentionally more conservative so it does not duplicate robot-configured tools unnecessarily.</p>
 */
public final class StandardTesters {

    private StandardTesters() {
    }

    /**
     * Adds the shared framework tester groups to an existing robot-owned top-level suite.
     *
     * <p>This is the embed-safe registration path. It only adds the generic categories that are a
     * clean fit underneath a robot-specific tester tree.</p>
     */
    public static void register(TesterSuite suite) {
        if (suite == null) return;

        suite.add(
                "Framework: Calibration & Localization",
                "Framework-owned camera-mount and AprilTag localization tools.",
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
                "Framework-owned camera, AprilTag, and generic Pinpoint tools.",
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
                .setMaxVisibleItems(8);

        guide.addStep(
                "HW: DcMotor Power",
                "Select each drivetrain motor and verify direction one at a time.",
                DcMotorPowerTester::new
        );

        guide.addStep(
                "Calib: Camera Mount",
                "Use the camera picker if needed, solve robot->camera, and paste the printed CameraMountConfig.ofDegrees(...).",
                CameraMountCalibrator::new
        );

        guide.addStep(
                "Loc: AprilTag Localization",
                "Sanity-check tag detection and the AprilTag-only field pose solve before fusing with odometry.",
                AprilTagLocalizationTester::new
        );

        guide.addStep(
                "Calib: Pinpoint Axis Check",
                "Pick the Pinpoint device name if needed, then verify +X forward, +Y left, heading CCW+.",
                StandardTesters::createGenericPinpointAxisCheckTester
        );

        guide.addStep(
                "Calib: Pinpoint Pod Offsets",
                "Pick the Pinpoint device name if needed. Manual by-hand samples work without drive wiring.",
                StandardTesters::createGenericPinpointPodOffsetTester
        );

        guide.addStep(
                "Loc: Pinpoint + AprilTag Fusion",
                "Pick the Pinpoint device name if needed. The tester still provides its own camera picker and uses an identity camera mount until you calibrate it.",
                StandardTesters::createGenericPinpointAprilTagFusionTester
        );

        return guide.build();
    }

    /**
     * Framework camera-mount and AprilTag tools that embed cleanly under a robot-specific tester tree.
     */
    public static TesterSuite createCalibrationAndLocalizationSuite() {
        TesterSuite suite = new TesterSuite()
                .setTitle("Framework Calibration & Localization")
                .setHelp("Camera mount and AprilTag bring-up tools.")
                .setMaxVisibleItems(8);

        suite.add(
                "Calib: Camera Mount",
                "Solve robotToCameraPose using known robot pose + fixed tag layout. Includes camera picker.",
                CameraMountCalibrator::new
        );

        suite.add(
                "Loc: AprilTag Localization",
                "Verify AprilTag detections and the field pose solve. Includes camera picker.",
                AprilTagLocalizationTester::new
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
                .setHelp("Camera, AprilTag, and generic Pinpoint bring-up tools.")
                .setMaxVisibleItems(8);

        suite.add(
                "Calib: Camera Mount",
                "Solve robotToCameraPose using known robot pose + fixed tag layout. Includes camera picker.",
                CameraMountCalibrator::new
        );

        suite.add(
                "Loc: AprilTag Localization",
                "Verify AprilTag detections and the field pose solve. Includes camera picker.",
                AprilTagLocalizationTester::new
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
                "Loc: Pinpoint + AprilTag Fusion",
                "Choose the Pinpoint device name at runtime. The tester still provides its own camera picker and uses an identity camera mount until calibrated.",
                StandardTesters::createGenericPinpointAprilTagFusionTester
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

    private static TeleOpTester createGenericPinpointAxisCheckTester() {
        return new HardwareSelectingTester(
                "Pinpoint Axis Check",
                GoBildaPinpointDriver.class,
                "Select Pinpoint",
                "Dpad: highlight | A: choose | B: refresh",
                PinpointPoseEstimator.Config.defaults().hardwareMapName,
                hardwareName -> {
                    PinpointAxisDirectionTester.Config cfg = PinpointAxisDirectionTester.Config.defaults();
                    cfg.pinpoint = PinpointPoseEstimator.Config.defaults().withHardwareMapName(hardwareName);
                    return new PinpointAxisDirectionTester(cfg);
                }
        );
    }

    private static TeleOpTester createGenericPinpointPodOffsetTester() {
        return new HardwareSelectingTester(
                "Pinpoint Pod Offset Calibrator",
                GoBildaPinpointDriver.class,
                "Select Pinpoint",
                "Dpad: highlight | A: choose | B: refresh",
                PinpointPoseEstimator.Config.defaults().hardwareMapName,
                hardwareName -> {
                    PinpointPodOffsetCalibrator.Config cfg = PinpointPodOffsetCalibrator.Config.defaults();
                    cfg.pinpoint = PinpointPoseEstimator.Config.defaults().withHardwareMapName(hardwareName);
                    return new PinpointPodOffsetCalibrator(cfg);
                }
        );
    }

    private static TeleOpTester createGenericPinpointAprilTagFusionTester() {
        return new HardwareSelectingTester(
                "Pinpoint + AprilTag Fusion",
                GoBildaPinpointDriver.class,
                "Select Pinpoint",
                "Dpad: highlight | A: choose | B: refresh",
                PinpointPoseEstimator.Config.defaults().hardwareMapName,
                hardwareName -> new PinpointAprilTagFusionLocalizationTester(
                        null,
                        null,
                        PinpointPoseEstimator.Config.defaults().withHardwareMapName(hardwareName)
                )
        );
    }
}
