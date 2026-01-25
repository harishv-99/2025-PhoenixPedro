package edu.ftcphoenix.robots.phoenix.tester;

import edu.ftcphoenix.fw.ftc.localization.PinpointPoseEstimator;
import edu.ftcphoenix.fw.tools.tester.TesterSuite;
import edu.ftcphoenix.fw.tools.tester.calibration.CameraMountCalibrator;
import edu.ftcphoenix.fw.tools.tester.calibration.PinpointAxisDirectionTester;
import edu.ftcphoenix.fw.tools.tester.calibration.PinpointPodOffsetCalibrator;
import edu.ftcphoenix.fw.tools.tester.localization.AprilTagLocalizationTester;
import edu.ftcphoenix.fw.tools.tester.localization.PinpointAprilTagFusionLocalizationTester;
import edu.ftcphoenix.robots.phoenix.RobotConfig;

/**
 * A guided, step-by-step calibration walkthrough for Phoenix.
 *
 * <p>This is intentionally a nested {@link TesterSuite}. It reuses the same menu controls,
 * and it can be launched from the main tester menu.
 *
 * <p>The goal is to make the "best next step" obvious without requiring students to read
 * through robot-specific tester source code.
 */
public final class PhoenixCalibrationWalkthrough {

    private PhoenixCalibrationWalkthrough() {
    }

    /**
     * Register the walkthrough entry into a parent suite.
     */
    public static void register(TesterSuite parent) {
        if (parent == null) return;

        parent.add(
                "Guide: Calibration Walkthrough (Phoenix)",
                "Recommended step order for Phoenix calibration & localization.",
                PhoenixCalibrationWalkthrough::createSuite
        );
    }

    /**
     * Build the nested walkthrough suite.
     */
    private static TesterSuite createSuite() {
        TesterSuite suite = new TesterSuite()
                .setTitle("Phoenix Calibration Walkthrough")
                .setHelp("Run steps in order. A: enter step | BACK: go back")
                .setMaxVisibleItems(7);

        int idx = 0;

        final int stepMotorDir = idx++;
        suite.add(
                "0) HW: Drivetrain Motor Direction",
                "Verify each motor drives forward (hold X/Y/A/B).",
                DrivetrainMotorDirectionTester::new
        );

        final int stepCamMount = idx++;
        suite.add(
                "1) Calib: Camera Mount",
                "Solve RobotConfig.Vision.cameraMount (camera extrinsics).",
                () -> new CameraMountCalibrator(RobotConfig.Vision.nameWebcam)
        );

        final int stepApriltagCheck = idx++;
        suite.add(
                "2) Loc: AprilTag Localization",
                "Sanity-check tag detection and field pose (needs camera mount).",
                () -> new AprilTagLocalizationTester(
                        RobotConfig.Vision.nameWebcam,
                        RobotConfig.Vision.cameraMount
                )
        );

        final int stepPinpointAxes = idx++;
        suite.add(
                "3) Calib: Pinpoint Axis Check",
                "Verify Pinpoint reports +X forward, +Y left, heading CCW+.",
                () -> {
                    PinpointAxisDirectionTester.Config cfg = PinpointAxisDirectionTester.Config.defaults();
                    cfg.pinpoint = RobotConfig.Localization.pinpoint;
                    return new PinpointAxisDirectionTester(cfg);
                }
        );

        final int stepPinpointOffsets = idx++;
        suite.add(
                "4) Calib: Pinpoint Pod Offsets",
                "Press PLAY, then rotate in place to estimate offsets (Y auto-sample; auto-computes when tags are available). Tag assist auto-enables when camera mount is OK.",
                () -> {
                    PinpointPodOffsetCalibrator.Config cfg = PinpointPodOffsetCalibrator.Config.defaults();
                    cfg.pinpoint = RobotConfig.Localization.pinpoint;
                    cfg.mecanumWiring = RobotConfig.DriveTrain.mecanumWiring();
                    // Use ~180° for the pod-offset solve.
                    // A full 360° turn is (nearly) degenerate for this math and can explode.
                    cfg.targetTurnRad = Math.PI;

                    // Optional AprilTag assist.
                    cfg.preferredCameraName = RobotConfig.Vision.nameWebcam;
                    cfg.cameraMount = RobotConfig.Vision.cameraMount;
                    cfg.enableAprilTagAssist = RobotConfig.Calibration.canUseAprilTagAssist();

                    // With AprilTag assist, auto-samples compute automatically after the turn.
                    cfg.autoComputeAfterAutoSample = true;

                    return new PinpointPodOffsetCalibrator(cfg);
                }
        );

        final int stepFusion = idx++;
        suite.add(
                "5) Loc: Pinpoint + AprilTag Fusion",
                "Verify fused global pose (needs Pinpoint offsets + camera mount).",
                () -> {
                    PinpointPoseEstimator.Config cfg = RobotConfig.Localization.pinpoint;
                    return new PinpointAprilTagFusionLocalizationTester(
                            RobotConfig.Vision.nameWebcam,
                            RobotConfig.Vision.cameraMount,
                            cfg
                    );
                }
        );

        // Pick a default selection: first missing required step.
        boolean camOk = RobotConfig.Calibration.cameraMountCalibrated();
        boolean axesOk = RobotConfig.Calibration.pinpointAxesVerified;
        boolean offsetsOk = RobotConfig.Calibration.pinpointPodOffsetsCalibrated
                || RobotConfig.Calibration.pinpointPodOffsetsNonDefault();

        int startAt = stepCamMount;
        if (camOk && !axesOk) startAt = stepPinpointAxes;
        if (camOk && axesOk && !offsetsOk) startAt = stepPinpointOffsets;
        if (camOk && axesOk && offsetsOk) startAt = stepFusion;

        suite.setSelectedIndex(startAt);
        return suite;
    }
}
