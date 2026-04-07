package edu.ftcphoenix.robots.phoenix.tester;

import edu.ftcphoenix.fw.tools.tester.TesterSuite;
import edu.ftcphoenix.fw.tools.tester.calibration.CalibrationWalkthroughBuilder;

/**
 * Guided, step-by-step Phoenix bring-up sequence.
 */
public final class PhoenixCalibrationWalkthrough {

    private PhoenixCalibrationWalkthrough() {
    }

    /**
     * Builds the recommended Phoenix calibration walkthrough.
     *
     * @return tester suite containing the ordered bring-up flow for Phoenix
     */
    public static TesterSuite createSuite() {
        CalibrationWalkthroughBuilder guide = new CalibrationWalkthroughBuilder("Phoenix Calibration Walkthrough")
                .setHelp("Run steps in order. A: enter step | BACK: go back")
                .setMaxVisibleItems(8);

        guide.addStep(
                "HW: Drivetrain Motor Direction",
                "Optional but recommended on first bring-up: verify each wheel would drive the robot forward.",
                PhoenixRobotTesters::drivetrainMotorDirection
        );

        guide.addStep(
                "Calib: Camera Mount",
                "Solve PhoenixProfile.current().localization.cameraMount and paste the printed CameraMountConfig.ofDegrees(...) value.",
                PhoenixRobotTesters::cameraMountStatus,
                PhoenixRobotTesters::cameraMountCalibrator
        );

        guide.addStep(
                "Loc: AprilTag Localization",
                "Sanity-check tag detection and the AprilTag-only field pose solve before fusing with odometry.",
                PhoenixRobotTesters::cameraMountStatus,
                PhoenixRobotTesters::aprilTagLocalization
        );

        guide.addStep(
                "Calib: Pinpoint Axis Check",
                "Verify Pinpoint reports +X forward, +Y left, and CCW-positive heading.",
                PhoenixRobotTesters::pinpointAxesStatus,
                PhoenixRobotTesters::pinpointAxisCheck
        );

        guide.addStep(
                "Calib: Pinpoint Pod Offsets",
                "Rotate in place to estimate Pinpoint pod offsets. AprilTag assist auto-enables once the camera mount is solved.",
                PhoenixRobotTesters::pinpointOffsetsStatus,
                PhoenixRobotTesters::pinpointPodOffsets
        );

        int fusionStep = guide.addStep(
                "Loc: Pinpoint + AprilTag Fusion",
                "Validate Phoenix's default global localizer after camera mount and Pinpoint calibration look good.",
                PhoenixRobotTesters::globalLocalizationStatus,
                PhoenixRobotTesters::pinpointAprilTagFusion
        );

        guide.addStep(
                "Loc: Pinpoint + AprilTag EKF (Optional)",
                "Compare the optional EKF-style localizer only after the default fusion tester looks trustworthy.",
                PhoenixRobotTesters::globalLocalizationStatus,
                PhoenixRobotTesters::pinpointAprilTagEkf
        );

        guide.setFallbackSelectedIndex(fusionStep);
        return guide.build();
    }
}
