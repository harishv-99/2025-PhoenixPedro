package edu.ftcphoenix.robots.phoenix.tester;

import edu.ftcphoenix.fw.tools.tester.TesterSuite;
import edu.ftcphoenix.fw.tools.tester.StandardTesters;
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

        int actuatorBringUpStep = guide.addStep(
                "HW: Actuator Bring-up",
                "Establish each raw motor or servo direction and safe endpoints; copy them into PhoenixScoring.Config.defaults() or PhoenixDriveConfiguration.current().",
                StandardTesters::createActuatorBringUp
        );

        guide.addStep(
                "HW: Configured Drivetrain Verification",
                "After drivetrain facts are copied into PhoenixDriveConfiguration.current(), verify each configured wheel with the chassis raised.",
                PhoenixRobotTesters::configuredDrivetrainVerification
        );

        guide.addStep(
                "Calib: Camera Mount",
                "Solve the selected backend mount in PhoenixVisionFactory.Config.defaults() and paste the printed CameraMountConfig.ofDegrees(...) value.",
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
                "Rotate in place to estimate Pinpoint pod offsets. Once the active camera mount is solved, AprilTag assist uses Phoenix's production field layout, freshness, and solver policy; successful INIT does not command the drive before START.",
                PhoenixRobotTesters::pinpointOffsetsStatus,
                PhoenixRobotTesters::pinpointPodOffsets
        );

        guide.addStep(
                "Loc: Pinpoint + Field Corrections",
                "Validate Phoenix's default corrected-global localizer after camera mount and Pinpoint calibration look good.",
                PhoenixRobotTesters::globalLocalizationStatus,
                PhoenixRobotTesters::pinpointAprilTagFusion
        );

        guide.addStep(
                "Loc: Pinpoint + Field Corrections EKF (Optional)",
                "Compare the optional covariance-aware corrected localizer only after the default fusion tester looks trustworthy.",
                PhoenixRobotTesters::globalLocalizationStatus,
                PhoenixRobotTesters::pinpointAprilTagEkf
        );

        // Hardware-fact steps intentionally have no profile-completion predicate. Override the
        // status-based builder suggestion so a new student is never dropped into a later camera
        // step before seeing the physical-safety entry point.
        TesterSuite suite = guide.build();
        suite.setSelectedIndex(actuatorBringUpStep);
        return suite;
    }
}
