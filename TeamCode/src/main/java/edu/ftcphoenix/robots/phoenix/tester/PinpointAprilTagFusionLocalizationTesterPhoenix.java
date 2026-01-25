package edu.ftcphoenix.robots.phoenix.tester;

import edu.ftcphoenix.fw.ftc.localization.PinpointPoseEstimator;
import edu.ftcphoenix.fw.tools.tester.TesterSuite;
import edu.ftcphoenix.fw.tools.tester.localization.PinpointAprilTagFusionLocalizationTester;
import edu.ftcphoenix.robots.phoenix.RobotConfig;

/**
 * Phoenix robot-specific wrapper for {@link PinpointAprilTagFusionLocalizationTester}.
 */
public final class PinpointAprilTagFusionLocalizationTesterPhoenix {

    private PinpointAprilTagFusionLocalizationTesterPhoenix() {
    }

    public static void register(TesterSuite suite) {
        if (suite == null) return;

        String mountStatus = RobotConfig.Calibration.cameraMountCalibrated()
                ? "camera mount: OK"
                : "camera mount: NOT CALIBRATED";

        String axesStatus = RobotConfig.Calibration.pinpointAxesVerified
                ? "axes: OK"
                : "axes: NOT VERIFIED";

        boolean offsetsOk = RobotConfig.Calibration.pinpointPodOffsetsCalibrated
                || RobotConfig.Calibration.pinpointPodOffsetsNonDefault();
        String offsetsStatus = offsetsOk
                ? "offsets: OK"
                : "offsets: NOT CALIBRATED";

        suite.add(
                "Loc: Pinpoint + AprilTag Fusion (Robot)",
                "Fused pose; " + mountStatus + ", " + axesStatus + ", " + offsetsStatus,
                () -> {
                    PinpointPoseEstimator.Config cfg = RobotConfig.Localization.pinpoint;

                    return new PinpointAprilTagFusionLocalizationTester(
                            RobotConfig.Vision.nameWebcam,
                            RobotConfig.Vision.cameraMount,
                            cfg
                    );
                }
        );
    }
}
