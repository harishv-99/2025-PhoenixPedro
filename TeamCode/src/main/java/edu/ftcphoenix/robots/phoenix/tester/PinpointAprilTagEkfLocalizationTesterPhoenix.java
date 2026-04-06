package edu.ftcphoenix.robots.phoenix.tester;

import edu.ftcphoenix.fw.ftc.localization.PinpointPoseEstimator;
import edu.ftcphoenix.fw.tools.tester.TesterSuite;
import edu.ftcphoenix.fw.tools.tester.localization.PinpointAprilTagFusionLocalizationTester;
import edu.ftcphoenix.robots.phoenix.RobotConfig;

/**
 * Phoenix robot-specific wrapper for the optional EKF flavor of
 * {@link PinpointAprilTagFusionLocalizationTester}.
 *
 * <p>This tester is intentionally separate from the default fusion tester so teams can compare the
 * two localizers side by side without changing production robot config first.</p>
 */
public final class PinpointAprilTagEkfLocalizationTesterPhoenix {

    private PinpointAprilTagEkfLocalizationTesterPhoenix() {
    }

    /**
     * Registers the Phoenix-specific EKF localization tester with the supplied tester suite.
     */
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
                "Loc: Pinpoint + AprilTag EKF (Robot)",
                "Optional advanced estimator; " + mountStatus + ", " + axesStatus + ", " + offsetsStatus,
                () -> {
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
        );
    }
}
