package edu.ftcphoenix.robots.phoenix.tester;

import edu.ftcphoenix.fw.tools.tester.TesterSuite;
import edu.ftcphoenix.fw.tools.tester.localization.AprilTagLocalizationTester;
import edu.ftcphoenix.robots.phoenix.RobotConfig;

/**
 * Phoenix robot-specific wrapper for the framework {@link AprilTagLocalizationTester}.
 *
 * <p>
 * The framework tester is robot-agnostic (it only needs a camera name and a
 * {@code robot→camera} mount pose). This wrapper provides those values from
 * {@link RobotConfig.Vision} so students can run a localization sanity check
 * without first going through the camera picker.
 * </p>
 *
 * <h2>When to use</h2>
 * <ul>
 *   <li>After you wire vision and want to confirm tags are detected.</li>
 *   <li>After running <b>Calib: Camera Mount</b> and updating
 *       {@link RobotConfig.Vision#cameraMount}, to validate field pose output.</li>
 * </ul>
 *
 * <p>
 * If the configured camera name is wrong or the camera fails to initialize,
 * the tester falls back to its built-in camera picker.
 * </p>
 */
public final class AprilTagLocalizationTesterPhoenix {

    private AprilTagLocalizationTesterPhoenix() {
    }

    /**
     * Register this tester into a suite (robot-side).
     *
     * <p>Typical usage in your robot OpMode:
     * <pre>{@code
     * TesterSuite suite = StandardTesters.createSuite();
     * AprilTagLocalizationTesterPhoenix.register(suite);
     * }</pre>
     */
    public static void register(TesterSuite suite) {
        if (suite == null) return;

        String mountStatus = RobotConfig.Calibration.cameraMountCalibrated()
                ? "camera mount: OK"
                : "camera mount: NOT CALIBRATED (run Calib: Camera Mount)";

        suite.add(
                "Loc: AprilTag Localization (Robot)",
                "Uses RobotConfig.Vision defaults; " + mountStatus,
                () -> new AprilTagLocalizationTester(
                        RobotConfig.Vision.nameWebcam,
                        RobotConfig.Vision.cameraMount
                )
        );
    }
}
