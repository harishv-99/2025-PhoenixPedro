package edu.ftcphoenix.robots.phoenix.tester;

import edu.ftcphoenix.fw.tools.tester.TesterSuite;
import edu.ftcphoenix.fw.tools.tester.calibration.PinpointAxisDirectionTester;
import edu.ftcphoenix.robots.phoenix.RobotConfig;

/**
 * Phoenix robot-specific wrapper for {@link PinpointAxisDirectionTester}.
 *
 * <p>This is the recommended "Step 0" before running the pod-offset calibrator:
 * confirm that Pinpoint reports +X forward, +Y left, and CCW-positive heading.</p>
 */
public final class PinpointAxisDirectionTesterPhoenix {

    private PinpointAxisDirectionTesterPhoenix() {
    }

    /**
     * Registers the Pinpoint axis-direction checker into the Phoenix tester suite.
     */
    public static void register(TesterSuite suite) {
        if (suite == null) return;

        String status = RobotConfig.Calibration.pinpointAxesVerified
                ? "axes: OK"
                : "axes: NOT VERIFIED (set RobotConfig.Calibration.pinpointAxesVerified=true after checking)";

        suite.add(
                "Calib: Pinpoint Axis Check (Robot)",
                "Step 0: verify Pinpoint axes; " + status,
                () -> {
                    PinpointAxisDirectionTester.Config cfg = PinpointAxisDirectionTester.Config.defaults();
                    cfg.pinpoint = RobotConfig.Localization.pinpoint;
                    return new PinpointAxisDirectionTester(cfg);
                }
        );
    }
}
