package edu.ftcphoenix.robots.phoenix.tester;

import edu.ftcphoenix.fw.tools.tester.TesterSuite;
import edu.ftcphoenix.fw.tools.tester.calibration.PinpointPodOffsetCalibrator;
import edu.ftcphoenix.robots.phoenix.RobotConfig;

/**
 * Phoenix robot-specific wrapper for {@link PinpointPodOffsetCalibrator}.
 */
public final class PinpointPodOffsetCalibratorPhoenix {

    private PinpointPodOffsetCalibratorPhoenix() {
    }

    /**
     * Registers the Pinpoint pod-offset calibrator into the Phoenix tester suite.
     */
    public static void register(TesterSuite suite) {
        if (suite == null) return;

        boolean canUseAssist = RobotConfig.Calibration.canUseAprilTagAssist();
        String assistStatus = canUseAssist
                ? "AprilTag assist: AUTO-ENABLED (camera mount OK)"
                : "AprilTag assist: DISABLED (calibrate camera mount first)";

        String offsetsStatus = RobotConfig.Calibration.pinpointPodOffsetsCalibrated
                ? "offsets: OK"
                : "offsets: NOT CALIBRATED";

        suite.add(
                "Calib: Pinpoint Pod Offsets (Robot)",
                "Press PLAY, then rotate in place to estimate pod offsets (Y auto-sample; auto-computes with tags). "
                        + assistStatus + "; " + offsetsStatus,
                () -> {
                    PinpointPodOffsetCalibrator.Config cfg = PinpointPodOffsetCalibrator.Config.defaults();
                    cfg.pinpoint = RobotConfig.Localization.pinpoint;

                    // Provide drivetrain wiring so the calibrator can rotate the robot.
                    cfg.mecanumWiring = RobotConfig.DriveTrain.mecanumWiring();

                    // IMPORTANT: don't use a full 360° turn here.
                    // This calibrator solves offsets from start/end translation drift. After a full
                    // turn, the ideal drift is ~0, so the math becomes ill-conditioned and will blow
                    // up on noise (you'll see absurd offsets like hundreds of inches).
                    //
                    // ~180° gives the strongest signal and a well-conditioned solve.
                    cfg.targetTurnRad = Math.PI;

                    // AprilTag assist is optional. Enable automatically once camera mount is calibrated.
                    cfg.enableAprilTagAssist = canUseAssist;

                    // With AprilTag assist, auto-samples compute automatically after the turn.
                    cfg.autoComputeAfterAutoSample = true;
                    cfg.preferredCameraName = RobotConfig.Vision.nameWebcam;
                    cfg.cameraMount = RobotConfig.Vision.cameraMount;

                    return new PinpointPodOffsetCalibrator(cfg);
                }
        );
    }
}