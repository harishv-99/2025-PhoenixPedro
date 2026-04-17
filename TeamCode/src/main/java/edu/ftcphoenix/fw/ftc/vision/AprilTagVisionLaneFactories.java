package edu.ftcphoenix.fw.ftc.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;

/**
 * Convenience builders for common {@link AprilTagVisionLaneFactory} variants.
 *
 * <p>These helpers are mainly useful in testers and robot-layer factories where code wants to keep
 * backend selection explicit without duplicating anonymous opener lambdas everywhere.</p>
 */
public final class AprilTagVisionLaneFactories {

    private AprilTagVisionLaneFactories() {
    }

    /**
     * Returns a deferred opener for a webcam-backed AprilTag lane.
     *
     * @param config webcam lane config to copy and reopen later
     * @return opener that creates {@link FtcWebcamAprilTagVisionLane}
     */
    public static AprilTagVisionLaneFactory webcam(FtcWebcamAprilTagVisionLane.Config config) {
        final FtcWebcamAprilTagVisionLane.Config cfg =
                Objects.requireNonNull(config, "config").copy();
        return new AprilTagVisionLaneFactory() {
            @Override
            public AprilTagVisionLane open(HardwareMap hardwareMap) {
                return new FtcWebcamAprilTagVisionLane(hardwareMap, cfg);
            }

            @Override
            public String description() {
                return "webcam: " + cfg.webcamName;
            }
        };
    }

    /**
     * Returns a deferred opener for a webcam-backed AprilTag lane using just the stable essentials.
     *
     * @param webcamName  webcam hardware-map name
     * @param cameraMount robot→camera extrinsics used by the lane
     * @return opener that creates {@link FtcWebcamAprilTagVisionLane}
     */
    public static AprilTagVisionLaneFactory webcam(String webcamName,
                                                   CameraMountConfig cameraMount) {
        FtcWebcamAprilTagVisionLane.Config cfg = FtcWebcamAprilTagVisionLane.Config.defaults();
        cfg.webcamName = webcamName;
        cfg.cameraMount = (cameraMount != null) ? cameraMount : CameraMountConfig.identity();
        return webcam(cfg);
    }

    /**
     * Returns a deferred opener for a Limelight-backed AprilTag lane.
     *
     * @param config Limelight lane config to copy and reopen later
     * @return opener that creates {@link FtcLimelightAprilTagVisionLane}
     */
    public static AprilTagVisionLaneFactory limelight(FtcLimelightAprilTagVisionLane.Config config) {
        final FtcLimelightAprilTagVisionLane.Config cfg =
                Objects.requireNonNull(config, "config").copy();
        return new AprilTagVisionLaneFactory() {
            @Override
            public AprilTagVisionLane open(HardwareMap hardwareMap) {
                return new FtcLimelightAprilTagVisionLane(hardwareMap, cfg);
            }

            @Override
            public String description() {
                return "limelight: " + cfg.hardwareName;
            }
        };
    }

    /**
     * Returns a deferred opener for a Limelight-backed AprilTag lane using just the stable essentials.
     *
     * @param hardwareName Limelight hardware-map name
     * @param cameraMount  robot→camera extrinsics used by the lane
     * @return opener that creates {@link FtcLimelightAprilTagVisionLane}
     */
    public static AprilTagVisionLaneFactory limelight(String hardwareName,
                                                      CameraMountConfig cameraMount) {
        FtcLimelightAprilTagVisionLane.Config cfg = FtcLimelightAprilTagVisionLane.Config.defaults();
        cfg.hardwareName = hardwareName;
        cfg.cameraMount = (cameraMount != null) ? cameraMount : CameraMountConfig.identity();
        return limelight(cfg);
    }
}
