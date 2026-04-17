package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.FtcLimelightAprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.FtcWebcamAprilTagVisionLane;

/**
 * Robot-owned factory that selects Phoenix's concrete AprilTag vision lane backend.
 *
 * <p>Phoenix keeps this selection wrapper in the robot layer so the rest of the robot consumes only
 * the backend-neutral {@link AprilTagVisionLane} seam. The framework still owns the concrete FTC
 * boundary implementations for each supported backend.</p>
 */
public final class PhoenixVisionFactory {

    private PhoenixVisionFactory() {
    }

    /**
     * Creates the concrete AprilTag vision lane requested by the supplied Phoenix profile.
     *
     * @param hardwareMap FTC hardware map used to acquire the chosen vision device
     * @param cfg Phoenix AprilTag backend-selection config
     * @return concrete vision lane for the active Phoenix backend
     */
    public static AprilTagVisionLane create(HardwareMap hardwareMap, PhoenixProfile.VisionConfig cfg) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        Objects.requireNonNull(cfg, "cfg");

        switch (cfg.backend) {
            case WEBCAM:
                return new FtcWebcamAprilTagVisionLane(hardwareMap, cfg.webcam);
            case LIMELIGHT:
                return new FtcLimelightAprilTagVisionLane(hardwareMap, cfg.limelight);
            default:
                throw new IllegalArgumentException("Unsupported vision backend: " + cfg.backend);
        }
    }
}
