package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.FtcWebcamAprilTagVisionLane;

/**
 * Phoenix-owned factory for AprilTag vision backend selection.
 *
 * <p>
 * The framework owns the concrete FTC-boundary lane implementations. Phoenix owns the decision
 * of which backend to instantiate for this robot profile. That keeps backend choice in the robot
 * layer while the rest of Phoenix depends only on {@link AprilTagVisionLane}.
 * </p>
 */
public final class PhoenixVisionFactory {

    private PhoenixVisionFactory() {
    }

    /**
     * Creates the configured AprilTag vision lane for Phoenix.
     *
     * @param hardwareMap FTC hardware map used to construct the active backend
     * @param config      Phoenix vision configuration; copied before use
     * @return backend-specific lane exposed through the shared {@link AprilTagVisionLane} seam
     */
    public static AprilTagVisionLane create(HardwareMap hardwareMap,
                                            PhoenixProfile.VisionConfig config) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        PhoenixProfile.VisionConfig cfg =
                Objects.requireNonNull(config, "config").copy();

        switch (cfg.backend) {
            case WEBCAM:
                return new FtcWebcamAprilTagVisionLane(hardwareMap, cfg.webcam);

            case LIMELIGHT:
                throw new UnsupportedOperationException(
                        "PhoenixProfile.vision.backend = LIMELIGHT is reserved for a later stage. "
                                + "Stage 1 introduces the backend-neutral AprilTagVisionLane seam and "
                                + "keeps the checked-in implementation on FtcWebcamAprilTagVisionLane."
                );

            default:
                throw new IllegalArgumentException("Unsupported Phoenix vision backend: " + cfg.backend);
        }
    }
}
