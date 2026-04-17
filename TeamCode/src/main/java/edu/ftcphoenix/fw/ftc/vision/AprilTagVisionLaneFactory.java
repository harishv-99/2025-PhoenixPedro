package edu.ftcphoenix.fw.ftc.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Deferred opener for a configured {@link AprilTagVisionLane}.
 *
 * <p>This seam is intentionally tiny. It lets higher-level code such as testers accept “whatever
 * AprilTag backend this robot wants to use” without baking webcam-specific FTC assumptions into the
 * tester itself. Concrete factories typically capture an immutable backend config snapshot and open
 * the real lane later when a {@link HardwareMap} becomes available.</p>
 */
@FunctionalInterface
public interface AprilTagVisionLaneFactory {

    /**
     * Opens a configured AprilTag vision lane against one FTC hardware map.
     *
     * @param hardwareMap FTC hardware map used to acquire the underlying camera device
     * @return opened vision lane; caller owns its lifecycle
     */
    AprilTagVisionLane open(HardwareMap hardwareMap);

    /**
     * Human-facing description of the backend/config this factory opens.
     *
     * <p>Testers use this only for telemetry so operators can see which backend/device they are
     * about to initialize. Implementations may return a broad label such as {@code "webcam"} or a
     * more specific label such as {@code "limelight: frontLL"}.</p>
     *
     * @return short driver-facing description
     */
    default String description() {
        return "AprilTag vision";
    }
}
