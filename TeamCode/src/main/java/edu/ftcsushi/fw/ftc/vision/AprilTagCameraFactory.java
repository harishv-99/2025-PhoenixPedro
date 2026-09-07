package edu.ftcsushi.fw.ftc.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Deferred camera acquisition for a diagnostic or robot factory. Every open creates a fresh
 * physical owner and returns an explicit owned handle, never a borrowed closeable capability.
 */
@FunctionalInterface
public interface AprilTagCameraFactory {
    /** Opens one new camera; caller must close the returned owned handle. */
    OwnedAprilTagCamera open(HardwareMap hardwareMap);
    /** Describes the captured backend without acquiring hardware. */
    default String description() { return "AprilTag camera"; }
}
