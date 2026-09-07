package edu.ftcsushi.fw.ftc.vision;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;

/**
 * Borrowed AprilTag observations and mount facts. This view cannot open, close, or select the
 * camera; its physical owner is retained separately by the composing robot or diagnostic.
 */
public interface AprilTagVision {
    /** Returns the stable shared tag source, preserving each camera capture timestamp. */
    AprilTagSensor tagSensor();
    /** Returns the fixed robot-to-camera transform used by this capability. */
    CameraMountConfig cameraMountConfig();
    /** Component readiness, distinct from whether any tag is visible. */
    VisionReadiness readiness(LoopClock clock);
    /** Formats only already available diagnostic state. */
    default void debugDump(DebugSink dbg, String prefix) {}
}
