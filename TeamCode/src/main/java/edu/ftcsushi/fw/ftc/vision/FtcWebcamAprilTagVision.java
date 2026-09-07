package edu.ftcsushi.fw.ftc.vision;

import java.util.Objects;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;

/** Stable borrowed tag capability; the webcam remains its sole lifecycle owner. */
final class FtcWebcamAprilTagVision implements AprilTagVision {
    private final FtcWebcamVisionLane owner;
    private final AprilTagProcessor processor;
    private final CameraMountConfig mount;
    private final AprilTagSensor sensor;

    FtcWebcamAprilTagVision(FtcWebcamVisionLane owner, AprilTagProcessor processor,
                           CameraMountConfig mount) {
        this.owner = Objects.requireNonNull(owner, "owner");
        this.processor = Objects.requireNonNull(processor, "processor");
        this.mount = Objects.requireNonNull(mount, "mount");
        this.sensor = new FtcWebcamAprilTagSupport.PortalAprilTagSensor(owner, processor);
    }

    @Override public AprilTagSensor tagSensor() { return sensor; }
    @Override public CameraMountConfig cameraMountConfig() { return mount; }
    @Override public VisionReadiness readiness(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        return owner.processorReadiness(processor);
    }
    @Override public void debugDump(DebugSink dbg, String prefix) {
        sensor.debugDump(dbg, prefix);
    }
}
