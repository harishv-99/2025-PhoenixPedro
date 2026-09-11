package edu.ftcsushi.robots.examples.tagalignment;

import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.vision.AprilTagVision;
import edu.ftcsushi.fw.ftc.vision.FtcWebcamVisionLane;

/** Lifecycle adapter for the one camera; guidance only borrows its AprilTag view. */
final class TagAlignmentCamera implements RobotProgram.Service {
    private final FtcWebcamVisionLane camera;
    private boolean stopped;

    TagAlignmentCamera(HardwareMap hardwareMap, FtcWebcamVisionLane.Config config) {
        camera = new FtcWebcamVisionLane(hardwareMap, config);
    }

    AprilTagVision tags() { return camera.aprilTags(); }

    @Override public void update(LoopClock clock) {
        // The camera's shared source captures a frame lazily when guidance samples it.
        // It needs no second clock and no separate frame poll here.
    }

    @Override public void stop() {
        if (stopped) return;
        stopped = true;
        camera.close();
    }
}
