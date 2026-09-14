package edu.ftcsushi.robots.examples.cameraonlypickup;

import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.vision.FtcWebcamVisionLane;
import edu.ftcsushi.fw.sensing.observation.TargetObservations2d;

/** Owns one fixed webcam lifetime; selection and pickup borrow its located-object source. */
final class CameraOnlyPickupCamera implements RobotProgram.Service {
    private final FtcWebcamVisionLane camera;
    private boolean stopped;

    /** Opens the complete configured camera graph once; the lane snapshots its configuration. */
    CameraOnlyPickupCamera(HardwareMap hardwareMap, FtcWebcamVisionLane.Config config) {
        camera = new FtcWebcamVisionLane(hardwareMap, config);
    }

    /** Returns robot-at-capture positions, not field memory. */
    Source<TargetObservations2d> objects() { return camera.floorObjects(); }

    /** Frame sources sample lazily in the Task phase; this owner needs no extra poll or clock. */
    @Override public void update(LoopClock clock) { }

    /** Closes the owned camera exactly once after dependent pickup cleanup. */
    @Override public void stop() {
        if (stopped) return;
        stopped = true;
        camera.close();
    }
}
