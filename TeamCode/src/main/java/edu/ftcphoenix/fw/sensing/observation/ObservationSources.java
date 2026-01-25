package edu.ftcphoenix.fw.sensing.observation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.CameraMountLogic;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagTarget;

/**
 * Convenience factories for common {@link ObservationSource2d} implementations.
 */
public final class ObservationSources {

    private ObservationSources() {
        // Utility class.
    }

    /**
     * Create an observation source from a {@link TagTarget} (AprilTag tracking) and a
     * {@link CameraMountConfig} describing where the camera is mounted on the robot.
     */
    public static ObservationSource2d aprilTag(TagTarget target, CameraMountConfig mount) {
        Objects.requireNonNull(target, "target must not be null");
        Objects.requireNonNull(mount, "mount must not be null");

        return new ObservationSource2d() {
            @Override
            public TargetObservation2d sample(LoopClock clock) {
                target.update(clock);
                if (!target.hasTarget()) {
                    return TargetObservation2d.none();
                }
                // Convert the latest camera observation into a robot-relative planar observation.
                return CameraMountLogic.robotObservation2d(target.last(), mount);
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) {
                    return;
                }
                String p = (prefix == null || prefix.isEmpty()) ? "obs2d" : prefix;
                dbg.addData(p + ".class", getClass().getSimpleName());
                dbg.addData(p + ".type", "apriltag");
                // TagTarget may track multiple IDs; expose both the set and the currently-tracked ID
                // (from the most recent observation) for easy debugging.
                dbg.addData(p + ".idsOfInterest", target.idsOfInterest().toString());
                dbg.addData(p + ".tagId", target.last().id);
            }
        };
    }
}
