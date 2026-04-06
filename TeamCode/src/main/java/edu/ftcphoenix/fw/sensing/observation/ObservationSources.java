package edu.ftcphoenix.fw.sensing.observation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.CameraMountLogic;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionSource;

/**
 * Convenience factories for common {@link ObservationSource2d} implementations.
 */
public final class ObservationSources {

    private ObservationSources() {
        // Utility class.
    }

    /**
     * Create an observation source from a shared {@link TagSelectionSource} and a
     * {@link CameraMountConfig} describing where the camera is mounted on the robot.
     *
     * <p>The selector decides which tag is semantically relevant; this helper simply converts the
     * selector's <em>fresh selected observation</em> into a planar robot-frame observation.
     * When no fresh selected observation exists, the source returns {@link TargetObservation2d#none()}.
     * </p>
     */
    public static ObservationSource2d aprilTag(TagSelectionSource selection, CameraMountConfig mount) {
        Objects.requireNonNull(selection, "selection must not be null");
        Objects.requireNonNull(mount, "mount must not be null");

        return new ObservationSource2d() {
            /**
             * {@inheritDoc}
             */
            @Override
            public TargetObservation2d sample(LoopClock clock) {
                TagSelectionResult sel = selection.get(clock);
                if (!sel.hasFreshSelectedObservation) {
                    return TargetObservation2d.none();
                }
                AprilTagObservation obs = sel.selectedObservation;
                return CameraMountLogic.robotObservation2d(obs, mount);
            }

            /**
             * {@inheritDoc}
             */
            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) {
                    return;
                }
                String p = (prefix == null || prefix.isEmpty()) ? "obs2d" : prefix;
                dbg.addData(p + ".candidateIds", selection.candidateIds().toString());
                dbg.addData(p + ".class", getClass().getSimpleName());
                dbg.addData(p + ".type", "apriltagSelection");
            }
        };
    }
}
