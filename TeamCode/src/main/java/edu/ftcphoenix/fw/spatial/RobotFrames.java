package edu.ftcphoenix.fw.spatial;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Convenience factories for {@code robot -> frame} providers used by {@link SpatialControlFrames}.
 */
public final class RobotFrames {

    private RobotFrames() {
        // utility holder
    }

    /**
     * Returns a constant robot-center frame provider ({@code robot -> robotCenter = identity}).
     */
    public static Source<Pose2d> robotCenter() {
        return rigid(Pose2d.zero());
    }

    /**
     * Returns a constant rigid robot-frame provider.
     */
    public static Source<Pose2d> rigid(Pose2d robotToFrame) {
        Objects.requireNonNull(robotToFrame, "robotToFrame");
        return new Source<Pose2d>() {
            @Override
            public Pose2d get(LoopClock clock) {
                return robotToFrame;
            }

            @Override
            public void debugDump(DebugSink dbg, String prefix) {
                if (dbg == null) {
                    return;
                }
                String p = (prefix == null || prefix.isEmpty()) ? "rigidFrame" : prefix;
                dbg.addData(p + ".pose", robotToFrame);
            }

            @Override
            public String toString() {
                return "RigidRobotFrame{" + robotToFrame + '}';
            }
        };
    }
}
