package edu.ftcphoenix.fw.spatial;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.source.TimeAwareSource;
import edu.ftcphoenix.fw.core.source.TimeAwareSources;
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
    public static TimeAwareSource<Pose2d> robotCenter() {
        return rigid(Pose2d.zero());
    }

    /**
     * Returns a constant rigid robot-frame provider.
     */
    public static TimeAwareSource<Pose2d> rigid(Pose2d robotToFrame) {
        Objects.requireNonNull(robotToFrame, "robotToFrame");
        return new TimeAwareSource<Pose2d>() {
            @Override
            public Pose2d getAt(LoopClock clock, double timestampSec) {
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

    /**
     * Wraps a normal current-loop pose source as a time-aware frame provider.
     *
     * <p>This is explicit current-only behavior. For fast moving mechanisms that are interpreted
     * against delayed camera frames, prefer a history-backed {@link TimeAwareSource}.</p>
     */
    public static TimeAwareSource<Pose2d> currentOnly(Source<Pose2d> source) {
        return TimeAwareSources.currentOnly(source);
    }
}
