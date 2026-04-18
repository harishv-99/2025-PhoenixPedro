package edu.ftcphoenix.fw.spatial;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.source.Source;

/**
 * Defines which robot-relative frame providers a spatial query should control.
 *
 * <p>Many tasks care about two different controlled frames:</p>
 * <ul>
 *   <li><b>translation frame</b>: the point that should move to the translation target</li>
 *   <li><b>aim frame</b>: the frame whose +X axis should aim at the aim target</li>
 * </ul>
 *
 * <p>Both frames are expressed as {@code robot -> frame} transforms. They may be rigid or dynamic.
 * For a rigid frame, pass a {@link Pose2d} directly. For a dynamic mechanism frame, pass a
 * {@link Source}&lt;{@link Pose2d}&gt; that samples the current robot-relative frame each loop.</p>
 */
public final class SpatialControlFrames {

    private final Source<Pose2d> translationFrame;
    private final Source<Pose2d> aimFrame;

    private SpatialControlFrames(Source<Pose2d> translationFrame, Source<Pose2d> aimFrame) {
        this.translationFrame = Objects.requireNonNull(translationFrame, "translationFrame");
        this.aimFrame = Objects.requireNonNull(aimFrame, "aimFrame");
    }

    /**
     * Returns control frames with both channels bound to the robot center.
     */
    public static SpatialControlFrames robotCenter() {
        return new SpatialControlFrames(RobotFrames.robotCenter(), RobotFrames.robotCenter());
    }

    /**
     * Creates control frames from explicit dynamic or rigid frame providers.
     */
    public static SpatialControlFrames of(Source<Pose2d> translationFrame, Source<Pose2d> aimFrame) {
        return new SpatialControlFrames(translationFrame, aimFrame);
    }

    /**
     * Creates control frames from explicit rigid robot->frame poses.
     */
    public static SpatialControlFrames of(Pose2d robotToTranslationFrame, Pose2d robotToAimFrame) {
        return new SpatialControlFrames(RobotFrames.rigid(robotToTranslationFrame), RobotFrames.rigid(robotToAimFrame));
    }

    /**
     * Returns the sampled provider for the translation control frame.
     */
    public Source<Pose2d> translationFrame() {
        return translationFrame;
    }

    /**
     * Returns the sampled provider for the aim control frame.
     */
    public Source<Pose2d> aimFrame() {
        return aimFrame;
    }

    /**
     * Returns a copy with a rigid translation frame pose.
     */
    public SpatialControlFrames withTranslationFrame(Pose2d robotToTranslationFrame) {
        return withTranslationFrame(RobotFrames.rigid(robotToTranslationFrame));
    }

    /**
     * Returns a copy with a dynamic or rigid translation-frame provider.
     */
    public SpatialControlFrames withTranslationFrame(Source<Pose2d> translationFrame) {
        return new SpatialControlFrames(translationFrame, this.aimFrame);
    }

    /**
     * Returns a copy with a rigid aim-frame pose.
     */
    public SpatialControlFrames withAimFrame(Pose2d robotToAimFrame) {
        return withAimFrame(RobotFrames.rigid(robotToAimFrame));
    }

    /**
     * Returns a copy with a dynamic or rigid aim-frame provider.
     */
    public SpatialControlFrames withAimFrame(Source<Pose2d> aimFrame) {
        return new SpatialControlFrames(this.translationFrame, aimFrame);
    }

    /**
     * Resets both underlying frame providers.
     */
    public void reset() {
        translationFrame.reset();
        aimFrame.reset();
    }

    /**
     * Emits debug information for both underlying frame providers.
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "spatialFrames" : prefix;
        translationFrame.debugDump(dbg, p + ".translation");
        aimFrame.debugDump(dbg, p + ".aim");
    }

    @Override
    public String toString() {
        return "SpatialControlFrames{translationFrame=" + translationFrame + ", aimFrame=" + aimFrame + '}';
    }
}
