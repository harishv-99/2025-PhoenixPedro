package edu.ftcphoenix.fw.spatial;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.source.TimeAwareSource;

/**
 * Defines which robot-relative frame providers a spatial query should control.
 *
 * <p>Many tasks care about two different controlled frames:</p>
 * <ul>
 *   <li><b>translation frame</b>: the point/frame that should move to the translation target</li>
 *   <li><b>facing frame</b>: the frame whose +X axis should face the facing target</li>
 * </ul>
 *
 * <p>Both frames are expressed as {@code robot -> frame} transforms. They may be rigid, dynamic, or
 * history-backed. A history-backed {@link TimeAwareSource} is important for fast moving mechanisms
 * such as a turret-mounted camera: delayed camera observations should be interpreted using the
 * control-frame pose from the camera-frame timestamp.</p>
 */
public final class SpatialControlFrames {

    private final TimeAwareSource<Pose2d> translationFrame;
    private final TimeAwareSource<Pose2d> facingFrame;

    private SpatialControlFrames(TimeAwareSource<Pose2d> translationFrame,
                                 TimeAwareSource<Pose2d> facingFrame) {
        this.translationFrame = Objects.requireNonNull(translationFrame, "translationFrame");
        this.facingFrame = Objects.requireNonNull(facingFrame, "facingFrame");
    }

    /**
     * Returns control frames with both channels bound to the robot center.
     */
    public static SpatialControlFrames robotCenter() {
        return new SpatialControlFrames(RobotFrames.robotCenter(), RobotFrames.robotCenter());
    }

    /**
     * Creates control frames from explicit time-aware frame providers.
     */
    public static SpatialControlFrames of(TimeAwareSource<Pose2d> translationFrame,
                                          TimeAwareSource<Pose2d> facingFrame) {
        return new SpatialControlFrames(translationFrame, facingFrame);
    }

    /**
     * Creates control frames from current-only dynamic frame providers.
     */
    public static SpatialControlFrames of(Source<Pose2d> translationFrame, Source<Pose2d> facingFrame) {
        return new SpatialControlFrames(RobotFrames.currentOnly(translationFrame), RobotFrames.currentOnly(facingFrame));
    }

    /**
     * Creates control frames from explicit rigid robot->frame poses.
     */
    public static SpatialControlFrames of(Pose2d robotToTranslationFrame, Pose2d robotToFacingFrame) {
        return new SpatialControlFrames(RobotFrames.rigid(robotToTranslationFrame), RobotFrames.rigid(robotToFacingFrame));
    }

    /**
     * Returns the sampled provider for the translation control frame.
     */
    public TimeAwareSource<Pose2d> translationFrame() {
        return translationFrame;
    }

    /**
     * Returns the sampled provider for the facing control frame.
     */
    public TimeAwareSource<Pose2d> facingFrame() {
        return facingFrame;
    }

    /** Returns a copy with a rigid translation frame pose. */
    public SpatialControlFrames withTranslationFrame(Pose2d robotToTranslationFrame) {
        return withTranslationFrame(RobotFrames.rigid(robotToTranslationFrame));
    }

    /** Returns a copy with a current-only dynamic translation-frame provider. */
    public SpatialControlFrames withTranslationFrame(Source<Pose2d> translationFrame) {
        return withTranslationFrame(RobotFrames.currentOnly(translationFrame));
    }

    /**
     * Returns a copy with a dynamic or rigid translation-frame provider.
     */
    public SpatialControlFrames withTranslationFrame(TimeAwareSource<Pose2d> translationFrame) {
        return new SpatialControlFrames(translationFrame, this.facingFrame);
    }

    /**
     * Returns a copy with a rigid facing-frame pose.
     */
    public SpatialControlFrames withFacingFrame(Pose2d robotToFacingFrame) {
        return withFacingFrame(RobotFrames.rigid(robotToFacingFrame));
    }

    /**
     * Returns a copy with a current-only dynamic facing-frame provider.
     */
    public SpatialControlFrames withFacingFrame(Source<Pose2d> facingFrame) {
        return withFacingFrame(RobotFrames.currentOnly(facingFrame));
    }

    /**
     * Returns a copy with a dynamic or rigid facing-frame provider.
     */
    public SpatialControlFrames withFacingFrame(TimeAwareSource<Pose2d> facingFrame) {
        return new SpatialControlFrames(this.translationFrame, facingFrame);
    }

    /** Resets both underlying frame providers. */
    public void reset() {
        translationFrame.reset();
        facingFrame.reset();
    }

    /** Emits debug information for both underlying frame providers. */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "spatialFrames" : prefix;
        translationFrame.debugDump(dbg, p + ".translation");
        facingFrame.debugDump(dbg, p + ".facing");
    }

    @Override
    public String toString() {
        return "SpatialControlFrames{translationFrame=" + translationFrame + ", facingFrame=" + facingFrame + '}';
    }
}
