package edu.ftcphoenix.fw.spatial;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.source.TimeAwareSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.field.TagLayout;

/**
 * One solve request issued by {@link SpatialQuery} to each {@link SpatialSolveLane}.
 *
 * <p>The request includes both current-loop sampled frame poses and time-aware frame providers.
 * Lanes that solve from current state can use {@link #robotToTranslationFrame} and
 * {@link #robotToFacingFrame}. Lanes that interpret delayed sensor measurements should call
 * {@link #robotToTranslationFrameAt(double)} or {@link #robotToFacingFrameAt(double)} with the
 * measurement timestamp so moving mechanism frames are interpreted at the right time.</p>
 */
public final class SpatialSolveRequest {

    public final LoopClock clock;
    public final TranslationTarget2d translationTarget;
    public final FacingTarget2d facingTarget;
    public final TimeAwareSource<Pose2d> translationFrame;
    public final TimeAwareSource<Pose2d> facingFrame;
    public final Pose2d robotToTranslationFrame;
    public final Pose2d robotToFacingFrame;
    public final TagLayout fixedAprilTagLayout;

    public SpatialSolveRequest(LoopClock clock,
                               TranslationTarget2d translationTarget,
                               FacingTarget2d facingTarget,
                               TimeAwareSource<Pose2d> translationFrame,
                               TimeAwareSource<Pose2d> facingFrame,
                               Pose2d robotToTranslationFrame,
                               Pose2d robotToFacingFrame,
                               TagLayout fixedAprilTagLayout) {
        this.clock = clock;
        this.translationTarget = translationTarget;
        this.facingTarget = facingTarget;
        this.translationFrame = translationFrame;
        this.facingFrame = facingFrame;
        this.robotToTranslationFrame = robotToTranslationFrame;
        this.robotToFacingFrame = robotToFacingFrame;
        this.fixedAprilTagLayout = fixedAprilTagLayout;
    }

    /**
     * Returns the translation frame at a measurement timestamp, falling back to current frame if needed.
     */
    public Pose2d robotToTranslationFrameAt(double timestampSec) {
        if (translationFrame == null || !Double.isFinite(timestampSec)) {
            return robotToTranslationFrame;
        }
        return translationFrame.getAt(clock, timestampSec);
    }

    /**
     * Returns the facing frame at a measurement timestamp, falling back to current frame if needed.
     */
    public Pose2d robotToFacingFrameAt(double timestampSec) {
        if (facingFrame == null || !Double.isFinite(timestampSec)) {
            return robotToFacingFrame;
        }
        return facingFrame.getAt(clock, timestampSec);
    }
}
