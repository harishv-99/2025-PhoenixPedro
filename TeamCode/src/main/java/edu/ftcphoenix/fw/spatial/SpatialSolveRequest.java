package edu.ftcphoenix.fw.spatial;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.field.TagLayout;

/**
 * One per-loop solve request issued by {@link SpatialQuery} to each {@link SpatialSolveLane}.
 *
 * <p>{@link SpatialQuery} samples the dynamic control frames exactly once per loop and places the
 * sampled poses in this request. Every lane therefore solves against the same control-frame
 * snapshot for that loop.</p>
 */
public final class SpatialSolveRequest {

    public final LoopClock clock;
    public final TranslationTarget2d translationTarget;
    public final AimTarget2d aimTarget;
    public final Pose2d robotToTranslationFrame;
    public final Pose2d robotToAimFrame;
    public final TagLayout fixedAprilTagLayout;

    public SpatialSolveRequest(LoopClock clock,
                               TranslationTarget2d translationTarget,
                               AimTarget2d aimTarget,
                               Pose2d robotToTranslationFrame,
                               Pose2d robotToAimFrame,
                               TagLayout fixedAprilTagLayout) {
        this.clock = clock;
        this.translationTarget = translationTarget;
        this.aimTarget = aimTarget;
        this.robotToTranslationFrame = robotToTranslationFrame;
        this.robotToAimFrame = robotToAimFrame;
        this.fixedAprilTagLayout = fixedAprilTagLayout;
    }
}
