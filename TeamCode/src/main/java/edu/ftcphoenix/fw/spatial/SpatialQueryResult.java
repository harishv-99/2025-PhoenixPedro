package edu.ftcphoenix.fw.spatial;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;

/**
 * One coherent per-loop sample from {@link SpatialQuery}.
 *
 * <p>The sample includes the control-frame poses used for this loop and the per-lane solve results
 * generated from those same poses. Consumers can therefore make fair comparisons between lanes
 * without worrying that each lane sampled a slightly different dynamic frame.</p>
 */
public final class SpatialQueryResult {

    public final TranslationTarget2d translationTarget;
    public final FacingTarget2d facingTarget;
    public final Pose2d robotToTranslationFrame;
    public final Pose2d robotToFacingFrame;

    private final List<SpatialLaneResult> laneResults;

    SpatialQueryResult(TranslationTarget2d translationTarget,
                       FacingTarget2d facingTarget,
                       Pose2d robotToTranslationFrame,
                       Pose2d robotToFacingFrame,
                       List<SpatialLaneResult> laneResults) {
        this.translationTarget = translationTarget;
        this.facingTarget = facingTarget;
        this.robotToTranslationFrame = Objects.requireNonNull(robotToTranslationFrame, "robotToTranslationFrame");
        this.robotToFacingFrame = Objects.requireNonNull(robotToFacingFrame, "robotToFacingFrame");
        this.laneResults = Collections.unmodifiableList(new ArrayList<SpatialLaneResult>(laneResults));
    }

    /**
     * Returns how many ordered lane results are present in this sample.
     */
    public int laneCount() {
        return laneResults.size();
    }

    /**
     * Returns one lane result by ordered index.
     */
    public SpatialLaneResult laneResult(int index) {
        return laneResults.get(index);
    }

    /**
     * Returns the immutable ordered lane-result list.
     */
    public List<SpatialLaneResult> laneResults() {
        return laneResults;
    }

    @Override
    public String toString() {
        return "SpatialQueryResult{translationTarget=" + translationTarget
                + ", facingTarget=" + facingTarget
                + ", robotToTranslationFrame=" + robotToTranslationFrame
                + ", robotToFacingFrame=" + robotToFacingFrame
                + ", laneResults=" + laneResults + '}';
    }
}
