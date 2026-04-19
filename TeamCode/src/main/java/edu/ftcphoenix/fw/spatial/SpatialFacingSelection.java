package edu.ftcphoenix.fw.spatial;

import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;

/**
 * Selected facing solution plus provenance from a {@link SpatialQueryResult}.
 */
public final class SpatialFacingSelection {
    public final int laneIndex;
    public final FacingSolution solution;
    public final TagSelectionResult tagSelection;

    SpatialFacingSelection(int laneIndex, FacingSolution solution, TagSelectionResult tagSelection) {
        this.laneIndex = laneIndex;
        this.solution = solution;
        this.tagSelection = tagSelection;
    }

    /**
     * Returns a stable source id suitable for scalar candidate provenance and telemetry.
     */
    public String sourceId() {
        return "spatial-facing-lane-" + laneIndex;
    }

    /**
     * Signed facing error in radians.
     */
    public double facingErrorRad() {
        return solution.facingErrorRad;
    }

    /**
     * Lane-specific quality score.
     */
    public double quality() {
        return solution.quality;
    }

    /**
     * Age of the underlying solution in seconds.
     */
    public double ageSec() {
        return solution.ageSec;
    }

    /**
     * Timestamp of the underlying solution, or NaN if unknown.
     */
    public double timestampSec() {
        return solution.timestampSec;
    }
}
