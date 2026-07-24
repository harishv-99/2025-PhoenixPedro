package edu.ftcphoenix.fw.spatial;

import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;

/**
 * Selected translation solution plus provenance from a {@link SpatialQueryResult}.
 */
public final class SpatialTranslationSelection {
    public final int laneIndex;
    public final TranslationSolution solution;
    public final TagSelectionResult tagSelection;

    SpatialTranslationSelection(int laneIndex, TranslationSolution solution, TagSelectionResult tagSelection) {
        this.laneIndex = laneIndex;
        this.solution = solution;
        this.tagSelection = tagSelection;
    }

    /**
     * Returns a stable source id suitable for scalar candidate provenance and telemetry.
     */
    public String sourceId() {
        return "spatial-translation-lane-" + laneIndex;
    }

    /**
     * Lane-specific quality score.
     */
    public double quality() {
        return solution.quality;
    }

    /**
     * Returns the epoch-safe timestamp of the underlying solution.
     */
    public LoopTimestamp timestamp() {
        return solution.timestamp;
    }
}
