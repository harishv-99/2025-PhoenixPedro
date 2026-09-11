package edu.ftcsushi.fw.spatial;

import edu.ftcsushi.fw.core.time.LoopTimestamp;

/**
 * Selected translation solution plus provenance from a {@link SpatialQueryResult}.
 */
public final class SpatialTranslationSelection {
    public final int laneIndex;
    public final TranslationSolution solution;
    /** Exact target-selection provenance from the chosen lane, not that lane's sensor evidence. */
    public final ReferenceSelectionResult selection;

    SpatialTranslationSelection(int laneIndex, TranslationSolution solution, ReferenceSelectionResult selection) {
        this.laneIndex = laneIndex;
        this.solution = solution;
        this.selection = selection;
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
