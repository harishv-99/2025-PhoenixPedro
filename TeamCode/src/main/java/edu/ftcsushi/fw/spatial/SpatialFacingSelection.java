package edu.ftcsushi.fw.spatial;

import edu.ftcsushi.fw.core.time.LoopTimestamp;

/**
 * Selected facing solution plus provenance from a {@link SpatialQueryResult}.
 */
public final class SpatialFacingSelection {
    public final int laneIndex;
    public final FacingSolution solution;
    /** Exact target-selection provenance from the chosen lane, not that lane's sensor evidence. */
    public final ReferenceSelectionResult selection;

    SpatialFacingSelection(int laneIndex, FacingSolution solution, ReferenceSelectionResult selection) {
        this.laneIndex = laneIndex;
        this.solution = solution;
        this.selection = selection;
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
     * Returns the epoch-safe timestamp of the underlying solution.
     */
    public LoopTimestamp timestamp() {
        return solution.timestamp;
    }
}
