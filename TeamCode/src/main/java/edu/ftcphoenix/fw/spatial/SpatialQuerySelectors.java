package edu.ftcphoenix.fw.spatial;

/**
 * Shared selectors for choosing one acceptable lane result from a {@link SpatialQueryResult}.
 *
 * <p>The base {@link SpatialQuery} intentionally returns every lane result. Selectors are used by
 * higher-level consumers such as drive guidance and scalar setpoint request builders when they need
 * a simple priority choice. More advanced consumers may inspect all lanes directly.</p>
 */
public final class SpatialQuerySelectors {

    private SpatialQuerySelectors() {
        // utility
    }

    /**
     * Returns the first facing solution, in solve-set order, that passes {@code gate}.
     */
    public static SpatialFacingSelection firstValidFacing(SpatialQueryResult result, SpatialSolutionGate gate) {
        if (result == null) {
            return null;
        }
        SpatialSolutionGate g = gate != null ? gate : SpatialSolutionGate.defaults();
        for (int i = 0; i < result.laneCount(); i++) {
            SpatialLaneResult lane = result.laneResult(i);
            if (lane != null && g.accepts(lane.facing)) {
                return new SpatialFacingSelection(i, lane.facing, lane.facingSelection);
            }
        }
        return null;
    }

    /**
     * Returns the first translation solution, in solve-set order, that passes {@code gate}.
     */
    public static SpatialTranslationSelection firstValidTranslation(SpatialQueryResult result,
                                                                    SpatialSolutionGate gate) {
        if (result == null) {
            return null;
        }
        SpatialSolutionGate g = gate != null ? gate : SpatialSolutionGate.defaults();
        for (int i = 0; i < result.laneCount(); i++) {
            SpatialLaneResult lane = result.laneResult(i);
            if (lane != null && g.accepts(lane.translation)) {
                return new SpatialTranslationSelection(i, lane.translation, lane.translationSelection);
            }
        }
        return null;
    }
}
