package edu.ftcphoenix.fw.sensing.vision.apriltag;

import java.util.Objects;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.Source;

/**
 * Small source-graph helpers built on top of {@link TagSelectionSource}.
 */
public final class TagSelectionSources {

    private TagSelectionSources() {
        // Utility class.
    }

    /**
     * True when the selector currently has a selected tag ID.
     */
    public static BooleanSource hasSelection(TagSelectionSource selection) {
        Objects.requireNonNull(selection, "selection");
        return selection.mapToBoolean(new java.util.function.Predicate<TagSelectionResult>() {
            /**
             * {@inheritDoc}
             */
            @Override
            public boolean test(TagSelectionResult result) {
                return result != null && result.hasSelection;
            }
        });
    }

    /**
     * True when the selector currently has a fresh observation for its selected tag.
     */
    public static BooleanSource hasFreshSelectedObservation(TagSelectionSource selection) {
        Objects.requireNonNull(selection, "selection");
        return selection.mapToBoolean(new java.util.function.Predicate<TagSelectionResult>() {
            /**
             * {@inheritDoc}
             */
            @Override
            public boolean test(TagSelectionResult result) {
                return result != null && result.hasFreshSelectedObservation;
            }
        });
    }

    /**
     * Selected tag ID, or {@code fallbackId} when no selection exists.
     */
    public static Source<Integer> selectedTagId(TagSelectionSource selection, int fallbackId) {
        Objects.requireNonNull(selection, "selection");
        return selection.map(new java.util.function.Function<TagSelectionResult, Integer>() {
            /**
             * {@inheritDoc}
             */
            @Override
            public Integer apply(TagSelectionResult result) {
                return (result != null && result.hasSelection) ? Integer.valueOf(result.selectedTagId) : Integer.valueOf(fallbackId);
            }
        });
    }

    /**
     * The currently selected fresh observation, or {@link AprilTagObservation#noTarget(double)}.
     */
    public static Source<AprilTagObservation> selectedObservation(TagSelectionSource selection) {
        Objects.requireNonNull(selection, "selection");
        return selection.map(new java.util.function.Function<TagSelectionResult, AprilTagObservation>() {
            /**
             * {@inheritDoc}
             */
            @Override
            public AprilTagObservation apply(TagSelectionResult result) {
                return (result != null && result.hasFreshSelectedObservation)
                        ? result.selectedObservation
                        : AprilTagObservation.noTarget(Double.POSITIVE_INFINITY);
            }
        });
    }
}
