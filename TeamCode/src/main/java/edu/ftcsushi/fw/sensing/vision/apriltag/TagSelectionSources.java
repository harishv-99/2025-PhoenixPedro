package edu.ftcsushi.fw.sensing.vision.apriltag;

import java.util.Objects;

import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.Source;

/** Borrowed read projections. Reset never resets the shared selector or any of its inputs. */
public final class TagSelectionSources {
    private TagSelectionSources() { }

    /** True when the selector has an identity, with or without current geometry. */
    public static BooleanSource hasSelection(TagSelectionSource selection) {
        Objects.requireNonNull(selection, "selection");
        return clock -> selection.get(clock).hasSelection;
    }

    /** True only for a current actual observation, never for pose-inferred geometry. */
    public static BooleanSource hasFreshSelectedObservation(TagSelectionSource selection) {
        Objects.requireNonNull(selection, "selection");
        return clock -> selection.get(clock).hasFreshSelectedObservation;
    }

    /** Current selected identity, or the caller's fallback ID when none is selected. */
    public static Source<Integer> selectedTagId(TagSelectionSource selection, int fallbackId) {
        Objects.requireNonNull(selection, "selection");
        return Source.of(clock -> {
            TagSelectionResult result = selection.get(clock);
            return result.hasSelection ? result.selectedTagId : fallbackId;
        });
    }

    /** Current actual observation or {@link AprilTagObservation#noTarget()}. */
    public static Source<AprilTagObservation> selectedObservation(TagSelectionSource selection) {
        Objects.requireNonNull(selection, "selection");
        return Source.of(clock -> selection.get(clock).selectedObservation);
    }
}
