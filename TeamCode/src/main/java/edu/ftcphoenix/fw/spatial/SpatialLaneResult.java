package edu.ftcphoenix.fw.spatial;

import java.util.Collections;

import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;

/**
 * Per-lane result from one {@link SpatialQuery} sample.
 */
public final class SpatialLaneResult {

    private static final TagSelectionResult NO_SELECTION =
            TagSelectionResult.none(Collections.<Integer>emptySet());

    public final TranslationSolution translation;
    public final AimSolution aim;
    public final TagSelectionResult translationSelection;
    public final TagSelectionResult aimSelection;

    private SpatialLaneResult(TranslationSolution translation,
                              AimSolution aim,
                              TagSelectionResult translationSelection,
                              TagSelectionResult aimSelection) {
        this.translation = translation;
        this.aim = aim;
        this.translationSelection = translationSelection != null ? translationSelection : NO_SELECTION;
        this.aimSelection = aimSelection != null ? aimSelection : NO_SELECTION;
    }

    /**
     * Returns an empty lane result for a lane that could not solve either channel this loop.
     */
    public static SpatialLaneResult none() {
        return new SpatialLaneResult(null, null, NO_SELECTION, NO_SELECTION);
    }

    /**
     * Creates a lane result from the solved channel outputs and any selection snapshots.
     */
    public static SpatialLaneResult of(TranslationSolution translation,
                                       AimSolution aim,
                                       TagSelectionResult translationSelection,
                                       TagSelectionResult aimSelection) {
        return new SpatialLaneResult(translation, aim, translationSelection, aimSelection);
    }

    /**
     * Returns whether this lane solved the translation channel.
     */
    public boolean hasTranslation() {
        return translation != null;
    }

    /**
     * Returns whether this lane solved the aim channel.
     */
    public boolean hasAim() {
        return aim != null;
    }

    /**
     * Returns whether this lane solved at least one requested channel.
     */
    public boolean valid() {
        return hasTranslation() || hasAim();
    }

    @Override
    public String toString() {
        return "SpatialLaneResult{translation=" + translation + ", aim=" + aim
                + ", translationSelection=" + translationSelection
                + ", aimSelection=" + aimSelection + '}';
    }
}
