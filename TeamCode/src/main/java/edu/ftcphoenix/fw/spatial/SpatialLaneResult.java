package edu.ftcphoenix.fw.spatial;

import java.util.Collections;

import edu.ftcphoenix.fw.sensing.vision.apriltag.TagSelectionResult;

/**
 * Per-lane result from one {@link SpatialQuery} sample.
 *
 * <p>A lane may solve translation, facing, both, or neither. The query deliberately keeps every
 * lane result visible so higher-level consumers can apply their own selection/fusion policy.</p>
 */
public final class SpatialLaneResult {

    private static final TagSelectionResult NO_SELECTION =
            TagSelectionResult.none(Collections.<Integer>emptySet());

    public final TranslationSolution translation;
    public final FacingSolution facing;
    public final TagSelectionResult translationSelection;
    public final TagSelectionResult facingSelection;

    private SpatialLaneResult(TranslationSolution translation,
                              FacingSolution facing,
                              TagSelectionResult translationSelection,
                              TagSelectionResult facingSelection) {
        this.translation = translation;
        this.facing = facing;
        this.translationSelection = translationSelection != null ? translationSelection : NO_SELECTION;
        this.facingSelection = facingSelection != null ? facingSelection : NO_SELECTION;
    }

    /**
     * Returns an empty lane result for a lane that could not solve either channel this loop.
     */
    public static SpatialLaneResult none() {
        return new SpatialLaneResult(null, null, NO_SELECTION, NO_SELECTION);
    }

    /** Creates a lane result from solved channel outputs and selection snapshots. */
    public static SpatialLaneResult of(TranslationSolution translation,
                                       FacingSolution facing,
                                       TagSelectionResult translationSelection,
                                       TagSelectionResult facingSelection) {
        return new SpatialLaneResult(translation, facing, translationSelection, facingSelection);
    }

    /** Returns whether this lane solved the translation channel. */
    public boolean hasTranslation() {
        return translation != null;
    }

    /**
     * Returns whether this lane solved the facing channel.
     */
    public boolean hasFacing() {
        return facing != null;
    }

    /** Returns whether this lane solved at least one requested channel. */
    public boolean valid() {
        return hasTranslation() || hasFacing();
    }

    @Override
    public String toString() {
        return "SpatialLaneResult{translation=" + translation + ", facing=" + facing
                + ", translationSelection=" + translationSelection
                + ", facingSelection=" + facingSelection + '}';
    }
}
