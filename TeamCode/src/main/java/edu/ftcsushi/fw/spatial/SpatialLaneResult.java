package edu.ftcsushi.fw.spatial;

/**
 * Per-lane result from one {@link SpatialQuery} sample.
 *
 * <p>A lane may solve translation, facing, both, or neither. The query deliberately keeps every
 * lane result visible so higher-level consumers can apply their own selection/fusion policy.</p>
 */
public final class SpatialLaneResult {

    private static final ReferenceSelectionResult NO_SELECTION =
            ReferenceSelectionResult.none();

    public final TranslationSolution translation;
    public final FacingSolution facing;
    /** Exact target selection provenance, even when the translation solve is unavailable. */
    public final ReferenceSelectionResult translationSelection;
    /** Exact target selection provenance, independent of the facing solve's evidence authority. */
    public final ReferenceSelectionResult facingSelection;

    private SpatialLaneResult(TranslationSolution translation,
                              FacingSolution facing,
                              ReferenceSelectionResult translationSelection,
                              ReferenceSelectionResult facingSelection) {
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

    /**
     * Creates a lane result from solved channel outputs and selection snapshots.
     * A null selection becomes {@link ReferenceSelectionResult#none()}; a retained domain result
     * is not discarded merely because its corresponding solution is null.
     */
    public static SpatialLaneResult of(TranslationSolution translation,
                                       FacingSolution facing,
                                       ReferenceSelectionResult translationSelection,
                                       ReferenceSelectionResult facingSelection) {
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
