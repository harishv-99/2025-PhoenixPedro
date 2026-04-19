package edu.ftcphoenix.fw.spatial;

import java.util.Objects;

import edu.ftcphoenix.fw.field.TagLayout;

/**
 * Immutable, controller-neutral description of a spatial relationship to solve.
 *
 * <p>A spec answers four questions:</p>
 * <ol>
 *   <li><b>What translation relationship matters?</b> optionally a {@link TranslationTarget2d}</li>
 *   <li><b>What facing relationship matters?</b> optionally a {@link FacingTarget2d}</li>
 *   <li><b>Which robot-relative frames are controlled?</b> {@link SpatialControlFrames}</li>
 *   <li><b>Which solve lanes may answer the query?</b> {@link SpatialSolveSet}</li>
 * </ol>
 *
 * <p>Use {@link SpatialQuery#builder()} for the common case. Use this spec when you want to create
 * multiple independent runtime queries with the same immutable description.</p>
 */
public final class SpatialQuerySpec {

    public final TranslationTarget2d translationTarget;
    public final FacingTarget2d facingTarget;
    public final SpatialControlFrames controlFrames;
    public final SpatialSolveSet solveSet;
    public final TagLayout fixedAprilTagLayout;

    public SpatialQuerySpec(TranslationTarget2d translationTarget,
                            FacingTarget2d facingTarget,
                            SpatialControlFrames controlFrames,
                            SpatialSolveSet solveSet,
                            TagLayout fixedAprilTagLayout) {
        if (translationTarget == null && facingTarget == null) {
            throw new IllegalArgumentException("SpatialQuerySpec needs translateTo(...), faceTo(...), or both");
        }
        this.translationTarget = translationTarget;
        this.facingTarget = facingTarget;
        this.controlFrames = Objects.requireNonNull(controlFrames, "controlFrames");
        this.solveSet = Objects.requireNonNull(solveSet, "solveSet");
        if (solveSet.size() <= 0) {
            throw new IllegalArgumentException("SpatialQuerySpec requires at least one solve lane");
        }
        this.fixedAprilTagLayout = fixedAprilTagLayout;
    }

    /**
     * Starts building a reusable immutable spatial query spec.
     */
    public static Builder builder() {
        return new Builder();
    }

    /**
     * Returns true when a translation channel is requested.
     */
    public boolean hasTranslationTarget() {
        return translationTarget != null;
    }

    /**
     * Returns true when a facing channel is requested.
     */
    public boolean hasFacingTarget() {
        return facingTarget != null;
    }

    /**
     * Builder for immutable specs.
     */
    public static final class Builder {
        private TranslationTarget2d translationTarget;
        private FacingTarget2d facingTarget;
        private SpatialControlFrames controlFrames = SpatialControlFrames.robotCenter();
        private SpatialSolveSet solveSet;
        private TagLayout fixedAprilTagLayout;

        /** Configures the translation target, or {@code null} for facing-only queries. */
        public Builder translateTo(TranslationTarget2d translationTarget) {
            this.translationTarget = translationTarget;
            return this;
        }

        /** Configures the facing target, or {@code null} for translation-only queries. */
        public Builder faceTo(FacingTarget2d facingTarget) {
            this.facingTarget = facingTarget;
            return this;
        }

        /** Supplies robot-relative control frames used by the query. */
        public Builder controlFrames(SpatialControlFrames controlFrames) {
            this.controlFrames = Objects.requireNonNull(controlFrames, "controlFrames");
            return this;
        }

        /** Supplies the ordered solve-lane set. */
        public Builder solveWith(SpatialSolveSet solveSet) {
            this.solveSet = Objects.requireNonNull(solveSet, "solveSet");
            return this;
        }

        /** Supplies the trusted fixed AprilTag layout used by lanes that need field-tag geometry. */
        public Builder fixedAprilTagLayout(TagLayout fixedAprilTagLayout) {
            this.fixedAprilTagLayout = fixedAprilTagLayout;
            return this;
        }

        /** Builds the immutable spec. */
        public SpatialQuerySpec build() {
            if (solveSet == null) {
                throw new IllegalStateException("SpatialQuerySpec builder requires solveWith(...)");
            }
            return new SpatialQuerySpec(translationTarget, facingTarget, controlFrames, solveSet, fixedAprilTagLayout);
        }
    }

    @Override
    public String toString() {
        return "SpatialQuerySpec{translationTarget=" + translationTarget
                + ", facingTarget=" + facingTarget
                + ", controlFrames=" + controlFrames
                + ", solveSet=" + solveSet
                + ", fixedAprilTagLayout=" + fixedAprilTagLayout + '}';
    }
}
