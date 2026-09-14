package edu.ftcsushi.fw.spatial;

import java.util.Objects;

import edu.ftcsushi.fw.field.TagLayout;
import edu.ftcsushi.fw.field.TagLayouts;

/**
 * Immutable, controller-neutral description of a spatial relationship to solve.
 *
 * <p>A spec answers these conceptual questions in order:</p>
 * <ol>
 *   <li><b>What target relationship matters?</b> translation, facing, or both.</li>
 *   <li><b>Which robot-relative frames are controlled?</b> defaults to robot center unless
 *       {@link SpatialControlFrames} are supplied.</li>
 *   <li><b>Which solve lanes may answer the query?</b> a non-empty {@link SpatialSolveSet}.</li>
 *   <li><b>Is a fixed AprilTag layout needed?</b> optional, only for lanes/targets that use
 *       trusted field-tag geometry.</li>
 * </ol>
 *
 * <p>The staged builder intentionally does not expose {@code build()} until a target and a solve
 * set are chosen. Use {@link SpatialQuery#builder()} for the common runtime source. Use this spec
 * when you want to create multiple independent runtime queries with the same immutable
 * description. If supplied, the fixed tag layout is validated and snapshotted when
 * {@code build()} completes.</p>
 *
 * <p>{@code approach(description)} answers both targets and their rigid control frames together.
 * It is the coupled point-facing path; separate authored points/headings and dynamic control
 * frames retain their existing explicit target stages.</p>
 */
public final class SpatialQuerySpec {

    public final TranslationTarget2d translationTarget;
    public final FacingTarget2d facingTarget;
    public final SpatialControlFrames controlFrames;
    public final SpatialSolveSet solveSet;
    public final TagLayout fixedAprilTagLayout;

    private SpatialQuerySpec(TranslationTarget2d translationTarget,
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
        this.fixedAprilTagLayout = fixedAprilTagLayout != null
                ? TagLayouts.snapshot(fixedAprilTagLayout)
                : null;
    }

    /**
     * Starts building a reusable immutable spatial query spec.
     *
     * <p>The first required question is which relationship the query solves. Choose
     * {@link TargetChoice#translateTo(TranslationTarget2d)} for a translation-only query,
     * {@link TargetChoice#faceTo(FacingTarget2d)} for a facing-only query, or add the second target
     * through {@code andFaceTo(...)} / {@code andTranslateTo(...)} before choosing solve lanes.</p>
     */
    public static TargetChoice builder() {
        return new Builder();
    }

    /**
     * First builder stage: choose the target relationship solved by the query.
     */
    public interface TargetChoice {
        /**
         * Chooses a coupled tool-relative point approach. The description supplies both targets
         * and their frames; additional target/frame answers, including through retained earlier
         * stages, are rejected. Evidence and optional fixed layout remain explicit.
         */
        ApproachTargetStage approach(SpatialApproach2d approach);

        /**
         * Starts a translation query. Add facing later with {@link TranslationTargetStage#andFaceTo(FacingTarget2d)} when both channels matter.
         */
        TranslationTargetStage translateTo(TranslationTarget2d translationTarget);

        /**
         * Starts a facing query. Add translation later with {@link FacingTargetStage#andTranslateTo(TranslationTarget2d)} when both channels matter.
         */
        FacingTargetStage faceTo(FacingTarget2d facingTarget);
    }

    /** A complete approach geometry answer; only evidence and optional field facts remain. */
    public interface ApproachTargetStage {
        /** Supplies trusted fixed tag geometry; the completed spec snapshots it. */
        ApproachTargetStage fixedAprilTagLayout(TagLayout fixedAprilTagLayout);

        /** Supplies the non-empty solve-lane set and moves to the ordinary build stage. */
        ReadyStage solveWith(SpatialSolveSet solveSet);
    }

    /**
     * Builder stage for a query that currently has only a translation target.
     */
    public interface TranslationTargetStage {
        /**
         * Adds a facing target so the query solves both channels.
         */
        BothTargetStage andFaceTo(FacingTarget2d facingTarget);

        /**
         * Supplies robot-relative control frames. Defaults to {@link SpatialControlFrames#robotCenter()}.
         */
        TranslationTargetStage controlFrames(SpatialControlFrames controlFrames);

        /**
         * Supplies a trusted fixed AprilTag layout for lanes or targets that need field-tag
         * geometry. The layout is validated and snapshotted when {@code build()} completes.
         */
        TranslationTargetStage fixedAprilTagLayout(TagLayout fixedAprilTagLayout);

        /**
         * Supplies the ordered non-empty solve-lane set and moves to the build stage.
         */
        ReadyStage solveWith(SpatialSolveSet solveSet);
    }

    /**
     * Builder stage for a query that currently has only a facing target.
     */
    public interface FacingTargetStage {
        /**
         * Adds a translation target so the query solves both channels.
         */
        BothTargetStage andTranslateTo(TranslationTarget2d translationTarget);

        /**
         * Supplies robot-relative control frames. Defaults to {@link SpatialControlFrames#robotCenter()}.
         */
        FacingTargetStage controlFrames(SpatialControlFrames controlFrames);

        /**
         * Supplies a trusted fixed AprilTag layout for lanes or targets that need field-tag
         * geometry. The layout is validated and snapshotted when {@code build()} completes.
         */
        FacingTargetStage fixedAprilTagLayout(TagLayout fixedAprilTagLayout);

        /**
         * Supplies the ordered non-empty solve-lane set and moves to the build stage.
         */
        ReadyStage solveWith(SpatialSolveSet solveSet);
    }

    /**
     * Builder stage for a query that has both translation and facing targets.
     */
    public interface BothTargetStage {
        /**
         * Supplies robot-relative control frames. Defaults to {@link SpatialControlFrames#robotCenter()}.
         */
        BothTargetStage controlFrames(SpatialControlFrames controlFrames);

        /**
         * Supplies a trusted fixed AprilTag layout for lanes or targets that need field-tag
         * geometry. The layout is validated and snapshotted when {@code build()} completes.
         */
        BothTargetStage fixedAprilTagLayout(TagLayout fixedAprilTagLayout);

        /**
         * Supplies the ordered non-empty solve-lane set and moves to the build stage.
         */
        ReadyStage solveWith(SpatialSolveSet solveSet);
    }

    /**
     * Final stage: solve lanes are known, optional fixed layout may still be supplied, then build.
     */
    public interface ReadyStage {
        /**
         * Supplies a trusted fixed AprilTag layout for lanes or targets that need field-tag
         * geometry. The layout is validated and snapshotted when {@code build()} completes.
         */
        ReadyStage fixedAprilTagLayout(TagLayout fixedAprilTagLayout);

        /**
         * Builds the immutable spec.
         */
        SpatialQuerySpec build();
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
     * Staged builder implementation. Users normally hold one of the stage interfaces returned by
     * {@link #builder()} rather than this concrete class.
     */
    static final class Builder implements TargetChoice,
            ApproachTargetStage,
            TranslationTargetStage,
            FacingTargetStage,
            BothTargetStage,
            ReadyStage {
        private TranslationTarget2d translationTarget;
        private FacingTarget2d facingTarget;
        private SpatialControlFrames controlFrames = SpatialControlFrames.robotCenter();
        private SpatialSolveSet solveSet;
        private TagLayout fixedAprilTagLayout;
        private boolean approachChosen;
        private boolean controlFramesChosen;

        Builder() {
            // staged builder; use SpatialQuerySpec.builder()
        }

        @Override
        public Builder approach(SpatialApproach2d approach) {
            if (approachChosen || translationTarget != null || facingTarget != null || controlFramesChosen) {
                throw new IllegalStateException("approach(...) cannot be mixed with earlier target or control-frame answers");
            }
            SpatialApproach2d value = Objects.requireNonNull(approach, "approach");
            SpatialTargets.ReferencePointTarget target = SpatialTargets.point(value.point());
            translationTarget = target;
            facingTarget = target;
            controlFrames = value.controlFrames();
            approachChosen = true;
            return this;
        }

        @Override
        public Builder translateTo(TranslationTarget2d translationTarget) {
            requireSeparateTargetAnswer();
            this.translationTarget = Objects.requireNonNull(translationTarget, "translationTarget");
            return this;
        }

        @Override
        public Builder faceTo(FacingTarget2d facingTarget) {
            requireSeparateTargetAnswer();
            this.facingTarget = Objects.requireNonNull(facingTarget, "facingTarget");
            return this;
        }

        @Override
        public Builder andFaceTo(FacingTarget2d facingTarget) {
            requireSeparateTargetAnswer();
            this.facingTarget = Objects.requireNonNull(facingTarget, "facingTarget");
            return this;
        }

        @Override
        public Builder andTranslateTo(TranslationTarget2d translationTarget) {
            requireSeparateTargetAnswer();
            this.translationTarget = Objects.requireNonNull(translationTarget, "translationTarget");
            return this;
        }

        @Override
        public Builder controlFrames(SpatialControlFrames controlFrames) {
            requireSeparateTargetAnswer();
            this.controlFrames = Objects.requireNonNull(controlFrames, "controlFrames");
            controlFramesChosen = true;
            return this;
        }

        /** Protects the coupled description even when an earlier stage reference was retained. */
        private void requireSeparateTargetAnswer() {
            if (approachChosen) {
                throw new IllegalStateException("approach(...) already supplies both targets and control frames");
            }
        }

        @Override
        public Builder solveWith(SpatialSolveSet solveSet) {
            this.solveSet = Objects.requireNonNull(solveSet, "solveSet");
            return this;
        }

        @Override
        public Builder fixedAprilTagLayout(TagLayout fixedAprilTagLayout) {
            this.fixedAprilTagLayout = fixedAprilTagLayout;
            return this;
        }

        @Override
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
