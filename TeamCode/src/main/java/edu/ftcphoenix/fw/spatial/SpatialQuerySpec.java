package edu.ftcphoenix.fw.spatial;

import java.util.Objects;

import edu.ftcphoenix.fw.field.TagLayout;

/**
 * Immutable, controller-neutral description of a spatial relationship to solve.
 *
 * <p>A {@link SpatialQuerySpec} answers four questions:</p>
 * <ol>
 *   <li><b>What translation relationship matters?</b> optionally a {@link TranslationTarget2d}</li>
 *   <li><b>What aim relationship matters?</b> optionally an {@link AimTarget2d}</li>
 *   <li><b>Which robot-relative frames are controlled?</b> {@link SpatialControlFrames}</li>
 *   <li><b>Which solve lanes may answer the query?</b> {@link SpatialSolveSet}</li>
 * </ol>
 *
 * <p>The spec is intentionally task-neutral. It can be consumed by {@code DriveGuidance}, a future
 * angular mechanism planner, or a custom manipulator planner.</p>
 */
public final class SpatialQuerySpec {

    public final TranslationTarget2d translationTarget;
    public final AimTarget2d aimTarget;
    public final SpatialControlFrames controlFrames;
    public final SpatialSolveSet solveSet;
    public final TagLayout fixedAprilTagLayout;

    public SpatialQuerySpec(TranslationTarget2d translationTarget,
                            AimTarget2d aimTarget,
                            SpatialControlFrames controlFrames,
                            SpatialSolveSet solveSet,
                            TagLayout fixedAprilTagLayout) {
        if (translationTarget == null && aimTarget == null) {
            throw new IllegalArgumentException("SpatialQuerySpec needs translateTo(...), aimTo(...), or both");
        }
        this.translationTarget = translationTarget;
        this.aimTarget = aimTarget;
        this.controlFrames = Objects.requireNonNull(controlFrames, "controlFrames");
        this.solveSet = Objects.requireNonNull(solveSet, "solveSet");
        if (solveSet.size() <= 0) {
            throw new IllegalArgumentException("SpatialQuerySpec requires at least one solve lane");
        }
        this.fixedAprilTagLayout = fixedAprilTagLayout;
    }

    public boolean hasTranslationTarget() {
        return translationTarget != null;
    }

    public boolean hasAimTarget() {
        return aimTarget != null;
    }

    @Override
    public String toString() {
        return "SpatialQuerySpec{translationTarget=" + translationTarget
                + ", aimTarget=" + aimTarget
                + ", controlFrames=" + controlFrames
                + ", solveSet=" + solveSet
                + ", fixedAprilTagLayout=" + fixedAprilTagLayout + '}';
    }
}
