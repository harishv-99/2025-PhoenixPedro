package edu.ftcphoenix.fw.spatial;

import java.util.ArrayList;
import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.field.TagLayout;

/**
 * Runtime source that solves one spatial relationship each loop.
 *
 * <p>A spatial query answers: <em>where is the requested target relative to the requested
 * robot-attached control frames?</em> It may ask several ordered {@link SpatialSolveLane}s to solve
 * the same relationship. It returns every lane result; it does not choose the final winner, blend
 * lanes, or produce actuator commands.</p>
 *
 * <h2>When to use it directly</h2>
 * <p>Use {@code SpatialQuery} directly when robot code needs raw geometry: a tag-relative point,
 * a facing error, a translation error, or a comparison between solve lanes. Use higher-level
 * planners such as drive guidance or scalar setpoint planning when you want a command or a plant
 * setpoint.</p>
 */
public final class SpatialQuery implements Source<SpatialQueryResult> {

    private final SpatialQuerySpec spec;

    private long lastCycle = Long.MIN_VALUE;
    private SpatialQueryResult lastResult = null;

    /**
     * Starts building a runtime {@link SpatialQuery}.
     *
     * <p>The staged builder mirrors {@link SpatialQuerySpec#builder()} but returns a stateful
     * runtime source from {@code build()}. It asks for the target relationship first, then the solve
     * lanes, so a query cannot be built before its required conceptual questions are answered.</p>
     *
     * <p>For a reusable immutable problem description, use {@link SpatialQuerySpec#builder()} and
     * then {@link #from(SpatialQuerySpec)}.</p>
     */
    public static TargetChoice builder() {
        return new Builder();
    }

    /**
     * First builder stage: choose the target relationship solved by the runtime query.
     */
    public interface TargetChoice {
        /**
         * Starts a translation query. Add facing later with {@link TranslationTargetStage#andFaceTo(FacingTarget2d)} when both channels matter.
         */
        TranslationTargetStage translateTo(TranslationTarget2d translationTarget);

        /**
         * Starts a facing query. Add translation later with {@link FacingTargetStage#andTranslateTo(TranslationTarget2d)} when both channels matter.
         */
        FacingTargetStage faceTo(FacingTarget2d facingTarget);
    }

    /**
     * Builder stage for a runtime query that currently has only a translation target.
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
         * Supplies a trusted fixed AprilTag layout for lanes or targets that need field-tag geometry.
         */
        TranslationTargetStage fixedAprilTagLayout(TagLayout fixedAprilTagLayout);

        /**
         * Supplies the ordered non-empty solve-lane set and moves to the build stage.
         */
        ReadyStage solveWith(SpatialSolveSet solveSet);
    }

    /**
     * Builder stage for a runtime query that currently has only a facing target.
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
         * Supplies a trusted fixed AprilTag layout for lanes or targets that need field-tag geometry.
         */
        FacingTargetStage fixedAprilTagLayout(TagLayout fixedAprilTagLayout);

        /**
         * Supplies the ordered non-empty solve-lane set and moves to the build stage.
         */
        ReadyStage solveWith(SpatialSolveSet solveSet);
    }

    /**
     * Builder stage for a runtime query that has both translation and facing targets.
     */
    public interface BothTargetStage {
        /**
         * Supplies robot-relative control frames. Defaults to {@link SpatialControlFrames#robotCenter()}.
         */
        BothTargetStage controlFrames(SpatialControlFrames controlFrames);

        /**
         * Supplies a trusted fixed AprilTag layout for lanes or targets that need field-tag geometry.
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
         * Supplies a trusted fixed AprilTag layout for lanes or targets that need field-tag geometry.
         */
        ReadyStage fixedAprilTagLayout(TagLayout fixedAprilTagLayout);

        /**
         * Builds the runtime query.
         */
        SpatialQuery build();
    }

    /**
     * Creates a runtime query from an immutable spec.
     */
    public static SpatialQuery from(SpatialQuerySpec spec) {
        return new SpatialQuery(spec);
    }

    /**
     * Creates a runtime query from one immutable spatial-query spec.
     */
    public SpatialQuery(SpatialQuerySpec spec) {
        this.spec = Objects.requireNonNull(spec, "spec");
    }

    /**
     * Returns the immutable spec owned by this runtime query.
     */
    public SpatialQuerySpec spec() {
        return spec;
    }

    /**
     * Returns the per-lane spatial result for this loop, caching by {@link LoopClock#cycle()}.
     */
    @Override
    public SpatialQueryResult get(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        long cycle = clock.cycle();
        if (cycle == lastCycle && lastResult != null) {
            return lastResult;
        }

        Pose2d robotToTranslationFrame = Objects.requireNonNull(
                spec.controlFrames.translationFrame().get(clock),
                "SpatialControlFrames.translationFrame().get(clock) returned null"
        );
        Pose2d robotToFacingFrame = Objects.requireNonNull(
                spec.controlFrames.facingFrame().get(clock),
                "SpatialControlFrames.facingFrame().get(clock) returned null"
        );

        SpatialSolveRequest request = new SpatialSolveRequest(
                clock,
                spec.translationTarget,
                spec.facingTarget,
                spec.controlFrames.translationFrame(),
                spec.controlFrames.facingFrame(),
                robotToTranslationFrame,
                robotToFacingFrame,
                spec.fixedAprilTagLayout
        );

        ArrayList<SpatialLaneResult> results = new ArrayList<SpatialLaneResult>(spec.solveSet.size());
        for (SpatialSolveLane lane : spec.solveSet.lanes()) {
            SpatialLaneResult result = lane.solve(request);
            results.add(result != null ? result : SpatialLaneResult.none());
        }

        lastCycle = cycle;
        lastResult = new SpatialQueryResult(
                spec.translationTarget,
                spec.facingTarget,
                robotToTranslationFrame,
                robotToFacingFrame,
                results
        );
        return lastResult;
    }

    /**
     * Clears query-local latch/caching state and resets owned solve lanes and frame providers.
     */
    @Override
    public void reset() {
        lastCycle = Long.MIN_VALUE;
        lastResult = null;
        spec.controlFrames.reset();
        for (SpatialSolveLane lane : spec.solveSet.lanes()) {
            lane.reset();
        }
        SpatialQuerySupport.resetSelections(spec.translationTarget);
        SpatialQuerySupport.resetSelections(spec.facingTarget);
    }

    /**
     * Emits query, frame, and lane debug state.
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "spatialQuery" : prefix;
        dbg.addData(p + ".spec", spec)
                .addData(p + ".lastResult", lastResult);
        spec.controlFrames.debugDump(dbg, p + ".frames");
        for (int i = 0; i < spec.solveSet.size(); i++) {
            spec.solveSet.lane(i).debugDump(dbg, p + ".lanes[" + i + "]");
        }
    }

    /**
     * Staged builder for runtime {@link SpatialQuery} objects.
     */
    static final class Builder implements TargetChoice,
            TranslationTargetStage,
            FacingTargetStage,
            BothTargetStage,
            ReadyStage {
        private final SpatialQuerySpec.Builder specBuilder;

        Builder() {
            this.specBuilder = new SpatialQuerySpec.Builder();
        }

        @Override
        public Builder translateTo(TranslationTarget2d translationTarget) {
            specBuilder.translateTo(translationTarget);
            return this;
        }

        @Override
        public Builder faceTo(FacingTarget2d facingTarget) {
            specBuilder.faceTo(facingTarget);
            return this;
        }

        @Override
        public Builder andFaceTo(FacingTarget2d facingTarget) {
            specBuilder.andFaceTo(facingTarget);
            return this;
        }

        @Override
        public Builder andTranslateTo(TranslationTarget2d translationTarget) {
            specBuilder.andTranslateTo(translationTarget);
            return this;
        }

        @Override
        public Builder controlFrames(SpatialControlFrames controlFrames) {
            specBuilder.controlFrames(controlFrames);
            return this;
        }

        @Override
        public Builder solveWith(SpatialSolveSet solveSet) {
            specBuilder.solveWith(solveSet);
            return this;
        }

        @Override
        public Builder fixedAprilTagLayout(TagLayout fixedAprilTagLayout) {
            specBuilder.fixedAprilTagLayout(fixedAprilTagLayout);
            return this;
        }

        @Override
        public SpatialQuery build() {
            return SpatialQuery.from(specBuilder.build());
        }
    }
}
