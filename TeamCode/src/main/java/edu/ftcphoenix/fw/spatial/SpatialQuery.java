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
     * <p>For a reusable immutable problem description, use {@link SpatialQuerySpec#builder()} and
     * then {@link #from(SpatialQuerySpec)}.</p>
     */
    public static Builder builder() {
        return new Builder(false);
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

    /** Builder for runtime {@link SpatialQuery} objects. */
    public static final class Builder {
        private final SpatialQuerySpec.Builder specBuilder;

        Builder(boolean specOnly) {
            this.specBuilder = new SpatialQuerySpec.Builder();
        }

        /** Configures the translation target, or {@code null} for a facing-only query. */
        public Builder translateTo(TranslationTarget2d translationTarget) {
            specBuilder.translateTo(translationTarget);
            return this;
        }

        /**
         * Configures the facing target, or {@code null} for a translation-only query.
         */
        public Builder faceTo(FacingTarget2d facingTarget) {
            specBuilder.faceTo(facingTarget);
            return this;
        }

        /** Supplies robot-relative control frames used by the query. */
        public Builder controlFrames(SpatialControlFrames controlFrames) {
            specBuilder.controlFrames(controlFrames);
            return this;
        }

        /** Supplies the ordered solve-lane set. */
        public Builder solveWith(SpatialSolveSet solveSet) {
            specBuilder.solveWith(solveSet);
            return this;
        }

        /** Supplies the trusted fixed AprilTag layout used by lanes that need field-tag geometry. */
        public Builder fixedAprilTagLayout(TagLayout fixedAprilTagLayout) {
            specBuilder.fixedAprilTagLayout(fixedAprilTagLayout);
            return this;
        }

        /**
         * Builds the runtime query.
         */
        public SpatialQuery build() {
            return SpatialQuery.from(specBuilder.build());
        }
    }
}
