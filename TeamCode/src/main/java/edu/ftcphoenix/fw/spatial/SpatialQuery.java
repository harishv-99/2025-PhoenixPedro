package edu.ftcphoenix.fw.spatial;

import java.util.ArrayList;
import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.field.TagLayout;

/**
 * Stateful runtime query that samples control frames once per loop and asks each solve lane to
 * compute the requested spatial relationship.
 *
 * <p>{@link SpatialQuery} intentionally stops at <em>solving relationships</em>. It does not decide
 * whether to prefer one lane over another, blend between lanes, or turn the result into a drive or
 * mechanism command. Those decisions belong to higher-level consumers.</p>
 */
public final class SpatialQuery implements Source<SpatialQuerySample> {

    private final SpatialQuerySpec spec;

    private long lastCycle = Long.MIN_VALUE;
    private SpatialQuerySample lastSample = null;

    /**
     * Starts building a new immutable {@link SpatialQuerySpec}.
     */
    public static Builder builder() {
        return new Builder();
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

    @Override
    public SpatialQuerySample get(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        long cycle = clock.cycle();
        if (cycle == lastCycle && lastSample != null) {
            return lastSample;
        }

        Pose2d robotToTranslationFrame = Objects.requireNonNull(
                spec.controlFrames.translationFrame().get(clock),
                "SpatialControlFrames.translationFrame().get(clock) returned null"
        );
        Pose2d robotToAimFrame = Objects.requireNonNull(
                spec.controlFrames.aimFrame().get(clock),
                "SpatialControlFrames.aimFrame().get(clock) returned null"
        );

        SpatialSolveRequest request = new SpatialSolveRequest(
                clock,
                spec.translationTarget,
                spec.aimTarget,
                robotToTranslationFrame,
                robotToAimFrame,
                spec.fixedAprilTagLayout
        );

        ArrayList<SpatialLaneResult> results = new ArrayList<SpatialLaneResult>(spec.solveSet.size());
        for (SpatialSolveLane lane : spec.solveSet.lanes()) {
            SpatialLaneResult result = lane.solve(request);
            results.add(result != null ? result : SpatialLaneResult.none());
        }

        lastCycle = cycle;
        lastSample = new SpatialQuerySample(
                spec.translationTarget,
                spec.aimTarget,
                robotToTranslationFrame,
                robotToAimFrame,
                results
        );
        return lastSample;
    }

    @Override
    public void reset() {
        lastCycle = Long.MIN_VALUE;
        lastSample = null;
        spec.controlFrames.reset();
        for (SpatialSolveLane lane : spec.solveSet.lanes()) {
            lane.reset();
        }
        SpatialQuerySupport.resetSelections(spec.translationTarget);
        SpatialQuerySupport.resetSelections(spec.aimTarget);
    }

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "spatialQuery" : prefix;
        dbg.addData(p + ".spec", spec)
                .addData(p + ".lastSample", lastSample);
        spec.controlFrames.debugDump(dbg, p + ".frames");
        for (int i = 0; i < spec.solveSet.size(); i++) {
            spec.solveSet.lane(i).debugDump(dbg, p + ".lanes[" + i + "]");
        }
    }

    /**
     * Builder for immutable {@link SpatialQuerySpec}s.
     */
    public static final class Builder {
        private TranslationTarget2d translationTarget;
        private AimTarget2d aimTarget;
        private SpatialControlFrames controlFrames = SpatialControlFrames.robotCenter();
        private SpatialSolveSet solveSet;
        private TagLayout fixedAprilTagLayout;

        /**
         * Configures the translation target to solve, or {@code null} for an aim-only query.
         */
        public Builder translateTo(TranslationTarget2d translationTarget) {
            this.translationTarget = translationTarget;
            return this;
        }

        /**
         * Configures the aim target to solve, or {@code null} for a translation-only query.
         */
        public Builder aimTo(AimTarget2d aimTarget) {
            this.aimTarget = aimTarget;
            return this;
        }

        /**
         * Supplies the robot-relative frames that should satisfy the query geometry.
         */
        public Builder controlFrames(SpatialControlFrames controlFrames) {
            this.controlFrames = Objects.requireNonNull(controlFrames, "controlFrames");
            return this;
        }

        /**
         * Supplies the ordered solve-lane set used by this query.
         */
        public Builder solveWith(SpatialSolveSet solveSet) {
            this.solveSet = Objects.requireNonNull(solveSet, "solveSet");
            return this;
        }

        /**
         * Supplies the fixed field tag layout used by lanes that need trusted field-tag geometry.
         */
        public Builder fixedAprilTagLayout(TagLayout fixedAprilTagLayout) {
            this.fixedAprilTagLayout = fixedAprilTagLayout;
            return this;
        }

        /**
         * Builds the immutable {@link SpatialQuerySpec}.
         */
        public SpatialQuerySpec build() {
            if (solveSet == null) {
                throw new IllegalStateException("SpatialQuery builder requires solveWith(...)");
            }
            return new SpatialQuerySpec(translationTarget, aimTarget, controlFrames, solveSet, fixedAprilTagLayout);
        }
    }
}
