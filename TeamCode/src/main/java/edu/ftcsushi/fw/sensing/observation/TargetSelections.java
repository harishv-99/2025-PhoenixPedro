package edu.ftcsushi.fw.sensing.observation;

import java.util.Objects;
import java.util.function.ToDoubleFunction;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Creates continuous geometric selection from currently observed, located objects.
 *
 * <p>Choose the original capture's freshness bound, then a ranking policy. The policy answer
 * constructs the source directly; there is no identity-latching mode or separate build step.
 * A new frame may choose another candidate. Freezing a destination or associating physical objects
 * is explicit robot behavior, never inferred from frame order or proximity here.</p>
 *
 * <pre>{@code
 * TargetSelectionSource selected = TargetSelections.fromVisibleObjects(objects)
 *         .freshWithinSec(0.20)
 *         .choose(TargetSelectionPolicies.nearestToRobot());
 * }</pre>
 *
 * <p>Construction does not sample the camera or a policy's borrowed sources. Each successful
 * selection publishes once per loop cycle; an exception can retry. Reset clears only the local
 * selection cache, never borrowed observation or bearing sources.</p>
 */
public final class TargetSelections {
    private TargetSelections() { }

    /**
     * Starts a selector over one borrowed observed-frame source, with optional capture-time field
     * coordinates already attached. This does not infer visibility from localization or memory.
     */
    public static FreshnessStep fromVisibleObjects(Source<TargetObservations2d> observations) {
        Source<TargetObservations2d> required = Objects.requireNonNull(observations, "observations");
        return maxAgeSec -> {
            double checkedAgeSec = nonnegative("maxAgeSec", maxAgeSec);
            return policy -> create(required, checkedAgeSec, Objects.requireNonNull(policy, "policy"));
        };
    }

    /** Required choice of how old the original camera observation may be. */
    public interface FreshnessStep {
        /** Requires a finite non-negative inclusive capture-age bound in seconds. */
        PolicyStep freshWithinSec(double maxAgeSec);
    }

    /** Choose the ranking rule to construct one independent continuous selector. */
    public interface PolicyStep {
        /**
         * Builds the typed source without sampling its inputs. Policies come from
         * {@link TargetSelectionPolicies}; another call creates another independent cache.
         */
        TargetSelectionSource choose(TargetSelectionPolicy policy);
    }

    /** Keep generic memoization around a local calculation, not around the borrowed input owner. */
    private static TargetSelectionSource create(Source<TargetObservations2d> source,
                                                double maxAgeSec, TargetSelectionPolicy policy) {
        Source<TargetSelectionResult> calculated = Source.of(clock -> {
            TargetObservations2d frame = Objects.requireNonNull(source.get(clock), "observations frame");
            if (!frame.isAvailable()) return TargetSelectionResult.none(frame, maxAgeSec, frame.reason());
            if (!frame.isFresh(clock, maxAgeSec)) {
                return TargetSelectionResult.none(frame, maxAgeSec, "observation frame stale, reset, or future");
            }
            if (frame.observations().isEmpty()) {
                return TargetSelectionResult.none(frame, maxAgeSec, "observed empty frame");
            }
            ToDoubleFunction<TargetObservation2d> cost = policy.prepare(frame, clock);
            TargetObservation2d best = null;
            double bestCost = Double.POSITIVE_INFINITY;
            for (TargetObservation2d candidate : frame.observations()) {
                if (!candidate.hasPosition()) continue;
                double value = cost.applyAsDouble(candidate);
                if (!Double.isFinite(value)) continue;
                if (best == null || value < bestCost
                        || (value == bestCost && compare(candidate, best) < 0)) {
                    best = candidate;
                    bestCost = value;
                }
            }
            return best == null
                    ? TargetSelectionResult.none(frame, maxAgeSec,
                            "no positioned candidate satisfies " + policy.description())
                    : TargetSelectionResult.selected(frame, best, maxAgeSec, bestCost, policy.description());
        });
        return new SelectionSource(calculated.memoized(), maxAgeSec, policy);
    }

    /** Typed domain view over the shared generic source's guarded, successful-cycle cache. */
    private static final class SelectionSource implements TargetSelectionSource {
        private final Source<TargetSelectionResult> calculated;
        private final double maxAgeSec;
        private final TargetSelectionPolicy policy;

        /** Receives only a locally owned calculation; observations and policy inputs stay borrowed. */
        private SelectionSource(Source<TargetSelectionResult> calculated, double maxAgeSec,
                                TargetSelectionPolicy policy) {
            this.calculated = calculated;
            this.maxAgeSec = maxAgeSec;
            this.policy = policy;
        }

        /** Samples or returns the exact successful result already published in this cycle. */
        @Override public TargetSelectionResult get(LoopClock clock) { return calculated.get(clock); }

        /** Resets the local memoized calculation, whose leaf has no borrowed reset behavior. */
        @Override public void reset() { calculated.reset(); }

        /** Reports immutable configuration without sampling camera, policy, or controller inputs. */
        @Override public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = prefix == null || prefix.isEmpty() ? "targetSelection" : prefix;
            dbg.addData(p + ".class", "TargetSelectionSource")
                    .addData(p + ".policy", policy.description())
                    .addData(p + ".maxAgeSec", maxAgeSec);
        }
    }

    /** Total ordering of observable geometric values, independent of original list order. */
    private static int compare(TargetObservation2d a, TargetObservation2d b) {
        double[] av = {a.forwardInches, a.leftInches, a.fieldXInches, a.fieldYInches,
                a.bearingRad, a.targetHeadingRad, a.targetId, a.quality};
        double[] bv = {b.forwardInches, b.leftInches, b.fieldXInches, b.fieldYInches,
                b.bearingRad, b.targetHeadingRad, b.targetId, b.quality};
        for (int index = 0; index < av.length; index++) {
            int compared = Double.compare(av[index], bv[index]);
            if (compared != 0) return compared;
        }
        return 0;
    }

    /** Validates the required finite, inclusive capture-age limit at its builder boundary. */
    private static double nonnegative(String name, double value) {
        if (!Double.isFinite(value)) throw new IllegalArgumentException(name + " must be finite");
        if (value < 0) throw new IllegalArgumentException(name + " must be >= 0");
        return value;
    }
}
