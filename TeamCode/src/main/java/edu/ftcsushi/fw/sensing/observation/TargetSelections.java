package edu.ftcsushi.fw.sensing.observation;

import java.util.Objects;
import java.util.function.ToDoubleFunction;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Creates continuous geometric selection from visible objects or recent field-location memory.
 *
 * <p>Choose the original capture's freshness bound, then a ranking policy. The policy answer
 * constructs the source directly; there is no identity-latching mode or separate build step.
 * A new frame may choose another candidate. For differently aged remembered locations, start from
 * {@link #fromRecentFieldLocations}; its separate policy/result types preserve field evidence
 * without pretending entries came from one image. Freezing a destination or claiming physical
 * identity is never inferred by either selector.</p>
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

    /**
     * Starts continuous selection from one passive borrowed memory view. The ordinary
     * {@code choose(...)} terminal inherits the memory's retention bound; optional
     * {@code freshWithinSec(...)} may only make that bound stricter. Construction reads the
     * immutable retention configuration, never the view's snapshot or its upstream source.
     */
    public static RecentFieldSelectionStep fromRecentFieldLocations(FieldTargetMemory.View memory) {
        FieldTargetMemory.View required = Objects.requireNonNull(memory, "memory");
        double retentionSec = nonnegative("memory retentionSec", required.retentionSec());
        return new RecentFieldSelectionStep() {
            @Override public FieldTargetSelectionSource choose(FieldTargetSelectionPolicy policy) {
                return createField(required, retentionSec, Objects.requireNonNull(policy, "policy"));
            }

            @Override public FieldPolicyStep freshWithinSec(double stricterAgeSec) {
                double ageSec = nonnegative("stricterAgeSec", stricterAgeSec);
                if (ageSec > retentionSec) {
                    throw new IllegalArgumentException("stricterAgeSec must be <= memory retentionSec ("
                            + retentionSec + ")");
                }
                return policy -> createField(required, ageSec, Objects.requireNonNull(policy, "policy"));
            }
        };
    }

    /** Use the inherited memory age directly, or narrow it once before choosing a policy. */
    public interface RecentFieldSelectionStep extends FieldPolicyStep {
        /** Requires finite non-negative seconds no greater than the memory's retention limit. */
        FieldPolicyStep freshWithinSec(double stricterAgeSec);
    }

    /** One field-location ranking answer constructs one independent continuous selector. */
    public interface FieldPolicyStep {
        /** Builds a source without sampling memory or the policy's borrowed localization owner. */
        FieldTargetSelectionSource choose(FieldTargetSelectionPolicy policy);
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

    /** Calculates one field choice while leaving memory advancement and lifetime with its owner. */
    private static FieldTargetSelectionSource createField(FieldTargetMemory.View source,
            double maxAgeSec, FieldTargetSelectionPolicy policy) {
        Source<FieldTargetSelectionResult> calculated = Source.of(clock -> {
            FieldTargetMemory.Snapshot snapshot = Objects.requireNonNull(source.get(clock),
                    "field memory snapshot");
            boolean hasEligible = false;
            for (FieldTargetMemory.Entry entry : snapshot.entries()) {
                if (eligible(entry, clock, maxAgeSec)) {
                    hasEligible = true;
                    break;
                }
            }
            if (!hasEligible) {
                return FieldTargetSelectionResult.none(snapshot, maxAgeSec,
                        "no eligible remembered field locations: " + snapshot.inputReason(), null);
            }
            FieldTargetSelectionPolicy.Ranking ranking = policy.prepare(clock);
            if (ranking.unavailableReason != null) {
                return FieldTargetSelectionResult.none(snapshot, maxAgeSec,
                        ranking.unavailableReason, ranking.pose);
            }
            FieldTargetMemory.Entry best = null;
            double bestDistance = Double.POSITIVE_INFINITY;
            for (FieldTargetMemory.Entry entry : snapshot.entries()) {
                if (!eligible(entry, clock, maxAgeSec)) continue;
                double distance = ranking.metric.applyAsDouble(entry);
                if (!Double.isFinite(distance)) continue;
                if (best == null || distance < bestDistance
                        || (distance == bestDistance && compareField(entry, best) < 0)) {
                    best = entry;
                    bestDistance = distance;
                }
            }
            return best == null
                    ? FieldTargetSelectionResult.none(snapshot, maxAgeSec,
                            "no remembered location satisfies " + policy.description(), ranking.pose)
                    : FieldTargetSelectionResult.selected(snapshot, best, maxAgeSec,
                            bestDistance, policy.description(), ranking.pose);
        });
        return new FieldSelectionSource(calculated.memoized(), maxAgeSec, policy);
    }

    /** Rechecks exact historical sighting age and current owner-backed membership. */
    private static boolean eligible(FieldTargetMemory.Entry entry, LoopClock clock, double maxAgeSec) {
        return entry.isUsable(clock) && entry.lastSighting().isFresh(clock, maxAgeSec);
    }

    /** Field coordinates, then owner sequence, decide ties without detector list-order influence. */
    private static int compareField(FieldTargetMemory.Entry a, FieldTargetMemory.Entry b) {
        TargetObservation2d av = a.lastSighting();
        TargetObservation2d bv = b.lastSighting();
        int x = compareCoordinate(av.fieldXInches, bv.fieldXInches);
        if (x != 0) return x;
        int y = compareCoordinate(av.fieldYInches, bv.fieldYInches);
        return y != 0 ? y : Long.compare(a.key().sequence(), b.key().sequence());
    }

    /** Signed zeros name the same coordinate rather than separate geometric priorities. */
    private static int compareCoordinate(double a, double b) {
        return Double.compare(a == 0 ? 0 : a, b == 0 ? 0 : b);
    }

    /** Domain view over one locally owned value calculation; no borrowed lifecycle propagates. */
    private static final class FieldSelectionSource implements FieldTargetSelectionSource {
        private final Source<FieldTargetSelectionResult> calculated;
        private final double maxAgeSec;
        private final FieldTargetSelectionPolicy policy;

        /** Retains the local memoizer and immutable configuration only. */
        private FieldSelectionSource(Source<FieldTargetSelectionResult> calculated,
                double maxAgeSec, FieldTargetSelectionPolicy policy) {
            this.calculated = calculated;
            this.maxAgeSec = maxAgeSec;
            this.policy = policy;
        }

        /** Samples once successfully per cycle; an exceptional value calculation may retry. */
        @Override public FieldTargetSelectionResult get(LoopClock clock) { return calculated.get(clock); }

        /** Clears only this selector's cache, never memory or the localization owner. */
        @Override public void reset() { calculated.reset(); }

        /** Reports configuration without reading memory, localization, or hardware. */
        @Override public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) return;
            String p = prefix == null || prefix.isEmpty() ? "fieldTargetSelection" : prefix;
            dbg.addData(p + ".class", "FieldTargetSelectionSource")
                    .addData(p + ".policy", policy.description())
                    .addData(p + ".maxAgeSec", maxAgeSec);
        }
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
