package edu.ftcsushi.fw.sensing.observation;

import java.util.Objects;
import java.util.function.ToDoubleFunction;

import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Immutable geometric ranking configuration created by {@link TargetSelectionPolicies}.
 *
 * <p>A policy supplies a finite cost for each eligible positioned observation; smaller wins.
 * Non-finite cost excludes a candidate. The selector owns freshness, deterministic tie-breaking
 * and its successful-cycle cache. A policy adds no physical identity, tracking or confidence.</p>
 *
 * <p>One policy may be reused by independent selectors. Borrowed bearing sources and custom cost
 * functions must obey their value-source contracts and have no external effects; they are not
 * reset by the policy or selector. Use {@link TargetSelectionPolicies#lowestCost} for a custom
 * robot-owned ranking rule.</p>
 */
public final class TargetSelectionPolicy {
    /** Prepares any per-frame borrowed input once before ranking candidates. */
    interface MetricFactory {
        /** Returns a cost function for this accepted, nonempty frame without mutating it. */
        ToDoubleFunction<TargetObservation2d> prepare(TargetObservations2d frame, LoopClock clock);
    }

    private final String description;
    private final MetricFactory factory;

    /** Package-local assembly keeps the ordinary construction path in TargetSelectionPolicies. */
    TargetSelectionPolicy(String description, MetricFactory factory) {
        this.description = Objects.requireNonNull(description, "policy description");
        if (description.trim().isEmpty()) throw new IllegalArgumentException("policy description must be nonblank");
        this.factory = Objects.requireNonNull(factory, "metric factory");
    }

    /** Prepare one cost function without committing selector state. */
    ToDoubleFunction<TargetObservation2d> prepare(TargetObservations2d frame, LoopClock clock) {
        return Objects.requireNonNull(factory.prepare(frame, clock), "policy cost function");
    }

    /** Human-readable ranking meaning; never a measured-quality description. */
    String description() { return description; }

    /** Returns immutable policy identity without sampling any borrowed input. */
    @Override public String toString() { return "TargetSelectionPolicy{" + description + '}'; }
}
