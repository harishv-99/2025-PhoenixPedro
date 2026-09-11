package edu.ftcsushi.fw.sensing.observation;

import java.util.Objects;
import java.util.function.ToDoubleFunction;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.localization.PoseEstimate;

/**
 * Immutable ranking configuration for remembered field locations, created only through
 * {@link FieldTargetSelectionPolicies}.
 *
 * <p>One value may be reused by independent selectors. The selector checks each entry's original
 * sighting age and owner-backed lifetime before ranking. Smaller finite distances win; canonical
 * field coordinates and owner-issued key sequence break ties. A remembered location is a
 * hypothesis, not a verified physical identity, current visibility, or expected intake yield.</p>
 *
 * <p>A pose-dependent rule borrows already-published localization evidence. It never updates or
 * resets the estimator, and a new robot pose never refreshes the object's last sighting.</p>
 */
public final class FieldTargetSelectionPolicy {
    /** Prepares the optional shared pose once for one successful selector calculation. */
    interface MetricFactory {
        /** Reads value evidence only; an exception leaves the selector free to retry. */
        Ranking prepare(LoopClock clock);
    }

    /** Exact per-calculation ranking evidence, separate from later field-solving evidence. */
    static final class Ranking {
        final ToDoubleFunction<FieldTargetMemory.Entry> metric;
        final PoseEstimate pose;
        final String unavailableReason;

        /** Retains the prepared metric and any exact pose read, including a rejected pose. */
        Ranking(ToDoubleFunction<FieldTargetMemory.Entry> metric, PoseEstimate pose,
                String unavailableReason) {
            this.metric = Objects.requireNonNull(metric, "metric");
            this.pose = pose;
            this.unavailableReason = unavailableReason;
        }
    }

    private final String description;
    private final MetricFactory factory;

    /** Package-only assembly keeps one public construction path in the policy factories. */
    FieldTargetSelectionPolicy(String description, MetricFactory factory) {
        this.description = Objects.requireNonNull(description, "policy description");
        if (description.trim().isEmpty()) {
            throw new IllegalArgumentException("policy description must be nonblank");
        }
        this.factory = Objects.requireNonNull(factory, "metric factory");
    }

    /** Prepares one pure ranking operation without changing selection or memory state. */
    Ranking prepare(LoopClock clock) {
        return Objects.requireNonNull(factory.prepare(clock), "policy ranking");
    }

    /** Returns ranking meaning, not confidence or a claim of physical identity. */
    String description() { return description; }

    /** Returns immutable configuration identity without reading the borrowed estimator. */
    @Override public String toString() { return "FieldTargetSelectionPolicy{" + description + '}'; }
}
