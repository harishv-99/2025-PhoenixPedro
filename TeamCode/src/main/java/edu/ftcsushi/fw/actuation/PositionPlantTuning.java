package edu.ftcsushi.fw.actuation;

import java.util.Objects;

import edu.ftcsushi.fw.core.time.LoopClock;

/**
 * Exclusive advanced handle for staging one current-position hold without realizing an output.
 * The bound Plant remains the only heartbeat and output owner.
 */
public final class PositionPlantTuning {

    /** Immutable evidence from one non-actuating recovery-hold preparation. */
    public static final class RecoveryHold {
        private final double measurement;
        private final double holdTarget;

        RecoveryHold(double measurement, double holdTarget) {
            this.measurement = measurement;
            this.holdTarget = holdTarget;
        }

        /** Same-cycle finite measured position in Plant units. */
        public double measurement() { return measurement; }

        /** Exact legal command staged through the Plant graph for the next normal update. */
        public double holdTarget() { return holdTarget; }

        /** Whether recovery targets an envelope boundary instead of the measured position. */
        public boolean wasClamped() {
            return Double.doubleToLongBits(measurement)
                    != Double.doubleToLongBits(holdTarget);
        }
    }

    private final MappedPositionPlant plant;

    PositionPlantTuning(MappedPositionPlant plant) {
        this.plant = Objects.requireNonNull(plant, "plant");
    }

    /** Whether the final target graph is one literal exact graph-owned command. */
    public boolean hasExactCommandTarget() {
        return plant.hasExactTuningCommandTarget();
    }

    /** Return the current cached reference-dependent target range without sampling. */
    public ScalarRange targetRange() {
        return plant.targetRange();
    }

    /** Whether the bound position coordinate currently has a physical reference. */
    public boolean isReferenced() {
        return plant.isReferenced();
    }

    /** Return the bound Plant's cached reference status. */
    public String referenceStatus() {
        return plant.referenceStatus();
    }

    /**
     * Sample the current position, establish an ASSUME_CURRENT reference when necessary, and set
     * the exact graph-owned command to that position without invoking normal Plant realization.
     * Calibration-search heartbeats may already have run, but no normal position realization may
     * have been attempted. The Plant supplies one handle and accepts one successful preparation;
     * an early rejection such as an unestablished reference may be corrected and retried.
     *
     * @return the finite held position in Plant units
     */
    public double prepareHoldAtCurrent(LoopClock clock) {
        return plant.prepareTuningHoldAtCurrent(Objects.requireNonNull(clock, "clock"));
    }

    /**
     * Sample current position before this cycle's normal realization and stage a legal recovery
     * hold through the exact command graph without actuating.
     *
     * <p>The finite bounded {@code allowedPhysicalRange} must lie inside the referenced Plant
     * range. A measurement inside it is held exactly; ordinary overshoot outside it recovers to
     * the nearest boundary. The initial hold must still use {@link #prepareHoldAtCurrent(LoopClock)}
     * so an out-of-envelope starting pose is rejected instead of auto-driven.</p>
     *
     * @return immutable same-cycle measurement and staged target evidence
     * @throws IllegalStateException if initial preparation has not completed, the Plant is
     *                               unreferenced/searching/stopped, or normal realization already
     *                               began in this cycle
     * @throws IllegalArgumentException if the allowed range is unusable or outside the Plant range
     */
    public RecoveryHold prepareRecoveryHoldWithin(ScalarRange allowedPhysicalRange,
                                                   LoopClock clock) {
        return plant.prepareTuningRecoveryHoldWithin(
                Objects.requireNonNull(allowedPhysicalRange, "allowedPhysicalRange"),
                Objects.requireNonNull(clock, "clock"));
    }
}
