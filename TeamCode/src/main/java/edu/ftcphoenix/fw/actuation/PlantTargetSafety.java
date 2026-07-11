package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.time.LoopClock;

/** Internal final target-safety policy shared by all framework Plant implementations. */
final class PlantTargetSafety {

    private PlantTargetSafety() {
    }

    /**
     * Reject a mapped Plant range that cannot contain any finite hardware command.
     */
    static void requireUsableConfiguredRange(ScalarRange range, String plantName) {
        Objects.requireNonNull(range, "range");
        String owner = cleanName(plantName);
        if (!range.valid) {
            throw new IllegalArgumentException(owner + " requires a valid configured target range; got "
                    + range + ". Configure a valid range; reference availability is handled separately.");
        }
        if (!Double.isFinite(range.clamp(0.0))) {
            throw new IllegalArgumentException(owner + " configured target range must contain at least one"
                    + " finite plant-unit command; got " + range);
        }
    }

    /**
     * Apply dynamic guards, then enforce the Plant's final finite/range postcondition.
     */
    static PlantTargetGuards.Result applyGuards(PlantTargetGuards guards,
                                                double candidate,
                                                PlantTargetStatus currentStatus,
                                                double previousApplied,
                                                ScalarRange range,
                                                String targetName,
                                                LoopClock clock) {
        Objects.requireNonNull(guards, "guards");
        Objects.requireNonNull(currentStatus, "currentStatus");
        Objects.requireNonNull(range, "range");
        String name = cleanName(targetName);

        // An unreferenced position Plant does not write its normal output or advance dynamic guards.
        if (!range.valid) {
            double retained = Double.isFinite(previousApplied) ? previousApplied : 0.0;
            return new PlantTargetGuards.Result(retained, currentStatus);
        }

        double safeCandidate = candidate;
        PlantTargetStatus safeStatus = currentStatus;
        if (!Double.isFinite(safeCandidate)) {
            safeCandidate = safeRecovery(previousApplied, Double.NaN, range, name);
            safeStatus = PlantTargetStatus.targetUnavailable(name
                    + " candidate was non-finite; retained a finite target before guards");
        } else if (!range.contains(safeCandidate)) {
            safeCandidate = checkedClamp(safeCandidate, range, name);
            safeStatus = PlantTargetStatus.clampedToRange(name
                    + " candidate was clamped before target guards");
        }

        PlantTargetGuards.Result guarded = guards.apply(
                safeCandidate, safeStatus, previousApplied, clock);
        if (Double.isFinite(guarded.target) && range.contains(guarded.target)) {
            return guarded;
        }

        final double corrected;
        final PlantTargetStatus correctedStatus;
        if (!Double.isFinite(guarded.target)) {
            corrected = safeRecovery(previousApplied, safeCandidate, range, name);
            correctedStatus = PlantTargetStatus.targetUnavailable(name
                    + " target guards produced a non-finite result; recovered to a finite in-range target"
                    + " (prior status: " + guarded.status + ")");
        } else {
            corrected = checkedClamp(guarded.target, range, name);
            correctedStatus = PlantTargetStatus.clampedToRange(name
                    + " target was re-clamped after target guards (prior status: "
                    + guarded.status + ")");
        }

        guards.reconcileAppliedTarget(corrected, correctedStatus, clock);
        return new PlantTargetGuards.Result(corrected, correctedStatus);
    }

    private static double safeRecovery(double previousApplied,
                                       double safeCandidate,
                                       ScalarRange range,
                                       String targetName) {
        if (Double.isFinite(previousApplied)) {
            double retained = range.clamp(previousApplied);
            if (Double.isFinite(retained)) return retained;
        }
        if (Double.isFinite(safeCandidate)) {
            double retained = range.clamp(safeCandidate);
            if (Double.isFinite(retained)) return retained;
        }
        double zeroInRange = range.clamp(0.0);
        if (Double.isFinite(zeroInRange)) return zeroInRange;
        throw new IllegalStateException(targetName
                + " has no finite target available inside its declared range " + range);
    }

    private static double checkedClamp(double value, ScalarRange range, String targetName) {
        double clamped = range.clamp(value);
        if (!Double.isFinite(clamped)) {
            throw new IllegalStateException(targetName
                    + " range could not produce a finite target from " + value + ": " + range);
        }
        return clamped;
    }

    private static String cleanName(String name) {
        return name == null || name.trim().isEmpty() ? "Plant" : name.trim();
    }
}
