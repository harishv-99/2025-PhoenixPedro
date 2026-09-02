package edu.ftcsushi.fw.actuation;

import java.util.Objects;

/** Immutable {@link PlantSnapshot} with the cached public facts specific to position Plants. */
public final class PositionPlantSnapshot extends PlantSnapshot {

    private final PositionPlant.Periodicity periodicity;
    private final double period;
    private final ScalarRange targetRange;
    private final boolean referenced;
    private final String referenceStatus;
    private final boolean supportsCalibrationSearch;

    /** Capture one PositionPlant through its cached public query surface. */
    PositionPlantSnapshot(PositionPlant source) {
        super(source);
        PositionPlant plant = Objects.requireNonNull(source, "source");
        periodicity = Objects.requireNonNull(
                plant.periodicity(), "PositionPlant.periodicity() returned null");
        period = plant.period();
        targetRange = Objects.requireNonNull(
                plant.targetRange(), "PositionPlant.targetRange() returned null");
        referenced = plant.isReferenced();
        referenceStatus = Objects.requireNonNull(
                plant.referenceStatus(), "PositionPlant.referenceStatus() returned null");
        supportsCalibrationSearch = plant.supportsCalibrationSearch();
    }

    /** Periodicity of the caller-facing position coordinate. */
    public PositionPlant.Periodicity periodicity() {
        return periodicity;
    }

    /** Period in Plant units, or {@link Double#NaN} for a non-periodic coordinate. */
    public double period() {
        return period;
    }

    /** Captured legal target range in Plant units, possibly invalid before reference. */
    public ScalarRange targetRange() {
        return targetRange;
    }

    /** Whether the position coordinate had a valid physical reference at capture. */
    public boolean isReferenced() {
        return referenced;
    }

    /** Captured human-readable position-reference status. */
    public String referenceStatus() {
        return referenceStatus;
    }

    /** Whether this position Plant supports temporary open-loop calibration search. */
    public boolean supportsCalibrationSearch() {
        return supportsCalibrationSearch;
    }

    @Override
    public String toString() {
        return "PositionPlantSnapshot{" + super.toString()
                + ", periodicity=" + periodicity
                + ", period=" + period
                + ", targetRange=" + targetRange
                + ", referenced=" + referenced
                + ", referenceStatus='" + referenceStatus + '\''
                + ", supportsCalibrationSearch=" + supportsCalibrationSearch + "}";
    }
}
