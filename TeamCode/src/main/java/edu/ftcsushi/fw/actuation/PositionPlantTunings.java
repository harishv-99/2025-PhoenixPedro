package edu.ftcsushi.fw.actuation;

import java.util.Objects;

/** Advanced factory for claiming one Plant-derived position-tuning preparation capability. */
public final class PositionPlantTunings {
    private PositionPlantTunings() { }

    /**
     * Claim the one non-actuating preparation handle from a Sushi mapped position Plant.
     * Boundary wrappers must retain their original delegate and claim this handle from that
     * delegate; there is no provider SPI or universal PositionPlant method. Calibration-search or
     * still-unreferenced heartbeats may already have run, but normal referenced realization must
     * not have begun.
     *
     * @throws IllegalArgumentException if {@code plant} is not the framework mapped realization
     * @throws IllegalStateException if the capability was already claimed, the Plant stopped, or
     *                               normal referenced realization already began
     */
    public static PositionPlantTuning claim(PositionPlant plant) {
        Objects.requireNonNull(plant, "plant");
        if (!(plant instanceof MappedPositionPlant)) {
            throw new IllegalArgumentException("PositionPlantTunings.claim(...) requires the "
                    + "completed mapped position Plant built by the Sushi Plant grammar; a "
                    + "boundary wrapper must claim from the original delegate it retains");
        }
        return ((MappedPositionPlant) plant).claimPositionPlantTuning();
    }
}
