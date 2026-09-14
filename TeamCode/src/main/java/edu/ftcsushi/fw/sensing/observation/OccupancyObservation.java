package edu.ftcsushi.fw.sensing.observation;

import java.util.Objects;

import edu.ftcsushi.fw.core.time.LoopTimestamp;

/**
 * Immutable occupied/empty sensor evidence at its original observation time.
 *
 * <p>Unavailable evidence does not mean empty. An occupied sample by itself does not prove a new
 * capture: the consuming behavior owns freshness, transition, and phase requirements. Neither a
 * motor command nor disappearance from a camera is an occupancy observation.</p>
 */
public final class OccupancyObservation {
    /** Whether the sensor supplied an observation, independently of its age. */
    public final boolean available;
    /** The sensor's occupied state; meaningful only when {@link #available} is true. */
    public final boolean occupied;
    /** Original clock identity, epoch, and observation time; never refreshed by reading this value. */
    public final LoopTimestamp timestamp;

    private OccupancyObservation(boolean available, boolean occupied, LoopTimestamp timestamp) {
        this.available = available;
        this.occupied = occupied;
        this.timestamp = Objects.requireNonNull(timestamp, "occupancy timestamp");
    }

    /**
     * Records the sensor's accepted occupied/empty observation at its original time.
     *
     * @param occupied true when the sensor reports occupied, false when it reports empty
     * @param timestamp available timestamp of the actual observation, not the later read
     * @return immutable evidence; the consumer still validates age and clock compatibility
     * @throws IllegalArgumentException if the timestamp is unavailable
     */
    public static OccupancyObservation observed(boolean occupied, LoopTimestamp timestamp) {
        if (!Objects.requireNonNull(timestamp, "timestamp").isAvailable()) {
            throw new IllegalArgumentException("observed occupancy requires a timestamp");
        }
        return new OccupancyObservation(true, occupied, timestamp);
    }

    /** Returns missing sensor evidence, which never means empty or captured. */
    public static OccupancyObservation unavailable() {
        return new OccupancyObservation(false, false, LoopTimestamp.unavailable());
    }
}
