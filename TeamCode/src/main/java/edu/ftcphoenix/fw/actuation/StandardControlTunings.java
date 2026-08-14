package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

/** Advanced factories for deriving one exclusive standard-control tuning capability from a Plant. */
public final class StandardControlTunings {
    private StandardControlTunings() { }

    /**
     * Claim the Plant-owned Phoenix standard velocity controller before its first heartbeat.
     *
     * @throws IllegalArgumentException if the Plant is not a framework mapped velocity Plant
     * @throws IllegalStateException if it is device-managed, custom-regulated, already updated,
     *                               stopped, or already claimed
     */
    public static StandardControlTuning claimVelocity(Plant plant) {
        Objects.requireNonNull(plant, "plant");
        if (!(plant instanceof MappedVelocityPlant)) {
            throw new IllegalArgumentException("StandardControlTunings.claimVelocity(...) requires "
                    + "a velocity Plant built by the Phoenix Plant grammar; custom Plant "
                    + "implementations do not expose a standard-controller tuning capability");
        }
        return ((MappedVelocityPlant) plant).claimStandardControlTuning();
    }

    /**
     * Claim the Plant-owned Phoenix standard position controller before its first heartbeat.
     *
     * @throws IllegalArgumentException if the Plant is not a framework mapped position Plant
     * @throws IllegalStateException if it is device-managed/direct, custom-regulated, already
     *                               updated, stopped, or already claimed
     */
    public static StandardControlTuning claimPosition(PositionPlant plant) {
        Objects.requireNonNull(plant, "plant");
        if (!(plant instanceof MappedPositionPlant)) {
            throw new IllegalArgumentException("StandardControlTunings.claimPosition(...) requires "
                    + "a position Plant built by the Phoenix Plant grammar; custom PositionPlant "
                    + "implementations do not expose a standard-controller tuning capability");
        }
        return ((MappedPositionPlant) plant).claimStandardControlTuning();
    }
}
