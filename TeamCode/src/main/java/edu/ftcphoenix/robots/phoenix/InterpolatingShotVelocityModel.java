package edu.ftcphoenix.robots.phoenix;

import java.util.Objects;

import edu.ftcphoenix.fw.core.math.InterpolatingTable1D;

/**
 * Shot-velocity model backed by a 1D interpolation table.
 *
 * <p>This is Phoenix's current production model: use measured AprilTag camera range as the table
 * input and linearly interpolate to a recommended flywheel velocity.</p>
 */
public final class InterpolatingShotVelocityModel implements ShotVelocityModel {

    private final InterpolatingTable1D table;

    /**
     * Creates an interpolating shot-velocity model.
     *
     * @param table immutable range-to-velocity table used for interpolation
     */
    public InterpolatingShotVelocityModel(InterpolatingTable1D table) {
        this.table = Objects.requireNonNull(table, "table");
    }

    /**
     * Returns the interpolated flywheel velocity recommendation for the supplied range.
     *
     * @param rangeInches measured target range in inches
     * @return interpolated flywheel velocity recommendation in native velocity units
     */
    @Override
    public double velocityForRangeInches(double rangeInches) {
        return table.interpolate(rangeInches);
    }

    /**
     * Returns the immutable interpolation table used by this model.
     *
     * @return range-to-velocity table backing the model
     */
    public InterpolatingTable1D table() {
        return table;
    }
}
