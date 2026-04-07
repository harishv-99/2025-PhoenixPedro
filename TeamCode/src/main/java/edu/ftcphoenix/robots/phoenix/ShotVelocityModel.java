package edu.ftcphoenix.robots.phoenix;

/**
 * Converts a measured scoring range into a recommended flywheel velocity.
 *
 * <p>This abstraction keeps range-to-shot calibration out of the shooter hardware owner so the
 * targeting layer can choose or swap shot models without changing actuator code.</p>
 */
public interface ShotVelocityModel {

    /**
     * Returns the recommended flywheel velocity for the supplied target range.
     *
     * @param rangeInches measured target range in inches
     * @return recommended flywheel velocity in the shooter's native velocity units
     */
    double velocityForRangeInches(double rangeInches);
}
