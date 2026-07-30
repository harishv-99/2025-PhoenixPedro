package edu.ftcphoenix.fw.actuation;

/** Shared precondition for finite normalized calibration-search power answers. */
final class CalibrationSearchPowerValidation {

    private static final double MIN_POWER = -1.0;
    private static final double MAX_POWER = 1.0;

    private CalibrationSearchPowerValidation() {
        // utility class
    }

    /**
     * Return a valid power or fail before its caller commits any lifecycle or output effect.
     *
     * @param power     candidate normalized search power
     * @param operation student-facing entry-point name used in an actionable diagnostic
     * @return the unchanged finite power in the inclusive {@code [-1.0, +1.0]} range
     * @throws IllegalArgumentException if {@code power} is non-finite or out of range
     */
    static double requireValid(double power, String operation) {
        if (!Double.isFinite(power) || power < MIN_POWER || power > MAX_POWER) {
            throw new IllegalArgumentException(operation
                    + " requires finite normalized search power in the inclusive "
                    + "[-1.0, +1.0] range, got " + power);
        }
        return power;
    }
}
