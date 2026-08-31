package edu.ftcsushi.fw.actuation;

/** Shared finite-value preconditions for position-calibration answers. */
final class PositionCalibrationValueValidation {

    private PositionCalibrationValueValidation() {
        // utility class
    }

    /**
     * Return a finite plant-unit value or fail before its caller stores or applies the answer.
     *
     * @param value       candidate value in plant units
     * @param operation   student-facing entry-point name used in the diagnostic
     * @param argumentName student-facing argument name used in the diagnostic
     * @return the unchanged finite value
     * @throws IllegalArgumentException if {@code value} is non-finite
     */
    static double requireFinitePlantValue(double value,
                                          String operation,
                                          String argumentName) {
        return requireFinite(value, operation, argumentName, "plant units");
    }

    /**
     * Return a finite native-unit value or fail before its caller stores or applies the answer.
     *
     * @param value       candidate value in native units
     * @param operation   student-facing entry-point name used in the diagnostic
     * @param argumentName student-facing argument name used in the diagnostic
     * @return the unchanged finite value
     * @throws IllegalArgumentException if {@code value} is non-finite
     */
    static double requireFiniteNativeValue(double value,
                                           String operation,
                                           String argumentName) {
        return requireFinite(value, operation, argumentName, "native units");
    }

    private static double requireFinite(double value,
                                        String operation,
                                        String argumentName,
                                        String units) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(operation + ": " + argumentName
                    + " must be finite in " + units + ", got " + value);
        }
        return value;
    }
}
