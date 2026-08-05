package edu.ftcphoenix.fw.ftc;

/**
 * Package-private preconditions for fixed FTC motor-controller configuration.
 *
 * <p>The coefficient bound matches the symmetric, non-saturating public conversion domain of the
 * FTC SDK 11.1 REV/Lynx controller path pinned by this repository. It is a representation bound,
 * not a recommendation about useful or safe gains. The raw signed 16Q16 field has one additional
 * negative endpoint that the SDK's absolute-magnitude-then-sign conversion cannot produce.</p>
 */
final class FtcControllerConfigurationValidation {

    private static final double CONTROLLER_COEFFICIENT_SCALE = 65536.0;
    private static final double MAX_CONTROLLER_COEFFICIENT =
            Integer.MAX_VALUE / CONTROLLER_COEFFICIENT_SCALE;
    private static final double MIN_CONTROLLER_COEFFICIENT = -MAX_CONTROLLER_COEFFICIENT;
    private static final int MIN_DEVICE_POSITION_TOLERANCE_TICKS = 0;
    private static final int MAX_DEVICE_POSITION_TOLERANCE_TICKS = 65535;

    private FtcControllerConfigurationValidation() {
        // precondition utility
    }

    /** Return one unchanged coefficient after checking the pinned controller conversion domain. */
    static double requireControllerCoefficient(double value,
                                               String operation,
                                               String argument) {
        if (!Double.isFinite(value)
                || value < MIN_CONTROLLER_COEFFICIENT
                || value > MAX_CONTROLLER_COEFFICIENT) {
            throw new IllegalArgumentException(operation + ": " + argument
                    + " must be finite in FTC controller coefficient units within the inclusive ["
                    + MIN_CONTROLLER_COEFFICIENT + ", " + MAX_CONTROLLER_COEFFICIENT
                    + "] REV conversion domain, got " + value);
        }
        return value;
    }

    /** Validate one complete PIDF tuple before returning a new private configuration array. */
    static double[] requireControllerPidf(double p,
                                          double i,
                                          double d,
                                          double f,
                                          String operation) {
        double checkedP = requireControllerCoefficient(p, operation, "p");
        double checkedI = requireControllerCoefficient(i, operation, "i");
        double checkedD = requireControllerCoefficient(d, operation, "d");
        double checkedF = requireControllerCoefficient(f, operation, "f");
        return new double[]{checkedP, checkedI, checkedD, checkedF};
    }

    /** Return one unchanged RUN_TO_POSITION maximum-power magnitude in normalized power units. */
    static double requireRunToPositionMaxPower(double value, String operation) {
        if (!Double.isFinite(value) || value < 0.0 || value > 1.0) {
            throw new IllegalArgumentException(operation
                    + ": maxPower must be finite normalized RUN_TO_POSITION maximum power within "
                    + "the inclusive [0.0, 1.0] domain, got " + value);
        }
        return value;
    }

    /** Return one unchanged native target-position tolerance supported by the pinned REV command. */
    static int requireDevicePositionToleranceTicks(int value, String operation) {
        if (value < MIN_DEVICE_POSITION_TOLERANCE_TICKS
                || value > MAX_DEVICE_POSITION_TOLERANCE_TICKS) {
            throw new IllegalArgumentException(operation
                    + ": ticks must be native FTC controller tolerance ticks within the inclusive ["
                    + MIN_DEVICE_POSITION_TOLERANCE_TICKS + ", "
                    + MAX_DEVICE_POSITION_TOLERANCE_TICKS + "] domain, got " + value);
        }
        return value;
    }
}
