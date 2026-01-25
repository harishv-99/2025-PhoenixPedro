package edu.ftcphoenix.fw.core.math;

/**
 * Common unit conversion helpers used throughout the Phoenix framework.
 *
 * <p>Phoenix tries to keep units explicit and consistent (for example, radians for angles).
 * These helpers exist so robot code can stay readable:</p>
 *
 * <pre>{@code
 * double flywheelOmega = Units.rpmToRadPerSec(4500);
 * }</pre>
 */
public final class Units {

    private Units() {
        // utility class; no instances
    }

    /**
     * Convert revolutions per minute (RPM) to radians per second.
     *
     * @param rpm value in revolutions per minute
     * @return the same rate expressed in radians per second
     */
    public static double rpmToRadPerSec(double rpm) {
        return rpm * (2.0 * Math.PI) / 60.0;
    }
}
