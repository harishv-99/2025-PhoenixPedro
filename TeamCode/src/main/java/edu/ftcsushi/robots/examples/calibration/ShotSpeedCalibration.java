package edu.ftcsushi.robots.examples.calibration;

import edu.ftcsushi.fw.core.math.InterpolatingTable1D;
import edu.ftcsushi.fw.core.math.InterpolatingTable2D;

/**
 * Calculation-only example of two alternative robot-owned shot-speed calibration maps.
 *
 * <p>All samples are illustrative, not measured or recommended shooter settings. A robot normally
 * chooses the map justified by its measurements; it does not need both. The one-input map uses
 * distance in inches. The two-input map uses target displacement from the robot center in the
 * robot frame: positive forward and positive left, both in inches. Both return encoder ticks per
 * second. The example performs no sensing, timekeeping, hardware commands or shot prediction.</p>
 *
 * <p>The private immutable tables are constructed once when this class is initialized. Queries
 * reuse them without changing state. Finite queries clamp to each authored axis; this numerical
 * behavior does not approve an operating range. Any non-finite input returns {@link Double#NaN}.
 * The adopting robot still owns input freshness, coordinate meaning, range acceptance, mechanism
 * limits and controlled physical validation before using a result as a request.</p>
 */
public final class ShotSpeedCalibration {
    private static final InterpolatingTable1D DISTANCE_TO_SPEED = InterpolatingTable1D.ofSorted(
            new double[] {24.0, 48.0},
            new double[] {3000.0, 3400.0});

    private static final InterpolatingTable2D OFFSET_TO_SPEED = InterpolatingTable2D.ofSorted(
            new double[] {24.0, 48.0},
            new double[] {-12.0, 0.0, 12.0},
            new double[][] {
                    {3100.0, 3000.0, 3100.0},
                    {3500.0, 3400.0, 3500.0}
            });

    private ShotSpeedCalibration() {
    }

    /**
     * Looks up the illustrative distance-only map without commanding a mechanism.
     *
     * @param distanceIn distance to the target, in inches; finite values clamp to 24 through 48
     * @return illustrative speed in encoder ticks per second, or {@code NaN} for non-finite input
     */
    public static double speedForDistanceTicksPerSec(double distanceIn) {
        return DISTANCE_TO_SPEED.interpolate(distanceIn);
    }

    /**
     * Looks up the alternative robot-relative offset map without commanding a mechanism.
     *
     * <p>Rows correspond to forward displacement; columns correspond to left displacement. Each
     * finite input clamps independently to its authored axis. Neither this lookup nor the table
     * converts field/camera coordinates or compensates for a displaced shooter.</p>
     *
     * @param targetForwardIn target displacement forward from robot center, in inches
     * @param targetLeftIn target displacement left from robot center, in inches
     * @return illustrative speed in encoder ticks per second, or {@code NaN} if either input is
     *         non-finite
     */
    public static double speedForOffsetTicksPerSec(double targetForwardIn, double targetLeftIn) {
        return OFFSET_TO_SPEED.interpolate(targetForwardIn, targetLeftIn);
    }
}
