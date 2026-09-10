package edu.ftcsushi.robots.examples.calibration;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * Optional reading checkpoint for the real calculation-only example, not a hardware simulation.
 *
 * <p>Keep real: the maintained example's private illustrative tables and Sushi interpolation.
 * Replace: a distance/offset observation with explicitly authored numeric input. No sensor,
 * controller, mechanism, clock or physical shot is present. Assertions describe returned numbers,
 * not accepted calibration, sensor freshness or permission to command a shooter.</p>
 */
public final class ShotSpeedCalibrationTest {
    /**
     * Question: what do the two alternative maps return between their illustrative samples?
     */
    @Test
    public void distanceAndOffsetAlternativesInterpolateIllustrativeSamples() {
        // ARRANGE: inputs only; the real example owns the tables, not a copy inside this test.
        double distanceIn = 36.0;
        double targetForwardIn = 36.0;
        double targetLeftIn = 6.0;

        // REQUEST: these calls calculate now. There is no hardware command or later heartbeat.
        double distanceSpeed = ShotSpeedCalibration.speedForDistanceTicksPerSec(distanceIn);
        double offsetSpeed = ShotSpeedCalibration.speedForOffsetTicksPerSec(
                targetForwardIn, targetLeftIn);

        // ASSERT: 36 is halfway from 24 to 48; 6 is halfway from left 0 to left 12.
        assertEquals(3200.0, distanceSpeed, 1e-9);
        assertEquals(3250.0, offsetSpeed, 1e-9);
        // NEXT GATE: accepted robot measurements, not these illustrative values, justify a map.
    }

    /**
     * Question: is an outside finite input different from an unavailable input?
     */
    @Test
    public void finiteClampingDoesNotTurnMissingInputIntoAnEndpoint() {
        // ARRANGE: 60 and -20 are outside the authored axes; NaN/infinity are not finite values.
        double beyondDistanceIn = 60.0;
        double beyondForwardIn = 60.0;
        double beyondRightIn = -20.0;

        // REQUEST: finite inputs clamp independently; missing input stays unavailable.
        double distanceEdge = ShotSpeedCalibration.speedForDistanceTicksPerSec(beyondDistanceIn);
        double offsetCorner = ShotSpeedCalibration.speedForOffsetTicksPerSec(
                beyondForwardIn, beyondRightIn);
        double missingDistance = ShotSpeedCalibration.speedForDistanceTicksPerSec(Double.NaN);
        double missingOffset = ShotSpeedCalibration.speedForOffsetTicksPerSec(
                36.0, Double.POSITIVE_INFINITY);

        // ASSERT: clamping is numerical behavior, not evidence these positions are accepted.
        assertEquals(3400.0, distanceEdge, 0.0);
        assertEquals(3500.0, offsetCorner, 0.0);
        assertTrue(Double.isNaN(missingDistance));
        assertTrue(Double.isNaN(missingOffset));
        // NEXT GATE: the robot must separately check freshness, accepted range and hardware limits.
    }
}
