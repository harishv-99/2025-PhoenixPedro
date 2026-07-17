package edu.ftcphoenix.fw.tools.tester.hardware;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/** Verifies the sampling semantics used by the external-encoder hardware comparison. */
public final class MotorVelocityComparisonTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void firstSampleBootstrapsThenUsesElapsedAcceptedSampleTime() {
        MotorVelocityComparison comparison = new MotorVelocityComparison();

        MotorVelocityComparison.Sample first = comparison.sample(1L, 2.0, 100, 900.0);
        assertFalse(first.derivedVelocityAvailable);
        assertTrue(Double.isNaN(first.derivedVelocityTicksPerSec));

        MotorVelocityComparison.Sample normal = comparison.sample(2L, 2.02, 120, 975.0);
        assertTrue(normal.derivedVelocityAvailable);
        assertEquals(20L, normal.deltaPositionTicks);
        assertEquals(0.02, normal.sampleIntervalSec, EPSILON);
        assertEquals(1000.0, normal.derivedVelocityTicksPerSec, EPSILON);
        assertEquals(-25.0, normal.directMinusDerivedTicksPerSec, EPSILON);

        MotorVelocityComparison.Sample irregular = comparison.sample(3L, 2.12, 220, 1005.0);
        assertEquals(1000.0, irregular.derivedVelocityTicksPerSec, EPSILON);
    }

    @Test
    public void repeatedCycleReturnsFirstSnapshotWithoutConsumingMovement() {
        MotorVelocityComparison comparison = new MotorVelocityComparison();
        comparison.sample(1L, 0.0, 0, 0.0);

        MotorVelocityComparison.Sample accepted = comparison.sample(2L, 0.02, 20, 900.0);
        MotorVelocityComparison.Sample duplicate = comparison.sample(2L, 0.03, 30, 2000.0);
        assertSame(accepted, duplicate);

        MotorVelocityComparison.Sample next = comparison.sample(3L, 0.04, 40, 1100.0);
        assertEquals(1000.0, next.derivedVelocityTicksPerSec, EPSILON);
    }

    @Test
    public void equalTimeDoesNotConsumePositionDelta() {
        MotorVelocityComparison comparison = new MotorVelocityComparison();
        comparison.sample(1L, 5.0, 100, 0.0);

        MotorVelocityComparison.Sample equalTime = comparison.sample(2L, 5.0, 125, 0.0);
        assertFalse(equalTime.derivedVelocityAvailable);

        MotorVelocityComparison.Sample recovered = comparison.sample(3L, 5.1, 200, 1000.0);
        assertEquals(100L, recovered.deltaPositionTicks);
        assertEquals(1000.0, recovered.derivedVelocityTicksPerSec, EPSILON);
    }

    @Test
    public void regressingTimeRebaselines() {
        MotorVelocityComparison comparison = new MotorVelocityComparison();
        comparison.sample(1L, 10.0, 100, 0.0);

        MotorVelocityComparison.Sample regressed = comparison.sample(2L, 2.0, 500, 0.0);
        assertFalse(regressed.derivedVelocityAvailable);

        MotorVelocityComparison.Sample afterRebase = comparison.sample(3L, 2.1, 530, 300.0);
        assertEquals(300.0, afterRebase.derivedVelocityTicksPerSec, EPSILON);
    }

    @Test
    public void nonFiniteTimeDoesNotPoisonValidBaseline() {
        MotorVelocityComparison comparison = new MotorVelocityComparison();
        comparison.sample(1L, 1.0, 10, 0.0);

        MotorVelocityComparison.Sample invalid = comparison.sample(2L, Double.NaN, 100, 0.0);
        assertFalse(invalid.derivedVelocityAvailable);

        MotorVelocityComparison.Sample recovered = comparison.sample(3L, 1.1, 110, 1000.0);
        assertEquals(1000.0, recovered.derivedVelocityTicksPerSec, EPSILON);
    }

    @Test
    public void signedPositionRolloverPreservesShortIntervalDelta() {
        MotorVelocityComparison comparison = new MotorVelocityComparison();
        comparison.sample(1L, 0.0, Integer.MAX_VALUE - 5, 0.0);

        MotorVelocityComparison.Sample wrapped = comparison.sample(
                2L,
                0.01,
                Integer.MIN_VALUE + 4,
                1000.0);

        assertEquals(10L, wrapped.deltaPositionTicks);
        assertEquals(1000.0, wrapped.derivedVelocityTicksPerSec, EPSILON);
    }

    @Test
    public void reversePositionAndReverseRolloverPreserveSign() {
        MotorVelocityComparison comparison = new MotorVelocityComparison();
        comparison.sample(1L, 0.0, 100, -900.0);
        MotorVelocityComparison.Sample reverse = comparison.sample(2L, 0.02, 80, -1000.0);
        assertEquals(-1000.0, reverse.derivedVelocityTicksPerSec, EPSILON);

        comparison.reset();
        comparison.sample(3L, 1.0, Integer.MIN_VALUE + 5, 0.0);
        MotorVelocityComparison.Sample wrapped = comparison.sample(
                4L,
                1.01,
                Integer.MAX_VALUE - 4,
                -1000.0);
        assertEquals(-10L, wrapped.deltaPositionTicks);
        assertEquals(-1000.0, wrapped.derivedVelocityTicksPerSec, EPSILON);
    }

    @Test
    public void unavailableDirectVelocityDoesNotHideDerivedMeasurement() {
        MotorVelocityComparison comparison = new MotorVelocityComparison();
        comparison.sample(1L, 0.0, 0, Double.NaN);

        MotorVelocityComparison.Sample sample = comparison.sample(2L, 0.1, 50, Double.NaN);
        assertTrue(sample.derivedVelocityAvailable);
        assertEquals(500.0, sample.derivedVelocityTicksPerSec, EPSILON);
        assertTrue(Double.isNaN(sample.directMinusDerivedTicksPerSec));
    }

    @Test
    public void resetRequiresFreshBaseline() {
        MotorVelocityComparison comparison = new MotorVelocityComparison();
        comparison.sample(1L, 0.0, 0, 0.0);
        assertTrue(comparison.sample(2L, 0.1, 100, 1000.0).derivedVelocityAvailable);

        comparison.reset();
        MotorVelocityComparison.Sample restarted = comparison.sample(2L, 0.1, 100, 1000.0);
        assertFalse(restarted.derivedVelocityAvailable);
    }
}
