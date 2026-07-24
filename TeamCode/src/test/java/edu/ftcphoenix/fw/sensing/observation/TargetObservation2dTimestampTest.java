package edu.ftcphoenix.fw.sensing.observation;

import org.junit.Test;

import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/** Verifies that generic target observations keep one epoch-safe measurement time. */
public final class TargetObservation2dTimestampTest {

    private static final double EPSILON = 1.0e-12;

    @Test
    public void retainedObservationAgeAdvancesFromItsOneTimestamp() {
        ManualLoopClock time = new ManualLoopClock(8.0);
        LoopTimestamp capturedAt = time.clock().nowTimestamp();
        TargetObservation2d observation = TargetObservation2d.ofRobotRelativePosition(
                4,
                24.0,
                -3.0,
                0.75,
                capturedAt
        );

        assertSame(capturedAt, observation.timestamp);
        assertEquals(0.0, observation.ageSec(time.clock()), EPSILON);
        assertTrue(observation.isFresh(time.clock(), 0.25));

        time.nextCycle(0.25);
        assertEquals(0.25, observation.ageSec(time.clock()), EPSILON);
        assertTrue(observation.isFresh(time.clock(), 0.25));

        time.nextCycle(0.001);
        assertFalse(observation.isFresh(time.clock(), 0.25));
    }

    @Test
    public void resetInvalidatesRetainedObservationWithoutConsumerEpochBookkeeping() {
        ManualLoopClock time = new ManualLoopClock(3.0);
        TargetObservation2d observation = TargetObservation2d.ofRobotRelativeBearing(
                0.2,
                1.0,
                time.clock().nowTimestamp()
        );

        time.clock().reset(3.0);

        assertTrue(Double.isNaN(observation.ageSec(time.clock())));
        assertFalse(observation.isFresh(time.clock(), 1.0));
    }

    @Test
    public void noTargetUsesTheSharedUnavailableTimestamp() {
        TargetObservation2d none = TargetObservation2d.none();
        ManualLoopClock time = new ManualLoopClock();

        assertFalse(none.hasTarget);
        assertSame(LoopTimestamp.unavailable(), none.timestamp);
        assertTrue(Double.isNaN(none.ageSec(time.clock())));
        assertFalse(none.isFresh(time.clock(), 100.0));
    }
}
