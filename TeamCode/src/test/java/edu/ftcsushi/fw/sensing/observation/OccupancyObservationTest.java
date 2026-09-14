package edu.ftcsushi.fw.sensing.observation;

import org.junit.Test;

import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** The shared value preserves evidence; phase and capture interpretation belong to consumers. */
public final class OccupancyObservationTest {
    @Test public void observedRetainsExactTimestampAndDoesNotRefreshAge() {
        ManualLoopClock time = new ManualLoopClock();
        LoopTimestamp timestamp = time.clock().nowTimestamp();
        OccupancyObservation value = OccupancyObservation.observed(true, timestamp);
        assertTrue(value.available);
        assertTrue(value.occupied);
        assertSame(timestamp, value.timestamp);
        time.nextCycle(0.2);
        assertFalse(value.timestamp.isFresh(time.clock(), 0.1));
    }

    @Test public void unavailableIsNotAnObservedEmptySample() {
        OccupancyObservation unavailable = OccupancyObservation.unavailable();
        OccupancyObservation empty = OccupancyObservation.observed(false,
                new ManualLoopClock().clock().nowTimestamp());
        assertFalse(unavailable.available);
        assertFalse(unavailable.timestamp.isAvailable());
        assertTrue(empty.available);
        assertFalse(empty.occupied);
    }

    @Test public void observedRequiresAvailableTimestamp() {
        assertThrows(NullPointerException.class, () -> OccupancyObservation.observed(false, null));
        assertThrows(IllegalArgumentException.class,
                () -> OccupancyObservation.observed(false, LoopTimestamp.unavailable()));
    }
}
