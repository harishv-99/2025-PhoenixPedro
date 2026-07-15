package edu.ftcphoenix.robots.phoenix;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

/** Verifies Phoenix Auto match-time policy defaults and defensive profile copying. */
public final class PhoenixProfileAutoConfigTest {

    @Test
    public void parkTakeoverDefaultsToTwentyFiveSeconds() {
        assertEquals(
                25.0,
                PhoenixProfile.defaults().auto.parkTakeoverElapsedSec,
                0.0
        );
    }

    @Test
    public void profileCopyRetainsIndependentParkTakeoverValue() {
        PhoenixProfile original = PhoenixProfile.defaults();
        original.auto.parkTakeoverElapsedSec = 23.5;

        PhoenixProfile copy = original.copy();
        original.auto.parkTakeoverElapsedSec = 19.0;

        assertEquals(23.5, copy.auto.parkTakeoverElapsedSec, 0.0);
        assertEquals(19.0, original.auto.parkTakeoverElapsedSec, 0.0);
    }
}
