package edu.ftcphoenix.fw.localization;

import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused value-contract coverage for timestamped predictor motion. */
public final class MotionDeltaTest {

    @Test
    public void usableDeltaRequiresStrictlyPositiveCurrentEpochInterval() {
        ManualLoopClock time = new ManualLoopClock(4.0);
        LoopTimestamp start = time.clock().nowTimestamp();
        time.nextCycle(0.02);
        LoopTimestamp end = time.clock().nowTimestamp();

        MotionDelta delta = new MotionDelta(
                new Pose3d(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                true,
                1.0,
                start,
                end
        );

        assertTrue(delta.hasDelta);
        assertEquals(0.02, delta.durationSec(), 1e-9);
    }

    @Test
    public void zeroDurationUsesExplicitNoDeltaValue() {
        ManualLoopClock time = new ManualLoopClock();
        LoopTimestamp timestamp = time.clock().nowTimestamp();

        try {
            new MotionDelta(Pose3d.zero(), true, 1.0, timestamp, timestamp);
            fail("Expected zero-duration usable delta to be rejected");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("strictly later"));
        }

        MotionDelta none = MotionDelta.none(timestamp);
        assertFalse(none.hasDelta);
        assertEquals(0.0, none.durationSec(), 0.0);
    }

    @Test
    public void priorEpochCannotConstructUsableDelta() {
        ManualLoopClock time = new ManualLoopClock(2.0);
        LoopTimestamp start = time.clock().nowTimestamp();
        time.nextCycle(0.05);
        LoopTimestamp end = time.clock().nowTimestamp();
        time.clock().reset(time.clock().nowSec());

        try {
            new MotionDelta(Pose3d.zero(), true, 1.0, start, end);
            fail("Expected prior-epoch usable delta to be rejected");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("same clock's current epoch"));
        }
    }
}
