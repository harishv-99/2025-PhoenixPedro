package edu.ftcphoenix.fw.ftc.vision;

import org.junit.Test;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused tests for stable FTC/vendor identity-to-loop-time anchoring. */
public final class FtcFrameTimestampAnchorTest {

    @Test
    public void invalidVendorTimingFailsClosedWithoutReplacingRetainedFrame() {
        ManualLoopClock time = new ManualLoopClock(5.0);
        FtcFrameTimestampAnchor anchor = new FtcFrameTimestampAnchor();

        assertFalse(anchor.anchor(time.clock(), null, 0.0).isAvailable());
        assertFalse(anchor.anchor(time.clock(), "frame-a", Double.NaN).isAvailable());
        assertFalse(anchor.anchor(time.clock(), "frame-a", -0.1).isAvailable());

        LoopTimestamp retained = anchor.anchor(time.clock(), "frame-a", 0.2);
        assertTrue(retained.isAvailable());
        assertSame(retained, anchor.anchor(time.clock(), "frame-a", 0.4));
        assertEquals(0.2, retained.ageSec(time.clock()), 1e-12);
    }

    @Test
    public void sameIdentityIsBlockedAfterSameValueClockResetUntilIdentityChanges() {
        ManualLoopClock time = new ManualLoopClock(7.0);
        FtcFrameTimestampAnchor anchor = new FtcFrameTimestampAnchor();
        Object frameA = new String("frame-a");
        LoopTimestamp beforeReset = anchor.anchor(time.clock(), frameA, 0.1);

        time.clock().reset(7.0);
        assertFalse(anchor.anchor(time.clock(), new String("frame-a"), 0.0).isAvailable());
        assertFalse(anchor.anchor(time.clock(), frameA, 0.0).isAvailable());
        assertTrue(Double.isNaN(beforeReset.ageSec(time.clock())));

        LoopTimestamp frameB = anchor.anchor(time.clock(), "frame-b", 0.05);
        assertTrue(frameB.isAvailable());
        assertEquals(0.05, frameB.ageSec(time.clock()), 1e-12);
    }

    @Test
    public void distinctFirstPostResetIdentityDoesNotLetOldIdentityReanchorLater() {
        ManualLoopClock time = new ManualLoopClock(9.0);
        FtcFrameTimestampAnchor anchor = new FtcFrameTimestampAnchor();
        anchor.anchor(time.clock(), "frame-a", 0.1);

        time.clock().reset(9.0);
        LoopTimestamp frameB = anchor.anchor(time.clock(), "frame-b", 0.0);

        assertTrue(frameB.isAvailable());
        assertFalse(anchor.anchor(time.clock(), "frame-a", 0.0).isAvailable());
        assertSame(frameB, anchor.anchor(time.clock(), "frame-b", 0.2));
    }

    @Test
    public void oneAnchorRequiresOneStableLoopClock() {
        FtcFrameTimestampAnchor anchor = new FtcFrameTimestampAnchor();
        LoopClock first = new ManualLoopClock(1.0).clock();
        LoopClock second = new ManualLoopClock(1.0).clock();
        anchor.anchor(first, "frame-a", 0.0);

        try {
            anchor.anchor(second, "frame-b", 0.0);
            fail("Expected stable-clock failure");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("one stable LoopClock"));
        }
    }
}
