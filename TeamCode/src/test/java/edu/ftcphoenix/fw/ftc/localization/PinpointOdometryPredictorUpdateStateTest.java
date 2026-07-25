package edu.ftcphoenix.fw.ftc.localization;

import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.localization.MotionDelta;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Deterministic cycle and motion-baseline coverage without an FTC hardware-device mock. */
public final class PinpointOdometryPredictorUpdateStateTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void successfulCycleIsClaimedBeforeEffectsAndDuplicatesAreNoOps() {
        ManualLoopClock time = new ManualLoopClock();
        PinpointOdometryPredictor.UpdateState state =
                new PinpointOdometryPredictor.UpdateState();

        assertTrue(state.beginUpdate(time.clock()));
        try {
            state.beginUpdate(time.clock());
            fail("Expected same-cycle reentry to fail fast");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("reentered"));
            assertTrue(expected.getMessage().contains("once per cycle"));
        }

        time.nextCycle(0.02);
        try {
            state.beginUpdate(time.clock());
            fail("Expected reentry to fail even if the clock was advanced incorrectly");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("still in progress"));
        }
        state.endUpdate();

        assertTrue(state.beginUpdate(time.clock()));
        state.endUpdate();
        assertFalse(state.beginUpdate(time.clock()));

        time.nextCycle(0.02);
        assertTrue(state.beginUpdate(time.clock()));
        state.endUpdate();
    }

    @Test
    public void failedCycleRethrowsFirstFailureAndNextCycleMayTryAgain() {
        ManualLoopClock time = new ManualLoopClock();
        PinpointOdometryPredictor.UpdateState state =
                new PinpointOdometryPredictor.UpdateState();
        RuntimeException firstFailure = new IllegalStateException("hardware failed");
        RuntimeException laterFailure = new IllegalArgumentException("later failure");

        assertTrue(state.beginUpdate(time.clock()));
        assertSame(firstFailure, state.recordFailure(firstFailure));
        assertSame(firstFailure, state.recordFailure(laterFailure));
        state.endUpdate();

        try {
            state.beginUpdate(time.clock());
            fail("Expected the first same-cycle failure to be retained");
        } catch (RuntimeException repeatedFailure) {
            assertSame(firstFailure, repeatedFailure);
        }

        time.nextCycle(0.02);
        assertTrue(state.beginUpdate(time.clock()));
        state.endUpdate();
    }

    @Test
    public void nullClockIsRejectedBeforeCycleClaim() {
        PinpointOdometryPredictor.UpdateState state =
                new PinpointOdometryPredictor.UpdateState();

        try {
            state.beginUpdate(null);
            fail("Expected a shared LoopClock to be required");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("shared LoopClock"));
        }
    }

    @Test
    public void zeroTimePollPublishesNoDeltaWithoutConsumingMotion() {
        ManualLoopClock time = new ManualLoopClock();
        PinpointOdometryPredictor.UpdateState state =
                new PinpointOdometryPredictor.UpdateState();
        LoopTimestamp baselineTimestamp = time.clock().nowTimestamp();

        MotionDelta initial = state.observeMotion(poseAtX(0.0), baselineTimestamp, 0.8);
        assertFalse(initial.hasDelta);

        time.nextCycle(0.0);
        MotionDelta zeroTime = state.observeMotion(
                poseAtX(3.0),
                time.clock().nowTimestamp(),
                0.8
        );
        assertFalse(zeroTime.hasDelta);

        time.nextCycle(1.0);
        LoopTimestamp positiveTimestamp = time.clock().nowTimestamp();
        MotionDelta accumulated = state.observeMotion(poseAtX(5.0), positiveTimestamp, 0.8);

        assertTrue(accumulated.hasDelta);
        assertEquals(5.0, accumulated.deltaPose.xInches, EPSILON);
        assertEquals(1.0, accumulated.durationSec(), EPSILON);
        assertSame(baselineTimestamp, accumulated.startTimestamp);
        assertSame(positiveTimestamp, accumulated.endTimestamp);
    }

    @Test
    public void resetEpochAndExplicitInvalidationRequireFreshBaselines() {
        ManualLoopClock time = new ManualLoopClock();
        PinpointOdometryPredictor.UpdateState state =
                new PinpointOdometryPredictor.UpdateState();

        state.observeMotion(poseAtX(0.0), time.clock().nowTimestamp(), 1.0);
        time.nextCycle(1.0);
        assertEquals(
                4.0,
                state.observeMotion(poseAtX(4.0), time.clock().nowTimestamp(), 1.0)
                        .deltaPose.xInches,
                EPSILON
        );

        time.clock().reset(time.clock().nowSec());
        MotionDelta afterEpochReset = state.observeMotion(
                poseAtX(20.0),
                time.clock().nowTimestamp(),
                1.0
        );
        assertFalse(afterEpochReset.hasDelta);

        time.nextCycle(1.0);
        MotionDelta afterFreshInterval = state.observeMotion(
                poseAtX(22.0),
                time.clock().nowTimestamp(),
                1.0
        );
        assertTrue(afterFreshInterval.hasDelta);
        assertEquals(2.0, afterFreshInterval.deltaPose.xInches, EPSILON);

        state.invalidateMotionBaseline();
        time.nextCycle(1.0);
        assertFalse(state.observeMotion(
                poseAtX(100.0),
                time.clock().nowTimestamp(),
                1.0
        ).hasDelta);
        time.nextCycle(1.0);
        MotionDelta afterInvalidation = state.observeMotion(
                poseAtX(103.0),
                time.clock().nowTimestamp(),
                1.0
        );
        assertTrue(afterInvalidation.hasDelta);
        assertEquals(3.0, afterInvalidation.deltaPose.xInches, EPSILON);
    }

    @Test
    public void poseRebaseAnchorsLaterMotionWithoutChangingCycleEligibility() {
        ManualLoopClock time = new ManualLoopClock();
        PinpointOdometryPredictor.UpdateState state =
                new PinpointOdometryPredictor.UpdateState();

        assertTrue(state.beginUpdate(time.clock()));
        state.endUpdate();
        state.rebaseMotionBaseline(poseAtX(50.0), time.clock().nowTimestamp());

        assertFalse(state.beginUpdate(time.clock()));

        time.nextCycle(0.5);
        MotionDelta afterRebase = state.observeMotion(
                poseAtX(52.5),
                time.clock().nowTimestamp(),
                0.7
        );
        assertTrue(afterRebase.hasDelta);
        assertEquals(2.5, afterRebase.deltaPose.xInches, EPSILON);
        assertEquals(0.5, afterRebase.durationSec(), EPSILON);
    }

    private static Pose3d poseAtX(double xInches) {
        return new Pose3d(xInches, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
}
