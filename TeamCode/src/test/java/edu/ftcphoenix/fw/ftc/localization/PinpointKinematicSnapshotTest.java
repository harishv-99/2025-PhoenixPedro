package edu.ftcphoenix.fw.ftc.localization;

import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Focused value and heading-wrap tests for Pinpoint's immutable kinematic sample. */
public final class PinpointKinematicSnapshotTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void sampledSnapshotCarriesSameCycleKinematics() {
        ManualLoopClock manualClock = new ManualLoopClock();
        manualClock.nextCycle(0.02);

        PinpointKinematicSnapshot sample = PinpointKinematicSnapshot.sampled(
                new Pose2d(12.0, -7.0, 0.4),
                true,
                manualClock.clock().cycle(),
                manualClock.clock().nowSec(),
                20.0,
                -3.0,
                1.2,
                7.5
        );

        assertTrue(sample.hasPose);
        assertTrue(sample.hasVelocity);
        assertTrue(sample.hasUsableKinematics());
        assertTrue(sample.isCurrentFor(manualClock.clock()));
        assertEquals(12.0, sample.fieldToRobotPose.xInches, EPSILON);
        assertEquals(-7.0, sample.fieldToRobotPose.yInches, EPSILON);
        assertEquals(20.0, sample.fieldVelocityXInchesPerSec, EPSILON);
        assertEquals(-3.0, sample.fieldVelocityYInchesPerSec, EPSILON);
        assertEquals(1.2, sample.angularVelocityRadPerSec, EPSILON);
        assertEquals(7.5, sample.totalHeadingRad, EPSILON);

        manualClock.nextCycle(0.02);
        assertFalse(sample.isCurrentFor(manualClock.clock()));
    }

    @Test
    public void poseRebasePreservesPhysicalMotionAndPollIdentity() {
        PinpointKinematicSnapshot sample = PinpointKinematicSnapshot.sampled(
                new Pose2d(1.0, 2.0, 0.3),
                true,
                9L,
                1.5,
                4.0,
                5.0,
                0.6,
                2.4
        );

        PinpointKinematicSnapshot rebased = sample.withRebasedPose(
                new Pose2d(40.0, 50.0, -1.0)
        );

        assertTrue(rebased.hasUsableKinematics());
        assertEquals(40.0, rebased.fieldToRobotPose.xInches, EPSILON);
        assertEquals(50.0, rebased.fieldToRobotPose.yInches, EPSILON);
        assertEquals(-1.0, rebased.fieldToRobotPose.headingRad, EPSILON);
        assertEquals(sample.cycle, rebased.cycle);
        assertEquals(sample.timestampSec, rebased.timestampSec, EPSILON);
        assertEquals(sample.fieldVelocityXInchesPerSec,
                rebased.fieldVelocityXInchesPerSec, EPSILON);
        assertEquals(sample.fieldVelocityYInchesPerSec,
                rebased.fieldVelocityYInchesPerSec, EPSILON);
        assertEquals(sample.angularVelocityRadPerSec,
                rebased.angularVelocityRadPerSec, EPSILON);
        assertEquals(sample.totalHeadingRad, rebased.totalHeadingRad, EPSILON);
    }

    @Test
    public void motionInvalidationZerosVelocityWithoutInventingPoseOrHeading() {
        PinpointKinematicSnapshot sample = PinpointKinematicSnapshot.sampled(
                new Pose2d(3.0, 4.0, 0.5),
                true,
                4L,
                0.8,
                6.0,
                7.0,
                0.9,
                -8.0
        );

        PinpointKinematicSnapshot invalidated = sample.withoutVelocity();

        assertTrue(invalidated.hasPose);
        assertFalse(invalidated.hasVelocity);
        assertFalse(invalidated.hasUsableKinematics());
        assertEquals(sample.fieldToRobotPose.xInches,
                invalidated.fieldToRobotPose.xInches, EPSILON);
        assertEquals(sample.fieldToRobotPose.yInches,
                invalidated.fieldToRobotPose.yInches, EPSILON);
        assertEquals(0.0, invalidated.fieldVelocityXInchesPerSec, EPSILON);
        assertEquals(0.0, invalidated.fieldVelocityYInchesPerSec, EPSILON);
        assertEquals(0.0, invalidated.angularVelocityRadPerSec, EPSILON);
        assertEquals(sample.totalHeadingRad, invalidated.totalHeadingRad, EPSILON);
    }

    @Test
    public void unavailableSnapshotHasNoFalseZeroVelocityMeasurement() {
        PinpointKinematicSnapshot unavailable = PinpointKinematicSnapshot.unavailable(
                PinpointKinematicSnapshot.NO_CYCLE,
                0.0,
                3.0
        );

        assertFalse(unavailable.hasPose);
        assertFalse(unavailable.hasVelocity);
        assertFalse(unavailable.hasUsableKinematics());
        assertEquals(0.0, unavailable.fieldVelocityXInchesPerSec, EPSILON);
        assertEquals(0.0, unavailable.fieldVelocityYInchesPerSec, EPSILON);
        assertEquals(0.0, unavailable.angularVelocityRadPerSec, EPSILON);
        assertEquals(3.0, unavailable.totalHeadingRad, EPSILON);
    }

    @Test
    public void unwrappedHeadingUsesShortestSignedTurnAcrossWrap() {
        double forwardAcrossWrap = PinpointOdometryPredictor.accumulateUnwrappedHeadingRad(
                10.0,
                Math.toRadians(179.0),
                Math.toRadians(-179.0)
        );
        double backwardAcrossWrap = PinpointOdometryPredictor.accumulateUnwrappedHeadingRad(
                10.0,
                Math.toRadians(-179.0),
                Math.toRadians(179.0)
        );

        assertEquals(10.0 + Math.toRadians(2.0), forwardAcrossWrap, EPSILON);
        assertEquals(10.0 - Math.toRadians(2.0), backwardAcrossWrap, EPSILON);
    }
}
