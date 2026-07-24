package edu.ftcphoenix.fw.sensing.vision;

import org.junit.Test;

import java.util.Collections;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.sensing.observation.TargetObservation2d;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/** Verifies that camera-mount conversion forwards only trustworthy frame timestamps. */
public final class CameraMountLogicTimestampTest {

    private static final double EPSILON = 1.0e-12;

    @Test
    public void framedObservationForwardsItsExactTimestamp() {
        ManualLoopClock time = new ManualLoopClock(4.0);
        LoopTimestamp frameTimestamp = time.clock().timestampSecondsAgo(0.15);
        AprilTagObservation geometry = AprilTagObservation.target(
                8,
                new Pose3d(20.0, 3.0, 1.0, 0.25, 0.0, 0.0)
        );
        AprilTagObservation framed = AprilTagDetections.fromFrame(
                frameTimestamp,
                Collections.singletonList(geometry)
        ).observations.get(0);

        TargetObservation2d converted = CameraMountLogic.robotObservation2d(
                framed,
                CameraMountConfig.identity(),
                time.clock()
        );

        assertTrue(converted.hasTarget);
        assertEquals(8, converted.targetId);
        assertEquals(20.0, converted.forwardInches, EPSILON);
        assertEquals(3.0, converted.leftInches, EPSILON);
        assertSame(frameTimestamp, converted.timestamp);
    }

    @Test
    public void unframedOrResetInvalidatedObservationFailsClosed() {
        ManualLoopClock time = new ManualLoopClock(4.0);
        AprilTagObservation geometry = AprilTagObservation.target(8, Pose3d.zero());

        assertFalse(CameraMountLogic.robotObservation2d(
                geometry,
                CameraMountConfig.identity(),
                time.clock()
        ).hasTarget);

        AprilTagObservation framed = AprilTagDetections.fromFrame(
                time.clock().nowTimestamp(),
                Collections.singletonList(geometry)
        ).observations.get(0);
        time.clock().reset(4.0);

        assertFalse(CameraMountLogic.robotObservation2d(
                framed,
                CameraMountConfig.identity(),
                time.clock()
        ).hasTarget);
    }

    @Test
    public void materiallyFutureObservationFailsClosed() {
        ManualLoopClock time = new ManualLoopClock(4.0);
        AprilTagObservation framed = AprilTagDetections.fromFrame(
                time.clock().nowTimestamp(),
                Collections.singletonList(AprilTagObservation.target(8, Pose3d.zero()))
        ).observations.get(0);

        time.clock().update(4.0 - 2.0e-6);

        assertFalse(CameraMountLogic.robotObservation2d(
                framed,
                CameraMountConfig.identity(),
                time.clock()
        ).hasTarget);
    }
}
