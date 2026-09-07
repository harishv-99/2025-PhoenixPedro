package edu.ftcsushi.fw.sensing.vision;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Vec3;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.sensing.observation.TargetObservation2d;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Pure model/math regressions; synthetic coordinates do not validate a physical camera. */
public final class FloorTargetProjectionTest {
    private static final double EPS = 1.0e-9;
    private final ManualLoopClock time = new ManualLoopClock(2.0);
    private final LoopTimestamp captured = time.clock().nowTimestamp();

    @Test public void planeAnglesLocateReferencePointWithoutManufacturingConfidenceOrIdentity() {
        CameraMountConfig mount = CameraMountConfig.of(3, -1, 12, 0, 0, 0);
        FloorTargetProjection.Result projected = FloorTargetProjection.projectAngles(
                Math.atan(0.5), -Math.PI / 4, mount, FloorTargetModel.atHeightInches(2), captured);
        assertTrue(projected.isAvailable());
        TargetObservation2d point = projected.observation();
        assertEquals(13, point.forwardInches, EPS);
        assertEquals(4, point.leftInches, EPS);
        assertEquals(Math.atan2(4, 13), point.bearingRad, EPS);
        assertFalse(point.hasQuality());
        assertFalse(point.hasTargetId());
        assertFalse(point.hasOrientation());
        assertFalse(point.hasFieldPosition());
        assertSame(captured, point.timestamp);
    }

    @Test public void fullMountRotationIsAppliedBeforeIntersectingThePlane() {
        CameraMountConfig mount = CameraMountConfig.of(2, 3, 12, Math.PI / 2, Math.PI / 4, 0);
        TargetObservation2d point = FloorTargetProjection.projectAngles(0, 0, mount,
                FloorTargetModel.atHeightInches(0), captured).observation();
        assertEquals(2, point.forwardInches, EPS);
        assertEquals(15, point.leftInches, EPS);
    }

    @Test public void nonzeroRollMixesOffAxisComponentsBeforePitchYawAndTranslation() {
        CameraMountConfig mount = CameraMountConfig.of(
                3, -2, 12, Math.PI, Math.PI / 2, Math.PI / 2);
        FloorTargetProjection.Result result = FloorTargetProjection.projectRay(
                new Vec3(2, -1, 0.5), mount, FloorTargetModel.atHeightInches(2), captured);

        // Independent right-hand turns, without the production rotation matrix:
        // roll (2,-1,0.5) -> (2,-0.5,-1), pitch -> (-1,-0.5,-2), half-turn yaw -> (1,0.5,-2).
        // Reaching z=2 from the translated lens at z=12 scales this ray by 5.
        // Add the lens's (3,-2) offset only after that intersection: (8,0.5).
        assertTrue(result.reason().toString(), result.isAvailable());
        TargetObservation2d point = result.observation();
        assertEquals(8.0, point.forwardInches, EPS);
        assertEquals(0.5, point.leftInches, EPS);
        assertSame(captured, point.timestamp);
    }

    @Test public void positiveRayScalingIncludingHugeAndTinyValuesDoesNotChangeLocation() {
        CameraMountConfig mount = CameraMountConfig.of(0, 0, 10, 0, 0, 0);
        for (double scale : new double[] {1, 1.0e300, 1.0e-300}) {
            TargetObservation2d point = FloorTargetProjection.projectRay(new Vec3(scale, 0, -scale),
                    mount, FloorTargetModel.atHeightInches(0), captured).observation();
            assertEquals(10, point.forwardInches, EPS);
            assertEquals(0, point.leftInches, EPS);
        }
    }

    @Test public void nearHorizonUpwardBackwardAndMalformedRaysAreUnavailable() {
        CameraMountConfig mount = CameraMountConfig.of(0, 0, 10, 0, 0, 0);
        for (Vec3 ray : new Vec3[] {new Vec3(1, 0, -1e-12), new Vec3(1, 0, 0),
                new Vec3(1, 0, 1), new Vec3(-1, 0, -1), new Vec3(0, 0, 0),
                new Vec3(Double.NaN, 0, -1), new Vec3(Double.POSITIVE_INFINITY, 0, -1)}) {
            FloorTargetProjection.Result result = FloorTargetProjection.projectRay(ray, mount,
                    FloorTargetModel.atHeightInches(0), captured);
            assertFalse(result.isAvailable());
            assertFalse(result.observation().hasTarget);
        }
    }

    @Test public void cameraBelowPlaneAndInvalidPlaneAnglesCannotBecomePositions() {
        CameraMountConfig mount = CameraMountConfig.of(0, 0, 2, 0, 0, 0);
        assertEquals(FloorTargetProjection.Reason.CAMERA_NOT_ABOVE_PLANE,
                FloorTargetProjection.projectAngles(0, -0.5, mount,
                        FloorTargetModel.atHeightInches(2), captured).reason());
        for (double angle : new double[] {Double.NaN, Double.POSITIVE_INFINITY, Math.PI / 2, -Math.PI / 2}) {
            assertEquals(FloorTargetProjection.Reason.INVALID_ANGLES,
                    FloorTargetProjection.projectAngles(angle, -0.5, mount,
                            FloorTargetModel.atHeightInches(0), captured).reason());
        }
    }

    @Test public void explicitRangeLimitIsInclusiveAndNotAPhysicalDefault() {
        CameraMountConfig mount = CameraMountConfig.of(0, 0, 16, 0, 0, 0);
        FloorTargetModel plane = FloorTargetModel.atHeightInches(0);
        assertEquals(Double.POSITIVE_INFINITY, plane.maxRangeInches(), 0);
        assertTrue(FloorTargetProjection.projectRay(new Vec3(0.6, 0, -0.8), mount,
                plane.withMaxRangeInches(20), captured).isAvailable());
        assertEquals(FloorTargetProjection.Reason.RANGE_EXCEEDED,
                FloorTargetProjection.projectRay(new Vec3(0.6, 0, -0.8), mount,
                        plane.withMaxRangeInches(19.9), captured).reason());
    }

    @Test public void overflowAndUnknownTimestampFailClosed() {
        CameraMountConfig mount = CameraMountConfig.of(0, 0, Double.MAX_VALUE, 0, 0, 0);
        assertEquals(FloorTargetProjection.Reason.NONFINITE_POSITION,
                FloorTargetProjection.projectAngles(0, -Math.PI / 4, mount,
                        FloorTargetModel.atHeightInches(0), captured).reason());
        assertEquals(FloorTargetProjection.Reason.TIMESTAMP_UNAVAILABLE,
                FloorTargetProjection.projectAngles(0, -Math.PI / 4, mount,
                        FloorTargetModel.atHeightInches(0), LoopTimestamp.unavailable()).reason());
    }

    @Test public void physicalModelRejectsInvalidAuthoredValuesAndRetainedCaptureAges() {
        for (double value : new double[] {-1, Double.NaN, Double.POSITIVE_INFINITY}) {
            expectInvalid(() -> FloorTargetModel.atHeightInches(value));
            expectInvalid(() -> FloorTargetModel.atHeightInches(0).withMaxRangeInches(value));
        }
        expectInvalid(() -> FloorTargetModel.atHeightInches(0).withMaxRangeInches(0));
        TargetObservation2d point = FloorTargetProjection.projectAngles(0, -Math.PI / 4,
                CameraMountConfig.of(0, 0, 10, 0, 0, 0), FloorTargetModel.atHeightInches(0), captured)
                .observation();
        time.nextCycle(0.3);
        assertEquals(0.3, point.ageSec(time.clock()), EPS);
        time.clock().reset(2.3);
        assertFalse(point.isFresh(time.clock(), 100));
    }

    private static void expectInvalid(Runnable action) {
        try { action.run(); fail("expected invalid configuration"); }
        catch (IllegalArgumentException expected) { assertFalse(expected.getMessage().isEmpty()); }
    }
}
