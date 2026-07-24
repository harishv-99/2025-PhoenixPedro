package edu.ftcphoenix.fw.sensing.vision.apriltag;

import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the epoch-safe timestamp contract of the portable AprilTag values. */
public final class AprilTagTimestampTest {

    private static final double EPSILON = 1.0e-12;

    @Test
    public void retainedFrameAndObservationAgeFromOneTimestamp() {
        ManualLoopClock time = new ManualLoopClock(20.0);
        LoopTimestamp frameTimestamp = time.clock().timestampSecondsAgo(0.10);
        AprilTagObservation observation = AprilTagObservation.target(
                5,
                Pose3d.zero(),
                frameTimestamp
        );
        AprilTagDetections detections = AprilTagDetections.of(
                frameTimestamp,
                Collections.singletonList(observation)
        );

        assertSame(frameTimestamp, detections.frameTimestamp());
        assertSame(frameTimestamp, observation.frameTimestamp());
        assertEquals(0.10, detections.frameAgeSec(time.clock()), EPSILON);
        assertSame(observation, detections.forId(time.clock(), 5, 0.11));

        time.nextCycle(0.15);

        assertEquals(0.25, detections.frameAgeSec(time.clock()), EPSILON);
        assertEquals(0.25, observation.frameAgeSec(time.clock()), EPSILON);
        assertFalse(detections.isFresh(time.clock(), 0.20));
        assertFalse(observation.isFresh(time.clock(), 0.20));
        assertFalse(detections.forId(time.clock(), 5, 0.20).hasTarget);
    }

    @Test
    public void sameValueResetInvalidatesRetainedAprilTagValues() {
        ManualLoopClock time = new ManualLoopClock(3.0);
        LoopTimestamp timestamp = time.clock().nowTimestamp();
        AprilTagObservation observation = AprilTagObservation.target(
                7,
                Pose3d.zero(),
                timestamp
        );
        AprilTagDetections detections = AprilTagDetections.of(
                timestamp,
                Collections.singletonList(observation)
        );

        time.clock().reset(3.0);

        assertTrue(Double.isNaN(detections.frameAgeSec(time.clock())));
        assertTrue(Double.isNaN(observation.frameAgeSec(time.clock())));
        assertFalse(detections.isFresh(time.clock(), 10.0));
        assertFalse(observation.isFresh(time.clock(), 10.0));
    }

    @Test
    public void timestampedEmptyFrameDiffersFromUnavailableSnapshot() {
        ManualLoopClock time = new ManualLoopClock();
        AprilTagDetections emptyFrame = AprilTagDetections.of(
                time.clock().nowTimestamp(),
                Collections.<AprilTagObservation>emptyList()
        );

        assertTrue(emptyFrame.isFresh(time.clock(), 0.0));
        assertTrue(emptyFrame.observations.isEmpty());
        assertFalse(AprilTagDetections.none().isFresh(time.clock(), 100.0));
        assertSame(LoopTimestamp.unavailable(),
                AprilTagDetections.none().frameTimestamp());
        assertSame(LoopTimestamp.unavailable(),
                AprilTagObservation.noTarget().frameTimestamp());
    }

    @Test
    public void frameConstructionRejectsMissingOrContradictoryTiming() {
        ManualLoopClock time = new ManualLoopClock();
        LoopTimestamp first = time.clock().nowTimestamp();
        time.nextCycle(0.01);
        LoopTimestamp second = time.clock().nowTimestamp();
        AprilTagObservation firstObservation = AprilTagObservation.target(
                1,
                Pose3d.zero(),
                first
        );
        AprilTagObservation secondObservation = AprilTagObservation.target(
                2,
                Pose3d.zero(),
                second
        );

        expectIllegalArgument(() -> AprilTagDetections.of(
                LoopTimestamp.unavailable(),
                Collections.<AprilTagObservation>emptyList()
        ));
        expectIllegalArgument(() -> AprilTagDetections.of(
                first,
                Arrays.asList(firstObservation, secondObservation)
        ));
        expectIllegalArgument(() -> AprilTagDetections.of(
                first,
                Collections.singletonList(AprilTagObservation.noTarget())
        ));
    }

    @Test
    public void differentClockUseIsAnActionableWiringFailure() {
        ManualLoopClock owner = new ManualLoopClock();
        LoopClock other = new ManualLoopClock().clock();
        LoopTimestamp timestamp = owner.clock().nowTimestamp();
        AprilTagDetections detections = AprilTagDetections.of(
                timestamp,
                Collections.<AprilTagObservation>emptyList()
        );

        expectIllegalArgument(() -> detections.frameAgeSec(other));
        expectIllegalArgument(() -> detections.isFresh(other, 1.0));
    }

    @Test
    public void freshnessSourceFactoriesRejectInvalidConfigurationImmediately() {
        Source<AprilTagDetections> detections = clock -> AprilTagDetections.none();
        double[] invalidMaxAges = {
                -0.01,
                Double.NaN,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY
        };

        for (double invalidMaxAge : invalidMaxAges) {
            expectIllegalArgument(() -> AprilTagSources.observationForId(
                    detections, 1, invalidMaxAge));
            expectIllegalArgument(() -> AprilTagSources.hasFreshAny(
                    detections, Collections.singleton(1), invalidMaxAge));
            expectIllegalArgument(() -> AprilTagSources.visibleIds(
                    detections, invalidMaxAge));
        }
    }

    private static void expectIllegalArgument(Runnable action) {
        try {
            action.run();
            fail("Expected IllegalArgumentException");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage() != null && !expected.getMessage().isEmpty());
        }
    }
}
