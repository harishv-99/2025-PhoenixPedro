package edu.ftcphoenix.fw.sensing.vision.apriltag;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the epoch-safe timestamp contract of the portable AprilTag values. */
public final class AprilTagTimestampTest {

    private static final double EPSILON = 1.0e-12;

    @Test
    public void retainedFrameAndObservationAgeFromOneTimestamp() {
        ManualLoopClock time = new ManualLoopClock(20.0);
        LoopTimestamp frameTimestamp = time.clock().timestampSecondsAgo(0.125);
        AprilTagObservation observation = AprilTagObservation.target(
                5,
                Pose3d.zero()
        );
        AprilTagDetections detections = AprilTagDetections.fromFrame(
                frameTimestamp,
                Collections.singletonList(observation)
        );
        AprilTagObservation framedObservation = detections.observations.get(0);

        assertSame(frameTimestamp, detections.frameTimestamp());
        assertSame(LoopTimestamp.unavailable(), observation.frameTimestamp());
        assertNotSame(observation, framedObservation);
        assertSame(frameTimestamp, framedObservation.frameTimestamp());
        assertEquals(0.125, detections.frameAgeSec(time.clock()), EPSILON);
        assertTrue(detections.isFresh(time.clock(), 0.125));
        assertTrue(framedObservation.isFresh(time.clock(), 0.125));
        assertSame(framedObservation, detections.forId(time.clock(), 5, 0.125));

        time.nextCycle(0.125);

        assertEquals(0.25, detections.frameAgeSec(time.clock()), EPSILON);
        assertEquals(0.25, framedObservation.frameAgeSec(time.clock()), EPSILON);
        assertFalse(detections.isFresh(time.clock(), 0.20));
        assertFalse(framedObservation.isFresh(time.clock(), 0.20));
        assertFalse(detections.forId(time.clock(), 5, 0.20).hasTarget);
    }

    @Test
    public void sameValueResetInvalidatesRetainedAprilTagValues() {
        ManualLoopClock time = new ManualLoopClock(3.0);
        LoopTimestamp timestamp = time.clock().nowTimestamp();
        AprilTagObservation observation = AprilTagObservation.target(
                7,
                Pose3d.zero()
        );
        AprilTagDetections detections = AprilTagDetections.fromFrame(
                timestamp,
                Collections.singletonList(observation)
        );
        observation = detections.observations.get(0);

        time.clock().reset(3.0);

        assertTrue(Double.isNaN(detections.frameAgeSec(time.clock())));
        assertTrue(Double.isNaN(observation.frameAgeSec(time.clock())));
        assertFalse(detections.isFresh(time.clock(), 10.0));
        assertFalse(observation.isFresh(time.clock(), 10.0));
    }

    @Test
    public void timestampedEmptyFrameDiffersFromUnavailableSnapshot() {
        ManualLoopClock time = new ManualLoopClock();
        AprilTagDetections emptyFrame = AprilTagDetections.fromFrame(
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
    public void geometryFactoriesStayUnframedUntilAFrameAttachesThem() {
        ManualLoopClock time = new ManualLoopClock();
        Pose3d cameraToTagPose = new Pose3d(12.0, 2.0, 1.0, 0.1, 0.2, 0.3);
        Pose3d fieldToRobotPose = new Pose3d(24.0, -12.0, 0.0, -0.2, 0.0, 0.0);
        AprilTagObservation geometryOnly = AprilTagObservation.target(1, cameraToTagPose);
        AprilTagObservation geometryWithFieldPose = AprilTagObservation.target(
                2,
                cameraToTagPose,
                fieldToRobotPose
        );

        assertSame(LoopTimestamp.unavailable(), geometryOnly.frameTimestamp());
        assertSame(LoopTimestamp.unavailable(), geometryWithFieldPose.frameTimestamp());
        assertTrue(Double.isNaN(geometryOnly.frameAgeSec(time.clock())));
        assertFalse(geometryOnly.isFresh(time.clock(), 1.0));
        assertFalse(geometryOnly.hasFieldToRobotPose());
        assertTrue(geometryWithFieldPose.hasFieldToRobotPose());
        assertSame(fieldToRobotPose, geometryWithFieldPose.fieldToRobotPose);
    }

    @Test
    public void frameConstructionAttachesCopiesAndRejectsRestamping() {
        ManualLoopClock time = new ManualLoopClock();
        LoopTimestamp first = time.clock().nowTimestamp();
        LoopTimestamp sameCoordinateButDistinct = time.clock().nowTimestamp();
        time.nextCycle(0.01);
        LoopTimestamp second = time.clock().nowTimestamp();
        AprilTagObservation geometry = AprilTagObservation.target(1, Pose3d.zero());
        AprilTagDetections firstFrame = AprilTagDetections.fromFrame(
                first,
                Collections.singletonList(geometry)
        );
        AprilTagObservation attached = firstFrame.observations.get(0);
        AprilTagDetections sameFrame = AprilTagDetections.fromFrame(
                first,
                Collections.singletonList(attached)
        );

        assertSame(first, attached.frameTimestamp());
        assertSame(attached, sameFrame.observations.get(0));
        assertSame(LoopTimestamp.unavailable(), geometry.frameTimestamp());

        expectIllegalArgument(() -> AprilTagDetections.fromFrame(
                LoopTimestamp.unavailable(),
                Collections.<AprilTagObservation>emptyList()
        ));
        expectIllegalArgument(() -> AprilTagDetections.fromFrame(
                sameCoordinateButDistinct,
                Collections.singletonList(attached)
        ));
        expectIllegalArgument(() -> AprilTagDetections.fromFrame(
                second,
                Collections.singletonList(attached)
        ));
        expectIllegalArgument(() -> AprilTagDetections.fromFrame(
                first,
                Collections.singletonList(AprilTagObservation.noTarget())
        ));
    }

    @Test
    public void frameSnapshotDefensivelyCopiesItsObservationList() {
        ManualLoopClock time = new ManualLoopClock();
        ArrayList<AprilTagObservation> input =
                new ArrayList<AprilTagObservation>(Arrays.asList(
                        AprilTagObservation.target(1, Pose3d.zero()),
                        AprilTagObservation.target(2, Pose3d.zero())
                ));
        AprilTagDetections detections = AprilTagDetections.fromFrame(
                time.clock().nowTimestamp(),
                input
        );

        input.clear();

        assertEquals(2, detections.observations.size());
        expectUnsupported(() -> detections.observations.clear());
    }

    @Test
    public void materiallyFutureFrameFailsFreshnessClosed() {
        ManualLoopClock time = new ManualLoopClock(10.0);
        AprilTagDetections detections = AprilTagDetections.fromFrame(
                time.clock().nowTimestamp(),
                Collections.singletonList(AprilTagObservation.target(1, Pose3d.zero()))
        );
        AprilTagObservation observation = detections.observations.get(0);

        time.clock().update(10.0 - 2.0e-6);

        assertTrue(Double.isNaN(detections.frameAgeSec(time.clock())));
        assertTrue(Double.isNaN(observation.frameAgeSec(time.clock())));
        assertFalse(detections.isFresh(time.clock(), 100.0));
        assertFalse(observation.isFresh(time.clock(), 100.0));
    }

    @Test
    public void cachedFrameExpiresInsideOrdinaryTagSelection() {
        ManualLoopClock time = new ManualLoopClock(2.0);
        AprilTagDetections cachedFrame = AprilTagDetections.fromFrame(
                time.clock().nowTimestamp(),
                Collections.singletonList(AprilTagObservation.target(5, Pose3d.zero()))
        );
        TagSelectionSource selection = TagSelections.from(clock -> cachedFrame)
                .among(Collections.singleton(5))
                .freshWithinSec(0.20)
                .choose(TagSelectionPolicies.closestRange())
                .continuous()
                .build();

        assertTrue(selection.get(time.clock()).hasFreshSelectedObservation);

        time.nextCycle(0.201);

        TagSelectionResult expired = selection.get(time.clock());
        assertFalse(expired.hasPreview);
        assertFalse(expired.hasFreshSelectedObservation);
    }

    @Test
    public void differentClockUseIsAnActionableWiringFailure() {
        ManualLoopClock owner = new ManualLoopClock();
        LoopClock other = new ManualLoopClock().clock();
        LoopTimestamp timestamp = owner.clock().nowTimestamp();
        AprilTagDetections detections = AprilTagDetections.fromFrame(
                timestamp,
                Collections.<AprilTagObservation>emptyList()
        );

        expectIllegalArgument(() -> detections.frameAgeSec(other));
        expectIllegalArgument(() -> detections.isFresh(other, 1.0));
    }

    @Test
    public void freshnessApisRejectInvalidConfigurationImmediately() {
        ManualLoopClock time = new ManualLoopClock();
        AprilTagDetections emptyFrame = AprilTagDetections.fromFrame(
                time.clock().nowTimestamp(),
                Collections.<AprilTagObservation>emptyList()
        );
        Source<AprilTagDetections> detections = clock -> AprilTagDetections.none();
        double[] invalidMaxAges = {
                -0.01,
                Double.NaN,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY
        };

        for (double invalidMaxAge : invalidMaxAges) {
            expectIllegalArgument(() -> AprilTagObservation.noTarget().isFresh(
                    time.clock(), invalidMaxAge));
            expectIllegalArgument(() -> emptyFrame.freshMatching(
                    time.clock(), Collections.<Integer>emptySet(), invalidMaxAge));
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

    private static void expectUnsupported(Runnable action) {
        try {
            action.run();
            fail("Expected UnsupportedOperationException");
        } catch (UnsupportedOperationException expected) {
            // Expected.
        }
    }
}
