package edu.ftcsushi.fw.sensing.observation;

import org.junit.Test;

import java.util.Collections;
import java.util.concurrent.atomic.AtomicInteger;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.TimeAwareSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.localization.PlanarPoseHistory;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.localization.PoseTrajectoryEstimator;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelectionPolicies;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelectionSource;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelections;
import edu.ftcsushi.fw.field.SimpleTagLayout;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Capture-time history, failed-lookup provenance, and borrowed-source ownership tests. */
public final class ObservationSourcesTest {
    private static final double EPS = 1.0e-9;

    @Test public void selectedTagProjectionUsesOneHistoricalMountAndRetainsActualHeadingAndTimestamp() {
        ManualLoopClock time = new ManualLoopClock();
        LoopTimestamp capture = time.clock().nowTimestamp();
        AprilTagDetections frame = AprilTagDetections.fromFrame(capture, Collections.singletonList(
                AprilTagObservation.target(7, new Pose3d(10, 2, 3, 0.4, 0.1, 0.2))));
        CameraMountConfig mount = CameraMountConfig.of(4, 1, 8, 0.3, -0.2, 0.1);
        AtomicInteger lookups = new AtomicInteger();
        TagSelectionSource selection = TagSelections.fromVisibleTags(Source.constant(frame),
                (clock, timestamp) -> {
                    assertSame(capture, timestamp);
                    lookups.incrementAndGet();
                    return mount;
                }).among(Collections.singleton(7)).freshWithinSec(0.2)
                .choose(TagSelectionPolicies.closestRange()).continuous().build();
        Source<TargetObservation2d> observed = ObservationSources.aprilTag(selection);
        time.nextCycle(0.1);
        Pose3d expected = mount.robotToCameraPose().then(frame.observations.get(0).cameraToTagPose);
        TargetObservation2d target = observed.get(time.clock());
        assertEquals(7, target.targetId);
        assertEquals(expected.xInches, target.forwardInches, EPS);
        assertEquals(expected.yInches, target.leftInches, EPS);
        assertEquals(expected.yawRad, target.targetHeadingRad, EPS);
        assertSame(capture, target.timestamp);
        assertFalse(target.hasQuality());
        observed.get(time.clock());
        assertEquals(1, lookups.get());
    }

    @Test public void heldOldTagDecisionNeverBecomesAFreshObservationAfterLossOrClockReset() {
        ManualLoopClock time = new ManualLoopClock();
        AprilTagDetections[] frame = {AprilTagDetections.fromFrame(time.clock().nowTimestamp(),
                Collections.singletonList(AprilTagObservation.target(7, new Pose3d(10, 0, 0, 0, 0, 0))))};
        TagSelectionSource selection = TagSelections.fromVisibleTags(clock -> frame[0], CameraMountConfig.identity())
                .among(Collections.singleton(7)).freshWithinSec(0.2).choose(TagSelectionPolicies.closestRange())
                .stickyWhen(BooleanSource.constant(true)).holdUntilDisabled().build();
        Source<TargetObservation2d> projected = ObservationSources.aprilTag(selection);
        assertTrue(projected.get(time.clock()).hasPosition());
        time.nextCycle(0.02);
        frame[0] = AprilTagDetections.fromFrame(time.clock().nowTimestamp(), Collections.emptyList());
        assertTrue(selection.get(time.clock()).hasSelection);
        assertFalse(projected.get(time.clock()).hasPosition());
        time.clock().reset(0);
        assertFalse(projected.get(time.clock()).hasPosition());
        assertFalse(selection.get(time.clock()).hasSelection);
    }

    @Test public void poseSelectedIdentityAndGeometryAreNotCameraObservations() {
        Fixture f = new Fixture();
        f.publish(0, 0, 0);
        TagSelectionSource selection = TagSelections.fromFieldPose(f.estimator,
                new SimpleTagLayout().addPose(7, new Pose3d(30, 2, 10, 0, 0, 0)), CameraMountConfig.identity())
                .among(Collections.singleton(7)).freshWithinSec(0.2).minQuality(0.1)
                .choose(TagSelectionPolicies.closestRange()).continuous().build();
        assertTrue(selection.get(f.time.clock()).hasSelection);
        assertNotNull(selection.get(f.time.clock()).currentSelectedCandidate);
        assertFalse(ObservationSources.aprilTag(selection).get(f.time.clock()).hasPosition());
    }

    @Test public void exactFieldProjectionPreservesRobotCoordinatesDetectorQualityAndCaptureTime() {
        Fixture f = new Fixture();
        f.publish(100, 20, Math.PI / 2);
        LoopTimestamp timestamp = f.time.clock().nowTimestamp();
        TargetObservation2d raw = TargetObservation2d.ofRobotRelativePosition(10, 2, Double.NaN, timestamp);
        TargetObservations2d frame = TargetObservations2d.fromFrame(timestamp, Collections.singletonList(raw));
        Source<TargetObservations2d> field = ObservationSources.inField(Source.constant(frame), f.history.lookupSource());
        TargetObservation2d point = field.get(f.time.clock()).observations().get(0);
        assertEquals(10, point.forwardInches, EPS);
        assertEquals(2, point.leftInches, EPS);
        assertEquals(98, point.fieldXInches, EPS);
        assertEquals(30, point.fieldYInches, EPS);
        assertSame(timestamp, point.timestamp);
        assertEquals(PlanarPoseHistory.Lookup.Kind.EXACT, point.fieldLookup().kind());
        assertFalse(point.hasQuality());
        assertEquals(0.8, point.fieldLookup().quality(), EPS);
        assertEquals(0, f.estimator.updates);
    }

    @Test public void interpolationUsesExposurePoseNotLatestRobotPose() {
        Fixture f = new Fixture();
        f.publish(0, 0, 0);
        f.time.nextCycle(0.1);
        f.publish(10, 0, 0);
        LoopTimestamp exposure = f.time.clock().timestampSecondsAgo(0.05);
        TargetObservations2d frame = TargetObservations2d.fromFrame(exposure,
                Collections.singletonList(TargetObservation2d.ofRobotRelativePosition(5, 0, Double.NaN, exposure)));
        TargetObservation2d point = ObservationSources.inField(Source.constant(frame), f.history.lookupSource())
                .get(f.time.clock()).observations().get(0);
        assertEquals(10, point.fieldXInches, EPS);
        assertEquals(PlanarPoseHistory.Lookup.Kind.INTERPOLATED, point.fieldLookup().kind());
        assertSame(exposure, point.fieldLookup().timestamp());
    }

    @Test public void historyResetIsVisibleEvenOnSameClockCycleAndSameCameraFrame() {
        Fixture f = new Fixture();
        f.publish(4, 5, 0);
        Source<TargetObservations2d> projected = f.projected(2, 3);
        TargetObservations2d first = projected.get(f.time.clock());
        projected.reset();
        assertTrue(projected.get(f.time.clock()).observations().get(0).hasFieldPosition());
        f.history.reset();
        TargetObservations2d afterReset = projected.get(f.time.clock());
        TargetObservation2d point = afterReset.observations().get(0);
        assertTrue(afterReset.isAvailable());
        assertTrue(point.hasPosition());
        assertFalse(point.hasFieldPosition());
        assertEquals(PlanarPoseHistory.Lookup.UnavailableReason.EMPTY, point.fieldLookup().unavailableReason());
        assertSame(first.timestamp(), afterReset.timestamp());
    }

    @Test public void historyEvictionDoesNotLeaveCachedFieldCoordinatesOnRepeatedImage() {
        Fixture f = new Fixture();
        f.publish(0, 0, 0);
        Source<TargetObservations2d> projected = f.projected(2, 3);
        assertTrue(projected.get(f.time.clock()).observations().get(0).hasFieldPosition());
        f.time.nextCycle(0.6);
        f.publish(1, 0, 0);
        TargetObservation2d retained = projected.get(f.time.clock()).observations().get(0);
        assertFalse(retained.hasFieldPosition());
        assertTrue(retained.hasPosition());
        assertEquals(PlanarPoseHistory.Lookup.UnavailableReason.EVICTED, retained.fieldLookup().unavailableReason());
        assertEquals(0.6, retained.ageSec(f.time.clock()), EPS);
    }

    @Test public void clockResetRetainsFailedLookupButCannotMakeOldObservationFresh() {
        Fixture f = new Fixture();
        f.publish(0, 0, 0);
        Source<TargetObservations2d> projected = f.projected(2, 3);
        projected.get(f.time.clock());
        f.time.clock().reset(0);
        TargetObservations2d frame = projected.get(f.time.clock());
        assertFalse(frame.isFresh(f.time.clock(), 100));
        TargetObservation2d point = frame.observations().get(0);
        assertFalse(point.hasFieldPosition());
        assertEquals(PlanarPoseHistory.Lookup.UnavailableReason.QUERY_TIMESTAMP_NOT_CURRENT,
                point.fieldLookup().unavailableReason());
    }

    @Test public void failedLookupsCanRetryAndNoProjectionResetPropagatesToBorrowedInputs() {
        Fixture f = new Fixture();
        f.publish(0, 0, 0);
        LoopTimestamp timestamp = f.time.clock().nowTimestamp();
        TargetObservations2d frame = TargetObservations2d.fromFrame(timestamp, Collections.emptyList());
        AtomicInteger resets = new AtomicInteger();
        AtomicInteger calls = new AtomicInteger();
        Source<TargetObservations2d> source = new Source<TargetObservations2d>() {
            @Override public TargetObservations2d get(LoopClock clock) { return frame; }
            @Override public void reset() { resets.incrementAndGet(); }
        };
        TimeAwareSource<PlanarPoseHistory.Lookup> history = new TimeAwareSource<PlanarPoseHistory.Lookup>() {
            @Override public PlanarPoseHistory.Lookup getAt(LoopClock clock, LoopTimestamp requested) {
                if (calls.incrementAndGet() == 1) throw new IllegalStateException("temporary lookup failure");
                return f.history.lookupSource().getAt(clock, requested);
            }
            @Override public void reset() { resets.incrementAndGet(); }
        };
        Source<TargetObservations2d> projected = ObservationSources.inField(source, history);
        try { projected.get(f.time.clock()); fail("first lookup must fail"); }
        catch (IllegalStateException expected) { }
        assertTrue(projected.get(f.time.clock()).isAvailable());
        projected.reset();
        assertEquals(0, resets.get());
    }

    @Test public void lookupMustRetainExactRequestedTimestampAndAnotherClockIsAnError() {
        Fixture f = new Fixture();
        f.publish(0, 0, 0);
        LoopTimestamp timestamp = f.time.clock().nowTimestamp();
        TargetObservations2d frame = TargetObservations2d.fromFrame(timestamp, Collections.emptyList());
        Source<TargetObservations2d> wrong = ObservationSources.inField(Source.constant(frame),
                (clock, requested) -> f.history.lookupSource().getAt(clock, clock.nowTimestamp()));
        try { wrong.get(f.time.clock()); fail("wrong requested timestamp"); }
        catch (IllegalArgumentException expected) { }
        Source<TargetObservations2d> projected = ObservationSources.inField(Source.constant(frame), f.history.lookupSource());
        try { projected.get(new ManualLoopClock().clock()); fail("wrong clock"); }
        catch (IllegalArgumentException expected) { }
    }

    @Test public void allTagProjectionKeepsIdOrientationAndUnknownConfidenceWithOneSourceGrammar() {
        ManualLoopClock time = new ManualLoopClock();
        LoopTimestamp stamp = time.clock().nowTimestamp();
        AprilTagDetections tags = AprilTagDetections.fromFrame(stamp, Collections.singletonList(
                AprilTagObservation.target(7, new Pose3d(10, 2, 0, 0.3, 0, 0))));
        TargetObservation2d point = ObservationSources.aprilTags(Source.constant(tags), CameraMountConfig.identity())
                .get(time.clock()).observations().get(0);
        assertEquals(7, point.targetId);
        assertEquals(0.3, point.targetHeadingRad, EPS);
        assertFalse(point.hasQuality());
        assertSame(stamp, point.timestamp);
    }

    /** Supplies real history with synthetic cached pose evidence, never an invented Lookup. */
    static final class Fixture {
        final ManualLoopClock time = new ManualLoopClock(0);
        final FakeTrajectory estimator = new FakeTrajectory();
        final PlanarPoseHistory history = new PlanarPoseHistory(estimator, PlanarPoseHistory.Config.defaults());

        void publish(double x, double y, double heading) {
            estimator.estimate = new PoseEstimate(new Pose3d(x, y, 0, heading, 0, 0), true,
                    0.8, time.clock().nowTimestamp());
            history.recordCurrent(time.clock());
        }

        Source<TargetObservations2d> projected(double x, double y) {
            LoopTimestamp timestamp = time.clock().nowTimestamp();
            TargetObservations2d frame = TargetObservations2d.fromFrame(timestamp,
                    Collections.singletonList(TargetObservation2d.ofRobotRelativePosition(x, y, Double.NaN, timestamp)));
            return ObservationSources.inField(Source.constant(frame), history.lookupSource());
        }
    }

    private static final class FakeTrajectory implements PoseTrajectoryEstimator {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        int updates;
        @Override public void update(LoopClock clock) { updates++; }
        @Override public PoseEstimate getEstimate() { return estimate; }
        @Override public long trajectorySegmentId() { return 0; }
    }
}
