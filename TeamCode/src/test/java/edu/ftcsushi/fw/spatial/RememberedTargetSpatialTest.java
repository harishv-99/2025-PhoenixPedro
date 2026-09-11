package edu.ftcsushi.fw.spatial;

import org.junit.Test;

import java.util.Collections;
import java.util.concurrent.atomic.AtomicInteger;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.guidance.DriveGuidance;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceQuery;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceSpec;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceStatus;
import edu.ftcsushi.fw.localization.PlanarPoseHistory;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.localization.PoseTrajectoryEstimator;
import edu.ftcsushi.fw.sensing.observation.FieldTargetMemory;
import edu.ftcsushi.fw.sensing.observation.FieldTargetSelectionPolicies;
import edu.ftcsushi.fw.sensing.observation.FieldTargetSelectionResult;
import edu.ftcsushi.fw.sensing.observation.FieldTargetSelectionSource;
import edu.ftcsushi.fw.sensing.observation.ObservationSources;
import edu.ftcsushi.fw.sensing.observation.TargetObservation2d;
import edu.ftcsushi.fw.sensing.observation.TargetObservations2d;
import edu.ftcsushi.fw.sensing.observation.TargetSelections;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Real memory, history, selection and solving; only camera/pose publications are authored. */
public final class RememberedTargetSpatialTest {
    @Test public void hiddenLocationUsesCurrentRobotPoseAndAppliesToolOffsetOnce() {
        Fixture f = new Fixture(1, 4);
        f.capture(20, 5);
        FieldTargetSelectionResult chosen = f.selected.get(f.time.clock());
        LoopTimestamp sighting = chosen.entry().lastSighting().timestamp;
        ReferencePoint2d point = References.selectedFieldTargetPoint(f.selected);
        SpatialQuery query = SpatialQuery.builder().translateTo(SpatialTargets.point(point))
                .andFaceTo(SpatialTargets.point(point))
                .controlFrames(SpatialControlFrames.robotCenter()
                        .withTranslationFrame(new Pose2d(2, 1, 0))
                        .withFacingFrame(new Pose2d(2, 1, 0)))
                .solveWith(SpatialSolveSet.builder().absolutePose(f.pose).build()).build();

        f.time.nextCycle(0.1);
        f.publishPose(4, 0, Math.PI / 2);
        f.raw = TargetObservations2d.unavailable("camera blocked");
        f.memory.update(f.time.clock());
        SpatialLaneResult result = query.get(f.time.clock()).laneResult(0);
        assertEquals(5, result.translation.robotForwardInches(), 1e-9);
        assertEquals(-16, result.translation.robotLeftInches(), 1e-9);
        assertEquals(3, result.translation.frameForwardInches(), 1e-9);
        assertEquals(-17, result.translation.frameLeftInches(), 1e-9);
        assertEquals(Math.atan2(-17, 3), result.facing.facingErrorRad, 1e-9);
        assertSame(sighting, result.translation.timestamp);
        assertSame(sighting, result.facing.targetObservationTimestamp);
        assertSame(f.pose.estimate.timestamp, result.translation.robotPoseTimestamp);
        assertTrue(result.translation.liveTarget); // An age constraint, not a visibility claim.
        assertEquals(ReferenceSelectionResult.Kind.REMEMBERED_TARGET, result.facingSelection.kind());
        assertSame(chosen.entry(), result.facingSelection.rememberedTarget().entry());
        assertTrue(Double.isNaN(result.facingSelection.rememberedTarget().entry().lastSighting().quality));
    }

    @Test public void referenceRejectsExpiredSightingDespiteFreshLocalization() {
        Fixture f = new Fixture(0.2, 4);
        f.capture(20, 5);
        FieldTargetSelectionResult retained = f.selected.get(f.time.clock());
        SpatialQuery query = f.query(Source.constant(retained));
        assertNotNull(query.get(f.time.clock()).laneResult(0).translation);
        f.time.nextCycle(0.21);
        f.publishPose(1, 0, 0);
        SpatialLaneResult result = query.get(f.time.clock()).laneResult(0);
        assertNull(result.translation);
        assertNull(result.facing);
        assertSame(retained, result.translationSelection.rememberedTarget());
        assertTrue(result.translationSelection.hasSelection());
        assertFalse(retained.isUsable(f.time.clock()));
    }

    @Test public void olderSolvingPoseRemainsTheLimitingEvidenceTime() {
        Fixture f = new Fixture(1, 4);
        PoseEstimate olderPose = f.pose.estimate;
        f.time.nextCycle(0.1);
        f.publishPose(0, 0, 0);
        f.capture(20, 5);
        FieldTargetSelectionResult retained = f.selected.get(f.time.clock());
        f.pose.estimate = olderPose; // Solving and capture-time projection retain separate evidence.
        SpatialLaneResult result = f.query(Source.constant(retained)).get(f.time.clock()).laneResult(0);
        assertSame(olderPose.timestamp, result.translation.timestamp);
        assertSame(olderPose.timestamp, result.facing.timestamp);
        assertSame(retained.entry().lastSighting().timestamp, result.translation.targetObservationTimestamp);
        assertSame(olderPose.timestamp, result.translation.robotPoseTimestamp);
    }

    @Test public void sameCycleResetRevokesConstantSelectionButDoesNotRewritePublishedQuery() {
        Fixture f = new Fixture(1, 4);
        f.capture(20, 5);
        FieldTargetSelectionResult retained = f.selected.get(f.time.clock());
        SpatialQuery query = f.query(Source.constant(retained));
        SpatialQueryResult published = query.get(f.time.clock());
        assertNotNull(published.laneResult(0).translation);
        f.memory.reset(f.time.clock());
        assertFalse(retained.isUsable(f.time.clock()));
        assertSame(published, query.get(f.time.clock()));
        query.reset(); // The owner resets its own cached consumer during a mid-cycle transition.
        assertNull(query.get(f.time.clock()).laneResult(0).translation);
        assertNotNull(published.laneResult(0).translation); // Immutable historical publication.
    }

    @Test public void evictionAndStopInvalidateRetainedConstantReferences() {
        Fixture f = new Fixture(1, 1);
        f.capture(20, 5);
        FieldTargetSelectionResult old = f.selected.get(f.time.clock());
        f.time.nextCycle(0.05);
        f.publishPose(0, 0, 0);
        f.capture(40, 5);
        assertFalse(old.isUsable(f.time.clock()));
        assertNull(f.query(Source.constant(old)).get(f.time.clock()).laneResult(0).facing);
        FieldTargetSelectionResult current = f.selected.get(f.time.clock());
        assertTrue(current.isUsable(f.time.clock()));
        f.memory.stop();
        assertFalse(current.isUsable(f.time.clock()));
        assertNull(f.query(Source.constant(current)).get(f.time.clock()).laneResult(0).facing);
    }

    @Test public void failureSuspendsRetainedEvidenceUntilMemoryRecovers() {
        Fixture f = new Fixture(1, 4);
        f.capture(20, 5);
        FieldTargetSelectionResult chosen = f.selected.get(f.time.clock());
        f.time.nextCycle(0.05);
        f.publishPose(0, 0, 0);
        f.inputFailure = new IllegalStateException("camera boundary failed");
        assertThrows(IllegalStateException.class, () -> f.memory.update(f.time.clock()));
        assertFalse(chosen.isUsable(f.time.clock()));
        assertNull(f.query(Source.constant(chosen)).get(f.time.clock()).laneResult(0).translation);
        f.time.nextCycle(0.05);
        f.publishPose(0, 0, 0);
        f.inputFailure = null;
        f.raw = TargetObservations2d.unavailable("no new camera frame");
        f.memory.update(f.time.clock());
        assertTrue(chosen.isUsable(f.time.clock()));
        assertNotNull(f.query(Source.constant(chosen)).get(f.time.clock()).laneResult(0).translation);
    }

    @Test public void rememberedEvidenceNeverSolvesAsDirectCameraFeedback() {
        Fixture f = new Fixture(1, 4);
        f.capture(20, 5);
        ReferencePoint2d point = References.selectedFieldTargetPoint(f.selected);
        SpatialQuery query = SpatialQuery.builder().translateTo(SpatialTargets.point(point))
                .andFaceTo(SpatialTargets.point(point))
                .solveWith(SpatialSolveSet.builder().observedPoints()
                        .relativeAprilTags(clock -> AprilTagDetections.fromFrame(
                                f.time.clock().nowTimestamp(), Collections.emptyList()),
                                CameraMountConfig.identity()).build()).build();
        for (int index = 0; index < 2; index++) {
            SpatialLaneResult result = query.get(f.time.clock()).laneResult(index);
            assertNull(result.translation);
            assertNull(result.facing);
            assertEquals(ReferenceSelectionResult.Kind.REMEMBERED_TARGET, result.translationSelection.kind());
        }
        IllegalStateException observedError = assertThrows(IllegalStateException.class,
                () -> DriveGuidance.plan().faceTo().point(point).solveWith()
                        .observedPoints(DriveGuidanceSpec.LossPolicy.PASS_THROUGH).build());
        assertTrue(observedError.getMessage().contains("absolutePose"));
        IllegalStateException tagError = assertThrows(IllegalStateException.class,
                () -> DriveGuidance.plan().faceTo().point(point).solveWith()
                        .relativeAprilTags(clock -> AprilTagDetections.none(), CameraMountConfig.identity())
                        .doneRelativeAprilTags().build());
        assertTrue(tagError.getMessage().contains("absolutePose"));
    }

    @Test public void guidanceRetainsMemoryPayloadEvenWhenLocalizationIsUnavailable() {
        Fixture f = new Fixture(1, 4);
        f.capture(20, 5);
        FieldTargetSelectionResult retained = f.selected.get(f.time.clock());
        DriveGuidanceQuery query = DriveGuidance.plan().faceTo()
                .point(References.selectedFieldTargetPoint(Source.constant(retained)))
                .solveWith().absolutePose(f.pose).doneAbsolutePose().build().query();
        DriveGuidanceStatus first = query.get(f.time.clock());
        assertTrue(first.hasOmegaError);
        assertSame(retained, first.facingSelection.rememberedTarget());
        f.time.nextCycle(0.05);
        f.pose.estimate = null;
        DriveGuidanceStatus missing = query.get(f.time.clock());
        assertFalse(missing.hasOmegaError);
        assertSame(retained, missing.facingSelection.rememberedTarget());
    }

    @Test public void referenceSamplingIsOncePerCycleRetryableAndNeverResetsBorrowedOwner() {
        Fixture f = new Fixture(1, 4);
        f.capture(20, 5);
        FieldTargetSelectionResult chosen = f.selected.get(f.time.clock());
        AtomicInteger reads = new AtomicInteger();
        Source<FieldTargetSelectionResult> borrowed = new Source<FieldTargetSelectionResult>() {
            @Override public FieldTargetSelectionResult get(LoopClock clock) {
                if (reads.incrementAndGet() == 1) throw new IllegalStateException("first read failed");
                return chosen;
            }
            @Override public void reset() { throw new AssertionError("borrowed reset"); }
        };
        SpatialQuery query = f.query(borrowed);
        assertEquals(0, reads.get());
        assertThrows(IllegalStateException.class, () -> query.get(f.time.clock()));
        SpatialQueryResult result = query.get(f.time.clock());
        assertSame(result, query.get(f.time.clock()));
        assertEquals(2, reads.get());
        query.reset();
        assertNotNull(query.get(f.time.clock()).laneResult(0).translation);
        assertEquals(2, reads.get());
        ReferenceSelectionResult provenance = result.laneResult(0).translationSelection;
        assertThrows(IllegalStateException.class, provenance::observedTarget);
        assertThrows(IllegalStateException.class, provenance::aprilTag);
        assertThrows(IllegalStateException.class, provenance::approach);
    }

    private static final class Fixture {
        final ManualLoopClock time = new ManualLoopClock(0);
        final Trajectory pose = new Trajectory();
        final PlanarPoseHistory history = new PlanarPoseHistory(pose, PlanarPoseHistory.Config.defaults());
        TargetObservations2d raw = TargetObservations2d.unavailable("not captured");
        RuntimeException inputFailure;
        final FieldTargetMemory memory;
        final FieldTargetSelectionSource selected;

        Fixture(double retention, int capacity) {
            publishPose(0, 0, 0);
            Source<TargetObservations2d> frames = Source.of(clock -> {
                if (inputFailure != null) throw inputFailure;
                return raw;
            });
            memory = FieldTargetMemory.fromFieldObjects(ObservationSources.inField(frames, history.lookupSource()))
                    .retainingForSec(retention).matchingWithinInches(1).maxEntries(capacity);
            selected = TargetSelections.fromRecentFieldLocations(memory.source())
                    .choose(FieldTargetSelectionPolicies.nearFieldPoint(0, 0, 100));
        }

        void publishPose(double x, double y, double yaw) {
            pose.estimate = new PoseEstimate(new Pose3d(x, y, 0, yaw, 0, 0), true, 1,
                    time.clock().nowTimestamp());
            history.recordCurrent(time.clock());
        }

        void capture(double forward, double left) {
            LoopTimestamp timestamp = time.clock().nowTimestamp();
            raw = TargetObservations2d.fromFrame(timestamp, Collections.singletonList(
                    TargetObservation2d.ofRobotRelativePosition(forward, left, Double.NaN, timestamp)));
            memory.update(time.clock());
        }

        SpatialQuery query(Source<FieldTargetSelectionResult> selection) {
            ReferencePoint2d point = References.selectedFieldTargetPoint(selection);
            return SpatialQuery.builder().translateTo(SpatialTargets.point(point))
                    .andFaceTo(SpatialTargets.point(point))
                    .solveWith(SpatialSolveSet.builder().absolutePose(pose).build()).build();
        }
    }

    private static final class Trajectory implements PoseTrajectoryEstimator {
        PoseEstimate estimate;
        @Override public void update(LoopClock clock) { throw new AssertionError("borrowed update"); }
        @Override public PoseEstimate getEstimate() { return estimate; }
        @Override public long trajectorySegmentId() { return 0; }
    }
}
