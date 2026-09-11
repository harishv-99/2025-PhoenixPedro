package edu.ftcsushi.fw.spatial;

import org.junit.Test;

import java.util.Collections;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveOverlay;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.drive.guidance.DriveGuidance;
import edu.ftcsushi.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceQuery;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceSpec;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceStatus;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcsushi.fw.localization.PlanarPoseHistory;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.localization.PoseTrajectoryEstimator;
import edu.ftcsushi.fw.sensing.observation.TargetObservation2d;
import edu.ftcsushi.fw.sensing.observation.TargetObservations2d;
import edu.ftcsushi.fw.sensing.observation.TargetSelectionResult;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Exact provenance and expired evidence are exercised through the maintained spatial/drive graph. */
public final class ReferenceSelectionResultTest {
    @Test public void typedResultsAreRetainedExactlyIncludingUnsuccessfulChoices() {
        ManualLoopClock time = new ManualLoopClock();
        TargetSelectionResult selected = selection(time.clock());
        TagSelectionResult tag = TagSelectionResult.forTagId(7);
        ApproachResult2d unavailable = ApproachResult2d.unavailable("no admissible approach");

        ReferenceSelectionResult tagResult = ReferenceSelectionResult.aprilTag(tag);
        assertEquals(ReferenceSelectionResult.Kind.APRIL_TAG, tagResult.kind());
        assertTrue(tagResult.hasSelection());
        assertSame(tag, tagResult.aprilTag());
        assertFalse(tagResult.aprilTag().hasFreshSelectedObservation);

        ReferenceSelectionResult objectResult = ReferenceSelectionResult.observedTarget(selected);
        assertEquals(ReferenceSelectionResult.Kind.OBSERVED_TARGET, objectResult.kind());
        assertTrue(objectResult.hasSelection());
        assertSame(selected, objectResult.observedTarget());
        assertEquals(-1, objectResult.observedTarget().observation().targetId);

        TargetSelectionResult empty = TargetSelectionResult.none(selected.frame(), 0.2, "none qualify");
        ReferenceSelectionResult emptyResult = ReferenceSelectionResult.observedTarget(empty);
        assertFalse(emptyResult.hasSelection());
        assertSame(empty, emptyResult.observedTarget());
        assertEquals("none qualify", emptyResult.observedTarget().reason());

        ReferenceSelectionResult approachResult = ReferenceSelectionResult.approach(unavailable);
        assertEquals(ReferenceSelectionResult.Kind.APPROACH, approachResult.kind());
        assertFalse(approachResult.hasSelection());
        assertSame(unavailable, approachResult.approach());
        assertSame(ReferenceSelectionResult.none(), ReferenceSelectionResult.none());
        assertFalse(ReferenceSelectionResult.none().hasSelection());
    }

    @Test public void wrongKindAndNullPayloadsFailActionably() {
        ReferenceSelectionResult none = ReferenceSelectionResult.none();
        for (Runnable wrong : new Runnable[]{none::aprilTag, none::observedTarget, none::approach}) {
            try {
                wrong.run();
                fail("A wrong domain must not fabricate an empty typed result");
            } catch (IllegalStateException expected) {
                assertTrue(expected.getMessage().contains("kind()"));
                assertTrue(expected.getMessage().contains("NONE"));
            }
        }
        for (Runnable invalid : new Runnable[]{
                () -> ReferenceSelectionResult.aprilTag(null),
                () -> ReferenceSelectionResult.observedTarget(null),
                () -> ReferenceSelectionResult.approach(null)}) {
            try {
                invalid.run();
                fail("An absent payload needs an explicit result, not null");
            } catch (NullPointerException expected) {
                assertNotNull(expected.getMessage());
            }
        }
    }

    @Test public void commonMetadataTraversesQueriesSelectorsAndDriveWithOneSuccessfulSourceReadPerCycle() {
        ManualLoopClock time = new ManualLoopClock();
        TargetSelectionResult selected = selection(time.clock());
        int[] reads = {0};
        ReferencePoint2d reference = References.selectedTargetPoint(Source.of(clock -> {
            reads[0]++;
            return selected;
        }));
        SpatialQuery query = directQuery(reference);
        DriveGuidancePlan plan = directPlan(reference);
        DriveGuidanceQuery guidance = plan.query();
        DriveOverlay overlay = plan.overlay();
        RecordingSink sink = new RecordingSink();
        DriveGuidanceTask task = plan.task(sink, new DriveGuidanceTask.Config());
        assertEquals(0, reads[0]);

        SpatialQueryResult result = query.get(time.clock());
        SpatialLaneResult lane = result.laneResult(0);
        assertSame(selected, lane.translationSelection.observedTarget());
        assertSame(selected, lane.facingSelection.observedTarget());
        assertSame(selected, SpatialQuerySelectors.firstValidTranslation(result,
                SpatialSolutionGate.defaults()).selection.observedTarget());
        assertSame(selected, SpatialQuerySelectors.firstValidFacing(result,
                SpatialSolutionGate.defaults()).selection.observedTarget());
        assertSame(result, query.get(time.clock()));

        DriveGuidanceStatus status = guidance.get(time.clock());
        assertSame(selected, status.translationSelection.observedTarget());
        assertSame(selected, status.facingSelection.observedTarget());
        assertSame(status, guidance.get(time.clock()));
        overlay.onEnable(time.clock());
        DriveSignal overlaySignal = overlay.get(time.clock()).signal;
        task.start(time.clock());
        task.update(time.clock());
        assertEquals(status.signal.axial, overlaySignal.axial, 0);
        assertEquals(status.signal.omega, sink.signal.omega, 0);
        assertEquals(1, reads[0]);

        query.reset();
        guidance.reset();
        assertSame(selected, query.get(time.clock()).laneResult(0).translationSelection.observedTarget());
        assertSame(selected, guidance.get(time.clock()).translationSelection.observedTarget());
        assertEquals(1, reads[0]);
        task.cancel();
        time.nextCycle(0.01);
        query.get(time.clock());
        assertEquals(2, reads[0]);
    }

    @Test public void retainedStaleObjectRemainsInspectableWithoutGeometryOrRefreshedTime() {
        ManualLoopClock time = new ManualLoopClock();
        TargetSelectionResult selected = selection(time.clock());
        ReferencePoint2d reference = References.selectedTargetPoint(Source.constant(selected));
        SpatialQuery spatial = directQuery(reference);
        DriveGuidanceQuery guidance = directPlan(reference).query();
        assertTrue(spatial.get(time.clock()).laneResult(0).valid());
        time.nextCycle(0.21);

        SpatialLaneResult expired = spatial.get(time.clock()).laneResult(0);
        assertFalse(expired.valid());
        assertTrue(expired.translationSelection.hasSelection());
        assertSame(selected, expired.translationSelection.observedTarget());
        assertFalse(expired.translationSelection.observedTarget().isUsable(time.clock()));
        assertSame(selected.observation().timestamp,
                expired.translationSelection.observedTarget().observation().timestamp);
        DriveGuidanceStatus status = guidance.get(time.clock());
        assertFalse(status.hasTranslationError);
        assertFalse(status.hasOmegaError);
        assertSame(selected, status.facingSelection.observedTarget());

        time.clock().reset(0);
        assertFalse(spatial.get(time.clock()).laneResult(0).valid());
        assertSame(selected, spatial.get(time.clock()).laneResult(0).translationSelection.observedTarget());
    }

    @Test public void missingFieldProjectionRetainsSelectionButCannotUseRobotPoseAsTargetEvidence() {
        ManualLoopClock time = new ManualLoopClock();
        TargetSelectionResult selected = selection(time.clock());
        ReferencePoint2d point = References.selectedTargetPoint(Source.constant(selected));
        SpatialLaneResult result = SpatialQuery.builder().translateTo(SpatialTargets.point(point))
                .andFaceTo(SpatialTargets.point(point)).solveWith(SpatialSolveSet.builder()
                        .absolutePose(new Trajectory(time.clock())).build()).build()
                .get(time.clock()).laneResult(0);
        assertFalse(result.valid());
        assertSame(selected, result.translationSelection.observedTarget());
        assertSame(selected, result.facingSelection.observedTarget());
    }

    @Test public void committedApproachMetadataKeepsItsOriginalObservationAfterObservationExpiry() {
        ManualLoopClock time = new ManualLoopClock();
        Trajectory pose = new Trajectory(time.clock());
        PlanarPoseHistory history = new PlanarPoseHistory(pose, PlanarPoseHistory.Config.defaults());
        history.recordCurrent(time.clock());
        TargetObservation2d original = selection(time.clock()).observation();
        TargetObservation2d located = original.withFieldPoseLookup(
                history.lookupSource().getAt(time.clock(), original.timestamp));
        ApproachResult2d approach = ApproachResult2d.forTarget(located, Pose2d.zero(), 2, 0, 0.2)
                .committedFor(time.clock(), 1.0);
        ReferenceFrame2d reference = References.approachFrame(Source.constant(approach));
        SpatialQuery query = SpatialQuery.builder().translateTo(SpatialTargets.point(References.framePoint(reference)))
                .andFaceTo(SpatialTargets.frameHeading(reference)).solveWith(SpatialSolveSet.builder()
                        .absolutePose(pose).build()).build();

        time.nextCycle(0.3);
        pose.publish(time.clock());
        SpatialLaneResult result = query.get(time.clock()).laneResult(0);
        assertTrue(result.valid());
        assertSame(approach, result.translationSelection.approach());
        assertSame(approach, result.facingSelection.approach());
        assertTrue(result.translationSelection.approach().isCommitted());
        assertSame(original.timestamp, result.translation.targetObservationTimestamp);
        assertSame(pose.estimate.timestamp, result.translation.robotPoseTimestamp);
        assertFalse(result.translation.liveTarget);

        time.nextCycle(0.71);
        pose.publish(time.clock());
        SpatialLaneResult expired = query.get(time.clock()).laneResult(0);
        assertFalse(expired.valid());
        assertSame(approach, expired.translationSelection.approach());
        assertFalse(expired.translationSelection.approach().isUsable(time.clock()));
    }

    @Test public void selectedTargetPointIsTheOnlyPublicFrameSelectionReferenceFactory() throws Exception {
        assertEquals(ReferencePoint2d.class,
                References.class.getMethod("selectedTargetPoint", Source.class).getReturnType());
        try {
            References.class.getMethod("observedPoint", Source.class);
            fail("The old public spelling must not remain as a competing construction path");
        } catch (NoSuchMethodException expected) {
            // The selected-target spelling is the one ordinary construction path.
        }
    }

    private static TargetSelectionResult selection(LoopClock clock) {
        TargetObservation2d observation = TargetObservation2d.ofRobotRelativePosition(
                20, 3, Double.NaN, clock.nowTimestamp());
        TargetObservations2d frame = TargetObservations2d.fromFrame(observation.timestamp,
                Collections.singletonList(observation));
        return TargetSelectionResult.selected(frame, observation, 0.2, 0, "test selection");
    }

    private static SpatialQuery directQuery(ReferencePoint2d reference) {
        return SpatialQuery.builder().translateTo(SpatialTargets.point(reference))
                .andFaceTo(SpatialTargets.point(reference))
                .solveWith(SpatialSolveSet.builder().observedPoints().build()).build();
    }

    private static DriveGuidancePlan directPlan(ReferencePoint2d reference) {
        return DriveGuidance.plan().translateTo().point(reference).andFaceTo().point(reference)
                .solveWith().observedPoints(DriveGuidanceSpec.LossPolicy.ZERO_OUTPUT).build();
    }

    private static final class RecordingSink implements DriveCommandSink {
        DriveSignal signal = DriveSignal.zero();
        @Override public void drive(DriveSignal signal) { this.signal = signal; }
        @Override public void stop() { signal = DriveSignal.zero(); }
    }

    private static final class Trajectory implements PoseTrajectoryEstimator {
        PoseEstimate estimate;
        Trajectory(LoopClock clock) { publish(clock); }
        void publish(LoopClock clock) { estimate = new PoseEstimate(Pose3d.zero(), true, 1, clock.nowTimestamp()); }
        @Override public void update(LoopClock clock) { throw new AssertionError("Borrowed estimator must not update"); }
        @Override public PoseEstimate getEstimate() { return estimate; }
        @Override public long trajectorySegmentId() { return 1; }
    }
}
