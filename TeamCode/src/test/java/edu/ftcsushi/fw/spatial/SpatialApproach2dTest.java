package edu.ftcsushi.fw.spatial;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Collections;
import java.util.concurrent.atomic.AtomicInteger;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.DriveOverlayMask;
import edu.ftcsushi.fw.drive.guidance.DriveGuidance;
import edu.ftcsushi.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceQuery;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceSpec;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceStatus;
import edu.ftcsushi.fw.field.SimpleTagLayout;
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
import edu.ftcsushi.fw.sensing.observation.TargetSelectionResult;
import edu.ftcsushi.fw.sensing.observation.TargetSelections;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelectionPolicies;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelectionSource;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelections;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Shared real geometry/query/controller contracts with authored camera and robot-pose evidence. */
public final class SpatialApproach2dTest {
    private static final double EPS = 1e-9;

    @Test public void yawedOffsetToolExpandsOnceAndSeparatesFacingFromTranslation() {
        Fixture f = new Fixture(18, 4);
        Pose2d tool = new Pose2d(6, 1, Math.PI / 2);
        SpatialApproach2d approach = SpatialApproach2d.facePoint(f.point, tool, 2);
        assertSame(f.point, approach.point());
        assertSame(tool, approach.robotToToolFrame());
        assertEquals(2, approach.standOffInches(), 0);
        assertPose(approach.robotToStandOffFrame(), 6, 3, Math.PI / 2);

        SpatialLaneResult lane = f.directQuery(approach).get(f.time.clock()).laneResult(0);
        assertPose(lane.translation.robotToTargetPoint, 18, 4, 0);
        assertEquals(1, lane.translation.frameForwardInches(), EPS);
        assertEquals(-12, lane.translation.frameLeftInches(), EPS);
        assertEquals(Math.atan2(-12, 3), lane.facing.facingErrorRad, EPS);
        assertSame(f.selected, lane.translationSelection.observedTarget());
        assertSame(f.selected, lane.facingSelection.observedTarget());
        assertSame(f.capture, lane.translation.timestamp);
        assertFalse(lane.translation.robotPoseTimestamp.isAvailable());
        assertTrue(Double.isNaN(lane.translation.quality));
    }

    @Test public void centeredOffsetToolDoesNotConfuseStandOffWithRadialRange() {
        Fixture f = new Fixture(20, 3);
        Pose2d tool = new Pose2d(6, 1, 0);
        SpatialApproach2d approach = SpatialApproach2d.facePoint(f.point, tool, 2);
        SpatialLaneResult lane = f.directQuery(approach).get(f.time.clock()).laneResult(0);
        assertEquals(12, lane.translation.frameForwardInches(), EPS);
        assertEquals(2, lane.translation.frameLeftInches(), EPS);
        assertEquals(Math.atan2(2, 14), lane.facing.facingErrorRad, EPS);
        SpatialQuery range = SpatialQuery.builder().translateTo(SpatialTargets.point(f.point))
                .controlFrames(SpatialControlFrames.robotCenter().withTranslationFrame(tool))
                .solveWith(observedLanes()).build();
        assertEquals(Math.hypot(14, 2), range.get(f.time.clock()).laneResult(0)
                .translation.frameDistanceInches(), EPS);
        assertEquals(Math.hypot(12, 2), lane.translation.frameDistanceInches(), EPS);
    }

    @Test public void completionLeavesPositiveStandOffAndZeroErrorsWithYawedTool() {
        Fixture f = new Fixture(6, 3);
        SpatialApproach2d approach = SpatialApproach2d.facePoint(f.point,
                new Pose2d(6, 1, Math.PI / 2), 2);
        SpatialLaneResult lane = f.directQuery(approach).get(f.time.clock()).laneResult(0);
        assertEquals(0, lane.translation.frameDistanceInches(), EPS);
        assertEquals(0, lane.facing.facingErrorRad, EPS);
        DriveGuidanceStatus status = directPlan(approach).query().get(f.time.clock());
        assertTrue(status.hasTranslationError);
        assertTrue(status.hasOmegaError);
        assertEquals(0, status.signal.axial, EPS);
        assertEquals(0, status.signal.lateral, EPS);
        assertEquals(0, status.signal.omega, EPS);
    }

    @Test public void runtimeSpecAndBothGuidanceLayersShareTheExactCentralFrameExpansion() {
        Fixture f = new Fixture(18, 4);
        SpatialApproach2d approach = SpatialApproach2d.facePoint(f.point, new Pose2d(6, 1, 0.3), 2);
        SpatialQuery query = f.directQuery(approach);
        SpatialQuerySpec spec = SpatialQuerySpec.builder().approach(approach)
                .solveWith(observedLanes()).build();
        DriveGuidancePlan plan = directPlan(approach);
        DriveGuidanceSpec guidanceSpec = DriveGuidance.spec().approach(approach)
                .solveWith().observedPoints(DriveGuidanceSpec.LossPolicy.ZERO_OUTPUT).build();
        assertSame(query.spec().controlFrames, spec.controlFrames);
        assertSame(spec.controlFrames, plan.spec.controlFrames);
        assertSame(spec.controlFrames, guidanceSpec.controlFrames);
        assertSame(plan.spec.spatialQuerySpec.translationTarget, plan.spec.translationTarget);
        assertSame(plan.spec.spatialQuerySpec.facingTarget, plan.spec.facingTarget);
        assertSame(spec.translationTarget, spec.facingTarget);
        assertEquals(DriveOverlayMask.ALL, plan.requestedMask());
        SpatialQuery independent = SpatialQuery.from(spec);
        assertNotSame(query, independent);
        assertEquals(query.get(f.time.clock()).laneResult(0).translation.frameForwardInches(),
                independent.get(f.time.clock()).laneResult(0).translation.frameForwardInches(), EPS);
        DriveGuidanceStatus first = plan.query().get(f.time.clock());
        DriveGuidanceStatus second = DriveGuidance.plan(guidanceSpec)
                .driveTuning().use(plan.tuning).doneDriveTuning().build().query().get(f.time.clock());
        assertEquals(first.signal.axial, second.signal.axial, EPS);
        assertEquals(first.signal.lateral, second.signal.lateral, EPS);
        assertEquals(first.signal.omega, second.signal.omega, EPS);
        assertEquals(DriveGuidanceSpec.SolveMode.OBSERVED_POINTS, second.solveMode);
    }

    @Test public void selectedTagApproachRetainsIdentityAndTheSolvingCamerasEvidence() {
        ManualLoopClock time = new ManualLoopClock();
        LoopTimestamp capture = time.clock().nowTimestamp();
        AprilTagDetections frame = AprilTagDetections.fromFrame(capture, Collections.singletonList(
                AprilTagObservation.target(5, new Pose3d(18, 4, 0, 0, 0, 0))));
        AprilTagSensor tags = clock -> frame;
        TagSelectionSource selection = TagSelections.fromVisibleTags(tags, CameraMountConfig.identity())
                .among(Collections.singleton(5)).freshWithinSec(0.2)
                .choose(TagSelectionPolicies.closestRange()).holdUntilReset();
        ReferencePoint2d point = References.relativeToSelectedTagPoint(selection, 0, 0);
        SpatialApproach2d approach = SpatialApproach2d.facePoint(point, new Pose2d(6, 1, 0), 2);
        DriveGuidanceQuery query = DriveGuidance.plan().approach(approach).solveWith()
                .relativeAprilTags(tags, CameraMountConfig.identity()).maxAgeSec(0.2)
                .doneRelativeAprilTags().build().query();
        DriveGuidanceStatus status = query.get(time.clock());
        TagSelectionResult selected = selection.get(time.clock());
        assertSame(selected, status.translationSelection.aprilTag());
        assertSame(selected, status.facingSelection.aprilTag());
        assertEquals(DriveGuidanceSpec.SolveMode.RELATIVE_APRIL_TAGS, status.solveMode);
        assertTrue(status.hasTranslationError);
        SpatialLaneResult lane = SpatialQuery.builder().approach(approach).solveWith(
                SpatialSolveSet.builder().relativeAprilTags(tags, CameraMountConfig.identity(), 0.2)
                        .build()).build().get(time.clock()).laneResult(0);
        assertSame(capture, lane.translation.timestamp);
        assertEquals(18, lane.translation.robotForwardInches(), EPS);
        // Tag is 18 inches forward; the stand-off origin is 6 + 2 inches forward.
        assertEquals(10, lane.translation.frameForwardInches(), EPS);
        time.nextCycle(0.21);
        assertFalse(query.get(time.clock()).hasTranslationError);
        assertTrue(query.get(time.clock()).translationSelection.hasSelection());
        AprilTagSensor otherCamera = clock -> AprilTagDetections.fromFrame(clock.nowTimestamp(),
                Collections.emptyList());
        DriveGuidanceStatus other = DriveGuidance.plan().approach(approach).solveWith()
                .relativeAprilTags(otherCamera, CameraMountConfig.identity())
                .doneRelativeAprilTags().build().query().get(time.clock());
        assertFalse(other.hasTranslationError);
        assertSame(selection.get(time.clock()), other.translationSelection.aprilTag());
    }

    @Test public void fieldLayoutStageSnapshotsFactsAndKeepsAuthoredHeadingSeparate() {
        ManualLoopClock time = new ManualLoopClock();
        Trajectory pose = new Trajectory();
        pose.publish(time.clock(), 0, 0, 0);
        SimpleTagLayout layout = new SimpleTagLayout().addPose(5, new Pose3d(20, 3, 0, 0, 0, 0));
        SpatialApproach2d approach = SpatialApproach2d.facePoint(References.relativeToTagPoint(5, 0, 0),
                new Pose2d(6, 1, 0), 2);
        SpatialQuerySpec spec = SpatialQuerySpec.builder().approach(approach)
                .fixedAprilTagLayout(layout).solveWith(SpatialSolveSet.builder().absolutePose(pose).build())
                .build();
        SpatialQuery query = SpatialQuery.builder().approach(approach).fixedAprilTagLayout(layout)
                .solveWith(SpatialSolveSet.builder().absolutePose(pose).build()).build();
        DriveGuidancePlan plan = DriveGuidance.plan().approach(approach).solveWith().absolutePose(pose)
                .fixedAprilTagLayout(layout).doneAbsolutePose().build();
        layout.clear();
        assertEquals(12, query.get(time.clock()).laneResult(0).translation.frameForwardInches(), EPS);
        assertEquals(12, SpatialQuery.from(spec).get(time.clock()).laneResult(0)
                .translation.frameForwardInches(), EPS);
        assertTrue(plan.query().get(time.clock()).hasTranslationError);
        ReferenceFrame2d frame = References.relativeToTagFrame(5, 0, 0, 0.7);
        SpatialQuery authored = SpatialQuery.builder().translateTo(SpatialTargets.point(References.framePoint(frame)))
                .andFaceTo(SpatialTargets.frameHeading(frame)).fixedAprilTagLayout(spec.fixedAprilTagLayout)
                .solveWith(spec.solveSet).build();
        assertEquals(0.7, authored.get(time.clock()).laneResult(0).facing.facingErrorRad, EPS);
        assertEquals(Math.atan2(2, 14), query.get(time.clock()).laneResult(0).facing.facingErrorRad, EPS);
    }

    @Test public void rememberedApproachRequiresPoseAndRetainsOriginalSightingAndOwnerLifetime() {
        ManualLoopClock time = new ManualLoopClock();
        Trajectory pose = new Trajectory();
        pose.publish(time.clock(), 0, 0, 0);
        PlanarPoseHistory history = new PlanarPoseHistory(pose, PlanarPoseHistory.Config.defaults());
        history.recordCurrent(time.clock());
        LoopTimestamp capture = time.clock().nowTimestamp();
        TargetObservations2d frame = TargetObservations2d.fromFrame(capture, Collections.singletonList(
                TargetObservation2d.ofRobotRelativePosition(20, 3, Double.NaN, capture)));
        FieldTargetMemory memory = FieldTargetMemory.fromFieldObjects(
                ObservationSources.inField(Source.constant(frame), history.lookupSource()))
                .retainingForSec(1).matchingWithinInches(1).maxEntries(4);
        memory.update(time.clock());
        FieldTargetSelectionSource selected = TargetSelections.fromRecentFieldLocations(memory.source())
                .choose(FieldTargetSelectionPolicies.nearFieldPoint(20, 3, 1));
        FieldTargetSelectionResult choice = selected.get(time.clock());
        SpatialApproach2d approach = SpatialApproach2d.facePoint(
                References.selectedFieldTargetPoint(Source.constant(choice)), new Pose2d(6, 1, 0), 2);
        time.nextCycle(0.1);
        pose.publish(time.clock(), 2, 0, 0);
        SpatialQuery query = SpatialQuery.builder().approach(approach)
                .solveWith(SpatialSolveSet.builder().absolutePose(pose).build()).build();
        SpatialLaneResult lane = query.get(time.clock()).laneResult(0);
        assertEquals(10, lane.translation.frameForwardInches(), EPS);
        assertSame(capture, lane.translation.timestamp);
        assertSame(capture, lane.facing.targetObservationTimestamp);
        assertSame(pose.estimate.timestamp, lane.translation.robotPoseTimestamp);
        assertSame(choice, lane.translationSelection.rememberedTarget());
        DriveGuidanceQuery guidance = DriveGuidance.plan().approach(approach).solveWith()
                .absolutePose(pose).doneAbsolutePose().build().query();
        assertSame(choice, guidance.get(time.clock()).facingSelection.rememberedTarget());
        assertThrows(IllegalStateException.class, () -> directPlan(approach));
        SpatialLaneResult noPose = SpatialQuery.builder().approach(approach)
                .solveWith(observedLanes()).build().get(time.clock()).laneResult(0);
        assertFalse(noPose.valid());
        assertSame(choice, noPose.translationSelection.rememberedTarget());
        memory.reset(time.clock());
        query.reset();
        guidance.reset();
        assertNull(query.get(time.clock()).laneResult(0).translation);
        assertFalse(guidance.get(time.clock()).hasTranslationError);
    }

    @Test public void staleObservedApproachDoesNotBecomeUsableFromAFreshRobotPose() {
        ManualLoopClock time = new ManualLoopClock();
        Trajectory pose = new Trajectory();
        pose.publish(time.clock(), 0, 0, 0);
        PlanarPoseHistory history = new PlanarPoseHistory(pose, PlanarPoseHistory.Config.defaults());
        history.recordCurrent(time.clock());
        LoopTimestamp capture = time.clock().nowTimestamp();
        TargetObservation2d observed = TargetObservation2d.ofRobotRelativePosition(20, 3, 0.5, capture)
                .withFieldPoseLookup(history.lookupSource().getAt(time.clock(), capture));
        TargetSelectionResult selection = selected(observed);
        SpatialApproach2d approach = SpatialApproach2d.facePoint(
                References.selectedTargetPoint(Source.constant(selection)), Pose2d.zero(), 2);
        DriveGuidanceQuery query = DriveGuidance.plan().approach(approach).solveWith()
                .absolutePose(pose).doneAbsolutePose().build().query();
        assertTrue(query.get(time.clock()).hasTranslationError);
        time.nextCycle(0.21);
        pose.publish(time.clock(), 0, 0, 0);
        DriveGuidanceStatus stale = query.get(time.clock());
        assertFalse(stale.hasTranslationError);
        assertFalse(stale.hasOmegaError);
        assertSame(selection, stale.translationSelection.observedTarget());
    }

    @Test public void constructionDoesNotPollAndFailedValueReadsRetryWithoutResettingBorrowedSource() {
        Fixture f = new Fixture(20, 3);
        AtomicInteger reads = new AtomicInteger();
        Source<TargetSelectionResult> borrowed = new Source<TargetSelectionResult>() {
            @Override public TargetSelectionResult get(LoopClock clock) {
                if (reads.incrementAndGet() == 1) throw new IllegalStateException("transient read");
                return f.selected;
            }
            @Override public void reset() { throw new AssertionError("borrowed reset"); }
        };
        SpatialApproach2d approach = SpatialApproach2d.facePoint(
                References.selectedTargetPoint(borrowed), Pose2d.zero(), 2);
        SpatialQuery query = f.directQuery(approach);
        DriveGuidanceQuery guidance = directPlan(approach).query();
        assertEquals(0, reads.get());
        assertThrows(IllegalStateException.class, () -> query.get(f.time.clock()));
        SpatialQueryResult result = query.get(f.time.clock());
        assertSame(result, query.get(f.time.clock()));
        guidance.get(f.time.clock());
        query.reset();
        guidance.reset();
        query.get(f.time.clock());
        assertEquals(2, reads.get());
    }

    @Test public void invalidAndOverflowedAuthoredGeometryFailBeforeAnySourceRead() {
        ReferencePoint2d point = References.fieldPoint(1, 2);
        assertThrows(NullPointerException.class, () -> SpatialApproach2d.facePoint(null, Pose2d.zero(), 1));
        assertThrows(NullPointerException.class, () -> SpatialApproach2d.facePoint(point, null, 1));
        for (double value : new double[]{0, -0.0, -1, Double.NaN, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY}) {
            assertThrows(IllegalArgumentException.class, () -> SpatialApproach2d.facePoint(point, Pose2d.zero(), value));
        }
        for (double value : new double[]{Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
            assertThrows(IllegalArgumentException.class, () -> SpatialApproach2d.facePoint(point, new Pose2d(value, 0, 0), 1));
            assertThrows(IllegalArgumentException.class, () -> SpatialApproach2d.facePoint(point, new Pose2d(0, value, 0), 1));
            assertThrows(IllegalArgumentException.class, () -> SpatialApproach2d.facePoint(point, new Pose2d(0, 0, value), 1));
        }
        assertThrows(IllegalArgumentException.class, () -> SpatialApproach2d.facePoint(point,
                new Pose2d(Double.MAX_VALUE, 0, 0), Double.MAX_VALUE));
        assertThrows(IllegalArgumentException.class, () -> SpatialApproach2d.facePoint(point,
                new Pose2d(1e16, 0, 0), 1));
        SpatialApproach2d hugeHeading = SpatialApproach2d.facePoint(point,
                new Pose2d(0, 0, Double.MAX_VALUE), 1);
        assertTrue(Double.isFinite(hugeHeading.robotToStandOffFrame().xInches));
        assertTrue(Double.isFinite(hugeHeading.robotToStandOffFrame().yInches));
    }

    @Test public void queryAndSpecRejectRetainedStageGeometryMutationInBothOrders() {
        SpatialApproach2d approach = SpatialApproach2d.facePoint(References.fieldPoint(10, 0), Pose2d.zero(), 2);
        SpatialTargets.FieldPoint other = SpatialTargets.fieldPoint(8, 0);
        SpatialQuery.TargetChoice runtime = SpatialQuery.builder();
        runtime.approach(approach);
        assertThrows(IllegalStateException.class, () -> runtime.translateTo(other));
        assertThrows(IllegalStateException.class, () -> runtime.faceTo(other));
        assertThrows(IllegalStateException.class, () -> runtime.approach(approach));
        assertThrows(IllegalStateException.class, () -> ((SpatialQuery.BothTargetStage) runtime)
                .controlFrames(SpatialControlFrames.robotCenter()));
        SpatialQuery.TargetChoice already = SpatialQuery.builder();
        already.translateTo(other).controlFrames(SpatialControlFrames.robotCenter());
        assertThrows(IllegalStateException.class, () -> already.approach(approach));

        SpatialQuerySpec.TargetChoice spec = SpatialQuerySpec.builder();
        spec.approach(approach);
        assertThrows(IllegalStateException.class, () -> spec.translateTo(other));
        assertThrows(IllegalStateException.class, () -> spec.faceTo(other));
        assertThrows(IllegalStateException.class, () -> ((SpatialQuerySpec.TranslationTargetStage) spec).andFaceTo(other));
        assertThrows(IllegalStateException.class, () -> ((SpatialQuerySpec.FacingTargetStage) spec).andTranslateTo(other));
        assertThrows(IllegalStateException.class, () -> ((SpatialQuerySpec.BothTargetStage) spec)
                .controlFrames(SpatialControlFrames.robotCenter()));
        SpatialQuerySpec.TargetChoice oldSpec = SpatialQuerySpec.builder();
        oldSpec.faceTo(other);
        assertThrows(IllegalStateException.class, () -> oldSpec.approach(approach));
    }

    @Test public void guidanceRejectsEarlierAnswersAndRetainedUnansweredTargetBranches() {
        Fixture f = new Fixture(20, 3);
        SpatialApproach2d approach = SpatialApproach2d.facePoint(f.point, Pose2d.zero(), 2);
        DriveGuidance.PlanBuilder0 plan = DriveGuidance.plan();
        DriveGuidance.TranslateToBuilder<DriveGuidance.PlanBuilder1> delayedTranslation = plan.translateTo();
        DriveGuidance.FaceToBuilder<DriveGuidance.PlanBuilder2> delayedFacing = plan.faceTo();
        plan.approach(approach);
        assertThrows(IllegalStateException.class, plan::translateTo);
        assertThrows(IllegalStateException.class, plan::faceTo);
        assertThrows(IllegalStateException.class, () -> delayedTranslation.point(f.point));
        assertThrows(IllegalStateException.class, () -> delayedFacing.fieldHeadingRad(0));
        assertThrows(IllegalStateException.class, () -> plan.approach(approach));
        DriveGuidance.PlanBuilder0 previous = DriveGuidance.plan();
        previous.faceTo().point(f.point).controlFrames(SpatialControlFrames.robotCenter());
        assertThrows(IllegalStateException.class, () -> previous.approach(approach));

        DriveGuidance.SpecBuilder0 spec = DriveGuidance.spec();
        DriveGuidance.TranslateToBuilder<DriveGuidance.SpecBuilder1> delayed = spec.translateTo();
        spec.approach(approach);
        assertThrows(IllegalStateException.class, spec::translateTo);
        assertThrows(IllegalStateException.class, spec::faceTo);
        assertThrows(IllegalStateException.class, () -> delayed.robotRelativePointInches(1, 2));
        DriveGuidance.SpecBuilder0 prior = DriveGuidance.spec();
        prior.translateTo().point(f.point);
        assertThrows(IllegalStateException.class, () -> prior.approach(approach));
    }

    @Test public void approachStagesExposeNoCompetingGeometryAnswersOrConstructionAliases() {
        for (Class<?> type : new Class<?>[]{SpatialQuery.ApproachTargetStage.class,
                SpatialQuerySpec.ApproachTargetStage.class, DriveGuidance.PlanApproachStage.class,
                DriveGuidance.SpecApproachStage.class}) {
            for (Method method : type.getMethods()) {
                assertFalse(method.getName(), method.getName().equals("controlFrames"));
                assertFalse(method.getName(), method.getName().contains("FaceTo"));
                assertFalse(method.getName(), method.getName().contains("TranslateTo"));
            }
        }
        assertEquals(0, SpatialApproach2d.class.getConstructors().length);
        int factories = 0;
        for (Method method : SpatialApproach2d.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers()) && Modifier.isStatic(method.getModifiers())
                    && method.getReturnType() == SpatialApproach2d.class) {
                assertEquals("facePoint", method.getName());
                factories++;
            }
        }
        assertEquals(1, factories);
    }

    private static SpatialSolveSet observedLanes() { return SpatialSolveSet.builder().observedPoints().build(); }

    private static DriveGuidancePlan directPlan(SpatialApproach2d approach) {
        return DriveGuidance.plan().approach(approach).solveWith()
                .observedPoints(DriveGuidanceSpec.LossPolicy.ZERO_OUTPUT).build();
    }

    private static TargetSelectionResult selected(TargetObservation2d observation) {
        TargetObservations2d frame = TargetObservations2d.fromFrame(observation.timestamp,
                Collections.singletonList(observation));
        return TargetSelectionResult.selected(frame, observation, 0.2, 0, "authored test choice");
    }

    private static void assertPose(Pose2d actual, double x, double y, double heading) {
        assertEquals(x, actual.xInches, EPS);
        assertEquals(y, actual.yInches, EPS);
        assertEquals(heading, actual.headingRad, EPS);
    }

    private static final class Fixture {
        final ManualLoopClock time = new ManualLoopClock();
        final LoopTimestamp capture = time.clock().nowTimestamp();
        final TargetSelectionResult selected;
        final ReferencePoint2d point;
        Fixture(double forward, double left) {
            selected = selected(TargetObservation2d.ofRobotRelativePosition(forward, left, Double.NaN, capture));
            point = References.selectedTargetPoint(Source.constant(selected));
        }
        SpatialQuery directQuery(SpatialApproach2d approach) {
            return SpatialQuery.builder().approach(approach).solveWith(observedLanes()).build();
        }
    }

    private static final class Trajectory implements PoseTrajectoryEstimator {
        PoseEstimate estimate;
        void publish(LoopClock clock, double x, double y, double yaw) {
            estimate = new PoseEstimate(new Pose3d(x, y, 0, yaw, 0, 0), true, 1, clock.nowTimestamp());
        }
        @Override public void update(LoopClock clock) { throw new AssertionError("borrowed pose update"); }
        @Override public PoseEstimate getEstimate() { return estimate; }
        @Override public long trajectorySegmentId() { return 0; }
    }
}
