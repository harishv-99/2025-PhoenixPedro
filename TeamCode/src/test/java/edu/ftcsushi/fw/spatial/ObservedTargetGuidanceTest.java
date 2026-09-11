package edu.ftcsushi.fw.spatial;

import java.util.Collections;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.guidance.DriveGuidance;
import edu.ftcsushi.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceQuery;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceSpec;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceStatus;
import edu.ftcsushi.fw.localization.PlanarPoseHistory;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.localization.PoseTrajectoryEstimator;
import edu.ftcsushi.fw.sensing.observation.TargetObservation2d;
import edu.ftcsushi.fw.sensing.observation.TargetObservations2d;
import edu.ftcsushi.fw.sensing.observation.TargetSelectionResult;
import edu.ftcsushi.fw.sensing.observation.TargetSelectionPolicies;
import edu.ftcsushi.fw.sensing.observation.TargetSelections;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Synthetic observations exercise the real reference/query/guidance path, not physical accuracy. */
public final class ObservedTargetGuidanceTest {
    @Test public void tagAndObjectPointUseIdenticalOffCenterGuidance() {
        ManualLoopClock time = new ManualLoopClock(2.0);
        LoopTimestamp capture = time.clock().nowTimestamp();
        TargetObservation2d ball = TargetObservation2d.ofRobotRelativePosition(18, 4, Double.NaN, capture);
        ReferencePoint2d point = References.selectedTargetPoint(selection(ball));
        SpatialControlFrames frames = SpatialControlFrames.robotCenter()
                .withFacingFrame(new Pose2d(6, 1, 0.2))
                .withTranslationFrame(new Pose2d(6, 1, 0.2));
        DriveGuidancePlan objectPlan = DriveGuidance.plan()
                .translateTo().point(point).andFaceTo().point(point).controlFrames(frames)
                .solveWith().observedPoints(DriveGuidanceSpec.LossPolicy.PASS_THROUGH).build();
        AprilTagDetections detections = AprilTagDetections.fromFrame(capture,
                Collections.singletonList(AprilTagObservation.target(5, new Pose3d(18, 4, 0, 0, 0, 0))));
        AprilTagSensor tags = clock -> detections;
        ReferencePoint2d tagPoint = References.relativeToTagPoint(5, 0, 0);
        DriveGuidancePlan tagPlan = DriveGuidance.plan()
                .translateTo().point(tagPoint).andFaceTo().point(tagPoint).controlFrames(frames)
                .solveWith().relativeAprilTags(tags, CameraMountConfig.identity()).doneRelativeAprilTags().build();
        DriveGuidanceStatus object = objectPlan.query().get(time.clock());
        DriveGuidanceStatus tag = tagPlan.query().get(time.clock());
        assertEquals(tag.signal.axial, object.signal.axial, 1e-12);
        assertEquals(tag.signal.lateral, object.signal.lateral, 1e-12);
        assertEquals(tag.signal.omega, object.signal.omega, 1e-12);
        assertEquals(DriveGuidanceSpec.SolveMode.OBSERVED_POINTS, object.solveMode);
        assertTrue(object.hasOmegaError);
    }

    @Test public void freshPoseCannotRefreshExpiredTarget() {
        ManualLoopClock time = new ManualLoopClock(0.0);
        Trajectory pose = new Trajectory();
        TargetObservation2d ball = fieldObservation(time.clock(), pose, 20, 3);
        ReferencePoint2d point = References.selectedTargetPoint(selection(ball));
        DriveGuidanceQuery query = DriveGuidance.plan().faceTo().point(point)
                .solveWith().absolutePose(pose).doneAbsolutePose().build().query();
        assertTrue(query.get(time.clock()).hasOmegaError);
        time.nextCycle(0.21);
        pose.publish(time.clock());
        assertFalse(query.get(time.clock()).hasOmegaError);
    }

    @Test public void fieldSolutionRetainsBothEvidenceTimes() {
        ManualLoopClock time = new ManualLoopClock(0.0);
        Trajectory pose = new Trajectory();
        TargetObservation2d ball = fieldObservation(time.clock(), pose, 20, 3);
        time.nextCycle(0.05);
        pose.publish(time.clock());
        SpatialQuery query = SpatialQuery.builder()
                .translateTo(SpatialTargets.point(References.selectedTargetPoint(selection(ball))))
                .solveWith(SpatialSolveSet.builder().absolutePose(pose).build()).build();
        SpatialLaneResult lane = query.get(time.clock()).laneResult(0);
        TranslationSolution solution = lane.translation;
        assertNotNull(solution);
        assertSame(ball, lane.translationSelection.observedTarget().observation());
        assertSame(ball.timestamp, solution.targetObservationTimestamp);
        assertSame(pose.estimate.timestamp, solution.robotPoseTimestamp);
        assertSame(ball.timestamp, solution.timestamp);
        assertTrue(solution.liveTarget);
    }

    @Test public void commitmentIsBoundedIntentNotRefreshedObservation() {
        ManualLoopClock time = new ManualLoopClock(0.0);
        Trajectory pose = new Trajectory();
        TargetObservation2d ball = fieldObservation(time.clock(), pose, 20, 3);
        ApproachResult2d goal = ApproachResult2d.forTarget(ball, new Pose2d(6, 1, 0), 2, 0, 0.2)
                .committedFor(time.clock(), 1.0);
        ReferenceFrame2d frame = References.approachFrame(Source.constant(goal));
        SpatialQuery query = SpatialQuery.builder().translateTo(SpatialTargets.point(References.framePoint(frame)))
                .andFaceTo(SpatialTargets.frameHeading(frame))
                .solveWith(SpatialSolveSet.builder().absolutePose(pose).build()).build();
        time.nextCycle(0.4);
        pose.publish(time.clock());
        TranslationSolution solution = query.get(time.clock()).laneResult(0).translation;
        assertNotNull(solution);
        assertEquals(12, solution.robotForwardInches(), 1e-12);
        assertEquals(2, solution.robotLeftInches(), 1e-12);
        assertSame(ball.timestamp, solution.targetObservationTimestamp);
        assertSame(pose.estimate.timestamp, solution.timestamp);
        assertFalse(solution.liveTarget);
        time.nextCycle(0.61);
        pose.publish(time.clock());
        assertNull(query.get(time.clock()).laneResult(0).translation);
    }

    @Test public void offsetIsRotatedOnceAndResetInvalidatesCommitment() {
        ManualLoopClock time = new ManualLoopClock(0.0);
        TargetObservation2d ball = fieldObservation(time.clock(), new Trajectory(), 20, 3);
        ApproachResult2d goal = ApproachResult2d.forTarget(ball, new Pose2d(6, 1, 0), 2, Math.PI / 2, 0.2);
        assertEquals(21, goal.fieldToRobotGoalPose().xInches, 1e-12);
        assertEquals(-5, goal.fieldToRobotGoalPose().yInches, 1e-12);
        ApproachResult2d committed = goal.committedFor(time.clock(), 2);
        time.clock().reset(0.0);
        assertFalse(committed.isUsable(time.clock()));
        assertFalse(goal.isUsable(time.clock()));
    }

    @Test public void cannotExtendCommitmentOrCommitStaleEvidence() {
        ManualLoopClock time = new ManualLoopClock(0.0);
        TargetObservation2d ball = fieldObservation(time.clock(), new Trajectory(), 20, 3);
        ApproachResult2d goal = ApproachResult2d.forTarget(ball, Pose2d.zero(), 0, 0, 0.2);
        ApproachResult2d committed = goal.committedFor(time.clock(), 1);
        try {
            committed.committedFor(time.clock(), 1);
            fail("A commitment cannot refresh itself");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("already committed"));
        }
        time.nextCycle(0.21);
        assertFalse(goal.committedFor(time.clock(), 1).hasApproach());
    }

    @Test public void missingQualityIsNotFabricatedAndExplicitQualityGateRejectsIt() {
        ManualLoopClock time = new ManualLoopClock(0.0);
        FacingSolution noScore = new FacingSolution(0.1, Double.NaN, time.clock().nowTimestamp());
        assertTrue(SpatialSolutionGate.defaults().accepts(noScore, time.clock().nowTimestamp()));
        assertFalse(SpatialSolutionGate.builder().minQuality(0).build()
                .accepts(noScore, time.clock().nowTimestamp()));
    }

    @Test public void observationModeRejectsAuthoredFieldPoint() {
        try {
            DriveGuidance.plan().faceTo().fieldPointInches(10, 2)
                    .solveWith().observedPoints(DriveGuidanceSpec.LossPolicy.PASS_THROUGH).build();
            fail("A field point requires field-pose evidence");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("selectedTargetPoint"));
        }
    }

    private static Source<TargetSelectionResult> selection(TargetObservation2d observation) {
        return TargetSelections.fromVisibleObjects(Source.constant(TargetObservations2d.fromFrame(observation.timestamp,
                Collections.singletonList(observation)))).freshWithinSec(0.2)
                .choose(TargetSelectionPolicies.nearestToRobot());
    }

    private static TargetObservation2d fieldObservation(LoopClock clock, Trajectory pose, double x, double y) {
        pose.publish(clock);
        PlanarPoseHistory history = new PlanarPoseHistory(pose, PlanarPoseHistory.Config.defaults());
        history.recordCurrent(clock);
        LoopTimestamp timestamp = clock.nowTimestamp();
        return TargetObservation2d.ofRobotRelativePosition(x, y, Double.NaN, timestamp)
                .withFieldPoseLookup(history.lookupSource().getAt(clock, timestamp));
    }

    private static final class Trajectory implements PoseTrajectoryEstimator {
        PoseEstimate estimate;
        void publish(LoopClock clock) { estimate = new PoseEstimate(Pose3d.zero(), true, 1, clock.nowTimestamp()); }
        @Override public void update(LoopClock clock) { throw new AssertionError("Borrowed estimator must not update"); }
        @Override public PoseEstimate getEstimate() { return estimate; }
        @Override public long trajectorySegmentId() { return 1; }
    }
}
