package edu.ftcsushi.fw.spatial;

import org.junit.Test;

import java.util.Collections;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.source.TimeAwareSource;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.guidance.DriveGuidance;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceSpec;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceStatus;
import edu.ftcsushi.fw.sensing.observation.ObservationSources;
import edu.ftcsushi.fw.sensing.observation.TargetObservation2d;
import edu.ftcsushi.fw.sensing.observation.TargetObservations2d;
import edu.ftcsushi.fw.sensing.observation.TargetSelections;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Adversarial parity checks for the ordinary tag and shared observed-point consumption paths. */
public final class ObservedTargetSpatialParityTest {
    @Test public void delayedTranslationUsesCaptureToolOriginAndRobotAxesNotCurrentToolOrigin() {
        ManualLoopClock time = new ManualLoopClock();
        LoopTimestamp capture = time.clock().nowTimestamp();
        time.nextCycle(0.1);
        // The tool has translated and rotated after capture; the robot itself is not compensated.
        TimeAwareSource<Pose2d> tool = (clock, requested) -> requested.secondsSince(capture) == 0
                ? new Pose2d(2, 1, Math.PI / 2) : new Pose2d(6, 4, -Math.PI / 2);
        TargetObservation2d point = TargetObservation2d.ofRobotRelativePosition(10, 3, Double.NaN, capture);
        ReferencePoint2d reference = References.observedPoint(TargetSelections.from(Source.constant(
                TargetObservations2d.fromFrame(capture, Collections.singletonList(point))))
                .freshWithinSec(0.2).nearestToRobot());
        SpatialControlFrames frames = SpatialControlFrames.robotCenter().withTranslationFrame(tool);
        SpatialQuery geometry = SpatialQuery.builder().translateTo(SpatialTargets.point(reference))
                .controlFrames(frames).solveWith(SpatialSolveSet.builder()
                        .add(new ObservedTargetSpatialSolveLane()).build()).build();
        TranslationSolution translation = geometry.get(time.clock()).laneResult(0).translation;
        assertEquals(2, translation.frameForwardInches(), 1e-12);
        assertEquals(-8, translation.frameLeftInches(), 1e-12);

        DriveGuidanceStatus status = DriveGuidance.plan().translateTo().point(reference)
                .controlFrames(frames).solveWith()
                .observedPoints(DriveGuidanceSpec.LossPolicy.PASS_THROUGH).build().query().get(time.clock());
        assertTrue(status.hasTranslationError);
        assertEquals(8, status.forwardErrorIn, 1e-12);
        assertEquals(2, status.leftErrorIn, 1e-12);
    }

    @Test public void rawTagAndObservedTagCarryUnknownQualityAndObeyTheSameExplicitScoreGate() {
        ManualLoopClock time = new ManualLoopClock();
        LoopTimestamp capture = time.clock().nowTimestamp();
        AprilTagDetections frame = AprilTagDetections.fromFrame(capture, Collections.singletonList(
                AprilTagObservation.target(5, new Pose3d(10, 3, 0, 0.4, 0, 0))));
        AprilTagSensor tags = clock -> frame;
        ReferencePoint2d direct = References.relativeToTagPoint(5, 0, 0);
        ReferencePoint2d observed = References.observedPoint(TargetSelections.from(
                ObservationSources.aprilTags(Source.constant(frame), CameraMountConfig.identity()))
                .freshWithinSec(0.2).nearestToRobot());
        SpatialQuery directQuery = SpatialQuery.builder().translateTo(SpatialTargets.point(direct))
                .andFaceTo(SpatialTargets.point(direct)).solveWith(SpatialSolveSet.builder()
                        .relativeAprilTags(tags, CameraMountConfig.identity()).build()).build();
        SpatialQuery observedQuery = SpatialQuery.builder().translateTo(SpatialTargets.point(observed))
                .andFaceTo(SpatialTargets.point(observed)).solveWith(SpatialSolveSet.builder()
                        .add(new ObservedTargetSpatialSolveLane()).build()).build();
        SpatialSolutionGate explicitScore = SpatialSolutionGate.builder().minQuality(0.9).build();
        for (SpatialQuery query : new SpatialQuery[]{directQuery, observedQuery}) {
            SpatialLaneResult lane = query.get(time.clock()).laneResult(0);
            assertNotNull(lane.translation);
            assertNotNull(lane.facing);
            assertTrue(Double.isNaN(lane.translation.quality));
            assertTrue(Double.isNaN(lane.facing.quality));
            assertSame(capture, lane.translation.targetObservationTimestamp);
            assertSame(capture, lane.facing.targetObservationTimestamp);
            assertFalse(lane.translation.robotPoseTimestamp.isAvailable());
            assertFalse(lane.facing.robotPoseTimestamp.isAvailable());
            assertTrue(SpatialSolutionGate.defaults().accepts(lane.translation, capture));
            assertTrue(SpatialSolutionGate.defaults().accepts(lane.facing, capture));
            assertFalse(explicitScore.accepts(lane.translation, capture));
            assertFalse(explicitScore.accepts(lane.facing, capture));
        }
    }

    @Test public void transformedOverflowCannotBecomeNanDriveOutputUnderEitherLossPolicy() {
        ManualLoopClock time = new ManualLoopClock();
        LoopTimestamp capture = time.clock().nowTimestamp();
        TargetObservation2d point = TargetObservation2d.ofRobotRelativePosition(
                Double.MAX_VALUE, 0, Double.NaN, capture);
        ReferencePoint2d reference = References.observedPoint(TargetSelections.from(Source.constant(
                TargetObservations2d.fromFrame(capture, Collections.singletonList(point))))
                .freshWithinSec(0.2).lowestCost(candidate -> 0));
        for (DriveGuidanceSpec.LossPolicy loss : DriveGuidanceSpec.LossPolicy.values()) {
            DriveGuidanceStatus status = DriveGuidance.plan().translateTo().point(reference)
                    .controlFrames(SpatialControlFrames.robotCenter().withTranslationFrame(
                            new Pose2d(-Double.MAX_VALUE, 0, 0)))
                    .solveWith().observedPoints(loss).build().query().get(time.clock());
            assertFalse(status.hasTranslationError);
            assertTrue(Double.isFinite(status.signal.axial));
            assertTrue(Double.isFinite(status.signal.lateral));
            assertEquals(0, status.signal.axial, 0);
            assertEquals(loss == DriveGuidanceSpec.LossPolicy.ZERO_OUTPUT, status.mask.overridesTranslation());
        }
    }
}
