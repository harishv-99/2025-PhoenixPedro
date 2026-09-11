package edu.ftcsushi.fw.spatial;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.source.TimeAwareSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.field.SimpleTagLayout;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.localization.apriltag.FixedTagFieldPoseSolver;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelectionPolicies;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelectionSource;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelections;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Public construction, independent sensor evidence, and selected-reference boundary regressions. */
public final class AprilTagSpatialSolveLaneBoundaryTest {
    private static final double EPS = 1e-9;

    @Test public void builtInLanesHaveOnlyTheCanonicalSpatialBuilderConstructionPath() throws Exception {
        for (Class<?> lane : Arrays.asList(AprilTagSpatialSolveLane.class,
                AbsolutePoseSpatialSolveLane.class, ObservedTargetSpatialSolveLane.class)) {
            assertFalse(Modifier.isPublic(lane.getModifiers()));
            assertEquals(0, lane.getConstructors().length);
        }
        for (Class<?> stage : Arrays.asList(SpatialSolveSet.FirstLaneStep.class,
                SpatialSolveSet.MoreLanesStep.class)) {
            assertEquals(SpatialSolveSet.MoreLanesStep.class, stage.getMethod("relativeAprilTags",
                    AprilTagSensor.class, CameraMountConfig.class).getReturnType());
            assertEquals(SpatialSolveSet.MoreLanesStep.class, stage.getMethod("relativeAprilTags",
                    AprilTagSensor.class, CameraMountConfig.class, double.class).getReturnType());
            assertEquals(SpatialSolveSet.MoreLanesStep.class, stage.getMethod("relativeAprilTags",
                    AprilTagSensor.class, TimeAwareSource.class, double.class).getReturnType());
            for (Method method : stage.getMethods()) {
                assertNotEquals("aprilTags", method.getName());
                for (Class<?> argument : method.getParameterTypes()) {
                    assertNotEquals(FixedTagFieldPoseSolver.class, argument);
                    assertNotEquals(FixedTagFieldPoseSolver.Config.class, argument);
                }
            }
        }
    }

    @Test public void rawFixedAndHistoricalLanesCannotUseEmbeddedFieldPoseToSolveFieldTargets() {
        ManualLoopClock time = new ManualLoopClock();
        AprilTagDetections frame = frame(time, AprilTagObservation.target(7,
                new Pose3d(30, 0, 0, 0, 0, 0), Pose3d.zero()));
        AprilTagSensor tags = clock -> frame;
        TimeAwareSource<CameraMountConfig> history = (clock, timestamp) -> CameraMountConfig.identity();
        for (SpatialSolveSet lanes : Arrays.asList(
                SpatialSolveSet.builder().relativeAprilTags(tags, CameraMountConfig.identity(), 0.2).build(),
                SpatialSolveSet.builder().relativeAprilTags(tags, history, 0.2).build())) {
            SpatialLaneResult result = SpatialQuery.builder().translateTo(SpatialTargets.fieldPoint(40, 10))
                    .andFaceTo(SpatialTargets.fieldHeading(0.5)).solveWith(lanes)
                    .fixedAprilTagLayout(new SimpleTagLayout().addPose(7, new Pose3d(30, 0, 0, 0, 0, 0)))
                    .build().get(time.clock()).laneResult(0);
            assertNull(result.translation);
            assertNull(result.facing);
        }
    }

    @Test public void selectedIdentityFromCameraADoesNotSmuggleCameraAGeometryIntoCameraB() {
        ManualLoopClock time = new ManualLoopClock();
        AprilTagDetections a = frame(time, tag(7, 100, 20, 0.1));
        TagSelectionSource selection = selected(Source.constant(a), CameraMountConfig.of(20, 0, 0, 0, 0, 0), ids(7));
        time.nextCycle(0.05);
        AprilTagDetections b = frame(time, tag(7, 10, 3, 0.2));
        CameraMountConfig mountB = CameraMountConfig.of(2, 1, 0, Math.PI / 2, 0, 0);
        ReferencePoint2d point = References.relativeToSelectedTagPoint(selection, 1, 2);
        ReferenceFrame2d heading = References.relativeToSelectedTagFrame(selection, 0, 0, 0.3);
        SpatialLaneResult result = SpatialQuery.builder().translateTo(SpatialTargets.point(point))
                .andFaceTo(SpatialTargets.frameHeading(heading))
                .solveWith(SpatialSolveSet.builder().relativeAprilTags(clock -> b, mountB, 0.2).build())
                .build().get(time.clock()).laneResult(0);
        Pose2d robotToTagB = mountB.robotToCameraPose().then(b.observations.get(0).cameraToTagPose).toPose2d();
        Pose2d expected = robotToTagB.then(new Pose2d(1, 2, 0));
        assertEquals(expected.xInches, result.translation.robotToTargetPoint.xInches, EPS);
        assertEquals(expected.yInches, result.translation.robotToTargetPoint.yInches, EPS);
        assertEquals(Pose2d.wrapToPi(robotToTagB.headingRad + 0.3), result.facing.facingErrorRad, EPS);
        assertSame(b.frameTimestamp(), result.translation.timestamp);
        assertSame(b.frameTimestamp(), result.facing.timestamp);
        assertSame(a.frameTimestamp(), result.translationSelection.aprilTag().currentSelectedCandidate.evidenceTimestamp);
        assertSame(selection.get(time.clock()), result.translationSelection.aprilTag());
    }

    @Test public void stricterSolveAgeCannotBorrowFreshnessFromSelectedCameraGeometry() {
        ManualLoopClock time = new ManualLoopClock();
        AprilTagDetections old = frame(time, tag(7, 30, 2, 0));
        time.nextCycle(0.3);
        AprilTagDetections current = frame(time, tag(7, 10, 1, 0));
        TagSelectionSource selection = selected(Source.constant(current), CameraMountConfig.identity(), ids(7));
        ReferencePoint2d point = References.relativeToSelectedTagPoint(selection, 0, 0);
        SpatialLaneResult result = SpatialQuery.builder().translateTo(SpatialTargets.point(point))
                .andFaceTo(SpatialTargets.point(point)).solveWith(SpatialSolveSet.builder()
                        .relativeAprilTags(clock -> old, CameraMountConfig.identity(), 0.1).build())
                .build().get(time.clock()).laneResult(0);
        assertTrue(selection.get(time.clock()).hasFreshSelectedObservation);
        assertNull(result.translation);
        assertNull(result.facing);
        assertEquals(7, result.translationSelection.aprilTag().selectedTagId);
    }

    @Test public void poseSelectedIdentityCanGuideAgainstActualSensorWithoutFakeSelectedObservation() {
        ManualLoopClock time = new ManualLoopClock();
        PoseEstimate estimate = new PoseEstimate(Pose3d.zero(), true, 1, time.clock().nowTimestamp());
        AbsolutePoseEstimator pose = new AbsolutePoseEstimator() {
            @Override public void update(LoopClock clock) { fail("borrowed estimator must not update"); }
            @Override public PoseEstimate getEstimate() { return estimate; }
        };
        TagSelectionSource selection = TagSelections.fromFieldPose(pose,
                new SimpleTagLayout().addPose(7, new Pose3d(100, 0, 0, 0, 0, 0)), CameraMountConfig.identity())
                .among(ids(7)).freshWithinSec(0.2).minQuality(0.1)
                .choose(TagSelectionPolicies.closestRange()).continuous();
        AprilTagDetections actual = frame(time, tag(7, 15, 3, 0));
        SpatialLaneResult result = SpatialQuery.builder().translateTo(SpatialTargets.point(
                References.relativeToSelectedTagPoint(selection, 0, 0))).solveWith(SpatialSolveSet.builder()
                        .relativeAprilTags(clock -> actual, CameraMountConfig.identity()).build())
                .build().get(time.clock()).laneResult(0);
        assertFalse(selection.get(time.clock()).hasFreshSelectedObservation);
        assertEquals(15, result.translation.robotToTargetPoint.xInches, EPS);
        assertSame(actual.frameTimestamp(), result.translation.targetObservationTimestamp);
    }

    @Test public void sharedAndPerIdPointFrameOffsetsRemainParallelAcrossChangingSelections() {
        ManualLoopClock time = new ManualLoopClock();
        AprilTagDetections[] frames = {frame(time, tag(7, 20, 0, Math.PI / 2))};
        TagSelectionSource selection = selected(clock -> frames[0], CameraMountConfig.identity(), ids(7, 8));
        Map<Integer, References.TagPointOffset> pointOffsets = new LinkedHashMap<>();
        pointOffsets.put(7, References.pointOffset(1, 2));
        pointOffsets.put(8, References.pointOffset(-3, 4));
        Map<Integer, References.TagFrameOffset> frameOffsets = new LinkedHashMap<>();
        frameOffsets.put(7, References.frameOffset(1, 2, 0.3));
        frameOffsets.put(8, References.frameOffset(-3, 4, -0.4));
        ReferencePoint2d mapped = References.relativeToSelectedTagPoint(selection, pointOffsets);
        ReferenceFrame2d mappedFrame = References.relativeToSelectedTagFrame(selection, frameOffsets);
        ReferencePoint2d shared = References.relativeToSelectedTagPoint(selection, 1, 2);
        ReferenceFrame2d sharedFrame = References.relativeToSelectedTagFrame(selection, 1, 2, 0.3);
        SpatialSolveSet lanes = SpatialSolveSet.builder()
                .relativeAprilTags(clock -> frames[0], CameraMountConfig.identity()).build();
        SpatialQuery mappedQuery = query(mapped, mappedFrame, lanes);
        SpatialQuery sharedQuery = query(shared, sharedFrame, lanes);
        SpatialLaneResult first = mappedQuery.get(time.clock()).laneResult(0);
        SpatialLaneResult firstShared = sharedQuery.get(time.clock()).laneResult(0);
        assertEquals(18, first.translation.robotToTargetPoint.xInches, EPS);
        assertEquals(1, first.translation.robotToTargetPoint.yInches, EPS);
        assertEquals(Math.PI / 2 + 0.3, first.facing.facingErrorRad, EPS);
        assertEquals(first.translation.robotToTargetPoint, firstShared.translation.robotToTargetPoint);
        time.nextCycle(0.02);
        frames[0] = frame(time, tag(8, 0, 20, 0));
        SpatialLaneResult next = mappedQuery.get(time.clock()).laneResult(0);
        SpatialLaneResult nextShared = sharedQuery.get(time.clock()).laneResult(0);
        assertEquals(8, next.translationSelection.aprilTag().selectedTagId);
        assertSame(next.translationSelection.aprilTag(), next.facingSelection.aprilTag());
        assertEquals(-3, next.translation.robotToTargetPoint.xInches, EPS);
        assertEquals(24, next.translation.robotToTargetPoint.yInches, EPS);
        assertEquals(-0.4, next.facing.facingErrorRad, EPS);
        assertEquals(1, nextShared.translation.robotToTargetPoint.xInches, EPS);
        assertEquals(22, nextShared.translation.robotToTargetPoint.yInches, EPS);
        assertEquals(0.3, nextShared.facing.facingErrorRad, EPS);
    }

    private static SpatialQuery query(ReferencePoint2d point, ReferenceFrame2d heading, SpatialSolveSet lanes) {
        return SpatialQuery.builder().translateTo(SpatialTargets.point(point))
                .andFaceTo(SpatialTargets.frameHeading(heading)).solveWith(lanes).build();
    }

    private static TagSelectionSource selected(Source<AprilTagDetections> frames, CameraMountConfig mount,
                                               Set<Integer> ids) {
        return TagSelections.fromVisibleTags(frames, mount).among(ids).freshWithinSec(1)
                .choose(TagSelectionPolicies.closestRange()).continuous();
    }

    private static Set<Integer> ids(Integer... ids) { return new LinkedHashSet<>(Arrays.asList(ids)); }

    private static AprilTagDetections frame(ManualLoopClock time, AprilTagObservation... observations) {
        return AprilTagDetections.fromFrame(time.clock().nowTimestamp(), Arrays.asList(observations));
    }

    private static AprilTagObservation tag(int id, double x, double y, double yaw) {
        return AprilTagObservation.target(id, new Pose3d(x, y, 0, yaw, 0, 0));
    }
}
