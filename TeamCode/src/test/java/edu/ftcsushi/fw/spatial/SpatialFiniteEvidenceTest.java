package edu.ftcsushi.fw.spatial;

import org.junit.Test;

import java.util.Collections;
import java.util.Set;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.TimeAwareSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelectionSource;
import edu.ftcsushi.fw.sensing.vision.apriltag.TagSelectionResult;
import edu.ftcsushi.fw.field.SimpleTagLayout;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Raw spatial results must not claim a solve merely because invalid geometry has a fresh time. */
public final class SpatialFiniteEvidenceTest {
    @Test public void unavailableAbsolutePoseRetainsAuthoredAndSelectedIdentityWithoutGeometry() {
        ManualLoopClock time = new ManualLoopClock();
        TagSelectionSource identity = new TagSelectionSource() {
            @Override public Set<Integer> candidateIds() { return Collections.singleton(7); }
            @Override public TagSelectionResult get(LoopClock clock) { return TagSelectionResult.forTagId(7); }
        };
        SpatialLaneResult result = SpatialQuery.builder().translateTo(SpatialTargets.point(
                References.relativeToSelectedTagPoint(identity, 0, 0)))
                .andFaceTo(SpatialTargets.point(References.relativeToTagPoint(8, 0, 0)))
                .solveWith(SpatialSolveSet.builder().absolutePose(estimator(
                        PoseEstimate.noPose(time.clock().nowTimestamp()))).build())
                .fixedAprilTagLayout(new SimpleTagLayout().addPose(7, Pose3d.zero()).addPose(8, Pose3d.zero()))
                .build().get(time.clock()).laneResult(0);
        assertNull(result.translation);
        assertNull(result.facing);
        assertEquals(7, result.translationSelection.aprilTag().selectedTagId);
        assertEquals(8, result.facingSelection.aprilTag().selectedTagId);
        assertFalse(result.translationSelection.aprilTag().hasFreshSelectedObservation);
        assertFalse(result.facingSelection.aprilTag().hasFreshSelectedObservation);
    }

    @Test public void invalidHistoricalTranslationFrameLosesOnlyTranslation() {
        ManualLoopClock time = new ManualLoopClock();
        LoopTimestamp capture = time.clock().nowTimestamp();
        time.nextCycle(0.1);
        TimeAwareSource<Pose2d> translation = (clock, timestamp) -> timestamp == capture
                ? new Pose2d(Double.NaN, 0, 0) : Pose2d.zero();
        SpatialLaneResult result = query(time, capture, Pose3d.zero(), SpatialControlFrames.robotCenter()
                .withTranslationFrame(translation));
        assertNull(result.translation);
        assertNotNull(result.facing);
        assertEquals(0.2, result.facing.facingErrorRad, 1e-12);
    }

    @Test public void invalidHistoricalFacingFrameLosesOnlyFacing() {
        ManualLoopClock time = new ManualLoopClock();
        LoopTimestamp capture = time.clock().nowTimestamp();
        time.nextCycle(0.1);
        TimeAwareSource<Pose2d> facing = (clock, timestamp) -> timestamp == capture
                ? new Pose2d(0, 0, Double.NaN) : Pose2d.zero();
        SpatialLaneResult result = query(time, capture, Pose3d.zero(), SpatialControlFrames.robotCenter()
                .withFacingFrame(facing));
        assertNotNull(result.translation);
        assertNull(result.facing);
    }

    @Test public void nonfinitePoseComponentRejectsBothChannelsEvenWhenUnusedByPlanarMath() {
        ManualLoopClock time = new ManualLoopClock();
        SpatialLaneResult result = query(time, time.clock().nowTimestamp(),
                new Pose3d(0, 0, 0, 0, Double.NaN, 0), SpatialControlFrames.robotCenter());
        assertNull(result.translation);
        assertNull(result.facing);
    }

    @Test public void finiteCoordinateOverflowCannotPublishATranslationSolution() {
        ManualLoopClock time = new ManualLoopClock();
        PoseEstimate estimate = new PoseEstimate(new Pose3d(-Double.MAX_VALUE, 0, 0, 0, 0, 0),
                true, 1, time.clock().nowTimestamp());
        SpatialLaneResult result = SpatialQuery.builder()
                .translateTo(SpatialTargets.fieldPoint(Double.MAX_VALUE, 0))
                .andFaceTo(SpatialTargets.fieldHeading(0.2))
                .solveWith(SpatialSolveSet.builder().absolutePose(estimator(estimate)).build())
                .build().get(time.clock()).laneResult(0);
        assertNull(result.translation);
        assertNotNull(result.facing);
    }

    private static SpatialLaneResult query(ManualLoopClock time, LoopTimestamp capture, Pose3d pose,
                                           SpatialControlFrames frames) {
        PoseEstimate estimate = new PoseEstimate(pose, true, 1, capture);
        return SpatialQuery.builder().translateTo(SpatialTargets.fieldPoint(10, 2))
                .andFaceTo(SpatialTargets.fieldHeading(0.2)).controlFrames(frames)
                .solveWith(SpatialSolveSet.builder().absolutePose(estimator(estimate)).build())
                .build().get(time.clock()).laneResult(0);
    }

    private static AbsolutePoseEstimator estimator(PoseEstimate estimate) {
        return new AbsolutePoseEstimator() {
            @Override public void update(LoopClock clock) { fail("borrowed estimator"); }
            @Override public PoseEstimate getEstimate() { return estimate; }
        };
    }
}
