package edu.ftcphoenix.fw.localization.apriltag;

import org.junit.Test;

import java.util.Collections;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.field.SimpleTagLayout;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/** Verifies that AprilTag localization preserves acquisition time instead of resampling it. */
public final class AprilTagPoseEstimatorTimestampTest {

    @Test
    public void cachedFrameKeepsExactTimestampThroughPoseEstimationAndExpires() {
        ManualLoopClock time = new ManualLoopClock(10.0);
        LoopTimestamp frameTimestamp = time.clock().timestampSecondsAgo(0.10);
        Pose3d solvedPose = new Pose3d(24.0, -12.0, 0.0, 0.25, 0.0, 0.0);
        AprilTagDetections retainedFrame = AprilTagDetections.fromFrame(
                frameTimestamp,
                Collections.singletonList(AprilTagObservation.target(
                        5,
                        Pose3d.zero(),
                        solvedPose
                ))
        );
        AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(
                clock -> retainedFrame,
                new SimpleTagLayout().addPose(5, solvedPose),
                AprilTagPoseEstimator.Config.defaults().withMaxDetectionAgeSec(0.50)
        );

        estimator.update(time.clock());

        PoseEstimate first = estimator.getEstimate();
        assertTrue(first.hasPose);
        assertSame(frameTimestamp, first.timestamp);

        time.nextCycle(0.20);
        estimator.update(time.clock());

        PoseEstimate repeated = estimator.getEstimate();
        assertTrue(repeated.hasPose);
        assertSame(frameTimestamp, repeated.timestamp);

        time.nextCycle(0.21);
        estimator.update(time.clock());

        assertFalse(estimator.getEstimate().hasPose);
    }
}
