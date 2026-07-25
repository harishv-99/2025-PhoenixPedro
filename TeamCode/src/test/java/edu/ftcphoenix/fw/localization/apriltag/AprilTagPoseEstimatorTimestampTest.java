package edu.ftcphoenix.fw.localization.apriltag;

import org.junit.Test;

import java.util.Collections;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.field.SimpleTagLayout;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies that AprilTag localization preserves acquisition time instead of resampling it. */
public final class AprilTagPoseEstimatorTimestampTest {

    private static final double EPSILON = 1e-9;

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

    @Test
    public void updateSamplesAndSolvesOncePerCycleThenRefreshesNextCycle() {
        ManualLoopClock time = new ManualLoopClock(2.0);
        RecordingSensor sensor = new RecordingSensor();
        Pose3d firstPose = new Pose3d(12.0, -3.0, 0.0, 0.2, 0.0, 0.0);
        Pose3d secondPose = new Pose3d(30.0, 4.0, 0.0, -0.3, 0.0, 0.0);
        sensor.next = frame(time.clock().nowTimestamp(), firstPose);
        AprilTagPoseEstimator estimator = estimator(sensor, firstPose);

        estimator.update(time.clock());
        PoseEstimate first = estimator.getEstimate();
        assertTrue(first.hasPose);
        assertEquals(1, sensor.getCount);

        // A source transition after this owner's update belongs to the next cycle.
        sensor.next = frame(time.clock().nowTimestamp(), secondPose);
        estimator.update(time.clock());

        assertEquals(1, sensor.getCount);
        assertSame(first, estimator.getEstimate());
        assertEquals(firstPose.xInches, estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);

        time.nextCycle(0.02);
        sensor.next = frame(time.clock().nowTimestamp(), secondPose);
        estimator.update(time.clock());

        assertEquals(2, sensor.getCount);
        assertFalse(first == estimator.getEstimate());
        assertEquals(secondPose.xInches,
                estimator.getEstimate().fieldToRobotPose.xInches, EPSILON);
    }

    @Test
    public void failedCycleRethrowsTheSameFailureWithoutResamplingAndRecoversNextCycle() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingSensor sensor = new RecordingSensor();
        RuntimeException failure = new IllegalStateException("camera sample failed");
        sensor.failure = failure;
        AprilTagPoseEstimator estimator = estimator(sensor, Pose3d.zero());

        assertSame(failure, captureFailure(() -> estimator.update(time.clock())));
        assertSame(failure, captureFailure(() -> estimator.update(time.clock())));
        assertEquals(1, sensor.getCount);

        time.nextCycle(0.02);
        sensor.failure = null;
        sensor.next = AprilTagDetections.fromFrame(
                time.clock().nowTimestamp(),
                Collections.<AprilTagObservation>emptyList()
        );
        estimator.update(time.clock());

        assertEquals(2, sensor.getCount);
        assertFalse(estimator.getEstimate().hasPose);
    }

    @Test
    public void recursiveUpdateFailsFastAndThatFailureIsRetainedForTheCycle() {
        ManualLoopClock time = new ManualLoopClock();
        ManualLoopClock otherTime = new ManualLoopClock();
        otherTime.nextCycle(0.02);
        RecordingSensor sensor = new RecordingSensor();
        AprilTagPoseEstimator estimator = estimator(sensor, Pose3d.zero());
        sensor.duringGet = () -> estimator.update(otherTime.clock());

        RuntimeException first = captureFailure(() -> estimator.update(time.clock()));
        assertTrue(first instanceof IllegalStateException);
        assertTrue(first.getMessage().contains("reentered"));
        assertSame(first, captureFailure(() -> estimator.update(time.clock())));
        assertEquals(1, sensor.getCount);

        time.nextCycle(0.02);
        sensor.duringGet = null;
        sensor.next = AprilTagDetections.fromFrame(
                time.clock().nowTimestamp(),
                Collections.<AprilTagObservation>emptyList()
        );
        estimator.update(time.clock());
        assertEquals(2, sensor.getCount);
    }

    @Test
    public void updateRequiresTheSharedLoopClock() {
        AprilTagPoseEstimator estimator = estimator(new RecordingSensor(), Pose3d.zero());

        RuntimeException failure = captureFailure(() -> estimator.update(null));

        assertTrue(failure instanceof NullPointerException);
        assertTrue(failure.getMessage().contains("clock"));
    }

    private static AprilTagPoseEstimator estimator(AprilTagSensor sensor, Pose3d tagPose) {
        AprilTagPoseEstimator.Config config = AprilTagPoseEstimator.Config.defaults();
        config.observationFieldPoseMaxDeltaInches = 1_000.0;
        config.observationFieldPoseMaxDeltaHeadingRad = Math.PI;
        return new AprilTagPoseEstimator(
                sensor,
                new SimpleTagLayout().addPose(5, tagPose),
                config
        );
    }

    private static AprilTagDetections frame(LoopTimestamp timestamp, Pose3d solvedPose) {
        return AprilTagDetections.fromFrame(
                timestamp,
                Collections.singletonList(AprilTagObservation.target(
                        5,
                        Pose3d.zero(),
                        solvedPose
                ))
        );
    }

    private static RuntimeException captureFailure(Runnable action) {
        try {
            action.run();
            fail("Expected update failure");
            return null;
        } catch (RuntimeException failure) {
            return failure;
        }
    }

    private static final class RecordingSensor implements AprilTagSensor {
        int getCount;
        AprilTagDetections next = AprilTagDetections.none();
        RuntimeException failure;
        Runnable duringGet;

        @Override
        public AprilTagDetections get(edu.ftcphoenix.fw.core.time.LoopClock clock) {
            getCount++;
            if (duringGet != null) {
                duringGet.run();
            }
            if (failure != null) {
                throw failure;
            }
            return next;
        }
    }
}
