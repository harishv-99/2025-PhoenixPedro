package edu.ftcsushi.fw.localization.apriltag;

import org.junit.Test;

import java.util.Collections;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.TimeAwareSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.field.SimpleTagLayout;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Capture-time mount evidence belongs to localization, not a competing guidance solver. */
public final class AprilTagPoseEstimatorMountHistoryTest {
    @Test public void historicalMountIsReadOnceAtExposureAndNeverAtConstruction() {
        ManualLoopClock time = new ManualLoopClock(4.0);
        LoopTimestamp exposure = time.clock().nowTimestamp();
        AprilTagDetections frame = frame(exposure, 1);
        History history = new History();
        AprilTagPoseEstimator estimator = estimator(clock -> frame, history);
        assertEquals(0, history.calls);

        time.nextCycle(0.20);
        estimator.update(time.clock());
        estimator.update(time.clock());

        assertEquals(1, history.calls);
        assertSame(exposure, history.timestamp);
        assertTrue(estimator.getEstimate().hasPose);
        // Field tag x=60, camera sees x=48, and exposure-time lens offset x=2: robot x=10.
        assertEquals(10.0, estimator.getEstimate().fieldToRobotPose.xInches, 1e-9);
        assertSame(exposure, estimator.getEstimate().timestamp);
    }

    @Test public void absentEmptyUnknownTagStaleAndOldEpochFramesDoNotReadHistory() {
        ManualLoopClock time = new ManualLoopClock(4.0);
        History history = new History();
        AprilTagDetections[] input = {AprilTagDetections.none()};
        AprilTagPoseEstimator estimator = estimator(clock -> input[0], history);
        estimator.update(time.clock());
        input[0] = AprilTagDetections.fromFrame(time.clock().nowTimestamp(), Collections.emptyList());
        estimator.update(time.nextCycle(0.01));
        input[0] = frame(time.clock().nowTimestamp(), 99);
        estimator.update(time.nextCycle(0.01));
        input[0] = frame(time.clock().nowTimestamp(), 1);
        estimator.update(time.nextCycle(0.51));
        input[0] = frame(time.clock().nowTimestamp(), 1);
        time.clock().reset(5.0);
        estimator.update(time.clock());
        assertEquals(0, history.calls);
        assertFalse(estimator.getEstimate().hasPose);
    }

    @Test public void failedLookupIsRetainedForTheCycleAndNextCycleMayRecover() {
        ManualLoopClock time = new ManualLoopClock();
        History history = new History();
        RuntimeException failure = new IllegalStateException("mount history unavailable");
        history.failure = failure;
        AprilTagPoseEstimator estimator = estimator(clock -> frame(clock.nowTimestamp(), 1), history);
        assertSame(failure, failureOf(() -> estimator.update(time.clock())));
        history.failure = null;
        assertSame(failure, failureOf(() -> estimator.update(time.clock())));
        assertEquals(1, history.calls);
        estimator.update(time.nextCycle(0.01));
        assertTrue(estimator.getEstimate().hasPose);
        assertEquals(2, history.calls);
    }

    @Test public void nullHistoryDoesNotSubstituteIdentityOrCurrentMount() {
        ManualLoopClock time = new ManualLoopClock();
        History history = new History();
        history.mount = null;
        AprilTagPoseEstimator estimator = estimator(clock -> frame(clock.nowTimestamp(), 1), history);
        RuntimeException failure = failureOf(() -> estimator.update(time.clock()));
        assertTrue(failure instanceof NullPointerException);
        assertTrue(failure.getMessage().contains("cameraMount.getAt"));
        assertSame(failure, failureOf(() -> estimator.update(time.clock())));
        assertEquals(1, history.calls);
        assertFalse(estimator.getEstimate().hasPose);
    }

    @Test public void fixedAndHistoricalOverloadsUseTheSameSolverPolicy() {
        ManualLoopClock time = new ManualLoopClock();
        AprilTagDetections frame = frame(time.clock().nowTimestamp(), 1);
        History history = new History();
        AprilTagPoseEstimator fixed = new AprilTagPoseEstimator(clock -> frame, layout(),
                history.mount, config());
        AprilTagPoseEstimator historical = estimator(clock -> frame, history);
        fixed.update(time.clock());
        historical.update(time.clock());
        assertEquals(fixed.getEstimate().fieldToRobotPose.xInches,
                historical.getEstimate().fieldToRobotPose.xInches, 0.0);
        assertEquals(fixed.getEstimate().quality, historical.getEstimate().quality, 0.0);
        assertSame(fixed.getEstimate().timestamp, historical.getEstimate().timestamp);
    }

    private static AprilTagPoseEstimator estimator(
            edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor source, History history) {
        return new AprilTagPoseEstimator(source, layout(), history, config());
    }

    private static SimpleTagLayout layout() {
        return new SimpleTagLayout().addPose(1, new Pose3d(60, 0, 0, 0, 0, 0));
    }

    private static AprilTagPoseEstimator.Config config() {
        AprilTagPoseEstimator.Config config = AprilTagPoseEstimator.Config.defaults();
        config.fieldPoseSolver.preferObservationFieldPose = false;
        return config;
    }

    private static AprilTagDetections frame(LoopTimestamp timestamp, int id) {
        return AprilTagDetections.fromFrame(timestamp, Collections.singletonList(
                AprilTagObservation.target(id, new Pose3d(48, 0, 0, 0, 0, 0))));
    }

    private static RuntimeException failureOf(Runnable action) {
        try { action.run(); fail("Expected lookup failure"); return null; }
        catch (RuntimeException failure) { return failure; }
    }

    private static final class History implements TimeAwareSource<CameraMountConfig> {
        int calls;
        LoopTimestamp timestamp;
        RuntimeException failure;
        CameraMountConfig mount = CameraMountConfig.of(2, 0, 0, 0, 0, 0);
        @Override public CameraMountConfig getAt(LoopClock clock, LoopTimestamp requested) {
            calls++;
            timestamp = requested;
            if (failure != null) throw failure;
            return mount;
        }
    }
}
