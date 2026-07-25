package edu.ftcphoenix.fw.ftc.localization;

import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.field.SimpleTagLayout;
import edu.ftcphoenix.fw.ftc.vision.AprilTagVisionLane;
import edu.ftcphoenix.fw.ftc.vision.VisionReadiness;
import edu.ftcphoenix.fw.localization.MotionDelta;
import edu.ftcphoenix.fw.localization.MotionPredictor;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused ownership coverage for the localization lane's injected-predictor path. */
public final class FtcOdometryAprilTagLocalizationLaneTest {

    @Test
    public void withPredictorUsesExactlyTheSuppliedPredictor() {
        RecordingPredictor predictor = new RecordingPredictor();
        FtcOdometryAprilTagLocalizationLane.Config cfg =
                FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.predictor.hardwareMapName = "retained-but-not-applied-by-injected-path";

        FtcOdometryAprilTagLocalizationLane lane =
                FtcOdometryAprilTagLocalizationLane.withPredictor(
                        predictor,
                        noDetectionsVisionLane(),
                        new SimpleTagLayout(),
                        cfg
                );

        assertSame(predictor, lane.predictor());
        assertEquals(
                "retained-but-not-applied-by-injected-path",
                lane.config().predictor.hardwareMapName
        );

        ManualLoopClock clock = new ManualLoopClock();
        lane.update(clock.nextCycle(0.02));

        assertEquals(1, predictor.updateCount);
        assertEquals(1.0, lane.globalEstimator().getEstimate().fieldToRobotPose.xInches, 1e-9);
    }

    @Test
    public void withPredictorStillDefensivelyCopiesCorrectionConfig() {
        FtcOdometryAprilTagLocalizationLane.Config cfg =
                FtcOdometryAprilTagLocalizationLane.Config.defaults();
        cfg.correctedEstimatorMode = FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.EKF;

        FtcOdometryAprilTagLocalizationLane lane =
                FtcOdometryAprilTagLocalizationLane.withPredictor(
                        new RecordingPredictor(),
                        noDetectionsVisionLane(),
                        new SimpleTagLayout(),
                        cfg
                );

        cfg.correctedEstimatorMode = FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.FUSION;

        assertEquals(
                FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.EKF,
                lane.config().correctedEstimatorMode
        );
    }

    @Test
    public void withPredictorRejectsMissingPredictorBeforeBuildingTheGraph() {
        try {
            FtcOdometryAprilTagLocalizationLane.withPredictor(null, null, null, null);
            fail("Expected missing predictor to fail fast");
        } catch (NullPointerException expected) {
            assertTrue(expected.getMessage().contains("predictor"));
        }
    }

    @Test
    public void laneTraversesEachEstimatorModeOncePerCycleAndRefreshesNextCycle() {
        for (FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode mode
                : FtcOdometryAprilTagLocalizationLane.GlobalEstimatorMode.values()) {
            RecordingPredictor predictor = new RecordingPredictor();
            RecordingVisionLane vision = new RecordingVisionLane();
            FtcOdometryAprilTagLocalizationLane.Config cfg =
                    FtcOdometryAprilTagLocalizationLane.Config.defaults();
            cfg.correctedEstimatorMode = mode;
            FtcOdometryAprilTagLocalizationLane lane =
                    FtcOdometryAprilTagLocalizationLane.withPredictor(
                            predictor,
                            vision,
                            new SimpleTagLayout(),
                            cfg
                    );
            ManualLoopClock time = new ManualLoopClock();

            lane.update(time.clock());
            PoseEstimate first = lane.globalEstimator().getEstimate();
            lane.update(time.clock());

            assertEquals(mode.name(), 1, predictor.updateCount);
            assertEquals(mode.name(), 1, vision.sensor.getCount);
            assertSame(mode.name(), first, lane.globalEstimator().getEstimate());

            time.nextCycle(0.02);
            lane.update(time.clock());

            assertEquals(mode.name(), 2, predictor.updateCount);
            assertEquals(mode.name(), 2, vision.sensor.getCount);
            assertFalse(mode.name(), first == lane.globalEstimator().getEstimate());
        }
    }

    @Test
    public void failedLaneCycleRethrowsTheSameFailureWithoutRetraversal() {
        RecordingPredictor predictor = new RecordingPredictor();
        RuntimeException failure = new IllegalStateException("predictor failed");
        predictor.failure = failure;
        RecordingVisionLane vision = new RecordingVisionLane();
        FtcOdometryAprilTagLocalizationLane lane = lane(predictor, vision);
        ManualLoopClock time = new ManualLoopClock();

        assertSame(failure, captureFailure(() -> lane.update(time.clock())));
        assertSame(failure, captureFailure(() -> lane.update(time.clock())));
        assertEquals(1, predictor.updateCount);
        assertEquals(0, vision.sensor.getCount);

        time.nextCycle(0.02);
        predictor.failure = null;
        lane.update(time.clock());

        assertEquals(2, predictor.updateCount);
        assertEquals(1, vision.sensor.getCount);
    }

    @Test
    public void recursiveLaneUpdateFailsFastAndIsRetainedForTheCycle() {
        RecordingPredictor predictor = new RecordingPredictor();
        RecordingVisionLane vision = new RecordingVisionLane();
        FtcOdometryAprilTagLocalizationLane lane = lane(predictor, vision);
        ManualLoopClock time = new ManualLoopClock();
        predictor.duringUpdate = () -> lane.update(time.clock());

        RuntimeException first = captureFailure(() -> lane.update(time.clock()));
        assertTrue(first instanceof IllegalStateException);
        assertTrue(first.getMessage().contains("reentered"));
        assertSame(first, captureFailure(() -> lane.update(time.clock())));
        assertEquals(1, predictor.updateCount);

        time.nextCycle(0.02);
        predictor.duringUpdate = null;
        lane.update(time.clock());
        assertEquals(2, predictor.updateCount);
    }

    @Test
    public void laneUpdateRequiresTheSharedLoopClock() {
        FtcOdometryAprilTagLocalizationLane lane = lane(
                new RecordingPredictor(),
                new RecordingVisionLane()
        );

        RuntimeException failure = captureFailure(() -> lane.update(null));

        assertTrue(failure instanceof NullPointerException);
        assertTrue(failure.getMessage().contains("clock"));
    }

    private static FtcOdometryAprilTagLocalizationLane lane(
            RecordingPredictor predictor,
            RecordingVisionLane vision
    ) {
        return FtcOdometryAprilTagLocalizationLane.withPredictor(
                predictor,
                vision,
                new SimpleTagLayout(),
                FtcOdometryAprilTagLocalizationLane.Config.defaults()
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

    private static AprilTagVisionLane noDetectionsVisionLane() {
        return new RecordingVisionLane();
    }

    private static final class RecordingVisionLane implements AprilTagVisionLane {
        final RecordingAprilTagSensor sensor = new RecordingAprilTagSensor();

        @Override
        public AprilTagSensor tagSensor() {
            return sensor;
        }

        @Override
        public CameraMountConfig cameraMountConfig() {
            return CameraMountConfig.identity();
        }

        @Override
        public VisionReadiness readiness(LoopClock clock) {
            return VisionReadiness.ready();
        }

        @Override
        public void close() {
            // No resources in this pure-JVM fixture.
        }
    }

    private static final class RecordingAprilTagSensor implements AprilTagSensor {
        int getCount;

        @Override
        public AprilTagDetections get(LoopClock clock) {
            getCount++;
            return AprilTagDetections.none();
        }
    }

    private static final class RecordingPredictor implements MotionPredictor {
        int updateCount;
        RuntimeException failure;
        Runnable duringUpdate;
        private PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        private MotionDelta delta = MotionDelta.none(LoopTimestamp.unavailable());

        @Override
        public void update(LoopClock clock) {
            updateCount++;
            if (duringUpdate != null) {
                duringUpdate.run();
            }
            if (failure != null) {
                throw failure;
            }
            LoopTimestamp timestamp = clock.nowTimestamp();
            estimate = new PoseEstimate(
                    new Pose3d(updateCount, 0.0, 0.0, 0.0, 0.0, 0.0),
                    true,
                    1.0,
                    timestamp
            );
            delta = MotionDelta.none(timestamp);
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }

        @Override
        public MotionDelta getLatestMotionDelta() {
            return delta;
        }
    }
}
