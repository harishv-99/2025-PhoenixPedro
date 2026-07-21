package edu.ftcphoenix.fw.ftc.localization;

import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
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

    private static AprilTagVisionLane noDetectionsVisionLane() {
        return new AprilTagVisionLane() {
            private final AprilTagSensor sensor = clock -> AprilTagDetections.none();

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
        };
    }

    private static final class RecordingPredictor implements MotionPredictor {
        int updateCount;
        private PoseEstimate estimate = PoseEstimate.noPose(0.0);
        private MotionDelta delta = MotionDelta.none(0.0);

        @Override
        public void update(LoopClock clock) {
            updateCount++;
            double nowSec = clock.nowSec();
            estimate = new PoseEstimate(
                    new Pose3d(updateCount, 0.0, 0.0, 0.0, 0.0, 0.0),
                    true,
                    1.0,
                    0.0,
                    nowSec
            );
            delta = MotionDelta.none(nowSec);
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
