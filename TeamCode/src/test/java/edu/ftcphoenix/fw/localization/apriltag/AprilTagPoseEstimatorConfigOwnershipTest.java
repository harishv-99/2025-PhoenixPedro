package edu.ftcphoenix.fw.localization.apriltag;

import org.junit.Test;

import java.util.Collections;
import java.util.Set;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.field.SimpleTagLayout;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Proves estimator-local validation, snapshot ownership, and effect ordering. */
public final class AprilTagPoseEstimatorConfigOwnershipTest {

    @Test
    public void invalidAgeFailsBeforeLayoutAccessWithExactFieldDomainAndValue() {
        AprilTagPoseEstimator.Config config = AprilTagPoseEstimator.Config.defaults();
        config.maxDetectionAgeSec = Double.NaN;
        RecordingLayout layout = new RecordingLayout();
        RecordingSensor sensor = new RecordingSensor();

        RuntimeException failure = capture(() -> new AprilTagPoseEstimator(
                sensor,
                layout,
                config
        ));

        assertTrue(failure instanceof IllegalArgumentException);
        assertTrue(failure.getMessage().contains(
                "AprilTagPoseEstimator.Config.maxDetectionAgeSec must be finite and >= 0, got NaN"));
        assertEquals(0, layout.accessCount);
        assertEquals(0, sensor.sampleCount);

        config.maxDetectionAgeSec = -Double.MIN_VALUE;
        RecordingLayout negativeLayout = new RecordingLayout();
        RecordingSensor negativeSensor = new RecordingSensor();
        RuntimeException negativeFailure = capture(() -> new AprilTagPoseEstimator(
                negativeSensor,
                negativeLayout,
                config
        ));
        assertTrue(negativeFailure instanceof IllegalArgumentException);
        assertTrue(negativeFailure.getMessage().contains(
                "AprilTagPoseEstimator.Config.maxDetectionAgeSec must be finite and >= 0, got "
                        + (-Double.MIN_VALUE)));
        assertEquals(0, negativeLayout.accessCount);
        assertEquals(0, negativeSensor.sampleCount);

        config.maxDetectionAgeSec = Double.POSITIVE_INFINITY;
        RecordingLayout infiniteLayout = new RecordingLayout();
        RecordingSensor infiniteSensor = new RecordingSensor();
        RuntimeException infiniteFailure = capture(() -> new AprilTagPoseEstimator(
                infiniteSensor,
                infiniteLayout,
                config
        ));
        assertTrue(infiniteFailure instanceof IllegalArgumentException);
        assertTrue(infiniteFailure.getMessage().contains("got Infinity"));
        assertEquals(0, infiniteLayout.accessCount);
        assertEquals(0, infiniteSensor.sampleCount);
    }

    @Test
    public void invalidNestedSolverAndNullMountFailBeforeLayoutAccess() {
        RecordingLayout layout = new RecordingLayout();
        RecordingSensor sensor = new RecordingSensor();
        AprilTagPoseEstimator.Config invalidSolver = AprilTagPoseEstimator.Config.defaults();
        invalidSolver.fieldPoseSolver.rangeSoftnessInches = 0.0;

        RuntimeException solverFailure = capture(() -> new AprilTagPoseEstimator(
                sensor,
                layout,
                invalidSolver
        ));
        assertTrue(solverFailure.getMessage().contains(
                "FixedTagFieldPoseSolver.Config.rangeSoftnessInches"));
        assertEquals(0, layout.accessCount);
        assertEquals(0, sensor.sampleCount);

        AprilTagPoseEstimator.Config nullMount = AprilTagPoseEstimator.Config.defaults();
        nullMount.cameraMount = null;
        RecordingSensor mountSensor = new RecordingSensor();
        RuntimeException mountFailure = capture(() -> new AprilTagPoseEstimator(
                mountSensor,
                layout,
                nullMount
        ));
        assertTrue(mountFailure instanceof NullPointerException);
        assertTrue(mountFailure.getMessage().contains("AprilTagPoseEstimator.Config.cameraMount"));
        assertEquals(0, layout.accessCount);
        assertEquals(0, mountSensor.sampleCount);
    }

    @Test
    public void nullNestedSolverAndNullConfigAreRejectedRatherThanDefaulted() {
        AprilTagPoseEstimator.Config nullSolver = AprilTagPoseEstimator.Config.defaults();
        nullSolver.fieldPoseSolver = null;
        RecordingSensor nullSolverSensor = new RecordingSensor();

        RuntimeException solverFailure = capture(() -> new AprilTagPoseEstimator(
                nullSolverSensor,
                new RecordingLayout(),
                nullSolver
        ));
        assertTrue(solverFailure instanceof NullPointerException);
        assertTrue(solverFailure.getMessage().contains("AprilTagPoseEstimator.Config.fieldPoseSolver"));
        assertEquals(0, nullSolverSensor.sampleCount);

        RecordingSensor nullConfigSensor = new RecordingSensor();
        RuntimeException configFailure = capture(() -> new AprilTagPoseEstimator(
                nullConfigSensor,
                new RecordingLayout(),
                null
        ));
        assertTrue(configFailure instanceof NullPointerException);
        assertTrue(configFailure.getMessage().contains("cfg"));
        assertEquals(0, nullConfigSensor.sampleCount);
    }

    @Test
    public void zeroAgeIsValidAndLaterDraftMutationCannotChangeEstimator() {
        ManualLoopClock time = new ManualLoopClock();
        Pose3d pose = new Pose3d(4.0, 2.0, 0.0, 0.1, 0.0, 0.0);
        AprilTagDetections frame = AprilTagDetections.fromFrame(
                time.clock().nowTimestamp(),
                Collections.singletonList(AprilTagObservation.target(1, Pose3d.zero(), pose))
        );
        AprilTagPoseEstimator.Config config = AprilTagPoseEstimator.Config.defaults();
        config.maxDetectionAgeSec = 0.0;
        AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(
                clock -> frame,
                new SimpleTagLayout().addPose(1, pose),
                config
        );

        config.maxDetectionAgeSec = -1.0;
        config.fieldPoseSolver.rangeSoftnessInches = 0.0;
        config.cameraMount = null;
        estimator.update(time.clock());

        assertTrue(estimator.getEstimate().hasPose);
        assertSame(frame.frameTimestamp(), estimator.getEstimate().timestamp);
    }

    private static RuntimeException capture(Runnable action) {
        try {
            action.run();
            fail("Expected constructor failure");
            return null;
        } catch (RuntimeException failure) {
            return failure;
        }
    }

    private static final class RecordingLayout implements TagLayout {
        int accessCount;

        @Override
        public Pose3d getFieldToTagPose(int id) {
            accessCount++;
            return Pose3d.zero();
        }

        @Override
        public Set<Integer> ids() {
            accessCount++;
            return Collections.singleton(1);
        }
    }

    private static final class RecordingSensor implements AprilTagSensor {
        int sampleCount;

        @Override
        public AprilTagDetections get(LoopClock clock) {
            sampleCount++;
            return AprilTagDetections.none();
        }
    }
}
