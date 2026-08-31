package edu.ftcsushi.fw.localization.apriltag;

import java.util.Collections;
import java.util.Set;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.field.TagLayout;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/** Verifies that AprilTag localization owns immutable fixed-field metadata. */
public final class AprilTagPoseEstimatorLayoutSnapshotTest {

    @Test
    public void constructorSnapshotSupportsRealSolveAfterAuthoredSourceStartsFailing() {
        ManualLoopClock time = new ManualLoopClock(3.0);
        LoopTimestamp frameTimestamp = time.clock().nowTimestamp();
        Pose3d solvedFieldToRobot = new Pose3d(18.0, -7.0, 0.0, 0.35, 0.0, 0.0);
        RecordingLayout authored = new RecordingLayout(solvedFieldToRobot);
        AprilTagDetections frame = AprilTagDetections.fromFrame(
                frameTimestamp,
                Collections.singletonList(AprilTagObservation.target(
                        6,
                        Pose3d.zero(),
                        solvedFieldToRobot
                ))
        );
        AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(
                new FixedFrameSensor(frame),
                authored,
                AprilTagPoseEstimator.Config.defaults()
        );

        assertEquals(1, authored.idsCalls);
        assertEquals(1, authored.poseCalls);
        authored.failOnAccess = true;

        estimator.update(time.clock());

        assertEquals(1, authored.idsCalls);
        assertEquals(1, authored.poseCalls);
        PoseEstimate estimate = estimator.getEstimate();
        assertTrue(estimate.hasPose);
        assertEquals(solvedFieldToRobot, estimate.fieldToRobotPose);
        assertSame(frameTimestamp, estimate.timestamp);
    }

    private static final class FixedFrameSensor implements AprilTagSensor {
        private final AprilTagDetections frame;

        FixedFrameSensor(AprilTagDetections frame) {
            this.frame = frame;
        }

        @Override
        public AprilTagDetections get(LoopClock clock) {
            return frame;
        }
    }

    private static final class RecordingLayout implements TagLayout {
        private final Pose3d pose;
        private int idsCalls;
        private int poseCalls;
        private boolean failOnAccess;

        RecordingLayout(Pose3d pose) {
            this.pose = pose;
        }

        @Override
        public Pose3d getFieldToTagPose(int id) {
            if (failOnAccess) {
                throw new AssertionError("Estimator read the mutable authored layout after construction");
            }
            poseCalls++;
            return pose;
        }

        @Override
        public Set<Integer> ids() {
            if (failOnAccess) {
                throw new AssertionError("Estimator enumerated the mutable authored layout after construction");
            }
            idsCalls++;
            return Collections.singleton(6);
        }
    }
}
