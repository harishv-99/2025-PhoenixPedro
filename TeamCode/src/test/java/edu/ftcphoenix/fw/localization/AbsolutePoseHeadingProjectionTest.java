package edu.ftcphoenix.fw.localization;

import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.time.LoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

public final class AbsolutePoseHeadingProjectionTest {
    @Test
    public void projectsCachedPoseHeadingWithoutAnotherUpdate() {
        LoopClock clock = new LoopClock();
        clock.reset(3.0);
        PoseEstimate pose = new PoseEstimate(
                new Pose3d(4.0, 5.0, 0.0, 1.2, 0.0, 0.0),
                true,
                0.8,
                clock.nowTimestamp()
        );
        FixedPoseEstimator estimator = new FixedPoseEstimator(pose);

        HeadingEstimate heading = estimator.getHeadingEstimate();

        assertTrue(heading.hasHeading);
        assertEquals(1.2, heading.fieldHeadingRad, 0.0);
        assertEquals(0.8, heading.quality, 0.0);
        assertSame(pose.timestamp, heading.timestamp);
        assertEquals(0, estimator.updates);
    }

    @Test
    public void unavailablePoseProjectsUnavailableHeading() {
        FixedPoseEstimator estimator = new FixedPoseEstimator(
                PoseEstimate.noPose(edu.ftcphoenix.fw.core.time.LoopTimestamp.unavailable())
        );

        assertFalse(estimator.getHeadingEstimate().hasHeading);
    }

    private static final class FixedPoseEstimator implements AbsolutePoseEstimator {
        private final PoseEstimate estimate;
        int updates;

        FixedPoseEstimator(PoseEstimate estimate) {
            this.estimate = estimate;
        }

        @Override
        public void update(LoopClock clock) {
            updates++;
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }
    }
}
