package edu.ftcphoenix.fw.drive.guidance;

import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveOverlay;
import edu.ftcphoenix.fw.drive.DriveOverlayOutput;
import edu.ftcphoenix.fw.drive.DriveOverlayStack;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;

/** Verifies cycle ownership for the built-in stateful pose-lock overlay. */
public final class PoseLockOverlayCycleSafetyTest {

    @Test
    public void repeatedReadsShareOneSuccessfulPoseSamplePerCycle() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingPoseEstimator estimator = new RecordingPoseEstimator();
        estimator.setPose(3.0, time);
        DriveOverlay poseLock = DriveGuidance.poseLock(estimator);
        poseLock.onEnable(time.clock());

        DriveOverlayOutput first = poseLock.get(time.clock());
        DriveOverlayOutput repeated = poseLock.get(time.clock());

        assertSame(first, repeated);
        assertEquals(2, estimator.getEstimateCount);

        time.nextCycle(0.02);
        DriveOverlayOutput nextCycle = poseLock.get(time.clock());
        assertNotSame(first, nextCycle);
        assertEquals(3, estimator.getEstimateCount);

        poseLock.onEnable(time.clock());
        DriveOverlayOutput reenabled = poseLock.get(time.clock());
        assertNotSame(nextCycle, reenabled);
        assertEquals(5, estimator.getEstimateCount);
    }

    @Test
    public void laterStackFailureCannotAdvancePoseLockTwiceInOneCycle() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingPoseEstimator estimator = new RecordingPoseEstimator();
        estimator.setPose(3.0, time);
        DriveOverlay poseLock = DriveGuidance.poseLock(estimator);
        FailsOnceOverlay later = new FailsOnceOverlay();
        BooleanSource enabled = BooleanSource.of(() -> true);
        DriveSource stack = DriveOverlayStack.on(clock -> DriveSignal.zero())
                .add("poseLock", enabled, poseLock)
                .add("later", enabled, later)
                .build();

        assertThrows(IllegalStateException.class, () -> stack.get(time.clock()));

        DriveSignal recovered = stack.get(time.clock());
        assertSame(recovered, stack.get(time.clock()));
        assertEquals(2, estimator.getEstimateCount);
        assertEquals(1, later.enableCount);
        assertEquals(2, later.getCount);
    }

    private static final class RecordingPoseEstimator implements AbsolutePoseEstimator {
        private PoseEstimate estimate;
        private int getEstimateCount;

        void setPose(double fieldXInches, ManualLoopClock time) {
            estimate = new PoseEstimate(
                    new Pose3d(fieldXInches, 0.0, 0.0, 0.0, 0.0, 0.0),
                    true,
                    1.0,
                    time.clock().nowTimestamp()
            );
        }

        @Override
        public void update(LoopClock clock) {
            // The test controls the immutable snapshot directly.
        }

        @Override
        public PoseEstimate getEstimate() {
            getEstimateCount++;
            return estimate;
        }
    }

    private static final class FailsOnceOverlay implements DriveOverlay {
        private int enableCount;
        private int getCount;
        private boolean shouldFail = true;

        @Override
        public void onEnable(LoopClock clock) {
            enableCount++;
        }

        @Override
        public DriveOverlayOutput get(LoopClock clock) {
            getCount++;
            if (shouldFail) {
                shouldFail = false;
                throw new IllegalStateException("transient later-layer failure");
            }
            return DriveOverlayOutput.zero();
        }
    }
}
