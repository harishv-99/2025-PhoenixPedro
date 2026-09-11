package edu.ftcsushi.fw.drive.guidance;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.DriveOverlay;
import edu.ftcsushi.fw.drive.DriveOverlayMask;
import edu.ftcsushi.fw.drive.DriveOverlayOutput;
import edu.ftcsushi.fw.drive.DriveOverlayStack;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

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

    @Test
    public void invalidPoseOrQualityAtEnableCannotCaptureALaterImplicitTarget() {
        for (int bad = 0; bad < 10; bad++) {
            ManualLoopClock time = new ManualLoopClock();
            RecordingPoseEstimator estimator = new RecordingPoseEstimator();
            estimator.estimate = invalidEstimate(bad, time.clock());
            DriveOverlay lock = DriveGuidance.poseLock(estimator);
            lock.onEnable(time.clock());
            assertEquals(DriveOverlayMask.NONE, lock.get(time.clock()).mask);

            time.nextCycle(0.02);
            estimator.setPose(10, time);
            assertEquals(DriveOverlayMask.NONE, lock.get(time.clock()).mask);
            lock.onEnable(time.clock());
            assertEquals(DriveOverlayMask.ALL, lock.get(time.clock()).mask);
        }
    }

    @Test
    public void invalidActiveEvidencePassesThroughAndRecoveryKeepsCapturedTarget() {
        for (int bad = 0; bad < 10; bad++) {
            ManualLoopClock time = new ManualLoopClock();
            RecordingPoseEstimator estimator = new RecordingPoseEstimator();
            estimator.setPose(3, time);
            DriveOverlay lock = DriveGuidance.poseLock(estimator);
            lock.onEnable(time.clock());
            time.nextCycle(0.02);
            estimator.estimate = invalidEstimate(bad, time.clock());
            DriveOverlayOutput unavailable = lock.get(time.clock());
            assertEquals(DriveOverlayMask.NONE, unavailable.mask);
            assertSame(DriveSignal.zero(), unavailable.signal);
            assertSame(unavailable, lock.get(time.clock()));

            time.nextCycle(0.02);
            estimator.setPose(10, time);
            DriveOverlayOutput recovered = lock.get(time.clock());
            assertEquals(DriveOverlayMask.ALL, recovered.mask);
            assertTrue(recovered.signal.axial < 0); // Still targets x=3, not the later x=10.
        }
    }

    @Test
    public void staleActivationRequiresReenableAndStaleFeedbackDoesNotCommand() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingPoseEstimator estimator = new RecordingPoseEstimator();
        estimator.setPose(99, time);
        time.nextCycle(0.51);
        DriveOverlay lock = DriveGuidance.poseLock(estimator);
        lock.onEnable(time.clock());
        assertEquals(DriveOverlayMask.NONE, lock.get(time.clock()).mask);
        time.nextCycle(0.02);
        estimator.setPose(3, time);
        assertEquals(DriveOverlayMask.NONE, lock.get(time.clock()).mask);
        lock.onEnable(time.clock());
        assertEquals(DriveOverlayMask.ALL, lock.get(time.clock()).mask);
        time.nextCycle(0.51);
        assertEquals(DriveOverlayMask.NONE, lock.get(time.clock()).mask);
    }

    @Test
    public void finitePosesWithOverflowingDifferenceDoNotProduceGuidance() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingPoseEstimator estimator = new RecordingPoseEstimator();
        estimator.setPose(Double.MAX_VALUE, time);
        DriveOverlay lock = DriveGuidance.poseLock(estimator);
        lock.onEnable(time.clock());
        time.nextCycle(0.02);
        estimator.setPose(-Double.MAX_VALUE, time);
        DriveOverlayOutput unavailable = lock.get(time.clock());
        assertEquals(DriveOverlayMask.NONE, unavailable.mask);
        assertSame(DriveSignal.zero(), unavailable.signal);
    }

    private static PoseEstimate invalidEstimate(int which, LoopClock clock) {
        double x = 99, y = 0, z = 0, yaw = 0, pitch = 0, roll = 0, quality = 1;
        switch (which) {
            case 0: x = Double.NaN; break;
            case 1: y = Double.POSITIVE_INFINITY; break;
            case 2: z = Double.NaN; break;
            case 3: yaw = Double.NaN; break;
            case 4: pitch = Double.POSITIVE_INFINITY; break;
            case 5: roll = Double.NaN; break;
            case 6: quality = Double.NaN; break;
            case 7: quality = Double.POSITIVE_INFINITY; break;
            case 8: quality = 1.01; break;
            case 9: quality = -0.01; break;
            default: throw new AssertionError("Unknown invalid test case");
        }
        return new PoseEstimate(new Pose3d(x, y, z, yaw, pitch, roll), true, quality,
                clock.nowTimestamp());
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
