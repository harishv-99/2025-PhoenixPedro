package edu.ftcsushi.fw.ftc.localization;

import org.junit.Test;

import java.util.concurrent.atomic.AtomicInteger;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.localization.HeadingEstimate;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public final class FtcImuHeadingEstimatorTest {
    @Test
    public void alignsRawYawToAuthoredInitialFieldHeading() {
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        MutableImu imu = new MutableImu(0.2);
        FtcImuHeadingEstimator estimator = new FtcImuHeadingEstimator(
                imu,
                () -> Math.PI / 2.0
        );

        estimator.start(clock);
        assertEquals(Math.PI / 2.0,
                estimator.getHeadingEstimate().fieldHeadingRad, 1e-9);

        imu.yaw = 0.5;
        clock.update(0.02);
        estimator.update(clock);
        assertEquals(Math.PI / 2.0 + 0.3,
                estimator.getHeadingEstimate().fieldHeadingRad, 1e-9);
    }

    @Test
    public void updateReadsImuAtMostOncePerCycleAndStopIsIdempotent() {
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        AtomicInteger reads = new AtomicInteger();
        FtcImuHeadingEstimator estimator = new FtcImuHeadingEstimator(
                () -> {
                    reads.incrementAndGet();
                    return 0.0;
                },
                () -> 0.0
        );
        estimator.start(clock);
        clock.update(0.02);

        estimator.update(clock);
        HeadingEstimate first = estimator.getHeadingEstimate();
        estimator.update(clock);

        assertEquals(2, reads.get());
        assertTrue(first.hasHeading);
        estimator.stop();
        estimator.stop();
        assertFalse(estimator.getHeadingEstimate().hasHeading);
    }

    @Test(expected = IllegalStateException.class)
    public void updateBeforeStartFailsFast() {
        FtcImuHeadingEstimator estimator = new FtcImuHeadingEstimator(() -> 0.0, () -> 0.0);
        estimator.update(new LoopClock());
    }

    private static final class MutableImu implements FtcImuHeadingEstimator.ImuAccess {
        double yaw;

        MutableImu(double yaw) {
            this.yaw = yaw;
        }

        @Override
        public double yawRad() {
            return yaw;
        }
    }
}
