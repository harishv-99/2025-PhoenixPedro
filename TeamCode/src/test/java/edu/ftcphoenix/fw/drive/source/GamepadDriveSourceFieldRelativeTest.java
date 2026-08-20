package edu.ftcphoenix.fw.drive.source;

import org.junit.Test;

import java.util.concurrent.atomic.AtomicInteger;

import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.localization.HeadingEstimate;
import edu.ftcphoenix.fw.localization.HeadingEstimator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

public final class GamepadDriveSourceFieldRelativeTest {
    private static final double EPS = 1e-9;

    @Test
    public void authoredUpAndRobotHeadingMayBeOrthogonal() {
        LoopClock clock = clockAt(0.0);
        MutableHeading heading = new MutableHeading();
        heading.publish(clock, Math.PI / 2.0, 1.0);

        DriveSignal signal = source(
                ScalarSource.constant(0.0), ScalarSource.constant(1.0),
                ScalarSource.constant(0.0), heading, () -> 0.0).get(clock);

        assertEquals(0.0, signal.axial, EPS);
        assertEquals(-1.0, signal.lateral, EPS);
    }

    @Test
    public void controlUpIsIndependentFromInitialRobotFacing() {
        LoopClock clock = clockAt(0.0);
        MutableHeading heading = new MutableHeading();
        heading.publish(clock, 0.0, 1.0);

        DriveSignal signal = source(
                ScalarSource.constant(0.0), ScalarSource.constant(1.0),
                ScalarSource.constant(0.0), heading, () -> Math.PI / 2.0).get(clock);

        assertEquals(0.0, signal.axial, EPS);
        assertEquals(1.0, signal.lateral, EPS);
    }

    @Test
    public void staleHeadingStopsTranslationButPreservesOmega() {
        LoopClock clock = clockAt(1.0);
        MutableHeading heading = new MutableHeading();
        heading.estimate = new HeadingEstimate(
                0.0, true, 1.0, clock.timestampSecondsAgo(0.5));
        GamepadDriveSource.Config config = GamepadDriveSource.Config.defaults();
        config.deadband = 0.0;
        DriveSource source = new GamepadDriveSource(
                ScalarSource.constant(0.0), ScalarSource.constant(1.0),
                ScalarSource.constant(0.4), config
        ).fieldRelativeTo(heading, () -> 0.0, 0.1, 0.0);

        DriveSignal signal = source.get(clock);

        assertEquals(0.0, signal.axial, EPS);
        assertEquals(0.0, signal.lateral, EPS);
        assertEquals(-Math.pow(0.4, 1.5), signal.omega, EPS);
    }

    @Test
    public void samplesAxesEvidenceAndConfiguredUpOncePerCycle() {
        LoopClock clock = clockAt(0.0);
        AtomicInteger axisReads = new AtomicInteger();
        AtomicInteger upReads = new AtomicInteger();
        MutableHeading heading = new MutableHeading();
        heading.publish(clock, 0.0, 1.0);
        ScalarSource axis = c -> {
            axisReads.incrementAndGet();
            return 0.0;
        };
        DriveSource source = source(axis, axis, axis, heading, () -> {
            upReads.incrementAndGet();
            return 0.0;
        });

        DriveSignal first = source.get(clock);
        DriveSignal second = source.get(clock);

        assertSame(first, second);
        assertEquals(3, axisReads.get());
        assertEquals(1, heading.reads);
        assertEquals(1, upReads.get());
    }

    @Test
    public void lowQualityAndDifferentClockEvidenceFailClosed() {
        LoopClock clock = clockAt(0.0);
        MutableHeading heading = new MutableHeading();
        heading.publish(clock, 0.0, 0.4);
        DriveSource source = new GamepadDriveSource(
                ScalarSource.constant(0.0), ScalarSource.constant(1.0),
                ScalarSource.constant(0.0), GamepadDriveSource.Config.defaults()
        ).fieldRelativeTo(heading, () -> 0.0, 0.25, 0.5);

        assertEquals(0.0, source.get(clock).axial, EPS);

        LoopClock otherClock = clockAt(0.02);
        clock.update(0.02);
        heading.publish(otherClock, 0.0, 1.0);
        assertEquals(0.0, source.get(clock).axial, EPS);
    }

    @Test
    public void nonFiniteConfiguredUpFailureIsRetainedWithinCycle() {
        LoopClock clock = clockAt(0.0);
        MutableHeading heading = new MutableHeading();
        heading.publish(clock, 0.0, 1.0);
        AtomicInteger reads = new AtomicInteger();
        DriveSource source = source(
                ScalarSource.constant(0.0), ScalarSource.constant(0.0),
                ScalarSource.constant(0.0), heading, () -> {
                    reads.incrementAndGet();
                    return Double.NaN;
                });

        RuntimeException first = captureFailure(() -> source.get(clock));
        RuntimeException second = captureFailure(() -> source.get(clock));

        assertSame(first, second);
        assertEquals(1, reads.get());
    }

    @Test
    public void resetRereadsConfiguredUpAndStructuralAxes() {
        LoopClock clock = clockAt(0.0);
        MutableHeading heading = new MutableHeading();
        heading.publish(clock, 0.0, 1.0);
        AtomicInteger upReads = new AtomicInteger();
        DriveSource source = source(
                ScalarSource.constant(0.0), ScalarSource.constant(0.0),
                ScalarSource.constant(0.0), heading, () -> {
                    upReads.incrementAndGet();
                    return 0.0;
                });
        source.get(clock);
        source.reset();
        clock.update(0.02);
        heading.publish(clock, 0.0, 1.0);
        source.get(clock);

        assertEquals(2, upReads.get());
    }

    @Test
    public void advancedThresholdsRejectInvalidAuthoredValues() {
        MutableHeading heading = new MutableHeading();
        GamepadDriveSource source = rawSource(
                ScalarSource.constant(0.0), ScalarSource.constant(0.0),
                ScalarSource.constant(0.0));

        assertTrue(captureFailure(() -> source.fieldRelativeTo(
                heading, () -> 0.0, -1.0, 0.0)).getMessage().contains("maxHeadingAgeSec"));
        assertTrue(captureFailure(() -> source.fieldRelativeTo(
                heading, () -> 0.0, 0.25, 2.0)).getMessage().contains("minHeadingQuality"));
    }

    private static DriveSource source(ScalarSource lateral,
                                      ScalarSource axial,
                                      ScalarSource omega,
                                      HeadingEstimator heading,
                                      java.util.function.DoubleSupplier up) {
        return rawSource(lateral, axial, omega).fieldRelativeTo(heading, up);
    }

    private static GamepadDriveSource rawSource(ScalarSource lateral,
                                                 ScalarSource axial,
                                                 ScalarSource omega) {
        return new GamepadDriveSource(
                lateral, axial, omega, GamepadDriveSource.Config.defaults());
    }

    private static LoopClock clockAt(double seconds) {
        LoopClock clock = new LoopClock();
        clock.reset(seconds);
        return clock;
    }

    private static RuntimeException captureFailure(Runnable action) {
        try {
            action.run();
        } catch (RuntimeException failure) {
            return failure;
        }
        throw new AssertionError("Expected RuntimeException");
    }

    private static final class MutableHeading implements HeadingEstimator {
        HeadingEstimate estimate;
        int reads;

        void publish(LoopClock clock, double headingRad, double quality) {
            estimate = new HeadingEstimate(
                    headingRad, true, quality, clock.nowTimestamp());
        }

        @Override
        public void update(LoopClock clock) {
        }

        @Override
        public HeadingEstimate getHeadingEstimate() {
            reads++;
            return estimate;
        }
    }
}
