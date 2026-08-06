package edu.ftcphoenix.robots.phoenix;

import org.junit.Test;

import java.util.Collections;
import java.util.Set;

import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies Phoenix's ordinary Source graph and successful-only targeting publication. */
public final class ScoringTargetingSourceTest {

    @Test
    public void lateCalculationFailureDoesNotAdvanceAimReadinessAndCanRetrySameCycle() {
        PhoenixProfile profile = PhoenixProfile.current();
        PhoenixProfile.AutoAimConfig autoAim = profile.autoAim.copy();
        autoAim.aimReadyDebounceSec = 0.5;

        int scoringTagId = autoAim.scoringTagIds().iterator().next();
        CurrentFrameAprilTagSensor tagSensor = new CurrentFrameAprilTagSensor(scoringTagId);
        LateFailingTagLayout fieldLayout = new LateFailingTagLayout(
                profile.field.fixedAprilTagLayout
        );
        ScoringTargeting targeting = new ScoringTargeting(
                autoAim,
                profile.localization.aprilTags.fieldPoseSolver.copy(),
                tagSensor,
                CameraMountConfig.identity(),
                new NoPoseEstimator(),
                fieldLayout,
                BooleanSource.constant(true),
                BooleanSource.constant(false),
                autoAim.shotVelocityTable
        );

        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        clock.update(0.25);
        RuntimeException injectedFailure =
                new RuntimeException("injected late field-layout failure");
        fieldLayout.failNextDirectHas(injectedFailure);

        try {
            targeting.status(clock);
            fail("expected injected late targeting-calculation failure");
        } catch (RuntimeException actual) {
            assertSame(injectedFailure, actual);
        }

        clock.update(0.5);
        ScoringTargeting.Status recovered = targeting.status(clock);
        assertTrue(recovered.selection.hasSelection);
        assertTrue(recovered.aimStatus.hasOmegaError);
        assertTrue(recovered.aimStatus.omegaWithin(Math.toRadians(autoAim.aimReadyToleranceDeg)));
        assertFalse("the failed prior cycle must not contribute its dt to readiness", recovered.aimReady);
        assertSame(recovered, targeting.status(clock));

        clock.update(0.75);
        assertTrue("two successful ready cycles should satisfy the configured debounce",
                targeting.status(clock).aimReady);

        targeting.reset();
        clock.update(1.0);
        RuntimeException sameCycleFailure =
                new RuntimeException("injected late same-cycle retry failure");
        fieldLayout.failNextDirectHas(sameCycleFailure);
        try {
            targeting.status(clock);
            fail("expected injected late same-cycle retry failure");
        } catch (RuntimeException actual) {
            assertSame(sameCycleFailure, actual);
        }

        ScoringTargeting.Status sameCycleRecovered = targeting.status(clock);
        assertTrue(sameCycleRecovered.selection.hasSelection);
        assertTrue(sameCycleRecovered.aimStatus.hasOmegaError);
        assertFalse(sameCycleRecovered.aimReady);
        assertSame(sameCycleRecovered, targeting.status(clock));
    }

    @Test
    public void failedCalculationRetriesWithoutPublishingDefaultAndBorrowedResetStopsAtView() {
        PhoenixProfile profile = PhoenixProfile.current();
        RuntimeException injectedFailure = new RuntimeException("injected override read failure");
        int[] overrideReads = {0};
        BooleanSource failOnceOverride = BooleanSource.of(() -> {
            overrideReads[0]++;
            if (overrideReads[0] == 1) {
                throw injectedFailure;
            }
            return false;
        });

        ScoringTargeting targeting = new ScoringTargeting(
                profile.autoAim,
                profile.localization.aprilTags.fieldPoseSolver.copy(),
                new EmptyAprilTagSensor(),
                CameraMountConfig.identity(),
                new NoPoseEstimator(),
                profile.field.fixedAprilTagLayout,
                BooleanSource.constant(true),
                failOnceOverride,
                profile.autoAim.shotVelocityTable
        );
        LoopClock clock = new LoopClock();
        clock.update(1.0);

        try {
            targeting.status(clock);
            fail("expected injected targeting input failure");
        } catch (RuntimeException actual) {
            assertSame(injectedFailure, actual);
        }

        ScoringTargeting.Status recovered = targeting.status(clock);
        assertTrue(recovered.autoAimEnabled);
        assertFalse(recovered.aimOverride);
        assertEquals(2, overrideReads[0]);
        assertSame(recovered, targeting.status(clock));

        targeting.aimOkToShootSource().reset();
        assertSame(recovered, targeting.status(clock));
        assertEquals(2, overrideReads[0]);

        targeting.reset();
        ScoringTargeting.Status afterOwnerReset = targeting.status(clock);
        assertNotSame(recovered, afterOwnerReset);
        assertTrue(afterOwnerReset.autoAimEnabled);
        assertEquals(3, overrideReads[0]);
    }

    private static final class CurrentFrameAprilTagSensor implements AprilTagSensor {
        private final int tagId;
        private long lastCycle = Long.MIN_VALUE;
        private AprilTagDetections last = AprilTagDetections.none();

        CurrentFrameAprilTagSensor(int tagId) {
            this.tagId = tagId;
        }

        @Override
        public AprilTagDetections get(LoopClock clock) {
            if (clock.cycle() != lastCycle) {
                last = AprilTagDetections.fromFrame(
                        clock.nowTimestamp(),
                        Collections.singletonList(AprilTagObservation.target(
                                tagId,
                                new Pose3d(36.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                        ))
                );
                lastCycle = clock.cycle();
            }
            return last;
        }
    }

    private static final class LateFailingTagLayout implements TagLayout {
        private final TagLayout delegate;
        private RuntimeException nextDirectHasFailure;

        LateFailingTagLayout(TagLayout delegate) {
            this.delegate = delegate;
        }

        void failNextDirectHas(RuntimeException failure) {
            nextDirectHasFailure = failure;
        }

        @Override
        public Pose3d getFieldToTagPose(int id) {
            return delegate.getFieldToTagPose(id);
        }

        @Override
        public Set<Integer> ids() {
            return delegate.ids();
        }

        @Override
        public boolean has(int id) {
            if (nextDirectHasFailure != null) {
                RuntimeException failure = nextDirectHasFailure;
                nextDirectHasFailure = null;
                throw failure;
            }
            return getFieldToTagPose(id) != null;
        }
    }

    /** Named test boundary adapter; direct Source implementation is not an ordinary robot recipe. */
    private static final class EmptyAprilTagSensor implements AprilTagSensor {
        @Override
        public AprilTagDetections get(LoopClock clock) {
            return AprilTagDetections.none();
        }
    }

    private static final class NoPoseEstimator implements AbsolutePoseEstimator {
        private final PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());

        @Override
        public void update(LoopClock clock) {
            // No stateful localization is needed for this targeting-source test.
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }
    }
}
