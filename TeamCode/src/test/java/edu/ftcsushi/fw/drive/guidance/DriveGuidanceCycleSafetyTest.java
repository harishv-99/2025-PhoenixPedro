package edu.ftcsushi.fw.drive.guidance;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.source.TimeAwareSource;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.DriveOverlayMask;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.spatial.FacingSolution;
import edu.ftcsushi.fw.spatial.SpatialControlFrames;
import edu.ftcsushi.fw.spatial.SpatialLaneResult;
import edu.ftcsushi.fw.spatial.SpatialQuerySpec;
import edu.ftcsushi.fw.spatial.SpatialSolveLane;
import edu.ftcsushi.fw.spatial.SpatialSolveRequest;
import edu.ftcsushi.fw.spatial.SpatialSolveSet;
import edu.ftcsushi.fw.spatial.SpatialTargets;
import edu.ftcsushi.fw.spatial.TranslationSolution;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies cycle-safe state advancement and reset boundaries in Drive Guidance. */
public final class DriveGuidanceCycleSafetyTest {

    @Test
    public void singleAuthorityAdvancesOnlyOnceForRepeatedSameCycleReads() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingLane localization = RecordingLane.localization();
        DriveGuidanceCore core = new DriveGuidanceCore(localizationPlan(time, localization));
        core.onEnable();

        time.nextCycle(0.03);
        DriveGuidanceCore.Step first = core.step(time.clock(), DriveOverlayMask.ALL);
        DriveGuidanceCore.Step repeated = core.step(time.clock(), DriveOverlayMask.ALL);

        assertSame(first, repeated);
        assertEquals(DriveGuidanceSpec.SolveMode.ABSOLUTE_POSE, first.solveMode);
        assertEquals(1, localization.solveCount);
    }

    @Test
    public void differentMasksInOneCycleFailWithRecoveryOptions() {
        ManualLoopClock time = new ManualLoopClock();
        DriveGuidanceQuery query = localizationPlan(time, RecordingLane.localization()).query();

        query.sample(time.clock(), DriveOverlayMask.TRANSLATION_ONLY);

        try {
            query.sample(time.clock(), DriveOverlayMask.OMEGA_ONLY);
            fail("Expected different same-cycle masks to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("different requested masks"));
            assertTrue(expected.getMessage().contains("natural mask"));
            assertTrue(expected.getMessage().contains("union mask"));
            assertTrue(expected.getMessage().contains("separate DriveGuidanceQuery"));
        }

        time.nextCycle(0.02);
        assertNotNull(query.sample(time.clock(), DriveOverlayMask.OMEGA_ONLY));
    }

    @Test
    public void independentRuntimesMayUseDifferentMasksInTheSameCycle() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingLane localization = RecordingLane.localization();
        DriveGuidancePlan plan = localizationPlan(time, localization);
        DriveGuidanceQuery translationQuery = plan.query();
        DriveGuidanceQuery omegaQuery = plan.query();

        DriveGuidanceStatus translation =
                translationQuery.sample(time.clock(), DriveOverlayMask.TRANSLATION_ONLY);
        DriveGuidanceStatus omega =
                omegaQuery.sample(time.clock(), DriveOverlayMask.OMEGA_ONLY);

        assertNotNull(translation);
        assertNotNull(omega);
        assertNotSame(translation, omega);
        assertEquals(DriveOverlayMask.TRANSLATION_ONLY, translation.mask);
        assertEquals(DriveOverlayMask.OMEGA_ONLY, omega.mask);
        assertEquals(2, localization.solveCount);
    }

    @Test
    public void failedStatusSampleCanRetryInTheSameCycle() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingLane lane = RecordingLane.localization();
        lane.failuresRemaining = 1;
        Source<DriveGuidanceStatus> status = DriveGuidanceSources.status(
                localizationPlan(time, lane).query()
        );

        try {
            status.get(time.clock());
            fail("Expected the first lane solve to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("transient test failure"));
        }

        DriveGuidanceStatus retried = status.get(time.clock());

        assertNotNull(retried);
        assertSame(retried, status.get(time.clock()));
        assertEquals(2, lane.solveCount);
    }

    @Test
    public void explicitQueryResetClearsSameCycleCaches() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingLane localization = RecordingLane.localization();
        DriveGuidanceQuery query = localizationPlan(time, localization).query();

        time.nextCycle(0.03);
        query.sample(time.clock());
        time.nextCycle(0.03);
        DriveGuidanceStatus beforeReset = query.sample(time.clock());

        query.reset();
        DriveGuidanceStatus afterReset = query.sample(time.clock());

        assertNotSame(beforeReset, afterReset);
        assertEquals(3, localization.solveCount);
    }

    @Test
    public void clockResetInvalidatesCycleCachesWithoutResettingGuidanceState() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingLane localization = RecordingLane.localization();
        DriveGuidanceQuery query = localizationPlan(time, localization).query();

        time.nextCycle(0.03);
        DriveGuidanceStatus beforeClockReset = query.sample(time.clock());

        time.clock().reset(time.clock().nowSec());
        DriveGuidanceStatus afterClockReset = query.sample(time.clock());

        assertNotSame(beforeClockReset, afterClockReset);
        assertEquals(2, localization.solveCount);
    }

    @Test
    public void explicitQueryResetRecapturesRobotRelativeAnchor() {
        ManualLoopClock time = new ManualLoopClock();
        MutablePoseEstimator estimator = new MutablePoseEstimator();
        estimator.setPose(0.0, time.clock().nowTimestamp());
        DriveGuidanceQuery query = DriveGuidance.plan()
                .translateTo()
                    .robotRelativePointInches(4.0, 0.0)
                .solveWith()
                    .absolutePose(estimator).doneAbsolutePose()
                .build()
                .query();

        DriveGuidanceStatus first = query.sample(time.clock());
        assertNotNull(first.fieldToTranslationFrameAnchor);
        assertEquals(0.0, first.fieldToTranslationFrameAnchor.xInches, 1e-9);

        time.nextCycle(0.02);
        estimator.setPose(10.0, time.clock().nowTimestamp());
        DriveGuidanceStatus stillLatched = query.sample(time.clock());
        assertEquals(0.0, stillLatched.fieldToTranslationFrameAnchor.xInches, 1e-9);

        query.reset();
        DriveGuidanceStatus recaptured = query.sample(time.clock());
        assertEquals(10.0, recaptured.fieldToTranslationFrameAnchor.xInches, 1e-9);
    }

    @Test
    public void invalidSixDofPoseOrQualityCannotCaptureRobotRelativeAnchor() {
        Pose3d[] badPoses = {
                new Pose3d(99, 0, Double.NaN, 0, 0, 0),
                new Pose3d(99, 0, 0, 0, Double.POSITIVE_INFINITY, 0),
                new Pose3d(99, 0, 0, 0, 0, Double.NaN),
                new Pose3d(99, 0, 0, 0, 0, 0),
                new Pose3d(99, 0, 0, 0, 0, 0)
        };
        double[] badQualities = {1.0, 1.0, 1.0, 1.01, Double.NaN};
        for (int i = 0; i < badPoses.length; i++) {
            ManualLoopClock time = new ManualLoopClock();
            MutablePoseEstimator estimator = new MutablePoseEstimator();
            estimator.estimate = new PoseEstimate(badPoses[i], true, badQualities[i],
                    time.clock().nowTimestamp());
            DriveGuidanceQuery query = DriveGuidance.plan().translateTo()
                    .robotRelativePointInches(4, 0)
                    .solveWith().absolutePose(estimator).doneAbsolutePose().build().query();
            DriveGuidanceStatus invalid = query.sample(time.clock());
            assertFalse(invalid.hasTranslationError);
            assertNull(invalid.fieldToTranslationFrameAnchor);

            time.nextCycle(0.02);
            estimator.setPose(10, time.clock().nowTimestamp());
            DriveGuidanceStatus valid = query.sample(time.clock());
            assertTrue(valid.hasTranslationError);
            assertEquals(10, valid.fieldToTranslationFrameAnchor.xInches, 1e-9);
            assertEquals(4, valid.forwardErrorIn, 1e-9);
        }
    }

    @Test
    public void illegalFrameCallbackClockChangeCannotCommitFirstAnchor() {
        ManualLoopClock time = new ManualLoopClock();
        MutablePoseEstimator estimator = new MutablePoseEstimator();
        estimator.setPose(99, time.clock().nowTimestamp());
        boolean[] advanceDuringSample = {true};
        TimeAwareSource<Pose2d> frame = (clock, timestamp) -> {
            if (advanceDuringSample[0]) time.nextCycle(0.01);
            return Pose2d.zero();
        };
        DriveGuidanceQuery query = DriveGuidance.plan().translateTo()
                .robotRelativePointInches(4, 0)
                .controlFrames(SpatialControlFrames.robotCenter().withTranslationFrame(frame))
                .solveWith().absolutePose(estimator).doneAbsolutePose().build().query();
        IllegalStateException failure = assertThrows(IllegalStateException.class,
                () -> query.sample(time.clock()));
        assertTrue(failure.getMessage().contains("LoopClock"));
        assertNull(query.last());

        advanceDuringSample[0] = false;
        estimator.setPose(10, time.clock().nowTimestamp());
        DriveGuidanceStatus retried = query.sample(time.clock());
        assertEquals(10, retried.fieldToTranslationFrameAnchor.xInches, 1e-9);
        assertEquals(4, retried.forwardErrorIn, 1e-9);
    }

    private static DriveGuidancePlan localizationPlan(ManualLoopClock time, RecordingLane lane) {
        SpatialTargets.FieldPoint target = SpatialTargets.fieldPoint(12.0, 2.0);
        SpatialControlFrames frames = SpatialControlFrames.robotCenter();
        SpatialQuerySpec spatialSpec = SpatialQuerySpec.builder()
                .translateTo(target)
                .andFaceTo(target)
                .controlFrames(frames)
                .solveWith(SpatialSolveSet.builder().add(lane).build())
                .build();
        DriveGuidanceSpec.AbsolutePose localization = new DriveGuidanceSpec.AbsolutePose(
                new FixedPoseEstimator(PoseEstimate.noPose(time.clock().nowTimestamp())), 0.5, 0.1
        );
        DriveGuidanceSpec.ResolveWith resolveWith = DriveGuidanceSpec.ResolveWith.create(
                DriveGuidanceSpec.SolveMode.ABSOLUTE_POSE,
                null,
                localization,
                null,
                DriveGuidanceSpec.LossPolicy.PASS_THROUGH
        );
        return new DriveGuidancePlan(
                new DriveGuidanceSpec(target, target, frames, resolveWith, spatialSpec),
                DriveGuidancePlan.Tuning.defaults()
        );
    }

    private static final class RecordingLane implements SpatialSolveLane {
        private final double forwardInches;
        private final boolean hasRange;
        private final double rangeInches;
        private int solveCount;
        private int failuresRemaining;

        private RecordingLane(double forwardInches, boolean hasRange, double rangeInches) {
            this.forwardInches = forwardInches;
            this.hasRange = hasRange;
            this.rangeInches = rangeInches;
        }

        static RecordingLane localization() {
            return new RecordingLane(10.0, false, Double.NaN);
        }

        @Override
        public SpatialLaneResult solve(SpatialSolveRequest request) {
            solveCount++;
            if (failuresRemaining > 0) {
                failuresRemaining--;
                throw new IllegalStateException("transient test failure");
            }
            LoopTimestamp timestamp = request.clock.nowTimestamp();
            Pose2d point = new Pose2d(forwardInches, 2.0, 0.0);
            return SpatialLaneResult.of(
                    new TranslationSolution(
                            point,
                            point,
                            hasRange,
                            rangeInches,
                            1.0,
                            timestamp
                    ),
                    new FacingSolution(0.5, 1.0, timestamp),
                    null,
                    null
            );
        }
    }

    private static final class FixedPoseEstimator implements AbsolutePoseEstimator {
        private final PoseEstimate estimate;

        FixedPoseEstimator(PoseEstimate estimate) {
            this.estimate = estimate;
        }

        @Override
        public void update(LoopClock clock) {
            // Fixed test snapshot.
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }
    }

    private static final class MutablePoseEstimator implements AbsolutePoseEstimator {
        private PoseEstimate estimate;

        void setPose(double fieldXInches, LoopTimestamp timestamp) {
            estimate = new PoseEstimate(
                    new Pose3d(fieldXInches, 0.0, 0.0, 0.0, 0.0, 0.0),
                    true,
                    1.0,
                    timestamp
            );
        }

        @Override
        public void update(LoopClock clock) {
            // The test controls the snapshot directly.
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }
    }
}
