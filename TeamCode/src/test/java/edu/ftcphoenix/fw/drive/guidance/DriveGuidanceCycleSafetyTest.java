package edu.ftcphoenix.fw.drive.guidance;

import org.junit.Test;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcphoenix.fw.spatial.FacingSolution;
import edu.ftcphoenix.fw.spatial.SpatialControlFrames;
import edu.ftcphoenix.fw.spatial.SpatialLaneResult;
import edu.ftcphoenix.fw.spatial.SpatialQuerySpec;
import edu.ftcphoenix.fw.spatial.SpatialSolveLane;
import edu.ftcphoenix.fw.spatial.SpatialSolveRequest;
import edu.ftcphoenix.fw.spatial.SpatialSolveSet;
import edu.ftcphoenix.fw.spatial.SpatialTargets;
import edu.ftcphoenix.fw.spatial.TranslationSolution;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies cycle-safe state advancement and reset boundaries in Drive Guidance. */
public final class DriveGuidanceCycleSafetyTest {

    @Test
    public void adaptiveBlendAdvancesOnlyOnceForRepeatedSameCycleReads() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingLane localization = RecordingLane.localization();
        RecordingLane aprilTags = RecordingLane.aprilTagsInRange();
        DriveGuidanceCore core = new DriveGuidanceCore(adaptivePlan(time, localization, aprilTags));
        core.onEnable();

        time.nextCycle(0.03);
        DriveGuidanceCore.Step first = core.step(time.clock(), DriveOverlayMask.ALL);
        DriveGuidanceCore.Step repeated = core.step(time.clock(), DriveOverlayMask.ALL);

        assertSame(first, repeated);
        assertEquals(0.20, first.blendTTranslate, 1e-9);
        assertEquals(0.20, first.blendTOmega, 1e-9);
        assertEquals(1, localization.solveCount);
        assertEquals(1, aprilTags.solveCount);
    }

    @Test
    public void differentMasksInOneCycleFailWithRecoveryOptions() {
        ManualLoopClock time = new ManualLoopClock();
        DriveGuidanceQuery query = adaptivePlan(
                time,
                RecordingLane.localization(),
                RecordingLane.aprilTagsInRange()
        ).query();

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
        RecordingLane aprilTags = RecordingLane.aprilTagsInRange();
        DriveGuidancePlan plan = adaptivePlan(time, localization, aprilTags);
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
        assertEquals(2, aprilTags.solveCount);
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
    public void explicitQueryResetClearsOwnedBlendAndSameCycleCaches() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingLane localization = RecordingLane.localization();
        RecordingLane aprilTags = RecordingLane.aprilTagsInRange();
        DriveGuidanceQuery query = adaptivePlan(time, localization, aprilTags).query();

        time.nextCycle(0.03);
        query.sample(time.clock());
        time.nextCycle(0.03);
        DriveGuidanceStatus beforeReset = query.sample(time.clock());
        assertEquals(0.40, beforeReset.blendTTranslate, 1e-9);

        query.reset();
        DriveGuidanceStatus afterReset = query.sample(time.clock());

        assertNotSame(beforeReset, afterReset);
        assertEquals(0.20, afterReset.blendTTranslate, 1e-9);
        assertEquals(3, localization.solveCount);
        assertEquals(3, aprilTags.solveCount);
    }

    @Test
    public void clockResetInvalidatesCycleCachesWithoutResettingGuidanceState() {
        ManualLoopClock time = new ManualLoopClock();
        RecordingLane localization = RecordingLane.localization();
        RecordingLane aprilTags = RecordingLane.aprilTagsInRange();
        DriveGuidanceQuery query = adaptivePlan(time, localization, aprilTags).query();

        time.nextCycle(0.03);
        DriveGuidanceStatus beforeClockReset = query.sample(time.clock());
        assertEquals(0.20, beforeClockReset.blendTTranslate, 1e-9);

        time.clock().reset(time.clock().nowSec());
        DriveGuidanceStatus afterClockReset = query.sample(time.clock());

        assertNotSame(beforeClockReset, afterClockReset);
        assertEquals(0.20, afterClockReset.blendTTranslate, 1e-9);
        assertEquals(2, localization.solveCount);
        assertEquals(2, aprilTags.solveCount);
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
                    .localizationOnlyWithDefaults(estimator)
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

    private static DriveGuidancePlan adaptivePlan(ManualLoopClock time,
                                                  RecordingLane localization,
                                                  RecordingLane aprilTags) {
        SpatialTargets.FieldPoint target = SpatialTargets.fieldPoint(12.0, 2.0);
        SpatialControlFrames frames = SpatialControlFrames.robotCenter();
        SpatialSolveSet solveSet = SpatialSolveSet.builder()
                .add(localization)
                .add(aprilTags)
                .build();
        SpatialQuerySpec spatialSpec = SpatialQuerySpec.builder()
                .translateTo(target)
                .andFaceTo(target)
                .controlFrames(frames)
                .solveWith(solveSet)
                .build();
        DriveGuidanceSpec.ResolveWith resolveWith = DriveGuidanceSpec.ResolveWith.create(
                DriveGuidanceSpec.SolveMode.ADAPTIVE,
                new DriveGuidanceSpec.AprilTags(NO_TAGS, CameraMountConfig.identity()),
                new DriveGuidanceSpec.Localization(
                        new FixedPoseEstimator(PoseEstimate.noPose(time.clock().nowTimestamp()))
                ),
                null,
                new DriveGuidanceSpec.TranslationTakeover(9.0, 12.0, 0.15),
                DriveGuidanceSpec.OmegaPolicy.PREFER_APRIL_TAGS_WHEN_VALID,
                DriveGuidanceSpec.LossPolicy.PASS_THROUGH
        );
        return new DriveGuidancePlan(
                new DriveGuidanceSpec(target, target, frames, resolveWith, spatialSpec, 0, 1),
                DriveGuidancePlan.Tuning.defaults()
        );
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
        DriveGuidanceSpec.Localization localization = new DriveGuidanceSpec.Localization(
                new FixedPoseEstimator(PoseEstimate.noPose(time.clock().nowTimestamp()))
        );
        DriveGuidanceSpec.ResolveWith resolveWith = DriveGuidanceSpec.ResolveWith.create(
                DriveGuidanceSpec.SolveMode.LOCALIZATION_ONLY,
                null,
                localization,
                null,
                null,
                null,
                DriveGuidanceSpec.LossPolicy.PASS_THROUGH
        );
        return new DriveGuidancePlan(
                new DriveGuidanceSpec(target, target, frames, resolveWith, spatialSpec, 0, -1),
                DriveGuidancePlan.Tuning.defaults()
        );
    }

    private static final AprilTagSensor NO_TAGS = new AprilTagSensor() {
        @Override
        public AprilTagDetections get(LoopClock clock) {
            return AprilTagDetections.none();
        }
    };

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

        static RecordingLane aprilTagsInRange() {
            return new RecordingLane(20.0, true, 5.0);
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
