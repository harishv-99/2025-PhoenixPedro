package edu.ftcsushi.fw.drive.guidance;

import org.junit.Test;

import java.util.LinkedHashMap;
import java.util.Map;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveOverlayMask;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.field.SimpleTagLayout;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the validated construction-time contract for autonomous guidance settings. */
public final class DriveGuidanceTaskConfigTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void publicDefaultsRemainExactAndNullConfigUsesThem() {
        DriveGuidanceTask.Config defaults = new DriveGuidanceTask.Config();

        assertEquals(1.5, defaults.positionTolInches, EPSILON);
        assertEquals(Math.toRadians(6.0), defaults.headingTolRad, EPSILON);
        assertEquals(3.0, defaults.timeoutSec, EPSILON);
        assertEquals(0.35, defaults.maxNoGuidanceSec, EPSILON);
        assertNull(defaults.requestedMask);

        ManualLoopClock clock = new ManualLoopClock();
        MutablePoseEstimator estimator = availableEstimator(clock, Pose3d.zero());
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask task = exactPosePlan(estimator).task(drive, null);

        task.start(clock.clock());
        task.update(clock.clock());

        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(1, drive.driveCount);
    }

    @Test
    public void tolerancesRejectEveryNonfiniteOrNegativeValueBeforeEffects() {
        assertInvalidValues(
                "positionTolInches",
                "finite and >= 0",
                new double[]{Double.NaN, Double.POSITIVE_INFINITY,
                        Double.NEGATIVE_INFINITY, -0.01},
                new ConfigAnswer() {
                    @Override
                    public void set(DriveGuidanceTask.Config config, double value) {
                        config.positionTolInches = value;
                    }
                }
        );
        assertInvalidValues(
                "headingTolRad",
                "finite and >= 0",
                new double[]{Double.NaN, Double.POSITIVE_INFINITY,
                        Double.NEGATIVE_INFINITY, -0.01},
                new ConfigAnswer() {
                    @Override
                    public void set(DriveGuidanceTask.Config config, double value) {
                        config.headingTolRad = value;
                    }
                }
        );
    }

    @Test
    public void timeoutsRejectEveryNonfiniteOrNonpositiveValueBeforeEffects() {
        double[] invalid = new double[]{Double.NaN, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY, -0.01, 0.0};
        assertInvalidValues(
                "timeoutSec",
                "finite and > 0",
                invalid,
                new ConfigAnswer() {
                    @Override
                    public void set(DriveGuidanceTask.Config config, double value) {
                        config.timeoutSec = value;
                    }
                }
        );
        assertInvalidValues(
                "maxNoGuidanceSec",
                "finite and > 0",
                invalid,
                new ConfigAnswer() {
                    @Override
                    public void set(DriveGuidanceTask.Config config, double value) {
                        config.maxNoGuidanceSec = value;
                    }
                }
        );
    }

    @Test
    public void zeroTolerancesRejectNonzeroErrorUntilThePoseIsExact() {
        ManualLoopClock clock = new ManualLoopClock();
        MutablePoseEstimator estimator = availableEstimator(clock, Pose3d.zero());
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask.Config config = new DriveGuidanceTask.Config();
        config.positionTolInches = 0.0;
        config.headingTolRad = 0.0;

        DriveGuidanceTask task = posePlan(estimator, 0.01, 0.01).task(drive, config);
        task.start(clock.clock());
        task.update(clock.clock());

        assertFalse(task.isComplete());

        clock.nextCycle(0.02);
        estimator.setAvailable(
                new Pose3d(0.01, 0.0, 0.0, 0.01, 0.0, 0.0),
                clock.clock().nowTimestamp()
        );
        task.update(clock.clock());

        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
    }

    @Test
    public void positionToleranceSnapshotDiffersFromDefaultAndLaterMutation() {
        ManualLoopClock clock = new ManualLoopClock();
        MutablePoseEstimator estimator = availableEstimator(clock, Pose3d.zero());
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask.Config config = new DriveGuidanceTask.Config();
        config.positionTolInches = 0.5;

        DriveGuidanceTask task = translationPlan(estimator, 1.0).task(drive, config);
        config.positionTolInches = 2.0;

        task.start(clock.clock());
        task.update(clock.clock());
        assertFalse("the retained 0.5-inch tolerance must reject 1 inch of error",
                task.isComplete());

        clock.nextCycle(0.02);
        estimator.setAvailable(
                new Pose3d(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                clock.clock().nowTimestamp()
        );
        task.update(clock.clock());

        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
    }

    @Test
    public void headingToleranceSnapshotDiffersFromDefaultAndLaterMutation() {
        ManualLoopClock clock = new ManualLoopClock();
        MutablePoseEstimator estimator = availableEstimator(clock, Pose3d.zero());
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask.Config config = new DriveGuidanceTask.Config();
        config.headingTolRad = 0.05;

        DriveGuidanceTask task = headingPlan(estimator, 0.08).task(drive, config);
        config.headingTolRad = 0.2;

        task.start(clock.clock());
        task.update(clock.clock());
        assertFalse("the retained 0.05-radian tolerance must reject 0.08 radians of error",
                task.isComplete());

        clock.nextCycle(0.02);
        estimator.setAvailable(
                new Pose3d(0.0, 0.0, 0.0, 0.08, 0.0, 0.0),
                clock.clock().nowTimestamp()
        );
        task.update(clock.clock());

        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
    }

    @Test
    public void callerMutationCannotDriftCompletionMaskOrDebugSettings() {
        ManualLoopClock clock = new ManualLoopClock();
        MutablePoseEstimator estimator = availableEstimator(clock, Pose3d.zero());
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask.Config config = new DriveGuidanceTask.Config();
        config.positionTolInches = 0.5;
        config.headingTolRad = 0.2;
        config.timeoutSec = 5.0;
        config.maxNoGuidanceSec = 0.75;
        config.requestedMask = DriveOverlayMask.ALL;

        DriveGuidanceTask task = offsetPosePlan(estimator).task(drive, config);

        config.positionTolInches = 0.0;
        config.headingTolRad = 0.0;
        config.timeoutSec = Double.NaN;
        config.maxNoGuidanceSec = 0.01;
        config.requestedMask = DriveOverlayMask.OMEGA_ONLY;

        task.start(clock.clock());
        task.update(clock.clock());

        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(1, drive.driveCount);

        RecordingDebugSink debug = new RecordingDebugSink();
        task.debugDump(debug, "guidance");
        assertEquals(
                0.75,
                ((Double) debug.values.get("guidance.maxNoGuidanceSec")).doubleValue(),
                EPSILON
        );

        try {
            offsetPosePlan(estimator).task(new RecordingDriveSink(), config);
            fail("A later Task must validate the caller's later Config values");
        } catch (IllegalArgumentException expected) {
            assertEquals(
                    "DriveGuidanceTask.Config.timeoutSec must be finite and > 0, got NaN.",
                    expected.getMessage()
            );
        }
    }

    @Test
    public void snapshottedTimeoutsKeepStrictExceededBoundary() {
        ManualLoopClock clock = new ManualLoopClock();
        MutablePoseEstimator unavailable = unavailableEstimator(clock);
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask.Config config = new DriveGuidanceTask.Config();
        config.timeoutSec = 5.0;
        config.maxNoGuidanceSec = 0.5;

        DriveGuidanceTask task = translationPlan(unavailable, 12.0).task(drive, config);
        config.timeoutSec = 0.01;
        config.maxNoGuidanceSec = 0.01;

        task.start(clock.clock());
        clock.nextCycle(0.5);
        task.update(clock.clock());
        assertFalse(task.isComplete());

        clock.nextCycle(0.001);
        task.update(clock.clock());
        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
    }

    @Test
    public void overallTimeoutStillRequiresTheDeadlineToBeExceeded() {
        ManualLoopClock clock = new ManualLoopClock();
        MutablePoseEstimator estimator = availableEstimator(clock, Pose3d.zero());
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask.Config config = new DriveGuidanceTask.Config();
        config.timeoutSec = 0.1;
        config.maxNoGuidanceSec = 1.0;

        DriveGuidanceTask task = translationPlan(estimator, 12.0).task(drive, config);
        task.start(clock.clock());

        clock.nextCycle(0.1);
        task.update(clock.clock());
        assertFalse(task.isComplete());

        clock.nextCycle(0.001);
        task.update(clock.clock());
        assertTrue(task.isComplete());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
    }

    @Test
    public void requestedMaskIsSnapshottedWithoutChangingNonePolicy() {
        ManualLoopClock clock = new ManualLoopClock();
        MutablePoseEstimator estimator = availableEstimator(clock, Pose3d.zero());
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask.Config config = new DriveGuidanceTask.Config();
        config.requestedMask = DriveOverlayMask.NONE;

        DriveGuidanceTask task = translationPlan(estimator, 12.0).task(drive, config);
        config.requestedMask = DriveOverlayMask.ALL;
        task.start(clock.clock());
        task.update(clock.clock());

        assertFalse(task.isComplete());
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        assertEquals(0, drive.driveCount);
    }

    @Test
    public void allGoToPoseFactoriesShareTheConfigValidationBoundary() {
        ManualLoopClock clock = new ManualLoopClock();
        MutablePoseEstimator estimator = availableEstimator(clock, Pose3d.zero());
        RecordingDriveSink drive = new RecordingDriveSink();
        DriveGuidanceTask.Config invalid = new DriveGuidanceTask.Config();
        invalid.timeoutSec = 0.0;
        DriveGuidancePlan.Tuning tuning = DriveGuidancePlan.Tuning.defaults();
        SimpleTagLayout layout = new SimpleTagLayout()
                .addPose(7, Pose3d.zero());

        expectInvalidTimeout(new TaskFactory() {
            @Override
            public Task create() {
                return GoToPoseTasks.goToPoseFieldRelative(
                        estimator, drive, Pose2d.zero(), tuning, invalid);
            }
        });
        expectInvalidTimeout(new TaskFactory() {
            @Override
            public Task create() {
                return GoToPoseTasks.goToPoseTagRelative(
                        estimator, drive, layout, 7, 0.0, 0.0, 0.0, tuning, invalid);
            }
        });
        expectInvalidTimeout(new TaskFactory() {
            @Override
            public Task create() {
                return GoToPoseTasks.holdPositionAndAimFieldHeading(
                        estimator, drive, 0.0, tuning, invalid);
            }
        });
        expectInvalidTimeout(new TaskFactory() {
            @Override
            public Task create() {
                return GoToPoseTasks.aimOnlyFieldHeading(
                        estimator, drive, 0.0, tuning, invalid);
            }
        });

        assertEquals(0, drive.updateCount);
        assertEquals(0, drive.driveCount);
        assertEquals(0, drive.stopCount);
    }

    private static void assertInvalidValues(String fieldName,
                                            String constraint,
                                            double[] values,
                                            ConfigAnswer answer) {
        ManualLoopClock clock = new ManualLoopClock();
        MutablePoseEstimator estimator = availableEstimator(clock, Pose3d.zero());
        DriveGuidancePlan plan = translationPlan(estimator, 12.0);

        for (double value : values) {
            RecordingDriveSink drive = new RecordingDriveSink();
            DriveGuidanceTask.Config config = new DriveGuidanceTask.Config();
            answer.set(config, value);

            try {
                plan.task(drive, config);
                fail("Expected invalid " + fieldName + " value to fail: " + value);
            } catch (IllegalArgumentException expected) {
                assertEquals(
                        "DriveGuidanceTask.Config." + fieldName + " must be " + constraint
                                + ", got " + value + ".",
                        expected.getMessage()
                );
            }

            assertEquals(0, drive.updateCount);
            assertEquals(0, drive.driveCount);
            assertEquals(0, drive.stopCount);
            assertEquals(0, estimator.getEstimateCount);
        }
    }

    private static void expectInvalidTimeout(TaskFactory factory) {
        try {
            factory.create();
            fail("Expected invalid timeoutSec to fail");
        } catch (IllegalArgumentException expected) {
            assertEquals(
                    "DriveGuidanceTask.Config.timeoutSec must be finite and > 0, got 0.0.",
                    expected.getMessage()
            );
        }
    }

    private static DriveGuidancePlan translationPlan(AbsolutePoseEstimator estimator,
                                                      double targetXInches) {
        return DriveGuidance.plan()
                .translateTo()
                    .fieldPointInches(targetXInches, 0.0)
                .solveWith()
                    .localizationOnlyWithDefaults(estimator)
                .build();
    }

    private static DriveGuidancePlan exactPosePlan(AbsolutePoseEstimator estimator) {
        return posePlan(estimator, 0.0, 0.0);
    }

    private static DriveGuidancePlan posePlan(AbsolutePoseEstimator estimator,
                                              double targetXInches,
                                              double targetHeadingRad) {
        return DriveGuidance.plan()
                .translateTo()
                    .fieldPointInches(targetXInches, 0.0)
                .andFaceTo()
                    .fieldHeadingRad(targetHeadingRad)
                .solveWith()
                    .localizationOnlyWithDefaults(estimator)
                .build();
    }

    private static DriveGuidancePlan headingPlan(AbsolutePoseEstimator estimator,
                                                  double targetHeadingRad) {
        return DriveGuidance.plan()
                .faceTo()
                    .fieldHeadingRad(targetHeadingRad)
                .solveWith()
                    .localizationOnlyWithDefaults(estimator)
                .build();
    }

    private static DriveGuidancePlan offsetPosePlan(AbsolutePoseEstimator estimator) {
        return DriveGuidance.plan()
                .translateTo()
                    .fieldPointInches(0.25, 0.0)
                .andFaceTo()
                    .fieldHeadingRad(0.1)
                .solveWith()
                    .localizationOnlyWithDefaults(estimator)
                .build();
    }

    private static MutablePoseEstimator availableEstimator(ManualLoopClock clock, Pose3d pose) {
        return new MutablePoseEstimator(new PoseEstimate(
                pose,
                true,
                1.0,
                clock.clock().nowTimestamp()
        ));
    }

    private static MutablePoseEstimator unavailableEstimator(ManualLoopClock clock) {
        return new MutablePoseEstimator(PoseEstimate.noPose(clock.clock().nowTimestamp()));
    }

    private interface ConfigAnswer {
        void set(DriveGuidanceTask.Config config, double value);
    }

    private interface TaskFactory {
        Task create();
    }

    private static final class MutablePoseEstimator implements AbsolutePoseEstimator {
        private PoseEstimate estimate;
        private int getEstimateCount;

        MutablePoseEstimator(PoseEstimate estimate) {
            this.estimate = estimate;
        }

        void setAvailable(Pose3d pose, LoopTimestamp timestamp) {
            estimate = new PoseEstimate(pose, true, 1.0, timestamp);
        }

        @Override
        public void update(LoopClock clock) {
            // The test controls snapshots directly.
        }

        @Override
        public PoseEstimate getEstimate() {
            getEstimateCount++;
            return estimate;
        }
    }

    private static final class RecordingDriveSink implements DriveCommandSink {
        private int updateCount;
        private int driveCount;
        private int stopCount;

        @Override
        public void update(LoopClock clock) {
            updateCount++;
        }

        @Override
        public void drive(DriveSignal signal) {
            driveCount++;
        }

        @Override
        public void stop() {
            stopCount++;
        }
    }

    private static final class RecordingDebugSink implements DebugSink {
        private final Map<String, Object> values = new LinkedHashMap<String, Object>();

        @Override
        public DebugSink addData(String key, Object value) {
            values.put(key, value);
            return this;
        }

        @Override
        public DebugSink addLine(String text) {
            return this;
        }
    }
}
