package edu.ftcsushi.robots.phoenix.scoring;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.math.InterpolatingTable1D;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveOverlay;
import edu.ftcsushi.fw.drive.DriveOverlayMask;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcsushi.fw.field.SimpleTagLayout;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.localization.MotionDelta;
import edu.ftcsushi.fw.localization.MotionPredictor;
import edu.ftcsushi.fw.localization.fusion.OdometryCorrectionFusionEstimator;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.robots.phoenix.PhoenixCapabilities;

import static org.junit.Assert.*;

/** Deterministic application policy checks; synthetic poses do not prove physical shot accuracy. */
public final class PhoenixTargetingSourceTest {
    @Test public void cameraToTagRangeUsesRotatedMountAndHeightAndLeavesTableDomainUnchanged() {
        PhoenixTargeting.Config config = config();
        config.shotVelocityTable = InterpolatingTable1D.ofSorted(
                new double[]{20.0, 40.0}, new double[]{1000.0, 1400.0});
        Fixture f = new Fixture(config,
                new SimpleTagLayout().addPose(24, pose(98, 54, 32, 0, 0, 0)),
                CameraMountConfig.of(10, 2, 14, 0.4, -0.2, 0.1));
        f.publish(pose(100, 20, 0, Math.PI / 2, 0, 0), 0.8);
        f.targeting.update(f.clock);
        PhoenixCapabilities.TargetingStatus status = f.targeting.status();
        // Camera origin (98,30,14): tag delta (0,24,18), whose 3D length is 30.
        assertEquals(30.0, status.cameraToTagRange3dInches, 1e-9);
        assertEquals(1200.0, status.suggestedVelocityNative, 1e-9);
        assertTrue(status.hasUsablePose);
        assertSame(f.estimator.estimate.timestamp, status.poseTimestamp);
        assertEquals(24, status.configuredTagId);
        assertEquals(1, f.estimator.reads);
        assertEquals(0, f.estimator.updates);
    }

    @Test public void noCameraVisibilityIsRequiredWhileCorrectedPoseRemainsUsable() {
        Fixture f = new Fixture();
        f.publish(Pose3d.zero(), 0.8);
        f.targeting.update(f.clock);
        assertTrue(f.targeting.status().aimReady);
        assertEquals(36.0, f.targeting.status().cameraToTagRange3dInches, 0.0);
        // The outside localization publication advances during camera occlusion. Targeting has
        // no camera dependency and cannot silently fall back to an image-only aim or range.
        f.clock.update(0.1);
        f.publish(pose(6, 0, 0, 0, 0, 0), 0.8);
        f.targeting.update(f.clock);
        assertTrue(f.targeting.status().hasSuggestedVelocity);
        assertTrue(f.targeting.status().aimReady);
        assertEquals(30.0, f.targeting.status().cameraToTagRange3dInches, 0.0);
        assertEquals(24, f.targeting.status().configuredTagId);
    }

    @Test public void realFusionKeepsTargetingThroughCorrectionLossButNotFrozenPoseEvidence() {
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        RecordingPredictor predictor = new RecordingPredictor();
        predictor.estimate = new PoseEstimate(Pose3d.zero(), true, 0.8, clock.nowTimestamp());
        RecordingPoseEstimator correction = new RecordingPoseEstimator();
        correction.estimate = new PoseEstimate(Pose3d.zero(), true, 1.0, clock.nowTimestamp());
        OdometryCorrectionFusionEstimator.Config fusionConfig =
                OdometryCorrectionFusionEstimator.Config.defaults();
        // Synthetic correction boundary only; no hardware resetter, camera or field calibration.
        fusionConfig.enablePushCorrectedPoseToPredictor = false;
        OdometryCorrectionFusionEstimator localization = new OdometryCorrectionFusionEstimator(
                predictor, correction, fusionConfig);
        PhoenixTargeting targeting = new PhoenixTargeting(config(), CameraMountConfig.identity(),
                localization, layout(), Source.constant(24),
                BooleanSource.constant(true), BooleanSource.constant(false));
        localization.update(clock);
        targeting.update(clock);
        assertEquals(1, localization.getAcceptedCorrectionCount());
        assertTrue(targeting.status().aimReady);
        LoopTimestamp earlier = predictor.estimate.timestamp;
        clock.update(0.1);
        predictor.estimate = new PoseEstimate(pose(6,0,0,0,0,0), true, 0.8, clock.nowTimestamp());
        predictor.delta = new MotionDelta(pose(6,0,0,0,0,0), true, 0.8,
                earlier, predictor.estimate.timestamp);
        correction.estimate = PoseEstimate.noPose(clock.nowTimestamp());
        localization.update(clock);
        targeting.update(clock);
        assertEquals(1, localization.getAcceptedCorrectionCount());
        assertEquals(30.0, targeting.status().cameraToTagRange3dInches, 1e-9);
        assertTrue(targeting.status().aimReady);
        assertTrue(targeting.status().hasSuggestedVelocity);
        clock.update(0.7); // Both inputs now retain old evidence; polling must not refresh it.
        localization.update(clock);
        targeting.update(clock);
        assertFalse(targeting.status().hasUsablePose);
        assertFalse(targeting.status().aimReady);
        assertFalse(targeting.status().hasSuggestedVelocity);
    }

    @Test public void aimRangeOverlayAndTaskReuseOnePublishedPoseInTheCycle() {
        Fixture f = new Fixture();
        f.publish(Pose3d.zero(), 1.0);
        f.targeting.update(f.clock);
        PhoenixCapabilities.TargetingStatus first = f.targeting.status();
        f.estimator.estimate = new PoseEstimate(pose(0, 0, 0, Math.PI / 2, 0, 0),
                true, 1.0, f.clock.nowTimestamp());
        DriveOverlay overlay = f.targeting.aimOverlay();
        overlay.onEnable(f.clock);
        overlay.get(f.clock);
        Task task = f.targeting.aimTask(new RecordingDriveSink(), null);
        task.start(f.clock);
        task.update(f.clock);
        f.targeting.update(f.clock);
        assertSame(first, f.targeting.status());
        assertEquals(0.0, first.aimStatus.omegaErrorRad, 0.0);
        assertEquals(1, f.estimator.reads);
        assertEquals(0, f.estimator.updates);
    }

    @Test public void invalidQualityAvailabilityGeometryAndAgeWithdrawBothAimAndSuggestion() {
        for (double quality : new double[]{Double.NaN, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY, -0.01, 0.099, 1.01}) {
            Fixture f = new Fixture();
            f.publish(Pose3d.zero(), quality);
            assertUnavailable(f);
        }
        for (int index = 0; index < 6; index++) {
            Fixture f = new Fixture();
            double[] values = new double[6];
            values[index] = Double.NaN;
            f.publish(pose(values[0], values[1], values[2], values[3], values[4], values[5]), 1.0);
            assertUnavailable(f);
        }
        Fixture unavailable = new Fixture();
        unavailable.estimator.estimate = PoseEstimate.noPose(unavailable.clock.nowTimestamp());
        assertUnavailable(unavailable);
        Fixture unknownTime = new Fixture();
        unknownTime.estimator.estimate = new PoseEstimate(Pose3d.zero(), true, 1.0,
                LoopTimestamp.unavailable());
        assertUnavailable(unknownTime);
        Fixture stale = new Fixture();
        stale.publish(Pose3d.zero(), 1.0);
        stale.clock.update(0.501);
        assertUnavailable(stale);
        Fixture oldEpoch = new Fixture();
        oldEpoch.publish(Pose3d.zero(), 1.0);
        oldEpoch.clock.reset(0.0);
        assertUnavailable(oldEpoch);
        Fixture absent = new Fixture();
        absent.estimator.estimate = null;
        assertUnavailable(absent);
    }

    @Test public void inclusivePoseAgeAndQualityBoundariesAreAccepted() {
        Fixture f = new Fixture();
        f.publish(Pose3d.zero(), 0.10);
        f.clock.update(0.50);
        f.targeting.update(f.clock);
        assertTrue(f.targeting.status().hasUsablePose);
        assertTrue(f.targeting.status().hasSuggestedVelocity);
        assertTrue(f.targeting.status().aimReady);
    }

    @Test public void rangeOverflowNeverBecomesAClampedVelocitySuggestion() {
        Fixture f = new Fixture(config(),
                new SimpleTagLayout().addPose(24, pose(Double.MAX_VALUE, 0, 0, 0, 0, 0)),
                CameraMountConfig.identity());
        f.publish(pose(-Double.MAX_VALUE, 0, 0, 0, 0, 0), 1.0);
        f.targeting.update(f.clock);
        assertFalse(f.targeting.status().hasSuggestedVelocity);
        assertTrue(Double.isNaN(f.targeting.status().cameraToTagRange3dInches));
        assertTrue(Double.isNaN(f.targeting.status().suggestedVelocityNative));
    }

    @Test public void stalePoseDoesNotEraseConfiguredTargetAndManualOverrideStillOwnsFeedBypass() {
        Fixture f = new Fixture();
        f.publish(Pose3d.zero(), 1.0);
        f.targeting.update(f.clock);
        f.clock.update(0.51);
        f.override = true;
        f.targeting.update(f.clock);
        assertEquals(24, f.targeting.status().configuredTagId);
        assertNotNull(f.targeting.status().fieldToSelectedTag);
        assertFalse(f.targeting.status().aimReady);
        assertTrue(f.targeting.status().aimOkToShoot);
        assertFalse(f.targeting.status().hasSuggestedVelocity);
        f.clock.update(0.52);
        f.override = false;
        f.autoAim = false;
        f.targeting.update(f.clock);
        assertTrue(f.targeting.status().aimOkToShoot);
        assertFalse(f.targeting.status().hasSuggestedVelocity);
    }

    @Test public void targetFreezesBeforePoseFailureAndOnlyOwnerResetChoosesAgain() {
        Fixture f = new Fixture();
        RuntimeException failure = new IllegalStateException("pose read failed");
        f.estimator.failure = failure;
        PhoenixCapabilities.TargetingStatus initial = f.targeting.status();
        assertSame(failure, assertThrows(RuntimeException.class, () -> f.targeting.update(f.clock)));
        assertSame(initial, f.targeting.status());
        f.selectedId = 20;
        f.estimator.failure = null;
        f.publish(Pose3d.zero(), 1.0);
        f.targeting.update(f.clock);
        assertEquals(24, f.targeting.status().configuredTagId);
        assertEquals(1, f.idReads);
        f.targeting.reset();
        assertEquals(0, f.idResets);
        f.clock.update(0.1);
        f.publish(Pose3d.zero(), 1.0);
        f.targeting.update(f.clock);
        assertEquals(20, f.targeting.status().configuredTagId);
        assertEquals(2, f.idReads);
        assertEquals(0, f.estimator.updates);
    }

    @Test public void failedCalculationDoesNotStartReadinessDebounceAndCanRetry() {
        PhoenixTargeting.Config config = config();
        config.aimReadyDebounceSec = 0.10;
        Fixture f = new Fixture(config, layout(), CameraMountConfig.identity());
        f.targeting.update(f.clock); // A successful no-pose sample establishes not-ready.
        PhoenixCapabilities.TargetingStatus previous = f.targeting.status();
        f.clock.update(0.06);
        f.publish(Pose3d.zero(), 1.0);
        f.estimator.failure = new IllegalStateException("first publication unavailable");
        assertThrows(IllegalStateException.class, () -> f.targeting.update(f.clock));
        assertSame(previous, f.targeting.status());
        f.estimator.failure = null;
        // Source debounce counts sampled dtSec, unlike a Task's own start-time deadline.
        // The retry may contribute this 0.06s interval only once, still below 0.10s.
        f.targeting.update(f.clock);
        assertFalse(f.targeting.status().aimReady);
        f.clock.update(0.11);
        f.publish(Pose3d.zero(), 1.0);
        f.targeting.update(f.clock);
        assertTrue(f.targeting.status().aimReady);
    }

    @Test public void configuredTargetAndFieldFactsAreCapturedBeforeLaterAuthorEdits() {
        PhoenixTargeting.Config config = config();
        SimpleTagLayout layout = layout();
        Fixture f = new Fixture(config, layout, CameraMountConfig.identity());
        config.scoringTargets.get(24).label = "mutated";
        config.scoringTargets.get(24).aimOffset.leftInches = 100.0;
        config.poseMaxAgeSec = 0.0;
        config.poseMinQuality = 1.0;
        f.publish(Pose3d.zero(), 0.5);
        f.targeting.update(f.clock);
        layout.addPose(24, pose(0, 100, 0, 0, 0, 0));
        f.clock.update(0.1);
        f.targeting.update(f.clock);
        assertEquals("Red scoring target", f.targeting.status().targetLabel);
        assertEquals(0.0, f.targeting.status().aimOffsetLeftInches, 0.0);
        assertEquals(36.0, f.targeting.status().cameraToTagRange3dInches, 0.0);
        assertTrue(f.targeting.status().hasUsablePose);
    }

    @Test public void selectedIdAndMissingCatalogOrFieldPoseFailBeforeBorrowedPoseRead() {
        for (Integer id : new Integer[]{null, -1, 999}) {
            Fixture f = new Fixture();
            f.selectedId = id;
            assertThrows(IllegalArgumentException.class, () -> f.targeting.update(f.clock));
            assertEquals(0, f.estimator.reads);
            assertEquals(-1, f.targeting.status().configuredTagId);
        }
        Fixture missing = new Fixture(config(), new SimpleTagLayout(), CameraMountConfig.identity());
        assertThrows(IllegalArgumentException.class, () -> missing.targeting.update(missing.clock));
        assertEquals(0, missing.estimator.reads);
    }

    @Test public void aimConsumersRequireManagedTargetingStartAndResetRequiresFreshOverlay() {
        Fixture f = new Fixture();
        DriveOverlay overlay = f.targeting.aimOverlay();
        Task task = f.targeting.aimTask(new RecordingDriveSink(), null);
        assertThrows(IllegalStateException.class, () -> overlay.onEnable(f.clock));
        assertThrows(IllegalStateException.class, () -> task.start(f.clock));
        f.publish(Pose3d.zero(), 1.0);
        f.targeting.update(f.clock);
        overlay.onEnable(f.clock);
        assertNotNull(overlay.get(f.clock));
        overlay.onDisable(f.clock);
        f.targeting.reset();
        f.clock.update(0.1);
        f.publish(Pose3d.zero(), 1.0);
        f.targeting.update(f.clock);
        assertThrows(IllegalStateException.class, () -> overlay.get(f.clock));
        DriveOverlay fresh = f.targeting.aimOverlay();
        fresh.onEnable(f.clock);
        assertNotNull(fresh.get(f.clock));
    }

    @Test public void aimTaskSnapshotsEachInvalidNumericConfigBeforeDeferredStart() {
        assertInvalidTaskConfig("positionTolInches", Double.NaN, 0.5, (c,v) -> c.positionTolInches=v);
        assertInvalidTaskConfig("headingTolRad", Double.POSITIVE_INFINITY, 0.1, (c,v) -> c.headingTolRad=v);
        assertInvalidTaskConfig("timeoutSec", 0.0, 1.0, (c,v) -> c.timeoutSec=v);
        assertInvalidTaskConfig("maxNoGuidanceSec", Double.NEGATIVE_INFINITY, 0.25,
                (c,v) -> c.maxNoGuidanceSec=v);
    }

    @Test public void aimTaskSnapshotsRequestedMaskBeforeDeferredStart() {
        Fixture f = new Fixture();
        RecordingDriveSink sink = new RecordingDriveSink();
        DriveGuidanceTask.Config config = new DriveGuidanceTask.Config();
        config.requestedMask = DriveOverlayMask.NONE;
        Task task = f.targeting.aimTask(sink, config);
        config.requestedMask = DriveOverlayMask.OMEGA_ONLY;
        f.publish(Pose3d.zero(), 1.0);
        f.targeting.update(f.clock);
        task.start(f.clock);
        task.update(f.clock);
        assertEquals(0, sink.driveCount);
    }

    private static void assertInvalidTaskConfig(String name, double invalid, double corrected,
                                                 ConfigAnswer answer) {
        Fixture f = new Fixture();
        RecordingDriveSink sink = new RecordingDriveSink();
        DriveGuidanceTask.Config config = new DriveGuidanceTask.Config();
        answer.set(config, invalid);
        Task task = f.targeting.aimTask(sink, config);
        answer.set(config, corrected);
        f.publish(Pose3d.zero(), 1.0);
        f.targeting.update(f.clock);
        IllegalArgumentException failure = assertThrows(IllegalArgumentException.class,
                () -> task.start(f.clock));
        assertTrue(failure.getMessage().contains(name));
        assertEquals(0, sink.updateCount);
        assertEquals(0, sink.driveCount);
        assertEquals(0, sink.stopCount);
    }

    private static void assertUnavailable(Fixture f) {
        f.targeting.update(f.clock);
        PhoenixCapabilities.TargetingStatus status = f.targeting.status();
        assertFalse(status.hasUsablePose);
        assertFalse(status.hasSuggestedVelocity);
        assertFalse(status.aimReady);
        assertFalse(status.aimStatus.hasOmegaError);
        assertTrue(Double.isNaN(status.suggestedVelocityNative));
        assertTrue(Double.isNaN(status.cameraToTagRange3dInches));
        assertEquals(24, status.configuredTagId);
    }

    private static PhoenixTargeting.Config config() {
        PhoenixTargeting.Config config = PhoenixTargeting.Config.defaults();
        config.aimReadyDebounceSec = 0.0;
        return config;
    }
    private static SimpleTagLayout layout() {
        return new SimpleTagLayout().addPose(24, pose(36,0,0,0,0,0))
                .addPose(20, pose(0,36,0,0,0,0));
    }
    private static Pose3d pose(double x, double y, double z, double yaw, double pitch, double roll) {
        return new Pose3d(x,y,z,yaw,pitch,roll);
    }
    private interface ConfigAnswer { void set(DriveGuidanceTask.Config config, double value); }

    private static final class Fixture {
        final LoopClock clock = new LoopClock();
        final RecordingPoseEstimator estimator = new RecordingPoseEstimator();
        final PhoenixTargeting targeting;
        Integer selectedId = 24;
        boolean autoAim = true;
        boolean override;
        int idReads;
        int idResets;
        Fixture() { this(config(), layout(), CameraMountConfig.identity()); }
        Fixture(PhoenixTargeting.Config config, SimpleTagLayout layout, CameraMountConfig mount) {
            clock.reset(0.0);
            targeting = new PhoenixTargeting(config, mount, estimator, layout,
                    new Source<Integer>() {
                        @Override public Integer get(LoopClock current) { idReads++; return selectedId; }
                        @Override public void reset() { idResets++; }
                    }, BooleanSource.of(() -> autoAim), BooleanSource.of(() -> override));
        }
        void publish(Pose3d pose, double quality) {
            estimator.estimate = new PoseEstimate(pose, true, quality, clock.nowTimestamp());
        }
    }

    private static final class RecordingPoseEstimator implements AbsolutePoseEstimator {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        RuntimeException failure;
        int reads;
        int updates;
        @Override public void update(LoopClock clock) { updates++; }
        @Override public PoseEstimate getEstimate() {
            reads++;
            if (failure != null) throw failure;
            return estimate;
        }
    }
    private static final class RecordingDriveSink implements DriveCommandSink {
        int updateCount;
        int driveCount;
        int stopCount;
        @Override public void update(LoopClock clock) { updateCount++; }
        @Override public void drive(DriveSignal signal) { driveCount++; }
        @Override public void stop() { stopCount++; }
    }

    private static final class RecordingPredictor implements MotionPredictor {
        PoseEstimate estimate;
        MotionDelta delta = MotionDelta.none(LoopTimestamp.unavailable());
        @Override public void update(LoopClock clock) { }
        @Override public PoseEstimate getEstimate() { return estimate; }
        @Override public MotionDelta getLatestMotionDelta() { return delta; }
        @Override public long trajectorySegmentId() { return 0L; }
    }
}
