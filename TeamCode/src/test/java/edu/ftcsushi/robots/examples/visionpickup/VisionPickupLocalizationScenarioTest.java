package edu.ftcsushi.robots.examples.visionpickup;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.MotionDelta;
import edu.ftcsushi.fw.localization.MotionPredictor;
import edu.ftcsushi.fw.localization.PlanarPoseHistory;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.localization.fusion.CorrectionStats;
import edu.ftcsushi.fw.localization.fusion.OdometryCorrectionFusionEstimator;
import edu.ftcsushi.fw.sensing.observation.ObservationSources;
import edu.ftcsushi.fw.sensing.observation.TargetObservations2d;
import edu.ftcsushi.fw.sensing.observation.TargetSelectionResult;
import edu.ftcsushi.fw.sensing.observation.TargetSelections;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/**
 * Keeps the real Fusion estimator, history, selector, guidance, and robot policy. Scripted
 * predictor/correction observations replace sensors; test-authored scores and jump bounds do not
 * establish physical accuracy, independence, or appropriate settings for an adopting robot.
 */
public final class VisionPickupLocalizationScenarioTest {
    @Test
    public void correctionLossAloneDoesNotVetoFreshSufficientPredictorEvidence() {
        Fixture f = new Fixture(0.8, Pose2d.zero(), 10, 4);
        f.pickup.setAimEnabled(true);
        f.pickup.update(f.clock());
        assertEquals(VisionPickup.AssistState.AIMING, f.pickup.status().assistState);
        for (int i = 0; i < 5; i++) {
            f.step(0.05, true, false, Pose2d.zero(), 10, 4);
            f.pickup.update(f.clock());
            assertTrue(f.fusion.getEstimate().hasPose);
            assertTrue(f.fusion.getEstimate().timestamp.isFresh(f.clock(), 0.0));
            assertEquals(VisionPickup.AssistState.AIMING, f.pickup.status().assistState);
        }
        assertEquals(0.8, f.fusion.getEstimate().quality, 1e-9);
        assertEquals(1, f.fusion.getCorrectionStats().acceptedCorrectionCount);
        assertFalse(f.correction.getEstimate().hasPose);
        assertEquals(0, f.pickup.status().assistLossCount);
    }

    @Test
    public void decayingAcceptedQualityCanEndAssistWhilePoseEvidenceRemainsFresh() {
        Fixture f = new Fixture(0.3, Pose2d.zero(), 10, 4);
        f.pickup.setAimEnabled(true);
        f.pickup.update(f.clock());
        assertEquals(0.9, f.fusion.getEstimate().quality, 1e-9);
        f.step(0.05, true, false, Pose2d.zero(), 10, 4);
        f.pickup.update(f.clock());
        assertEquals(0.675, f.fusion.getEstimate().quality, 1e-9);
        assertEquals(VisionPickup.AssistState.AIMING, f.pickup.status().assistState);
        f.step(0.05, true, false, Pose2d.zero(), 10, 4);
        f.pickup.update(f.clock());
        assertEquals(0.45, f.fusion.getEstimate().quality, 1e-9);
        assertTrue(f.fusion.getEstimate().timestamp.isFresh(f.clock(), 0.0));
        assertEquals(VisionPickup.AssistState.LOST, f.pickup.status().assistState);
        assertTrue(f.pickup.status().assistReason.contains("quality"));
        assertManual(f);

        f.step(0.05, true, true, Pose2d.zero(), 10, 4);
        f.pickup.update(f.clock());
        assertEquals(0.9, f.fusion.getEstimate().quality, 1e-9);
        assertEquals(VisionPickup.AssistState.LOST, f.pickup.status().assistState);
        assertManual(f); // New acceptance is evidence, not a new operator request.
        f.pickup.setAimEnabled(false);
        f.pickup.setAimEnabled(true);
        f.step(0.01, true, true, Pose2d.zero(), 10, 4);
        f.pickup.update(f.clock());
        assertEquals(VisionPickup.AssistState.AIMING, f.pickup.status().assistState);
    }

    @Test
    public void missingPredictorWithoutANewCorrectionMakesFinalLocalizationUnavailable() {
        Fixture f = new Fixture(0.8, Pose2d.zero(), 10, 4);
        f.pickup.setAimEnabled(true);
        f.pickup.update(f.clock());
        f.step(0.05, false, false, Pose2d.zero(), 10, 4);
        f.pickup.update(f.clock());
        assertFalse(f.fusion.getEstimate().hasPose);
        assertEquals(VisionPickup.AssistState.LOST, f.pickup.status().assistState);
        assertTrue(f.pickup.status().assistReason.contains("unavailable"));
        assertFalse(f.pickup.status().assistReason.contains("predictor"));
        assertManual(f);
        f.step(0.05, true, true, Pose2d.zero(), 10, 4);
        f.pickup.update(f.clock());
        assertTrue(f.fusion.getEstimate().hasPose);
        assertEquals(VisionPickup.AssistState.LOST, f.pickup.status().assistState);
        assertEquals(1, f.pickup.status().assistLossCount);
    }

    @Test
    public void anIncorporatedCorrectionCanSupplyPoseWithoutAReportedPredictorPose() {
        Fixture f = new Fixture(0.8, Pose2d.zero(), 10, 4);
        f.pickup.setAimEnabled(true);
        f.pickup.update(f.clock());
        f.step(0.05, false, true, Pose2d.zero(), 10, 4);
        f.pickup.update(f.clock());
        assertFalse(f.predictor.getEstimate().hasPose);
        assertTrue(f.fusion.getEstimate().hasPose);
        assertTrue(f.fusion.getEstimate().timestamp.isFresh(f.clock(), 0.0));
        assertEquals(VisionPickup.AssistState.AIMING, f.pickup.status().assistState);
        assertEquals(2, f.fusion.getCorrectionStats().acceptedCorrectionCount);

        // A repeated accepted frame is not another incorporation in the next loop.
        PoseEstimate retainedCorrection = f.correction.estimate;
        f.time.nextCycle(0.05);
        f.predictor.publish(f.clock(), false, Pose2d.zero(), f.predictorQuality);
        f.correction.estimate = retainedCorrection;
        f.publishEnvironment(Pose2d.zero(), 10, 4);
        f.pickup.update(f.clock());
        assertFalse(f.fusion.getEstimate().hasPose);
        assertEquals(VisionPickup.AssistState.LOST, f.pickup.status().assistState);
        assertEquals(2, f.fusion.getCorrectionStats().acceptedCorrectionCount);
        assertEquals(1, f.fusion.getCorrectionStats().skippedDuplicateCorrectionCount);
    }

    @Test
    public void frozenHighQualityPredictorCannotBeRestampedByNewLoopOrTarget() {
        Fixture f = new Fixture(0.8, Pose2d.zero(), 10, 4);
        f.pickup.setAimEnabled(true);
        f.pickup.update(f.clock());
        LoopTimestamp evidence = f.fusion.getEstimate().timestamp;
        f.time.nextCycle(0.11); // Deliberately leave predictor's estimate and delta frozen.
        f.correction.estimate = PoseEstimate.noPose(f.clock().nowTimestamp());
        f.publishEnvironment(Pose2d.zero(), 10, 4);
        f.pickup.update(f.clock());
        assertTrue(f.fusion.getEstimate().hasPose);
        assertEquals(0.0, f.fusion.getEstimate().timestamp.secondsSince(evidence), 0);
        assertTrue(f.fusion.getEstimate().quality >= 0.5);
        assertTrue(f.raw.isFresh(f.clock(), 0.0));
        assertEquals(VisionPickup.AssistState.LOST, f.pickup.status().assistState);
        assertTrue(f.pickup.status().assistReason.contains("stale"));
        assertManual(f);
    }

    @Test
    public void configuredTranslationJumpRejectsAContradictoryCorrectionWithoutInventingAVeto() {
        Fixture f = new Fixture(0.8, Pose2d.zero(), 10, 4);
        f.pickup.setAimEnabled(true);
        f.pickup.update(f.clock());
        f.time.nextCycle(0.05);
        f.predictor.publish(f.clock(), true, Pose2d.zero(), f.predictorQuality);
        // A same-time 12-inch disagreement exceeds this fixture's explicit 2-inch policy.
        // The fixture knows the input; CorrectionStats alone does not name the rejection cause.
        f.correction.estimate = VisionPickupTestRig.estimate(
                new Pose2d(12, 0, 0), 1.0, f.clock().nowTimestamp());
        f.publishEnvironment(Pose2d.zero(), 10, 4);
        f.pickup.update(f.clock());
        CorrectionStats stats = f.fusion.getCorrectionStats();
        assertEquals(1, stats.rejectedCorrectionCount);
        assertEquals(1, stats.acceptedCorrectionCount);
        assertEquals(0, f.fusion.getEstimate().toPose2d().xInches, 0);
        assertTrue(f.fusion.getEstimate().timestamp.isFresh(f.clock(), 0.0));
        assertEquals(VisionPickup.AssistState.AIMING, f.pickup.status().assistState);
        assertEquals(0, f.pickup.status().assistLossCount);
        assertTrue(f.pickup.driveSource().get(f.clock()).omega > 0);
    }

    @Test
    public void autoFinalIntakeCancelsOnFinalPoseLossAndDoesNotReviveOnRecovery() {
        Fixture f = new Fixture(0.8, new Pose2d(2, 0, 0), 10, 0);
        f.manual = DriveSignal.zero(); // Auto supplies zero idle intent, never synthetic driver input.
        Task auto = f.pickup.createPickupTask(clock -> true);
        auto.start(f.clock());
        assertEquals(VisionPickup.Phase.RECHECK, f.pickup.status().phase);
        f.step(0.05, true, true, new Pose2d(2, 0, 0), 10, 0);
        f.pickup.update(f.clock());
        auto.update(f.clock());
        assertEquals(VisionPickup.Phase.FINAL_INTAKE, f.pickup.status().phase);
        f.step(0.05, false, false, new Pose2d(2, 0, 0));
        f.pickup.update(f.clock());
        auto.update(f.clock());
        assertEquals(TaskOutcome.CANCELLED, auto.getOutcome());
        assertEquals(Arrays.asList(true, false), f.intakeRequests);
        assertEquals(0, f.pickup.driveSource().get(f.clock()).axial, 0);
        f.step(0.05, true, true, new Pose2d(2, 0, 0), 10, 0);
        f.pickup.update(f.clock());
        auto.update(f.clock());
        assertEquals(TaskOutcome.CANCELLED, auto.getOutcome());
        assertEquals(Arrays.asList(true, false), f.intakeRequests);
        f.pickup.stop();
        assertEquals(VisionPickup.AssistState.STOPPED, f.pickup.status().assistState);
        assertEquals(0, f.pickup.driveSource().get(f.clock()).axial, 0);
    }

    private static void assertManual(Fixture f) {
        DriveSignal actual = f.pickup.driveSource().get(f.clock());
        assertEquals(f.manual.axial, actual.axial, 0);
        assertEquals(f.manual.lateral, actual.lateral, 0);
        assertEquals(f.manual.omega, actual.omega, 0);
    }

    /** The estimator graph advances once before capture-time history and the pickup consumer. */
    private static final class Fixture {
        final ManualLoopClock time = new ManualLoopClock();
        final ScriptedPredictor predictor = new ScriptedPredictor();
        final ScriptedCorrection correction = new ScriptedCorrection();
        final OdometryCorrectionFusionEstimator fusion;
        final PlanarPoseHistory history;
        final VisionPickup pickup;
        final List<Boolean> intakeRequests = new ArrayList<>();
        final double predictorQuality;
        TargetObservations2d raw;
        VisionPickup.CaptureFeedback feedback;
        DriveSignal manual = new DriveSignal(0.3, -0.2, -0.4);

        Fixture(double predictorQuality, Pose2d pose, double... targets) {
            this.predictorQuality = predictorQuality;
            OdometryCorrectionFusionEstimator.Config cfg = OdometryCorrectionFusionEstimator.Config.defaults();
            cfg.maxCorrectionAgeSec = 0.20;
            cfg.minCorrectionQuality = 0.05;
            cfg.correctionPositionGain = 0.25;
            cfg.correctionHeadingGain = 0.35;
            cfg.maxCorrectionPositionJumpIn = 2.0;
            cfg.maxCorrectionHeadingJumpRad = 0.5;
            cfg.correctionConfidenceHoldSec = 0.20;
            cfg.enableInitializeFromCorrection = true;
            cfg.enablePushCorrectedPoseToPredictor = false;
            fusion = new OdometryCorrectionFusionEstimator(predictor, correction, cfg);
            history = new PlanarPoseHistory(fusion, PlanarPoseHistory.Config.defaults());
            predictor.publish(clock(), true, pose, predictorQuality);
            correction.estimate = VisionPickupTestRig.estimate(pose, 0.9, clock().nowTimestamp());
            publishEnvironment(pose, targets);
            Source<TargetSelectionResult> selected = TargetSelections.from(ObservationSources.inField(
                    Source.of(clock -> raw), history.lookupSource()))
                    .freshWithinSec(0.20).nearestToRobot();
            pickup = new VisionPickup(VisionPickupTestRig.configured(), selected, fusion,
                    Source.of(clock -> feedback), intakeRequests::add, clock -> manual);
        }

        LoopClock clock() { return time.clock(); }

        void step(double seconds, boolean predictorAvailable, boolean correctionAvailable,
                  Pose2d pose, double... targets) {
            time.nextCycle(seconds);
            predictor.publish(clock(), predictorAvailable, pose, predictorQuality);
            correction.estimate = correctionAvailable
                    ? VisionPickupTestRig.estimate(pose, 0.9, clock().nowTimestamp())
                    : PoseEstimate.noPose(clock().nowTimestamp());
            publishEnvironment(pose, targets);
        }

        void publishEnvironment(Pose2d pose, double... targets) {
            fusion.update(clock());
            history.recordCurrent(clock());
            raw = VisionPickupTestRig.frame(pose, clock().nowTimestamp(), targets);
            feedback = VisionPickup.CaptureFeedback.observed(false, clock().nowTimestamp());
        }
    }

    /** Scripted cached sensor boundary, retaining an accepted motion baseline across missing data. */
    private static final class ScriptedPredictor implements MotionPredictor {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        MotionDelta delta = MotionDelta.none(LoopTimestamp.unavailable());
        PoseEstimate previous;

        void publish(LoopClock clock, boolean available, Pose2d pose, double quality) {
            LoopTimestamp now = clock.nowTimestamp();
            if (!available) {
                estimate = PoseEstimate.noPose(now);
                delta = MotionDelta.none(now);
                return;
            }
            estimate = VisionPickupTestRig.estimate(pose, quality, now);
            delta = previous != null && now.secondsSince(previous.timestamp) > 0
                    ? new MotionDelta(previous.fieldToRobotPose.inverse().then(estimate.fieldToRobotPose),
                    true, quality, previous.timestamp, now) : MotionDelta.none(now);
            previous = estimate;
        }

        @Override public long trajectorySegmentId() { return 0; }
        @Override public void update(LoopClock clock) { }
        @Override public PoseEstimate getEstimate() { return estimate; }
        @Override public MotionDelta getLatestMotionDelta() { return delta; }
    }

    /** Authored field observations, not a camera or an independence claim. */
    private static final class ScriptedCorrection implements AbsolutePoseEstimator {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        @Override public void update(LoopClock clock) { }
        @Override public PoseEstimate getEstimate() { return estimate; }
    }
}
