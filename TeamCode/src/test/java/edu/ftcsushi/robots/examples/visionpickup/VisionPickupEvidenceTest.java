package edu.ftcsushi.robots.examples.visionpickup;

import org.junit.Test;

import java.util.Arrays;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.sensing.observation.TargetObservations2d;
import edu.ftcsushi.fw.sensing.observation.TargetSelectionResult;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Real policy scenarios with scripted observations; neither an accurate camera nor safe motion
 * is established by these tests. A quality threshold is an authored software policy, not a score
 * that can determine physical accuracy or independence of underlying sensors.
 */
public final class VisionPickupEvidenceTest {
    @Test
    public void enabledMotionRequiresAnExplicitFiniteQualityDecision() {
        assertTrue(Double.isNaN(VisionPickup.Config.defaults().minPoseQuality));
        new VisionPickupTestRig(VisionPickup.Config.defaults(), Pose2d.zero(), 10, 0);
        for (double invalid : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -0.01, 1.01}) {
            VisionPickup.Config config = VisionPickupTestRig.configured();
            config.minPoseQuality = invalid;
            try {
                new VisionPickupTestRig(config, Pose2d.zero(), 10, 0);
                fail("enabled motion must reject minPoseQuality=" + invalid);
            } catch (IllegalArgumentException expected) {
                assertTrue(expected.getMessage().contains("minPoseQuality"));
            }
        }
    }

    @Test
    public void explicitZeroFloorAcceptsValidZeroQualityButNotMalformedQuality() {
        VisionPickup.Config config = VisionPickupTestRig.configured();
        config.minPoseQuality = 0.0;
        VisionPickupTestRig rig = new VisionPickupTestRig(config, Pose2d.zero(), 10, 4);
        rig.localizer.estimate = VisionPickupTestRig.estimate(
                Pose2d.zero(), 0.0, rig.clock().nowTimestamp());
        rig.pickup.setAimEnabled(true);
        rig.pickup.update(rig.clock());
        assertEquals(VisionPickup.AssistState.AIMING, rig.pickup.status().assistState);
        for (double invalid : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -0.01, 1.01}) {
            VisionPickupTestRig bad = new VisionPickupTestRig(config, Pose2d.zero(), 10, 4);
            bad.localizer.estimate = VisionPickupTestRig.estimate(
                    Pose2d.zero(), invalid, bad.clock().nowTimestamp());
            bad.pickup.setAimEnabled(true);
            bad.pickup.update(bad.clock());
            assertEquals(VisionPickup.AssistState.LOST, bad.pickup.status().assistState);
            assertTrue(bad.pickup.status().assistReason.contains("quality"));
        }
    }

    @Test
    public void ownerSnapshotsItsQualityPolicyRatherThanRetainingTheDraft() {
        VisionPickup.Config config = VisionPickupTestRig.configured();
        VisionPickupTestRig rig = new VisionPickupTestRig(config, Pose2d.zero(), 10, 4);
        config.minPoseQuality = 0.0;
        rig.localizer.estimate = VisionPickupTestRig.estimate(
                Pose2d.zero(), 0.4, rig.clock().nowTimestamp());
        rig.pickup.setAimEnabled(true);
        rig.pickup.update(rig.clock());
        assertEquals(VisionPickup.AssistState.LOST, rig.pickup.status().assistState);
        assertTrue(rig.pickup.status().assistReason.contains("quality"));
    }

    @Test
    public void qualityFloorIsInclusiveAndAlignmentUsesActualGuidanceError() {
        VisionPickupTestRig aligned = new VisionPickupTestRig(
                VisionPickupTestRig.configured(), Pose2d.zero(), 10, 0);
        aligned.localizer.estimate = VisionPickupTestRig.estimate(
                Pose2d.zero(), 0.5, aligned.clock().nowTimestamp());
        aligned.pickup.setAimEnabled(true);
        aligned.pickup.update(aligned.clock());
        assertEquals(VisionPickup.AssistState.ALIGNED, aligned.pickup.status().assistState);
        assertEquals(0, aligned.drive().omega, 0);

        VisionPickupTestRig aiming = new VisionPickupTestRig(
                VisionPickupTestRig.configured(), Pose2d.zero(), 10, 4);
        aiming.pickup.setAimEnabled(true);
        aiming.pickup.update(aiming.clock());
        assertEquals(VisionPickup.AssistState.AIMING, aiming.pickup.status().assistState);
        assertTrue(aiming.drive().omega > 0);
    }

    @Test
    public void invalidOrInsufficientPoseQualityCancelsEveryPickupPhase() {
        for (double rejected : new double[]{Double.NaN, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY, -0.01, 1.01, 0.49}) {
            for (int phase = 0; phase < 4; phase++) {
                Pose2d start = phase == 2 ? new Pose2d(2, 0, 0) : Pose2d.zero();
                VisionPickupTestRig rig = new VisionPickupTestRig(
                        VisionPickupTestRig.configured(), start, 10, 0);
                Task task = phase == 3 ? rig.enterFinal()
                        : rig.pickup.createPickupTask(clock -> true);
                if (phase == 1 || phase == 2) task.start(rig.clock());
                Pose2d pose = phase >= 2 ? new Pose2d(2, 0, 0) : Pose2d.zero();
                rig.step(0.01, pose, 10, 0);
                rig.localizer.estimate = VisionPickupTestRig.estimate(
                        pose, rejected, rig.clock().nowTimestamp());
                rig.pickup.update(rig.clock());
                if (phase == 0) task.start(rig.clock());
                else task.update(rig.clock());
                String description = "quality=" + rejected + ", phase=" + phase;
                assertEquals(description, TaskOutcome.CANCELLED, task.getOutcome());
                assertTrue(description, rig.pickup.status().reason.contains("quality"));
                assertEquals(description, 0, rig.drive().axial, 0);
                if (phase == 3) assertEquals(Arrays.asList(true, false), rig.intakeRequests);
                else assertTrue(description, rig.intakeRequests.isEmpty());
            }
        }
    }

    @Test
    public void targetReturnCannotRestartAHeldLostAimSession() {
        VisionPickupTestRig rig = new VisionPickupTestRig(
                VisionPickupTestRig.configured(), Pose2d.zero(), 10, 4);
        rig.manual = new DriveSignal(0.3, -0.2, -0.4);
        rig.pickup.setAimEnabled(true);
        rig.pickup.update(rig.clock());
        long session = rig.pickup.status().aimSessionId;
        assertTrue(rig.drive().omega > 0);
        rig.step(0.05, Pose2d.zero());
        rig.pickup.update(rig.clock());
        assertEquals(VisionPickup.AssistState.LOST, rig.pickup.status().assistState);
        assertEquals(1, rig.pickup.status().assistLossCount);
        assertManual(rig);

        for (int cycle = 0; cycle < 3; cycle++) {
            rig.step(0.05, Pose2d.zero(), 10, 4);
            rig.pickup.setAimEnabled(true);
            rig.pickup.update(rig.clock());
            rig.pickup.update(rig.clock());
            assertEquals(VisionPickup.AssistState.LOST, rig.pickup.status().assistState);
            assertEquals(session, rig.pickup.status().aimSessionId);
            assertEquals(1, rig.pickup.status().assistLossCount);
            assertManual(rig);
        }
        rig.pickup.setAimEnabled(false);
        assertManual(rig);
        rig.pickup.setAimEnabled(true);
        assertEquals(VisionPickup.AssistState.REQUESTED, rig.pickup.status().assistState);
        assertEquals(session + 1, rig.pickup.status().aimSessionId);
        rig.pickup.update(rig.clock());
        assertEquals(VisionPickup.AssistState.REQUESTED, rig.pickup.status().assistState);
        assertManual(rig); // A new request cannot reuse an earlier service-cycle result.
        rig.step(0.05, Pose2d.zero(), 10, 4);
        rig.pickup.update(rig.clock());
        assertEquals(VisionPickup.AssistState.AIMING, rig.pickup.status().assistState);
        assertTrue(rig.drive().omega > 0);
        assertEquals(1, rig.pickup.status().assistLossCount);
    }

    @Test
    public void initiallyUnavailableFrameRetainsItsReasonAndNeedsAnotherRequest() {
        VisionPickupTestRig rig = new VisionPickupTestRig(
                VisionPickupTestRig.configured(), Pose2d.zero(), 10, 4);
        rig.raw = TargetObservations2d.unavailable("scripted camera frame unavailable");
        rig.pickup.setAimEnabled(true);
        rig.pickup.update(rig.clock());
        assertEquals(VisionPickup.AssistState.LOST, rig.pickup.status().assistState);
        assertTrue(rig.pickup.status().assistReason.contains("scripted camera frame unavailable"));
        rig.step(0.05, Pose2d.zero(), 10, 4);
        rig.pickup.update(rig.clock());
        assertEquals(VisionPickup.AssistState.LOST, rig.pickup.status().assistState);
        assertEquals(1, rig.pickup.status().assistLossCount);
    }

    @Test
    public void capabilityAgeBoundCannotBeRelaxedByAMorePermissiveSelector() {
        VisionPickupTestRig rig = new VisionPickupTestRig(
                VisionPickupTestRig.configured(), 1.0, Pose2d.zero(), 10, 4);
        rig.manual = new DriveSignal(0.3, -0.2, -0.4);
        rig.pickup.setAimEnabled(true);
        rig.pickup.update(rig.clock());
        TargetObservations2d oldFrame = rig.raw;
        rig.step(0.21, Pose2d.zero(), 10, 4);
        rig.raw = oldFrame;
        assertTrue(rig.selected.get(rig.clock()).isUsable(rig.clock()));
        assertTrue(rig.localizer.estimate.timestamp.isFresh(rig.clock(), 0.10));
        rig.pickup.update(rig.clock());
        assertEquals(VisionPickup.AssistState.LOST, rig.pickup.status().assistState);
        assertTrue(rig.pickup.status().assistReason.contains("stale"));
        assertManual(rig);
    }

    @Test
    public void stricterSelectorAgeStillWinsOverTheCapabilityBound() {
        VisionPickupTestRig rig = new VisionPickupTestRig(
                VisionPickupTestRig.configured(), 0.05, Pose2d.zero(), 10, 4);
        TargetObservations2d oldFrame = rig.raw;
        rig.step(0.06, Pose2d.zero(), 10, 4);
        rig.raw = oldFrame;
        rig.pickup.setAimEnabled(true);
        rig.pickup.update(rig.clock());
        assertEquals(VisionPickup.AssistState.LOST, rig.pickup.status().assistState);
        assertTrue(rig.pickup.status().assistReason.contains("stale"));
    }

    @Test
    public void freshTargetCannotRefreshAnOldOrMissingRobotPose() {
        for (boolean missing : new boolean[]{false, true}) {
            VisionPickupTestRig rig = new VisionPickupTestRig(
                    VisionPickupTestRig.configured(), Pose2d.zero(), 10, 4);
            PoseEstimate prior = rig.localizer.estimate;
            rig.step(0.11, Pose2d.zero(), 10, 4);
            rig.localizer.estimate = missing ? PoseEstimate.noPose(rig.clock().nowTimestamp()) : prior;
            rig.pickup.setAimEnabled(true);
            rig.pickup.update(rig.clock());
            assertEquals(VisionPickup.AssistState.LOST, rig.pickup.status().assistState);
            assertTrue(rig.pickup.status().assistReason,
                    rig.pickup.status().assistReason.contains(missing ? "unavailable" : "stale"));
            assertTrue(rig.raw.isFresh(rig.clock(), 0.20));
        }
    }

    @Test
    public void retainedFieldSelectionCannotCrossAKnownTrajectoryChange() {
        VisionPickupTestRig rig = new VisionPickupTestRig(
                VisionPickupTestRig.configured(), Pose2d.zero(), 10, 4);
        rig.pickup.setAimEnabled(true);
        rig.pickup.update(rig.clock());
        TargetSelectionResult oldSelection = rig.selected.get(rig.clock());
        rig.localizer.segment++;
        rig.step(0.05, Pose2d.zero(), 10, 4);
        rig.selectionOverride = oldSelection; // Adversarial cached pre-rebase field location.
        assertTrue(oldSelection.isUsable(rig.clock()));
        rig.pickup.update(rig.clock());
        assertEquals(VisionPickup.AssistState.LOST, rig.pickup.status().assistState);
        assertTrue(rig.pickup.status().assistReason.contains("trajectory"));
        rig.pickup.setAimEnabled(false);
        rig.pickup.setAimEnabled(true);
        rig.step(0.05, Pose2d.zero(), 10, 4);
        rig.pickup.update(rig.clock());
        assertTrue(oldSelection.isUsable(rig.clock()));
        assertEquals(VisionPickup.AssistState.LOST, rig.pickup.status().assistState);
        assertTrue(rig.pickup.status().assistReason.contains("trajectory"));
        rig.pickup.setAimEnabled(false);
        rig.pickup.setAimEnabled(true);
        rig.selectionOverride = null;
        rig.step(0.05, Pose2d.zero(), 10, 4);
        rig.pickup.update(rig.clock());
        assertEquals(VisionPickup.AssistState.AIMING, rig.pickup.status().assistState);
    }

    @Test
    public void captureTimeProjectionFailureIsNotHiddenByCurrentLocalization() {
        VisionPickupTestRig rig = new VisionPickupTestRig(
                VisionPickupTestRig.configured(), Pose2d.zero(), 10, 4);
        TargetObservations2d oldFrame = rig.raw;
        rig.step(0.05, Pose2d.zero(), 10, 4);
        rig.history.reset();
        rig.history.recordCurrent(rig.clock());
        rig.raw = oldFrame;
        assertFalse(rig.selected.get(rig.clock()).observation().hasFieldPosition());
        rig.pickup.setAimEnabled(true);
        rig.pickup.update(rig.clock());
        assertEquals(VisionPickup.AssistState.LOST, rig.pickup.status().assistState);
        assertTrue(rig.pickup.status().assistReason.contains("field"));
    }

    @Test
    public void committedStagingDestinationDoesNotBecomeLiveTargetEvidence() {
        VisionPickupTestRig rig = new VisionPickupTestRig(
                VisionPickupTestRig.configured(), Pose2d.zero(), 10, 0);
        Task task = rig.pickup.createPickupTask(clock -> true);
        task.start(rig.clock());
        rig.step(0.21, new Pose2d(0.5, 0, 0));
        rig.pickup.update(rig.clock());
        task.update(rig.clock());
        assertEquals(VisionPickup.Phase.STAGING, rig.pickup.status().phase);
        assertFalse(rig.pickup.status().approach.observation().isFresh(rig.clock(), 0.20));
        assertTrue(rig.pickup.status().approach.isCommitted());
        assertTrue(rig.drive().axial > 0);
        rig.step(0.05, new Pose2d(2, 0, 0));
        rig.pickup.update(rig.clock());
        task.update(rig.clock());
        assertEquals(VisionPickup.Phase.RECHECK, rig.pickup.status().phase);
        rig.step(0.05, new Pose2d(2, 0, 0));
        rig.pickup.update(rig.clock());
        task.update(rig.clock());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(rig.intakeRequests.isEmpty());
        assertTrue(rig.pickup.status().reason.contains("lost"));
    }

    private static void assertManual(VisionPickupTestRig rig) {
        assertEquals(rig.manual.axial, rig.drive().axial, 0);
        assertEquals(rig.manual.lateral, rig.drive().lateral, 0);
        assertEquals(rig.manual.omega, rig.drive().omega, 0);
    }
}
