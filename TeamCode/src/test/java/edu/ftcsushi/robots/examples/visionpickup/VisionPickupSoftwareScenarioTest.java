package edu.ftcsushi.robots.examples.visionpickup;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcsushi.fw.input.binding.Bindings;
import edu.ftcsushi.fw.localization.PlanarPoseHistory;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.localization.PoseTrajectoryEstimator;
import edu.ftcsushi.fw.sensing.observation.ObservationSources;
import edu.ftcsushi.fw.sensing.observation.TargetObservation2d;
import edu.ftcsushi.fw.sensing.observation.TargetObservations2d;
import edu.ftcsushi.fw.sensing.observation.TargetSelectionResult;
import edu.ftcsushi.fw.sensing.observation.TargetSelections;
import edu.ftcsushi.fw.spatial.AxisAlignedBoxRegion2d;
import edu.ftcsushi.fw.spatial.RobotFrameRectangle2d;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskBindings;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.task.TaskRunner;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Authored software experiments retaining real selection, pose history, guidance, and pickup policy.
 * Camera frames, localization samples, manual input, and capture sensor evidence are substituted.
 * These tests prove no physical clearance, contact force, motor behavior, or real object capture.
 */
public final class VisionPickupSoftwareScenarioTest {

    @Test
    public void arrivalIsNotCaptureAndOnlyNewFeedbackCompletesPickupSuccessfully() {
        Fixture fixture = new Fixture(configured(), Pose2d.zero(), 10, 0);
        Task task = fixture.pickup.createPickupTask(clock -> true);
        task.start(fixture.clock());
        assertEquals(VisionPickup.Phase.STAGING, fixture.pickup.status().phase);
        assertTrue(fixture.pickup.driveSource().get(fixture.clock()).axial > 0);
        fixture.step(0.05, new Pose2d(2, 0, 0), 10, 0);
        task.update(fixture.clock());
        assertEquals(VisionPickup.Phase.RECHECK, fixture.pickup.status().phase);
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        assertTrue(fixture.intakeRequests.isEmpty());
        fixture.step(0.05, new Pose2d(2, 0, 0), 10, 0);
        task.update(fixture.clock());
        assertEquals(VisionPickup.Phase.FINAL_INTAKE, fixture.pickup.status().phase);
        assertEquals(Collections.singletonList(true), fixture.intakeRequests);

        // Inject a new sensor transition, while the camera can no longer see the object.
        fixture.step(0.05, new Pose2d(3, 0, 0));
        fixture.feedback = VisionPickup.CaptureFeedback.observed(true, fixture.clock().nowTimestamp());
        task.update(fixture.clock());
        assertEquals(TaskOutcome.SUCCESS, task.getOutcome());
        assertEquals(Arrays.asList(true, false), fixture.intakeRequests);
        assertEquals(VisionPickup.Phase.DONE, fixture.pickup.status().phase);
        assertEquals(0, fixture.pickup.driveSource().get(fixture.clock()).axial, 0);
    }

    @Test
    public void defaultsDoNotEnableAutomaticMotionOrTouchIntake() {
        Fixture fixture = new Fixture(VisionPickup.Config.defaults(), Pose2d.zero(), 10, 0);
        fixture.pickup.setAimEnabled(true);
        fixture.pickup.update(fixture.clock());
        Task task = fixture.pickup.createPickupTask(clock -> true);
        task.start(fixture.clock());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(fixture.pickup.status().reason.contains("unconfigured"));
        assertTrue(fixture.intakeRequests.isEmpty());
        assertEquals(0, fixture.pickup.driveSource().get(fixture.clock()).omega, 0);
    }

    @Test
    public void ambiguousAndLostRechecksAbortWithoutStartingIntake() {
        for (double[] targets : new double[][]{{}, {10, 0, 10.2, 0.1}}) {
            Fixture fixture = new Fixture(configured(), new Pose2d(2, 0, 0), 10, 0);
            Task task = fixture.pickup.createPickupTask(clock -> true);
            task.start(fixture.clock());
            assertEquals(VisionPickup.Phase.RECHECK, fixture.pickup.status().phase);
            fixture.step(0.05, new Pose2d(2, 0, 0), targets);
            task.update(fixture.clock());
            assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
            assertTrue(fixture.intakeRequests.isEmpty());
            assertTrue(fixture.pickup.status().reason.contains(targets.length == 0 ? "lost" : "ambiguous"));
        }
    }

    @Test
    public void finalOcclusionWithoutFeedbackCannotClaimCapture() {
        Fixture fixture = new Fixture(configured(), Pose2d.zero(), 10, 0);
        Task task = fixture.enterFinal();
        fixture.step(0.4, new Pose2d(2, 0, 0));
        task.update(fixture.clock());
        assertFalse(task.isComplete());
        fixture.step(0.11, new Pose2d(2, 0, 0));
        task.update(fixture.clock());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertFalse(fixture.intakeRequests.get(fixture.intakeRequests.size() - 1));
    }

    @Test
    public void explicitlyMissingCaptureFeedbackEndsUnknownNeverSuccess() {
        VisionPickup.Config config = configured();
        config.allowUnconfirmedCapture = true;
        Fixture fixture = new Fixture(config, Pose2d.zero(), 10, 0);
        fixture.feedbackAvailable = false;
        fixture.feedback = VisionPickup.CaptureFeedback.unavailable();
        Task task = fixture.enterFinal();
        fixture.step(0.51, new Pose2d(2, 0, 0));
        task.update(fixture.clock());
        assertEquals(TaskOutcome.UNKNOWN, task.getOutcome());
        assertTrue(fixture.pickup.status().reason.contains("unconfirmed"));
    }

    @Test
    public void initialOccupiedMissingAndReplayedFeedbackNeverCountAsNewCapture() {
        Fixture occupied = new Fixture(configured(), Pose2d.zero(), 10, 0);
        occupied.feedback = VisionPickup.CaptureFeedback.observed(true, occupied.clock().nowTimestamp());
        Task occupiedTask = occupied.pickup.createPickupTask(clock -> true);
        occupiedTask.start(occupied.clock());
        assertEquals(TaskOutcome.CANCELLED, occupiedTask.getOutcome());

        Fixture missing = new Fixture(configured(), Pose2d.zero(), 10, 0);
        missing.feedback = VisionPickup.CaptureFeedback.unavailable();
        Task missingTask = missing.pickup.createPickupTask(clock -> true);
        missingTask.start(missing.clock());
        assertEquals(TaskOutcome.CANCELLED, missingTask.getOutcome());

        Fixture replayed = new Fixture(configured(), Pose2d.zero(), 10, 0);
        Task replayedTask = replayed.enterFinal();
        LoopTimestamp previous = replayed.feedback.timestamp;
        replayed.step(0.05, new Pose2d(2, 0, 0));
        replayed.feedback = VisionPickup.CaptureFeedback.observed(true, previous);
        replayedTask.update(replayed.clock());
        assertEquals(TaskOutcome.CANCELLED, replayedTask.getOutcome());
        assertTrue(replayed.pickup.status().reason.contains("same timestamp"));
    }

    @Test
    public void cancellationIsActiveOnlySingleUseAndClearsOwnedRequests() {
        Fixture fixture = new Fixture(configured(), Pose2d.zero(), 10, 0);
        Task task = fixture.pickup.createPickupTask(clock -> true);
        task.cancel();
        assertFalse(task.isComplete());
        task.start(fixture.clock());
        task.cancel();
        task.cancel();
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(fixture.intakeRequests.isEmpty());
        assertEquals(0, fixture.pickup.driveSource().get(fixture.clock()).axial, 0);
        try { task.start(fixture.clock()); fail("single-use"); }
        catch (IllegalStateException expected) { assertTrue(expected.getMessage().contains("single-use")); }
        Task fresh = fixture.pickup.createPickupTask(clock -> true);
        assertNotSame(task, fresh);
        try { fresh.update(fixture.clock()); fail("update before start"); }
        catch (IllegalStateException expected) { assertTrue(expected.getMessage().contains("start")); }

        Fixture finalFixture = new Fixture(configured(), Pose2d.zero(), 10, 0);
        Task finalTask = finalFixture.enterFinal();
        finalFixture.pickup.stop();
        finalTask.cancel();
        finalFixture.pickup.stop();
        assertEquals(TaskOutcome.CANCELLED, finalTask.getOutcome());
        assertEquals(Arrays.asList(true, false), finalFixture.intakeRequests);
        assertEquals(0, finalFixture.pickup.driveSource().get(finalFixture.clock()).axial, 0);
    }

    @Test
    public void sameCycleUpdatesAndLargePreStartDtCannotExpireNewFinalCommand() {
        Fixture fixture = new Fixture(configured(), new Pose2d(2, 0, 0), 10, 0);
        fixture.step(10.0, new Pose2d(2, 0, 0), 10, 0);
        Task task = fixture.pickup.createPickupTask(clock -> true);
        task.start(fixture.clock());
        task.update(fixture.clock());
        assertEquals(VisionPickup.Phase.RECHECK, fixture.pickup.status().phase);
        fixture.step(0.05, new Pose2d(2, 0, 0), 10, 0);
        task.update(fixture.clock());
        task.update(fixture.clock());
        assertEquals(Collections.singletonList(true), fixture.intakeRequests);
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        assertTrue(fixture.pickup.driveSource().get(fixture.clock()).axial > 0);
    }

    @Test
    public void trajectoryChangeAndClockResetInvalidateACommittedAttempt() {
        Fixture changed = new Fixture(configured(), Pose2d.zero(), 10, 0);
        Task task = changed.enterFinal();
        changed.localizer.segment++;
        changed.step(0.05, new Pose2d(2, 0, 0));
        task.update(changed.clock());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(changed.pickup.status().reason.contains("trajectory"));

        Fixture reset = new Fixture(configured(), Pose2d.zero(), 10, 0);
        Task resetTask = reset.enterFinal();
        reset.clock().reset(0);
        resetTask.update(reset.clock());
        assertEquals(TaskOutcome.CANCELLED, resetTask.getOutcome());
    }

    @Test
    public void finalTravelAndCorridorBoundsWinOverCoincidentCaptureEvidence() {
        Fixture traveled = new Fixture(configured(), Pose2d.zero(), 10, 0);
        Task traveledTask = traveled.enterFinal();
        traveled.step(0.05, new Pose2d(8, 0, 0));
        traveled.feedback = VisionPickup.CaptureFeedback.observed(true, traveled.clock().nowTimestamp());
        traveledTask.update(traveled.clock());
        assertEquals(TaskOutcome.CANCELLED, traveledTask.getOutcome());
        assertTrue(traveled.pickup.status().reason.contains("travel"));

        Fixture corridor = new Fixture(configured(), Pose2d.zero(), 10, 0);
        Task corridorTask = corridor.enterFinal();
        corridor.step(0.05, new Pose2d(2, 2, 0));
        corridor.feedback = VisionPickup.CaptureFeedback.observed(true, corridor.clock().nowTimestamp());
        corridorTask.update(corridor.clock());
        assertEquals(TaskOutcome.CANCELLED, corridorTask.getOutcome());
        assertTrue(corridor.pickup.status().reason.contains("corridor"));
    }

    @Test
    public void wallAndCornerContactNeedExplicitPermissionAndBoundedGeometry() {
        VisionPickup.Config wall = configured();
        wall.maxFinalTravelInches = 8;
        wall.contactCommandExtensionInches = 1;
        wall.templates = Collections.singletonList(new VisionPickup.Template("right wall",
                new AxisAlignedBoxRegion2d(48, 50, -3, 3), 0, VisionPickup.Contact.MAX_X));
        Fixture refused = new Fixture(wall, new Pose2d(41, 0, 0), 49, 0);
        Task refusedTask = refused.pickup.createPickupTask(clock -> true);
        refusedTask.start(refused.clock());
        assertEquals(TaskOutcome.CANCELLED, refusedTask.getOutcome());

        wall.allowWallContact = true;
        Fixture admitted = new Fixture(wall, new Pose2d(41, 0, 0), 49, 0);
        Task wallTask = admitted.pickup.createPickupTask(clock -> true);
        wallTask.start(admitted.clock());
        assertEquals(VisionPickup.Phase.RECHECK, admitted.pickup.status().phase);
        assertEquals(VisionPickup.Contact.MAX_X, admitted.pickup.status().permittedContact);

        VisionPickup.Config corner = configured();
        corner.allowWallContact = true;
        corner.contactCommandExtensionInches = 1;
        corner.templates = Collections.singletonList(new VisionPickup.Template("upper right corner",
                new AxisAlignedBoxRegion2d(48, 50, 48, 50), Math.PI / 4,
                VisionPickup.Contact.MAX_X_MAX_Y));
        double staging = 49 - 8 / Math.sqrt(2);
        Fixture cornerFixture = new Fixture(corner, new Pose2d(staging, staging, Math.PI / 4), 49, 49);
        Task cornerTask = cornerFixture.pickup.createPickupTask(clock -> true);
        cornerTask.start(cornerFixture.clock());
        assertEquals(VisionPickup.Phase.RECHECK, cornerFixture.pickup.status().phase);
        assertEquals(VisionPickup.Contact.MAX_X_MAX_Y, cornerFixture.pickup.status().permittedContact);
        assertFalse(cornerTask.isComplete()); // Accepted command geometry is not arrival/capture proof.
    }

    @Test
    public void aimAssistPreservesTranslationAndReleasesAllChannelsOnTargetLoss() {
        Fixture fixture = new Fixture(configured(), Pose2d.zero(), 10, 4);
        fixture.manual = new DriveSignal(0.3, -0.2, -0.4);
        fixture.pickup.setAimEnabled(true);
        fixture.pickup.update(fixture.clock());
        DriveSignal aimed = fixture.pickup.driveSource().get(fixture.clock());
        assertEquals(0.3, aimed.axial, 0);
        assertEquals(-0.2, aimed.lateral, 0);
        assertTrue(aimed.omega > 0);
        fixture.step(0.05, Pose2d.zero());
        fixture.pickup.update(fixture.clock());
        assertEquals(-0.4, fixture.pickup.driveSource().get(fixture.clock()).omega, 0);
    }

    @Test
    public void cancellationInsidePermissionSelectionAndCaptureCannotReviveWork() {
        for (int cancellationSite = 0; cancellationSite < 3; cancellationSite++) {
            Fixture fixture = new Fixture(configured(), Pose2d.zero(), 10, 0);
            if (cancellationSite == 1) fixture.selectionHook = fixture.pickup::cancelPickup;
            if (cancellationSite == 2) fixture.captureHook = fixture.pickup::cancelPickup;
            final boolean cancelPermission = cancellationSite == 0;
            Task task = fixture.pickup.createPickupTask(clock -> {
                if (cancelPermission) fixture.pickup.cancelPickup();
                return true;
            });
            task.start(fixture.clock());
            assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
            assertEquals(VisionPickup.Phase.DONE, fixture.pickup.status().phase);
            assertTrue(fixture.intakeRequests.isEmpty());
            assertEquals(0, fixture.pickup.driveSource().get(fixture.clock()).axial, 0);
        }
    }

    @Test
    public void cancellationOrThrowInsideIntakeRequestCannotRestoreNonzeroDrive() {
        Fixture cancelled = new Fixture(configured(), new Pose2d(2, 0, 0), 10, 0);
        cancelled.intakeHook = enabled -> { if (enabled) cancelled.pickup.cancelPickup(); };
        Task task = cancelled.pickup.createPickupTask(clock -> true);
        task.start(cancelled.clock());
        cancelled.step(0.05, new Pose2d(2, 0, 0), 10, 0);
        task.update(cancelled.clock());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(Arrays.asList(true, false), cancelled.intakeRequests);
        assertEquals(0, cancelled.pickup.driveSource().get(cancelled.clock()).axial, 0);

        Fixture thrown = new Fixture(configured(), new Pose2d(2, 0, 0), 10, 0);
        thrown.intakeHook = enabled -> { if (enabled) throw new IllegalStateException("mutated then failed"); };
        Task failed = thrown.pickup.createPickupTask(clock -> true);
        failed.start(thrown.clock());
        thrown.step(0.05, new Pose2d(2, 0, 0), 10, 0);
        try { failed.update(thrown.clock()); fail("request failure"); }
        catch (IllegalStateException expected) { assertEquals("mutated then failed", expected.getMessage()); }
        assertEquals(TaskOutcome.CANCELLED, failed.getOutcome());
        assertEquals(Arrays.asList(true, false), thrown.intakeRequests);
    }

    @Test
    public void recheckAndActiveFeedbackCallbacksCannotResurrectCancelledPhases() {
        Fixture recheck = new Fixture(configured(), new Pose2d(2, 0, 0), 10, 0);
        Task checking = recheck.pickup.createPickupTask(clock -> true);
        checking.start(recheck.clock());
        recheck.step(0.05, new Pose2d(2, 0, 0), 10, 0);
        recheck.selectionHook = recheck.pickup::stop;
        checking.update(recheck.clock());
        assertEquals(TaskOutcome.CANCELLED, checking.getOutcome());
        assertEquals(VisionPickup.Phase.DONE, recheck.pickup.status().phase);
        assertTrue(recheck.intakeRequests.isEmpty());
        assertEquals(0, recheck.pickup.driveSource().get(recheck.clock()).axial, 0);

        Fixture finalPhase = new Fixture(configured(), Pose2d.zero(), 10, 0);
        Task active = finalPhase.enterFinal();
        finalPhase.step(0.05, new Pose2d(2, 0, 0));
        finalPhase.captureHook = finalPhase.pickup::cancelPickup;
        active.update(finalPhase.clock());
        assertEquals(TaskOutcome.CANCELLED, active.getOutcome());
        assertEquals(Arrays.asList(true, false), finalPhase.intakeRequests);
        assertEquals(0, finalPhase.pickup.driveSource().get(finalPhase.clock()).axial, 0);
    }

    @Test
    public void exhaustedFinalDeadlineWinsOverACaptureFirstReadAfterTheBound() {
        Fixture fixture = new Fixture(configured(), Pose2d.zero(), 10, 0);
        Task task = fixture.enterFinal();
        fixture.step(0.51, new Pose2d(2, 0, 0));
        fixture.feedback = VisionPickup.CaptureFeedback.observed(true, fixture.clock().nowTimestamp());
        task.update(fixture.clock());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertEquals(Arrays.asList(true, false), fixture.intakeRequests);
    }

    @Test
    public void failedIntakeCleanupLatchesOwnerStoppedAndCannotAdmitReplacementWork() {
        Fixture fixture = new Fixture(configured(), Pose2d.zero(), 10, 0);
        Task task = fixture.enterFinal();
        fixture.manual = new DriveSignal(0.4, 0.2, 0.1);
        fixture.intakeHook = enabled -> {
            if (!enabled) throw new IllegalStateException("stop request failed");
        };
        try { task.cancel(); fail("cleanup failure"); }
        catch (IllegalStateException expected) { assertEquals("stop request failed", expected.getMessage()); }
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertEquals(0, fixture.pickup.driveSource().get(fixture.clock()).axial, 0);
        Task replacement = fixture.pickup.createPickupTask(clock -> true);
        replacement.start(fixture.clock());
        assertEquals(TaskOutcome.CANCELLED, replacement.getOutcome());
        assertEquals(Arrays.asList(true, false), fixture.intakeRequests);
    }

    @Test
    public void nearbyCandidateAcrossTemplateBoundaryStillMakesRecheckAmbiguous() {
        VisionPickup.Config config = configured();
        config.templates = Collections.singletonList(new VisionPickup.Template("bounded strip",
                new AxisAlignedBoxRegion2d(0, 10.1, -5, 5), 0, VisionPickup.Contact.NONE));
        Fixture fixture = new Fixture(config, new Pose2d(2, 0, 0), 10, 0);
        Task task = fixture.pickup.createPickupTask(clock -> true);
        task.start(fixture.clock());
        fixture.step(0.05, new Pose2d(2, 0, 0), 10, 0, 10.2, 0);
        task.update(fixture.clock());
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(fixture.pickup.status().reason.contains("ambiguous"));
        assertTrue(fixture.intakeRequests.isEmpty());
    }

    @Test
    public void recheckDoesNotRelaxTheSelectorsStricterFreshnessPolicy() {
        Fixture fixture = new Fixture(configured(), new Pose2d(2, 0, 0), 10, 0);
        Task task = fixture.pickup.createPickupTask(clock -> true);
        task.start(fixture.clock());
        fixture.step(0.05, new Pose2d(2, 0, 0), 10, 0);
        TargetObservations2d earlier = ObservationSources.inField(
                Source.of(clock -> fixture.raw), fixture.history.lookupSource()).get(fixture.clock());
        fixture.selectionOverride = TargetSelectionResult.none(earlier, 0.05, "selector observation expired");
        fixture.step(0.10, new Pose2d(2, 0, 0), 10, 0);
        task.update(fixture.clock());
        assertEquals(VisionPickup.Phase.RECHECK, fixture.pickup.status().phase);
        assertTrue(fixture.intakeRequests.isEmpty());
    }

    @Test
    public void teleOpAndAutoUseTheSameFreshTaskAndReleaseRejectsQueuedRequests() {
        Fixture fixture = new Fixture(configured(), Pose2d.zero(), 10, 0);
        boolean[] inputs = {false, false, false};
        Bindings bindings = new Bindings();
        TaskRunner runner = new TaskRunner();
        VisionPickupControls controls = new VisionPickupControls(
                clock -> inputs[0], clock -> inputs[1], clock -> inputs[2]);
        controls.bind(bindings, TaskBindings.of(bindings, runner), fixture.pickup);
        bindings.update(fixture.clock());
        inputs[1] = true;
        fixture.step(0.05, Pose2d.zero(), 10, 0);
        bindings.update(fixture.clock()); // Enqueued, but not started yet.
        inputs[1] = false;
        fixture.step(0.05, Pose2d.zero(), 10, 0);
        bindings.update(fixture.clock());
        runner.update(fixture.clock());
        assertEquals(TaskOutcome.CANCELLED, fixture.pickup.status().outcome);
        assertTrue(fixture.intakeRequests.isEmpty());

        // An Auto client deliberately provides continuing permission to this same factory.
        Task auto = fixture.pickup.createPickupTask(clock -> true);
        auto.start(fixture.clock());
        assertEquals(VisionPickup.Phase.STAGING, fixture.pickup.status().phase);
        auto.cancel();
    }

    /** Complete synthetic configuration, intentionally confined to tests rather than robot defaults. */
    private static VisionPickup.Config configured() {
        VisionPickup.Config c = VisionPickup.Config.defaults();
        c.enableMotion = true;
        c.robotToIntake = new Pose2d(3, 0, 0);
        c.robotEnvelope = RobotFrameRectangle2d.centeredInches(4, 4);
        c.fieldInterior = new AxisAlignedBoxRegion2d(-50, 50, -50, 50);
        c.templates = Collections.singletonList(new VisionPickup.Template("open floor",
                new AxisAlignedBoxRegion2d(-40, 40, -40, 40), 0, VisionPickup.Contact.NONE));
        c.guidanceTuning = DriveGuidancePlan.Tuning.defaults().withMaxTranslateCmd(0.2).withMaxOmegaCmd(0.2);
        c.stagingStandOffInches = 5;
        c.stagingWallMarginInches = 1;
        c.finalTranslateCommand = 0.1;
        c.maxAttemptTravelInches = 30;
        c.maxFinalTravelInches = 6;
        c.maxAttemptSec = 3;
        c.maxFinalSec = 0.5;
        c.recheckTimeoutSec = 0.3;
        c.recheckRadiusInches = 1;
        c.finalCorridorHalfWidthInches = 0.5;
        c.arrivalToleranceInches = 0.1;
        c.headingToleranceRad = 0.1;
        return c;
    }

    /** Test-only outside-world substitution; history, selector, guidance, and policy remain real. */
    private static final class Fixture {
        final ManualLoopClock time = new ManualLoopClock();
        final FakeLocalizer localizer = new FakeLocalizer();
        final PlanarPoseHistory history = new PlanarPoseHistory(localizer, PlanarPoseHistory.Config.defaults());
        final List<Boolean> intakeRequests = new ArrayList<>();
        final VisionPickup pickup;
        TargetObservations2d raw;
        VisionPickup.CaptureFeedback feedback;
        DriveSignal manual = DriveSignal.zero();
        boolean feedbackAvailable = true;
        Runnable selectionHook;
        Runnable captureHook;
        Consumer<Boolean> intakeHook;
        TargetSelectionResult selectionOverride;

        Fixture(VisionPickup.Config config, Pose2d pose, double... fieldTargets) {
            publish(pose, fieldTargets);
            Source<TargetSelectionResult> selected = TargetSelections.from(
                    ObservationSources.inField(Source.of(clock -> raw), history.lookupSource()))
                    .freshWithinSec(config.maxObservationAgeSec).nearestToRobot();
            pickup = new VisionPickup(config, Source.of(clock -> {
                if (selectionHook != null) selectionHook.run();
                if (selectionOverride != null) return selectionOverride;
                return selected.get(clock);
            }), localizer, Source.of(clock -> {
                if (captureHook != null) captureHook.run();
                return feedback;
            }), enabled -> {
                intakeRequests.add(enabled);
                if (intakeHook != null) intakeHook.accept(enabled);
            }, clock -> manual);
        }

        LoopClock clock() { return time.clock(); }

        void step(double seconds, Pose2d pose, double... targets) {
            time.nextCycle(seconds);
            publish(pose, targets);
        }

        private void publish(Pose2d pose, double... fieldTargets) {
            LoopTimestamp timestamp = clock().nowTimestamp();
            localizer.estimate = new PoseEstimate(new Pose3d(pose.xInches, pose.yInches, 0,
                    pose.headingRad, 0, 0), true, 1, timestamp);
            history.recordCurrent(clock());
            List<TargetObservation2d> targets = new ArrayList<>();
            for (int i = 0; i < fieldTargets.length; i += 2) {
                double dx = fieldTargets[i] - pose.xInches;
                double dy = fieldTargets[i + 1] - pose.yInches;
                targets.add(TargetObservation2d.ofRobotRelativePosition(
                        Math.cos(pose.headingRad) * dx + Math.sin(pose.headingRad) * dy,
                        -Math.sin(pose.headingRad) * dx + Math.cos(pose.headingRad) * dy,
                        Double.NaN, timestamp));
            }
            raw = TargetObservations2d.fromFrame(timestamp, targets);
            feedback = feedbackAvailable ? VisionPickup.CaptureFeedback.observed(false, timestamp)
                    : VisionPickup.CaptureFeedback.unavailable();
        }

        Task enterFinal() {
            Task task = pickup.createPickupTask(clock -> true);
            task.start(clock());
            step(0.05, new Pose2d(2, 0, 0), 10, 0);
            task.update(clock());
            step(0.05, new Pose2d(2, 0, 0), 10, 0);
            task.update(clock());
            assertEquals(VisionPickup.Phase.FINAL_INTAKE, pickup.status().phase);
            return task;
        }
    }

    private static final class FakeLocalizer implements PoseTrajectoryEstimator {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        long segment;
        @Override public long trajectorySegmentId() { return segment; }
        @Override public void update(LoopClock clock) { }
        @Override public PoseEstimate getEstimate() { return estimate; }
    }
}
