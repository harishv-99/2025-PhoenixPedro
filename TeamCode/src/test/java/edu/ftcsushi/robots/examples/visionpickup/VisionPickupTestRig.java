package edu.ftcsushi.robots.examples.visionpickup;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcsushi.fw.localization.PlanarPoseHistory;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.localization.PoseTrajectoryEstimator;
import edu.ftcsushi.fw.sensing.observation.ObservationSources;
import edu.ftcsushi.fw.sensing.observation.TargetObservation2d;
import edu.ftcsushi.fw.sensing.observation.TargetObservations2d;
import edu.ftcsushi.fw.sensing.observation.TargetSelectionResult;
import edu.ftcsushi.fw.sensing.observation.TargetSelectionPolicies;
import edu.ftcsushi.fw.sensing.observation.TargetSelections;
import edu.ftcsushi.fw.spatial.AxisAlignedBoxRegion2d;
import edu.ftcsushi.fw.spatial.RobotFrameRectangle2d;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;

/**
 * Software-only outside-world substitution shared by the degradation scenarios.
 * Selection, history, guidance, and pickup remain real; these numbers authorize no robot motion.
 */
final class VisionPickupTestRig {
    final ManualLoopClock time = new ManualLoopClock();
    final MutableLocalizer localizer = new MutableLocalizer();
    final PlanarPoseHistory history = new PlanarPoseHistory(
            localizer, PlanarPoseHistory.Config.defaults());
    final List<Boolean> intakeRequests = new ArrayList<>();
    final Source<TargetSelectionResult> selected;
    final VisionPickup pickup;
    TargetObservations2d raw;
    VisionPickup.CaptureFeedback feedback;
    DriveSignal manual = DriveSignal.zero();
    TargetSelectionResult selectionOverride;
    Runnable selectionHook;
    Runnable captureHook;
    Consumer<Boolean> intakeHook;

    VisionPickupTestRig(VisionPickup.Config config, Pose2d pose, double... fieldTargets) {
        this(config, config.maxObservationAgeSec, pose, fieldTargets);
    }

    VisionPickupTestRig(VisionPickup.Config config, double selectorMaxAgeSec,
                        Pose2d pose, double... fieldTargets) {
        publishPose(pose, 1.0);
        publishFrame(pose, fieldTargets);
        feedback = VisionPickup.CaptureFeedback.observed(false, clock().nowTimestamp());
        selected = TargetSelections.fromVisibleObjects(ObservationSources.inField(
                Source.of(clock -> raw), history.lookupSource()))
                .freshWithinSec(selectorMaxAgeSec).choose(TargetSelectionPolicies.nearestToRobot());
        pickup = new VisionPickup(config, Source.of(clock -> {
            if (selectionHook != null) selectionHook.run();
            return selectionOverride == null ? selected.get(clock) : selectionOverride;
        }), localizer, Source.of(clock -> {
            if (captureHook != null) captureHook.run();
            return feedback;
        }), requested -> {
            intakeRequests.add(requested);
            if (intakeHook != null) intakeHook.accept(requested);
        }, clock -> manual);
    }

    LoopClock clock() { return time.clock(); }

    DriveSignal drive() { return pickup.driveSource().get(clock()); }

    /** Advances outside-world evidence only; the caller explicitly advances services and Tasks. */
    void step(double seconds, Pose2d pose, double... fieldTargets) {
        time.nextCycle(seconds);
        publishPose(pose, 1.0);
        publishFrame(pose, fieldTargets);
        feedback = VisionPickup.CaptureFeedback.observed(false, clock().nowTimestamp());
    }

    /** Publishes a synthetic current robot sample and records the real capture-time history. */
    void publishPose(Pose2d pose, double quality) {
        localizer.estimate = estimate(pose, quality, clock().nowTimestamp());
        history.recordCurrent(clock());
    }

    /** Authors a current camera frame from field coordinates using the supplied capture pose. */
    void publishFrame(Pose2d pose, double... fieldTargets) {
        raw = frame(pose, clock().nowTimestamp(), fieldTargets);
    }

    /** Enters the existing synthetic open-floor final phase; no physical arrival is claimed. */
    Task enterFinal() {
        Task task = pickup.createPickupTask(clock -> true);
        task.start(clock());
        step(0.05, new Pose2d(2, 0, 0), 10, 0);
        pickup.update(clock());
        task.update(clock());
        step(0.05, new Pose2d(2, 0, 0), 10, 0);
        pickup.update(clock());
        task.update(clock());
        assertEquals(VisionPickup.Phase.FINAL_INTAKE, pickup.status().phase);
        return task;
    }

    static PoseEstimate estimate(Pose2d pose, double quality, LoopTimestamp timestamp) {
        return new PoseEstimate(new Pose3d(pose.xInches, pose.yInches, 0,
                pose.headingRad, 0, 0), true, quality, timestamp);
    }

    static TargetObservations2d frame(Pose2d pose, LoopTimestamp timestamp,
                                       double... fieldTargets) {
        List<TargetObservation2d> observations = new ArrayList<>();
        for (int i = 0; i < fieldTargets.length; i += 2) {
            double dx = fieldTargets[i] - pose.xInches;
            double dy = fieldTargets[i + 1] - pose.yInches;
            observations.add(TargetObservation2d.ofRobotRelativePosition(
                    Math.cos(pose.headingRad) * dx + Math.sin(pose.headingRad) * dy,
                    -Math.sin(pose.headingRad) * dx + Math.cos(pose.headingRad) * dy,
                    Double.NaN, timestamp));
        }
        return TargetObservations2d.fromFrame(timestamp, observations);
    }

    /** Complete illustrative software policy; no measured geometry or useful physical thresholds. */
    static VisionPickup.Config configured() {
        VisionPickup.Config c = VisionPickup.Config.defaults();
        c.enableMotion = true;
        c.allowWallContact = false;
        c.allowUnconfirmedCapture = false;
        c.robotToIntake = new Pose2d(3, 0, 0);
        c.robotEnvelope = RobotFrameRectangle2d.centeredInches(4, 4);
        c.fieldInterior = new AxisAlignedBoxRegion2d(-50, 50, -50, 50);
        c.templates = Collections.singletonList(new VisionPickup.Template("synthetic open floor",
                new AxisAlignedBoxRegion2d(-40, 40, -40, 40), 0, VisionPickup.Contact.NONE));
        c.guidanceTuning = DriveGuidancePlan.Tuning.defaults()
                .withMaxTranslateCmd(0.2).withMaxOmegaCmd(0.2);
        c.stagingStandOffInches = 5;
        c.stagingWallMarginInches = 1;
        c.contactCommandExtensionInches = 0;
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
        c.maxObservationAgeSec = 0.20;
        c.maxPoseAgeSec = 0.10;
        c.maxCaptureAgeSec = 0.10;
        c.minPoseQuality = 0.5;
        return c;
    }

    /** Cached scripted snapshot; the actual owner remains the test's outside-world publisher. */
    static final class MutableLocalizer implements PoseTrajectoryEstimator {
        PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        long segment;
        @Override public long trajectorySegmentId() { return segment; }
        @Override public void update(LoopClock clock) { }
        @Override public PoseEstimate getEstimate() { return estimate; }
    }
}
