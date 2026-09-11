package edu.ftcsushi.robots.examples.tagalignment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.junit.Test;

import java.util.Collections;

import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceQuery;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceSpec;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceStatus;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagObservation;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Authored camera geometry and real guidance/drive composition; not physical camera validation. */
public final class TagAlignmentTest {
    @Test public void disabledEntrypointsAndProfileRequirePhysicalReview() {
        assertNotNull(TagAlignmentTeleOp.class.getAnnotation(Disabled.class));
        assertNotNull(TagAlignmentAuto.class.getAnnotation(Disabled.class));
        TagAlignmentProfile profile = TagAlignmentProfile.example();
        IllegalStateException error = assertThrows(IllegalStateException.class, profile::requireMotionAllowed);
        assertTrue(error.getMessage().contains("allowMotion"));
        assertNotNull(profile.camera.aprilTags);
        assertNull(profile.camera.floorObjects);
        assertTrue(profile.auto.timeoutSec > profile.auto.maxNoGuidanceSec);
    }

    @Test public void approachUsesActualTagAndMountNotSuppliedFieldRobotPose() {
        Rig rig = new Rig();
        rig.capture(20, 6);
        DriveGuidanceQuery query = rig.plan.query();
        DriveGuidanceStatus status = query.get(rig.time.clock());
        // Camera is four inches ahead of center. Tag faces us, so +18 tag-X is toward us.
        assertEquals(6, status.forwardErrorIn, 1e-9); // 4 + 20 - 18.
        assertEquals(6, status.leftErrorIn, 1e-9);
        assertEquals(0, status.omegaErrorRad, 1e-9);
        assertEquals(DriveGuidanceSpec.SolveMode.RELATIVE_APRIL_TAGS, status.solveMode);
        assertNull(status.fieldToTranslationFrameAnchor);
        assertNull(rig.plan.spec.resolveWith.absolutePose);
        assertNull(rig.plan.spec.resolveWith.fixedAprilTagLayout);

        rig.time.nextCycle(0.02);
        rig.capture(30, 6); // A different actual fixture position changes the direct target.
        assertEquals(16, query.get(rig.time.clock()).forwardErrorIn, 1e-9);
    }

    @Test public void sidewaysCameraMountRotatesBothApproachPointAndHeading() {
        Rig rig = new Rig();
        rig.capture(30, 6);
        CameraMountConfig sideways = CameraMountConfig.of(2, 3, 8, Math.PI / 2, 0, 0);
        DriveGuidanceStatus status = TagAlignment.plan(rig.profile, rig.sensor, sideways)
                .query().get(rig.time.clock());
        assertEquals(-4, status.forwardErrorIn, 1e-9);
        assertEquals(15, status.leftErrorIn, 1e-9);
        assertEquals(Math.PI / 2, status.omegaErrorRad, 1e-9);
    }

    @Test public void builtPlanDoesNotDriftWithEditedProfile() {
        Rig rig = new Rig();
        rig.capture(20, 6);
        rig.profile.tagId = 999;
        rig.profile.tagForwardInches = 1000;
        rig.profile.tagHeadingRad = 0;
        rig.profile.maxTagAgeSec = 0;
        DriveGuidanceStatus status = rig.plan.query().get(rig.time.clock());
        assertEquals(6, status.forwardErrorIn, 1e-9);
        assertEquals(0, status.omegaErrorRad, 1e-9);
        assertEquals(0.20, rig.plan.spec.resolveWith.relativeAprilTags.maxAgeSec, 0);
    }

    @Test public void heldAssistReleaseLossAndReacquisitionMatchCachedDriverText() {
        Rig rig = new Rig();
        rig.capture(20, 6);
        boolean[] hold = {false};
        DriveSignal manual = new DriveSignal(-0.10, -0.08, -0.05);
        TagAlignmentControls controls = new TagAlignmentControls(clock -> manual,
                clock -> hold[0], rig.plan);
        DriveSource drive = controls.driveSource();
        assertSame(manual, drive.get(rig.time.clock()));
        assertTrue(controls.status().startsWith("Manual control"));
        assertEquals(0, rig.reads);

        hold[0] = true;
        rig.time.nextCycle(0.02);
        DriveSignal assisted = drive.get(rig.time.clock());
        assertTrue(assisted.axial > 0);
        assertTrue(assisted.lateral > 0);
        assertTrue(controls.status().startsWith("Assisting position and heading"));
        int reads = rig.reads;
        controls.status();
        controls.status();
        assertEquals(reads, rig.reads); // A presenter never reads vision or advances guidance.

        hold[0] = false;
        assertSame(assisted, drive.get(rig.time.clock())); // One result per active cycle.
        assertTrue(controls.status().startsWith("Assisting"));
        rig.time.nextCycle(0.02);
        assertSame(manual, drive.get(rig.time.clock()));
        assertTrue(controls.status().startsWith("Manual control"));

        hold[0] = true;
        rig.time.nextCycle(0.02);
        rig.frame = AprilTagDetections.none();
        assertSame(manual, drive.get(rig.time.clock()));
        assertTrue(controls.status().startsWith("Tag unavailable: manual control"));
        rig.time.nextCycle(0.02);
        rig.capture(20, 6);
        assertTrue(drive.get(rig.time.clock()).axial > 0);
        assertTrue(controls.status().startsWith("Assisting"));
    }

    @Test public void staleFrameRequestsStopThenTimeoutNeverArrival() {
        Rig rig = new Rig();
        rig.capture(20, 6);
        RecordingDrive drive = new RecordingDrive();
        DriveGuidanceTask task = rig.plan.task(drive, rig.profile.auto);
        task.start(rig.time.clock());
        task.update(rig.time.clock());
        assertTrue(drive.last.axial > 0);
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        int commands = drive.commands;

        rig.time.nextCycle(0.21); // Do not restamp a retained image.
        task.update(rig.time.clock());
        assertEquals(commands, drive.commands);
        assertEquals(DriveSignal.zero(), drive.last);
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        rig.time.nextCycle(0.31);
        task.update(rig.time.clock());
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertEquals(commands, drive.commands);
        int stops = drive.stops;
        task.update(rig.time.clock());
        task.cancel();
        assertEquals(stops, drive.stops);
    }

    @Test public void cancellationStopsOnceAndAnotherActionUsesFreshTask() {
        Rig rig = new Rig();
        rig.capture(20, 6);
        RecordingDrive drive = new RecordingDrive();
        DriveGuidanceTask first = rig.plan.task(drive, rig.profile.auto);
        first.cancel(); // Before start is inert.
        assertEquals(0, drive.stops);
        first.start(rig.time.clock());
        first.update(rig.time.clock());
        int commands = drive.commands;
        first.cancel();
        assertEquals(TaskOutcome.CANCELLED, first.getOutcome());
        assertEquals(DriveSignal.zero(), drive.last);
        int stops = drive.stops;
        first.cancel();
        first.update(rig.time.clock());
        assertEquals(stops, drive.stops);
        assertEquals(commands, drive.commands);
        assertThrows(IllegalStateException.class, () -> first.start(rig.time.clock()));

        DriveGuidanceTask second = rig.plan.task(drive, rig.profile.auto);
        assertNotSame(first, second);
        rig.time.nextCycle(0.02);
        rig.capture(14, 0); // Center is already at the requested 18-inch approach point.
        second.start(rig.time.clock());
        second.update(rig.time.clock());
        assertEquals(TaskOutcome.SUCCESS, second.getOutcome());
        assertEquals(DriveSignal.zero(), drive.last);
    }

    private static final class Rig {
        final ManualLoopClock time = new ManualLoopClock();
        final TagAlignmentProfile profile = TagAlignmentProfile.example();
        AprilTagDetections frame = AprilTagDetections.none();
        int reads;
        final AprilTagSensor sensor = clock -> { reads++; return frame; };
        final DriveGuidancePlan plan = TagAlignment.plan(profile, sensor, profile.camera.cameraMount);

        void capture(double forward, double left) {
            frame = AprilTagDetections.fromFrame(time.clock().nowTimestamp(), Collections.singletonList(
                    AprilTagObservation.target(1, new Pose3d(forward, left, 0, Math.PI, 0, 0),
                            new Pose3d(999, -999, 0, 1, 0, 0))));
        }
    }

    private static final class RecordingDrive implements DriveCommandSink {
        int commands;
        int stops;
        DriveSignal last = DriveSignal.zero();
        @Override public void drive(DriveSignal signal) { commands++; last = signal; }
        @Override public void stop() { stops++; last = DriveSignal.zero(); }
    }
}
