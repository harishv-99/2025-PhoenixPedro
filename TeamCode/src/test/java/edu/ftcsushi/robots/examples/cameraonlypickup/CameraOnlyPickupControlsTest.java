package edu.ftcsushi.robots.examples.cameraonlypickup;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.junit.Test;

import edu.ftcsushi.fw.drive.guidance.GuidedApproach;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskBindings;
import edu.ftcsushi.fw.task.TaskOutcome;

import static org.junit.Assert.*;

/** Maintainer controls/lifetime coverage over the independent teaching graph. */
public final class CameraOnlyPickupControlsTest {
    @Test public void disabledEntrypointAndProfileRequirePhysicalReview() {
        assertNotNull(CameraOnlyPickupTeleOp.class.getAnnotation(Disabled.class));
        CameraOnlyPickupProfile profile = CameraOnlyPickupProfile.example();
        assertThrows(IllegalStateException.class, profile::requireMotionAllowed);
        assertNull(profile.camera.aprilTags);
        assertNotNull(profile.camera.floorObjects);
    }

    @Test public void motionGateRejectsEveryIntakeDriveNameCollisionBeforeDeviceAcquisition() {
        CameraOnlyPickupProfile profile = CameraOnlyPickupProfile.example();
        profile.allowMotion = true;
        String[] driveNames = {profile.drive.wiring.frontLeftName, profile.drive.wiring.frontRightName,
                profile.drive.wiring.backLeftName, profile.drive.wiring.backRightName};
        for (String driveName : driveNames) {
            profile.intake.motorName = "  " + driveName + "  ";
            IllegalStateException error = assertThrows(IllegalStateException.class,
                    profile::requireMotionAllowed);
            assertTrue(error.getMessage().contains("intake.motorName"));
            assertTrue(error.getMessage().contains("drive.wiring."));
            assertTrue(error.getMessage().contains("distinct motor names"));
        }
        profile.intake.motorName = " intakeMotor ";
        profile.requireMotionAllowed(); // No hardware registry is acquired by this data-only gate.
    }

    @Test public void motionGateAlsoRejectsDuplicateDriveAndMissingMotorNames() {
        CameraOnlyPickupProfile profile = CameraOnlyPickupProfile.example();
        profile.allowMotion = true;
        profile.drive.wiring.backRightName = "  " + profile.drive.wiring.frontLeftName + "  ";
        IllegalStateException collision = assertThrows(IllegalStateException.class,
                profile::requireMotionAllowed);
        assertTrue(collision.getMessage().contains("frontLeftName"));
        assertTrue(collision.getMessage().contains("backRightName"));
        profile.drive.wiring.backRightName = "backRightMotor";
        profile.intake.motorName = " ";
        assertThrows(IllegalStateException.class, profile::requireMotionAllowed);
    }

    @Test public void releaseThenRepressCannotAuthorizeAnOlderQueuedGesture() {
        CameraOnlyPickupTestRig r = new CameraOnlyPickupTestRig();
        r.held = true;
        r.cycle(0.02, 14, false);
        Task obsolete = r.runner.nextQueuedTaskOrNull();
        r.held = false;
        r.cycle(0.02, 14, false);
        r.held = true;
        r.cycle(0.02, 14, true);
        assertEquals(TaskOutcome.CANCELLED, obsolete.getOutcome());
        assertTrue(r.runner.hasActiveTask());
        assertNotSame(obsolete, r.runner.currentTaskOrNull());
        assertEquals(GuidedApproach.Phase.GUIDE, r.pickup.status().phase);
    }

    @Test public void overrideWithdrawsPendingGestureAndItsReleaseDoesNotRestartHeldButton() {
        CameraOnlyPickupTestRig r = new CameraOnlyPickupTestRig();
        r.held = true;
        r.cycle(0.02, 14, false);
        Task obsolete = r.runner.nextQueuedTaskOrNull();
        r.override = true;
        r.cycle(0.02, 14, false);
        r.override = false;
        r.cycle(0.02, 14, true);
        assertEquals(TaskOutcome.CANCELLED, obsolete.getOutcome());
        assertTrue(r.runner.isIdle());
    }

    @Test public void releaseRestoresManualAndStopRemainsTerminal() {
        CameraOnlyPickupTestRig r = new CameraOnlyPickupTestRig();
        r.held = true;
        r.cycle(0.02, 14, true);
        Task attempt = r.runner.currentTaskOrNull();
        r.held = false;
        r.cycle(0.02, 14, true);
        assertEquals(TaskOutcome.CANCELLED, attempt.getOutcome());
        assertEquals(r.manual.axial, r.pickup.driveSource().get(r.clock()).axial, 0);
        r.pickup.stop();
        r.intake.stop();
        r.held = true;
        r.cycle(0.02, 14, true);
        assertEquals(GuidedApproach.Phase.STOPPED, r.pickup.status().phase);
        assertEquals(0, r.pickup.driveSource().get(r.clock()).axial, 0);
        assertEquals(0, r.motor.power(), 0);
    }

    @Test public void bindingTwiceDoesNotRegisterAnotherTaskFactory() {
        CameraOnlyPickupTestRig r = new CameraOnlyPickupTestRig();
        assertThrows(IllegalStateException.class,
                () -> r.controls.bind(r.bindings, TaskBindings.of(r.bindings, r.runner), r.pickup));
        r.held = true;
        r.cycle(0.02, 14, false);
        assertEquals(1, r.runner.queuedCount());
    }
}
