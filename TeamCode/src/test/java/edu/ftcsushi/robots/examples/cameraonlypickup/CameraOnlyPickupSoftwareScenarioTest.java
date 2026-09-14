package edu.ftcsushi.robots.examples.cameraonlypickup;

import org.junit.Test;

import edu.ftcsushi.fw.drive.guidance.GuidedApproach;
import edu.ftcsushi.fw.sensing.observation.TargetObservations2d;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;

import static org.junit.Assert.*;

/** One causal reading checkpoint; authored camera/sensor evidence is not a physical simulation. */
public final class CameraOnlyPickupSoftwareScenarioTest {
    @Test public void visibleVerificationAllowsBlindFinalButOnlyTheSensorConfirmsCapture() {
        // ARRANGE: real example owners; initially inject an empty intake and a target 14 inches ahead.
        CameraOnlyPickupTestRig r = new CameraOnlyPickupTestRig();
        r.held = true;
        r.cycle(0.02, 14, true);
        Task attempt = r.runner.currentTaskOrNull();
        assertEquals(GuidedApproach.Phase.GUIDE, r.pickup.status().phase);

        // INJECT EVIDENCE: target at 3-inch tool offset + 5-inch stand-off, not modeled robot motion.
        r.cycle(0.02, 8, true);
        assertEquals(GuidedApproach.Phase.VERIFY, r.pickup.status().phase);
        assertEquals(0, r.pickup.driveSource().get(r.clock()).axial, 0);
        r.cycle(0.11, 8, true); // One new qualifying image captured after the settling interval.
        assertEquals(1, r.pickup.status().verifiedCaptureCount);
        r.cycle(0.02, 8, true); // A second distinct image authorizes the final command.
        assertEquals(GuidedApproach.Phase.FINAL_INTAKE, r.pickup.status().phase);
        assertEquals(0.20, r.motor.power(), 0);
        assertEquals(TaskOutcome.NOT_DONE, attempt.getOutcome());

        // HEARTBEAT: expected occlusion after authorization is not success and does not stop final.
        r.time.nextCycle(0.02);
        r.frame = TargetObservations2d.unavailable("intake now hides the ball");
        r.runPhases(true);
        assertEquals(TaskOutcome.NOT_DONE, attempt.getOutcome());
        assertEquals(0.10, r.pickup.driveSource().get(r.clock()).axial, 1e-9);

        // INJECT EVIDENCE / ASSERT: only a later independent occupied sensor sample confirms capture.
        r.sensor.setHigh(false);
        r.time.nextCycle(0.02);
        r.runPhases(true);
        assertEquals(TaskOutcome.SUCCESS, attempt.getOutcome());
        assertEquals(0, r.motor.power(), 0);
        // NEXT GATE: this proves software sequencing, not visibility, physical braking or collection.
    }
}
