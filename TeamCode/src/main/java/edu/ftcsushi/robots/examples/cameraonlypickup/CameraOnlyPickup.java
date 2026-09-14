package edu.ftcsushi.robots.examples.cameraonlypickup;

import java.util.Objects;
import java.util.function.Consumer;

import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.drive.guidance.GuidedApproach;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.sensing.observation.OccupancyObservation;
import edu.ftcsushi.fw.sensing.observation.TargetObservations2d;
import edu.ftcsushi.fw.sensing.observation.TargetSelectionPolicies;
import edu.ftcsushi.fw.sensing.observation.TargetSelectionSource;
import edu.ftcsushi.fw.sensing.observation.TargetSelections;
import edu.ftcsushi.fw.task.Task;

/**
 * Independent robot capability: one camera-only open-floor pickup shared by TeleOp and Auto.
 *
 * <p>This service owns one core approach lifetime, not camera, intake, drive hardware, or another
 * Task runner. The managed Task phase advances attempts; the service callback intentionally does
 * not. STOP terminates the approach before its borrowed camera is closed. No localization, field
 * memory, wall clearance, measured distance limit, or automatic retry is claimed.</p>
 */
public final class CameraOnlyPickup implements RobotProgram.Service {
    private final GuidedApproach approach;

    /**
     * Snapshots the relevant profile answers into the selector and approach without sampling inputs.
     * Camera and intake outputs retain their own managed lifecycle; the setter changes intake intent.
     */
    public CameraOnlyPickup(CameraOnlyPickupProfile profile,
                            Source<TargetObservations2d> objects,
                            Source<OccupancyObservation> occupancy,
                            Consumer<Boolean> setCollecting, DriveSource idleDrive) {
        CameraOnlyPickupProfile config = Objects.requireNonNull(profile, "profile");
        TargetSelectionSource selected = TargetSelections.fromVisibleObjects(objects)
                .freshWithinSec(config.maxObservationAgeSec)
                .choose(TargetSelectionPolicies.nearestToRobot());
        approach = GuidedApproach.cameraOnly(selected)
                .throughTool(config.robotToIntake, config.standOffInches)
                .driveTuning(config.tuning)
                .verifyWithZeroCommand(config.settleSec, config.positionToleranceInches,
                        config.headingToleranceRad, config.verificationTimeoutSec)
                .finalIntake(setCollecting, config.finalCommand, config.maxFinalSec)
                .captureFeedback(occupancy, config.maxCaptureAgeSec)
                .idleFrom(idleDrive)
                .withinSec(config.maxAttemptSec);
    }

    /** Returns a fresh single-use attempt; admission and each active cycle sample permission. */
    public Task createPickupTask(BooleanSource permission) {
        return approach.createPickupTask(permission);
    }

    /** Cancels only active pickup; queued gesture permission belongs to the controls. */
    public void cancelPickup() { approach.cancelPickup(); }

    /** Stable source for the program's one drive declaration; it never advances an attempt. */
    public DriveSource driveSource() { return approach.driveSource(); }

    /** Cached facts only; neither phase completion nor intake command proves physical capture. */
    public GuidedApproach.Status status() { return approach.status(); }

    /** The managed Task runner advances work; this service owns lifetime but needs no heartbeat. */
    @Override public void update(LoopClock clock) { }

    /** Terminal, idempotent shutdown; final hardware stop remains with the registered outputs. */
    @Override public void stop() { approach.stop(); }
}
