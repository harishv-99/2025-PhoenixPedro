package edu.ftcphoenix.fw.drive.guidance;

import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveOverlay;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.drive.DriveOverlayOutput;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.localization.PoseEstimator;

/**
 * Overlay that “locks” the robot in place by holding the current field pose.
 *
 * <p>This is most useful in TeleOp when you want the drivetrain to resist small bumps while an
 * operator performs a precise manipulator action (dropping a sample, lining up an intake, etc.).</p>
 *
 * <p><b>Requires a field pose estimator.</b> If you do not have reliable localization, prefer a
 * simpler heading-hold overlay or no lock at all.</p>
 */
final class PoseLockOverlay implements DriveOverlay {

    private final PoseEstimator poseEstimator;
    private final DriveGuidancePlan.Tuning tuning;

    private Pose2d targetFieldToRobot = null;
    private DriveOverlayOutput lastOut = DriveOverlayOutput.zero();

    PoseLockOverlay(PoseEstimator poseEstimator, DriveGuidancePlan.Tuning tuning) {
        this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator");
        this.tuning = Objects.requireNonNull(tuning, "tuning");
    }

    @Override
    public void onEnable(LoopClock clock) {
        PoseEstimate est = poseEstimator.getEstimate();
        if (est != null && est.hasPose) {
            targetFieldToRobot = est.toPose2d();
        } else {
            targetFieldToRobot = null;
        }
    }

    @Override
    public DriveOverlayOutput get(LoopClock clock) {
        PoseEstimate est = poseEstimator.getEstimate();

        if (targetFieldToRobot == null || est == null || !est.hasPose) {
            // No valid pose: do not override anything.
            lastOut = DriveOverlayOutput.zero();
            return lastOut;
        }

        // Basic age/quality gating.
        if (est.ageSec > DriveGuidancePlan.FieldPose.DEFAULT_MAX_AGE_SEC
                || est.quality < DriveGuidancePlan.FieldPose.DEFAULT_MIN_QUALITY) {
            lastOut = DriveOverlayOutput.zero();
            return lastOut;
        }

        Pose2d fieldToRobot = est.toPose2d();

        // Error from current robot pose to target robot pose, expressed in robot frame.
        Pose2d robotToTarget = fieldToRobot.inverse().then(targetFieldToRobot);

        // Translate to reduce position error.
        DriveSignal t = DriveGuidanceControllers.translationCmd(
                robotToTarget.xInches,
                robotToTarget.yInches,
                tuning
        );

        // Rotate to reduce heading error.
        double headingErr = Pose2d.wrapToPi(robotToTarget.headingRad);
        double omega = DriveGuidanceControllers.omegaCmd(headingErr, tuning);

        DriveSignal cmd = new DriveSignal(t.axial, t.lateral, omega);
        lastOut = new DriveOverlayOutput(cmd, DriveOverlayMask.ALL);
        return lastOut;
    }

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "poseLock" : prefix;
        dbg.addData(p + ".target", targetFieldToRobot);
        dbg.addData(p + ".lastOut", lastOut);
    }
}
