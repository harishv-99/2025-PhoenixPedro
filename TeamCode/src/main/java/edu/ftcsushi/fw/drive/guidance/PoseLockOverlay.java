package edu.ftcsushi.fw.drive.guidance;

import java.util.Objects;

import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.DriveOverlay;
import edu.ftcsushi.fw.drive.DriveOverlayMask;
import edu.ftcsushi.fw.drive.DriveOverlayOutput;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.localization.AbsolutePoseEstimator;
import edu.ftcsushi.fw.localization.PoseEstimate;

/**
 * Overlay that “locks” the robot in place by holding the current field pose.
 *
 * <p>This is most useful in TeleOp when you want the drivetrain to resist small bumps while an
 * operator performs a precise manipulator action (dropping a sample, lining up an intake, etc.).</p>
 *
 * <p><b>Requires a field pose estimator.</b> If you do not have reliable localization, prefer a
 * simpler heading-hold overlay or no lock at all.</p>
 *
 * <p>This overlay owns its captured target and protects pose sampling by
 * {@link LoopClock#cycle()}. Repeated reads after one successful evaluation return the same
 * result; {@link #onEnable(LoopClock)} establishes a fresh activation and cache boundary.</p>
 *
 * <p>Capture and active feedback require fresh, finite six-dimensional pose evidence and finite
 * quality in the default absolute-pose admission range. If activation cannot capture a valid
 * target, release and re-enable; later evidence does not silently establish a different target.
 * After a valid capture, temporary feedback loss passes through until valid evidence returns.</p>
 */
final class PoseLockOverlay implements DriveOverlay {

    private final AbsolutePoseEstimator poseEstimator;
    private final DriveGuidancePlan.Tuning tuning;

    private Pose2d targetFieldToRobot = null;
    private DriveOverlayOutput lastOut = DriveOverlayOutput.zero();
    private long lastCycle = Long.MIN_VALUE;

    PoseLockOverlay(AbsolutePoseEstimator poseEstimator, DriveGuidancePlan.Tuning tuning) {
        this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator");
        this.tuning = Objects.requireNonNull(tuning, "tuning");
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void onEnable(LoopClock clock) {
        // Establish a fresh activation boundary before touching a dependency that may fail.
        targetFieldToRobot = null;
        lastOut = DriveOverlayOutput.zero();
        lastCycle = Long.MIN_VALUE;

        Objects.requireNonNull(clock, "clock");
        PoseEstimate est = poseEstimator.getEstimate();
        if (isUsable(est, clock)) {
            targetFieldToRobot = est.toPose2d();
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public DriveOverlayOutput get(LoopClock clock) {
        Objects.requireNonNull(clock, "clock");
        long cycle = clock.cycle();
        if (cycle == lastCycle) {
            return lastOut;
        }

        PoseEstimate est = poseEstimator.getEstimate();

        if (targetFieldToRobot == null || !isUsable(est, clock)) {
            // No valid pose: do not override anything.
            return rememberSuccessfulResult(cycle, DriveOverlayOutput.zero());
        }

        Pose2d fieldToRobot = est.toPose2d();

        // Error from current robot pose to target robot pose, expressed in robot frame.
        Pose2d robotToTarget = fieldToRobot.inverse().then(targetFieldToRobot);
        if (!Double.isFinite(robotToTarget.xInches) || !Double.isFinite(robotToTarget.yInches)
                || !Double.isFinite(Math.hypot(robotToTarget.xInches, robotToTarget.yInches))
                || !Double.isFinite(robotToTarget.headingRad)) {
            return rememberSuccessfulResult(cycle, DriveOverlayOutput.zero());
        }

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
        return rememberSuccessfulResult(
                cycle, new DriveOverlayOutput(cmd, DriveOverlayMask.ALL));
    }

    private DriveOverlayOutput rememberSuccessfulResult(long cycle, DriveOverlayOutput result) {
        lastOut = result;
        lastCycle = cycle;
        return result;
    }

    private static boolean isUsable(PoseEstimate est, LoopClock clock) {
        if (est == null || !est.hasPose
                || !est.timestamp.isFresh(clock, DriveGuidanceSpec.AbsolutePose.DEFAULT_MAX_AGE_SEC)
                || !Double.isFinite(est.quality)
                || est.quality < DriveGuidanceSpec.AbsolutePose.DEFAULT_MIN_QUALITY
                || est.quality > 1.0) return false;
        Pose3d pose = est.fieldToRobotPose;
        return Double.isFinite(pose.xInches) && Double.isFinite(pose.yInches)
                && Double.isFinite(pose.zInches) && Double.isFinite(pose.yawRad)
                && Double.isFinite(pose.pitchRad) && Double.isFinite(pose.rollRad);
    }

    /**
     * {@inheritDoc}
     */
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
