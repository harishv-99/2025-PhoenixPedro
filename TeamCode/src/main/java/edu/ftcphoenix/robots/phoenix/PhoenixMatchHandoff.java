package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.ftc.FtcAutoToTeleOpHandoff;
import edu.ftcphoenix.fw.localization.PoseEstimate;

/**
 * Phoenix-owned one-shot transfer of the final autonomous pose into the next TeleOp.
 *
 * <p>The framework carrier owns only typed, process-local delivery semantics. This wrapper owns
 * the robot meaning: Phoenix publishes one immutable field pose after a successful match Auto
 * shutdown and may apply it once during the following TeleOp INIT. It deliberately does not expose
 * the generic channel, its payload type, or localization internals to OpModes.</p>
 *
 * <p>This is a best-effort handoff within one Robot Controller process. A missing, stale, or
 * already-consumed value leaves TeleOp's normally initialized pose unchanged. It is not persistent
 * calibration storage and is not evidence that the robot was physically placed at a field pose.</p>
 */
public final class PhoenixMatchHandoff {

    /** Maximum accepted delay between Auto publication and TeleOp consumption. */
    private static final double MAX_AGE_SEC = 60.0;

    private static final FtcAutoToTeleOpHandoff<MatchSnapshot> AUTO_TO_TELEOP =
            FtcAutoToTeleOpHandoff.create(
                    "Phoenix final Auto pose",
                    MatchSnapshot.class,
                    MAX_AGE_SEC
            );

    /** Result of attempting to restore Phoenix's final Auto pose during TeleOp INIT. */
    public enum RestoreResult {
        /** A fresh snapshot was consumed and applied to TeleOp localization. */
        RESTORED,
        /** No Auto snapshot was available, so the normal TeleOp pose remains active. */
        MISSING,
        /** The snapshot exceeded Phoenix's freshness window and was discarded. */
        STALE,
        /** The snapshot had already been consumed and cannot be applied again. */
        ALREADY_CONSUMED
    }

    private PhoenixMatchHandoff() {
    }

    /**
     * Clears any pending or consumed snapshot before a new Phoenix mode sequence.
     *
     * <p>Match Auto calls this during INIT so a prior run cannot leak into the new sequence.
     * Phoenix diagnostic/test OpModes also clear it so they cannot accidentally preserve match
     * state.</p>
     */
    public static void clear() {
        AUTO_TO_TELEOP.clear();
    }

    /**
     * Publishes one valid final Phoenix field pose from a successfully stopped match Auto.
     *
     * <p>Only the planar field pose crosses the mode boundary. Estimate age, quality, hardware
     * owners, Pedro objects, and season strategy remain in the Auto runtime that produced them.</p>
     *
     * @param autoOpMode FTC match Auto instance publishing the snapshot
     * @param finalPose  final Phoenix field-pose estimate captured before Auto cleanup
     * @throws NullPointerException if either argument is null
     * @throws IllegalArgumentException if the estimate has no pose or any planar component is not
     *                                  finite
     * @throws IllegalStateException if this mode sequence already published a snapshot
     */
    public static void publishFromAuto(OpMode autoOpMode, PoseEstimate finalPose) {
        OpMode requiredAuto = Objects.requireNonNull(
                autoOpMode,
                "Phoenix match Auto OpMode is required"
        );
        PoseEstimate requiredEstimate = Objects.requireNonNull(
                finalPose,
                "Phoenix final Auto pose estimate is required"
        );
        if (!requiredEstimate.hasPose) {
            throw new IllegalArgumentException(
                    "Cannot publish the Phoenix Auto-to-TeleOp pose because the final estimate "
                            + "hasPose=false"
            );
        }

        Pose2d pose = requiredEstimate.toPose2d();
        requireFinitePose(pose);
        AUTO_TO_TELEOP.publishFromAuto(requiredAuto, new MatchSnapshot(pose));
    }

    /**
     * Consumes and, when fresh, applies Phoenix's final Auto pose during TeleOp INIT.
     *
     * <p>Call this after {@link PhoenixRobot#initTeleOp()} and before
     * {@link PhoenixRobot#startAny(double)}. Non-delivery results are explicit and leave the
     * normally initialized TeleOp localization pose unchanged.</p>
     *
     * @param teleOpMode FTC TeleOp instance consuming the snapshot
     * @param robot      initialized Phoenix TeleOp composition root
     * @return whether a pose was restored or why no pose was applied
     * @throws NullPointerException if either argument is null
     * @throws IllegalStateException if a fresh snapshot arrives before TeleOp localization is
     *                               initialized or after the FTC START boundary
     */
    public static RestoreResult restoreForTeleOp(OpMode teleOpMode, PhoenixRobot robot) {
        PhoenixRobot requiredRobot = Objects.requireNonNull(
                robot,
                "Phoenix TeleOp robot is required"
        );
        return restoreForTeleOp(teleOpMode, requiredRobot::restoreTeleOpPose);
    }

    /**
     * Host-test seam that proves delivery and pose forwarding without constructing FTC hardware.
     */
    static RestoreResult restoreForTeleOp(OpMode teleOpMode, PoseReceiver receiver) {
        Objects.requireNonNull(receiver, "Phoenix TeleOp pose receiver is required");
        FtcAutoToTeleOpHandoff.ConsumeResult<MatchSnapshot> result =
                AUTO_TO_TELEOP.consumeForTeleOp(
                        Objects.requireNonNull(
                                teleOpMode,
                                "Phoenix TeleOp OpMode is required"
                        )
                );

        RestoreResult restoreResult = restoreResultFor(result.status());
        if (restoreResult == RestoreResult.RESTORED) {
            MatchSnapshot snapshot = result.payloadOrNull();
            if (snapshot == null) {
                throw new IllegalStateException(
                        "Phoenix Auto-to-TeleOp handoff reported DELIVERED without a snapshot"
                );
            }
            receiver.restore(snapshot.fieldToRobotPose);
        }
        return restoreResult;
    }

    /** Pure status mapping kept package-private so every carrier outcome is host-testable. */
    static RestoreResult restoreResultFor(FtcAutoToTeleOpHandoff.ConsumeStatus status) {
        switch (Objects.requireNonNull(status, "Phoenix handoff status is required")) {
            case DELIVERED:
                return RestoreResult.RESTORED;

            case MISSING:
                return RestoreResult.MISSING;

            case STALE:
                return RestoreResult.STALE;

            case ALREADY_CONSUMED:
                return RestoreResult.ALREADY_CONSUMED;

            default:
                throw new IllegalStateException(
                        "Unsupported Phoenix Auto-to-TeleOp handoff result: " + status
                );
        }
    }

    private static void requireFinitePose(Pose2d pose) {
        if (pose == null
                || !Double.isFinite(pose.xInches)
                || !Double.isFinite(pose.yInches)
                || !Double.isFinite(pose.headingRad)) {
            throw new IllegalArgumentException(
                    "Phoenix final Auto pose must have finite xInches, yInches, and headingRad"
            );
        }
    }

    /** Internal receiver used only to keep host tests independent of FTC hardware construction. */
    interface PoseReceiver {
        void restore(Pose2d fieldToRobotPose);
    }

    /** Immutable robot-owned payload; no estimator or vendor runtime crosses the mode boundary. */
    private static final class MatchSnapshot {
        private final Pose2d fieldToRobotPose;

        private MatchSnapshot(Pose2d fieldToRobotPose) {
            this.fieldToRobotPose = new Pose2d(
                    fieldToRobotPose.xInches,
                    fieldToRobotPose.yInches,
                    fieldToRobotPose.headingRad
            );
        }
    }
}
