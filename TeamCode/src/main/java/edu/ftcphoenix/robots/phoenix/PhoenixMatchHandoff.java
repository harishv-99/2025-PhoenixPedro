package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.Objects;
import java.util.function.Consumer;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.ftc.FtcAutoToTeleOpHandoff;
import edu.ftcphoenix.fw.ftc.RobotProgram;
import edu.ftcphoenix.fw.localization.PoseEstimate;

/**
 * Phoenix-owned one-shot transfer of final match-Auto facts into the next TeleOp.
 *
 * <p>The framework carrier owns only typed, process-local delivery semantics. This wrapper owns
 * the robot meaning: Phoenix publishes one immutable field pose plus its frozen alliance after a
 * successful match Auto shutdown and may consume them once during the following TeleOp INIT. The
 * pose restores localization; the alliance only seeds TeleOp's still-editable INIT draft. TeleOp's
 * own prestart freeze remains the sole targeting-eligibility authority. This class deliberately
 * does not expose the generic channel, its payload type, or localization internals to OpModes.</p>
 *
 * <p>This is a best-effort handoff within one Robot Controller process. A missing, stale, or
 * already-consumed value leaves TeleOp's normally initialized pose and RED alliance draft
 * unchanged. It is not persistent calibration storage and is not evidence that the robot was
 * physically placed at a field pose.</p>
 */
public final class PhoenixMatchHandoff {

    /** Maximum accepted delay between Auto publication and TeleOp consumption. */
    private static final double MAX_AGE_SEC = 60.0;

    private static final FtcAutoToTeleOpHandoff<MatchSnapshot> AUTO_TO_TELEOP =
            FtcAutoToTeleOpHandoff.create(
                    "Phoenix final Auto match snapshot",
                    MatchSnapshot.class,
                    MAX_AGE_SEC
            );

    /** Result of attempting to restore Phoenix's final Auto match snapshot during TeleOp INIT. */
    public enum RestoreResult {
        /** A fresh snapshot restored localization and seeded the editable alliance draft. */
        RESTORED,
        /** No snapshot was available, so TeleOp keeps its normal pose and entry alliance default. */
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
     * Publishes one valid final Phoenix field pose and frozen alliance from a successfully stopped
     * match Auto.
     *
     * <p>Only the planar field pose and immutable alliance enum cross the mode boundary. Estimate
     * age, quality, hardware owners, Pedro objects, and route strategy remain in the Auto runtime
     * that produced them.</p>
     *
     * @param autoOpMode FTC match Auto instance publishing the snapshot
     * @param finalPose  final Phoenix field-pose estimate captured before Auto cleanup
     * @param alliance   immutable alliance frozen by that Auto at FTC START
     * @throws NullPointerException if any argument is null
     * @throws IllegalArgumentException if the estimate has no pose or any planar component is not
     *                                  finite
     * @throws IllegalStateException if this mode sequence already published a snapshot
     */
    public static void publishFromAuto(OpMode autoOpMode,
                                       PoseEstimate finalPose,
                                       PhoenixAlliance alliance) {
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
        AUTO_TO_TELEOP.publishFromAuto(
                requiredAuto,
                new MatchSnapshot(
                        pose,
                        Objects.requireNonNull(
                                alliance,
                                "Phoenix frozen Auto alliance is required"
                        )
                )
        );
    }

    /**
     * Consumes and, when fresh, applies Phoenix's final Auto pose and seeds TeleOp's alliance draft
     * during INIT.
     *
     * <p>Call this after {@link PhoenixRobot#declareTeleOp(RobotProgram, Source)} has declared
     * localization and before the managed program reaches FTC START. Non-delivery results are
     * explicit and leave the normally initialized TeleOp localization pose and entry alliance
     * default unchanged. The seed is not a targeting command: the operator may still choose
     * either alliance until TeleOp's prestart owner freezes at FTC START.</p>
     *
     * @param teleOpMode FTC TeleOp instance consuming the snapshot
     * @param robot      initialized Phoenix TeleOp composition root
     * @param seedAllianceDraft callback that seeds the still-editable TeleOp alliance draft
     * @return whether the snapshot was restored or why it was not applied
     * @throws NullPointerException if any argument is null
     * @throws IllegalStateException if a fresh snapshot arrives before TeleOp localization is
     *                               initialized or after the FTC START boundary
     */
    public static RestoreResult restoreForTeleOp(
            OpMode teleOpMode,
            PhoenixRobot robot,
            Consumer<? super PhoenixAlliance> seedAllianceDraft) {
        PhoenixRobot requiredRobot = Objects.requireNonNull(
                robot,
                "Phoenix TeleOp robot is required"
        );
        Consumer<? super PhoenixAlliance> requiredSeed = Objects.requireNonNull(
                seedAllianceDraft,
                "Phoenix TeleOp alliance-draft seed is required"
        );
        return restoreForTeleOp(teleOpMode, (fieldToRobotPose, alliance) -> {
            requiredRobot.restoreTeleOpPose(fieldToRobotPose);
            requiredSeed.accept(alliance);
        });
    }

    /**
     * Host-test seam that proves delivery of both match facts without constructing FTC hardware.
     */
    static RestoreResult restoreForTeleOp(OpMode teleOpMode, MatchReceiver receiver) {
        Objects.requireNonNull(receiver, "Phoenix TeleOp match receiver is required");
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
            receiver.restore(snapshot.fieldToRobotPose, snapshot.alliance);
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
    interface MatchReceiver {
        void restore(Pose2d fieldToRobotPose, PhoenixAlliance alliance);
    }

    /** Immutable robot-owned payload; no estimator or vendor runtime crosses the mode boundary. */
    private static final class MatchSnapshot {
        private final Pose2d fieldToRobotPose;
        private final PhoenixAlliance alliance;

        private MatchSnapshot(Pose2d fieldToRobotPose, PhoenixAlliance alliance) {
            this.fieldToRobotPose = new Pose2d(
                    fieldToRobotPose.xInches,
                    fieldToRobotPose.yInches,
                    fieldToRobotPose.headingRad
            );
            this.alliance = Objects.requireNonNull(alliance, "alliance");
        }
    }
}
