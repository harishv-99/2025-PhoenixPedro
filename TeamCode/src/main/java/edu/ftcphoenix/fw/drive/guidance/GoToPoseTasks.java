package edu.ftcphoenix.fw.drive.guidance;

import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.PoseEstimator;
import edu.ftcphoenix.fw.task.Task;

/**
 * Convenience factory methods that build go-to-pose behaviors using the
 * {@link DriveGuidancePlan} infrastructure.
 *
 * <p>This keeps the “go to pose” concept (very common in autonomous) while reusing
 * the same aim/translate/feedback/tuning machinery as TeleOp DriveGuidance overlays.</p>
 */
public final class GoToPoseTasks {

    private GoToPoseTasks() {
        // utility
    }

    /**
     * Drive to an absolute pose in the field frame.
     */
    public static Task goToPoseFieldRelative(
            PoseEstimator poseEstimator,
            MecanumDrivebase drivebase,
            Pose2d targetFieldPose,
            DriveGuidancePlan.Tuning tuning,
            DriveGuidanceTask.Config taskCfg) {

        Objects.requireNonNull(poseEstimator, "poseEstimator");
        Objects.requireNonNull(drivebase, "drivebase");
        Objects.requireNonNull(targetFieldPose, "targetFieldPose");
        Objects.requireNonNull(tuning, "tuning");

        DriveGuidancePlan plan = DriveGuidance.plan()
                .translateTo()
                .fieldPointInches(targetFieldPose.xInches, targetFieldPose.yInches)
                .doneTranslateTo()
                .aimTo()
                .fieldHeadingRad(targetFieldPose.headingRad)
                .doneAimTo()
                .feedback()
                .fieldPose(poseEstimator)
                .doneFeedback()
                .tuning(tuning)
                .build();

        return new DriveGuidanceTask(drivebase, plan, taskCfg);
    }

    /**
     * Drive to a pose defined relative to an AprilTag.
     *
     * <p>The final translation is {@code (forward, left)} in the tag's coordinate frame.
     * The final heading is the tag's heading + π (to face the tag) plus {@code headingOffsetRad}.</p>
     */
    public static Task goToPoseTagRelative(
            PoseEstimator poseEstimator,
            MecanumDrivebase drivebase,
            TagLayout tagLayout,
            int tagId,
            double forwardInches,
            double leftInches,
            double headingOffsetRad,
            DriveGuidancePlan.Tuning tuning,
            DriveGuidanceTask.Config taskCfg) {

        Objects.requireNonNull(poseEstimator, "poseEstimator");
        Objects.requireNonNull(drivebase, "drivebase");
        Objects.requireNonNull(tagLayout, "tagLayout");
        Objects.requireNonNull(tuning, "tuning");

        DriveGuidancePlan plan = DriveGuidance.plan()
                .translateTo()
                .tagRelativePointInches(tagId, forwardInches, leftInches)
                .doneTranslateTo()
                .aimTo()
                .tagFaceTagRad(tagId, headingOffsetRad)
                .doneAimTo()
                .feedback()
                .fieldPose(poseEstimator, tagLayout)
                .doneFeedback()
                .tuning(tuning)
                .build();

        return new DriveGuidanceTask(drivebase, plan, taskCfg);
    }

    /**
     * Hold the robot's starting position (in the field pose estimate) while turning to face a field heading.
     *
     * <p>This is the most common “aim only but keep position” autonomous behavior.</p>
     */
    public static Task holdPositionAndAimFieldHeading(
            PoseEstimator poseEstimator,
            MecanumDrivebase drivebase,
            double targetFieldHeadingRad,
            DriveGuidancePlan.Tuning tuning,
            DriveGuidanceTask.Config taskCfg) {

        Objects.requireNonNull(poseEstimator, "poseEstimator");
        Objects.requireNonNull(drivebase, "drivebase");
        Objects.requireNonNull(tuning, "tuning");

        DriveGuidancePlan plan = DriveGuidance.plan()
                .translateTo()
                // Captures a translation-frame anchor when the plan first becomes active.
                .robotRelativePointInches(0.0, 0.0)
                .doneTranslateTo()
                .aimTo()
                .fieldHeadingRad(targetFieldHeadingRad)
                .doneAimTo()
                .feedback()
                .fieldPose(poseEstimator)
                .doneFeedback()
                .tuning(tuning)
                .build();

        return new DriveGuidanceTask(drivebase, plan, taskCfg);
    }

    /**
     * Turn to face a field heading without holding position.
     */
    public static Task aimOnlyFieldHeading(
            PoseEstimator poseEstimator,
            MecanumDrivebase drivebase,
            double targetFieldHeadingRad,
            DriveGuidancePlan.Tuning tuning,
            DriveGuidanceTask.Config taskCfg) {

        Objects.requireNonNull(poseEstimator, "poseEstimator");
        Objects.requireNonNull(drivebase, "drivebase");
        Objects.requireNonNull(tuning, "tuning");

        DriveGuidancePlan plan = DriveGuidance.plan()
                .aimTo()
                .fieldHeadingRad(targetFieldHeadingRad)
                .doneAimTo()
                .feedback()
                .fieldPose(poseEstimator)
                .doneFeedback()
                .tuning(tuning)
                .build();

        DriveGuidanceTask.Config cfg = taskCfg != null ? taskCfg : new DriveGuidanceTask.Config();
        return new DriveGuidanceTask(drivebase, plan, cfg);
    }
}
