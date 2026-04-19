package edu.ftcphoenix.fw.drive.guidance;

import java.util.Objects;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.spatial.ReferenceFrame2d;
import edu.ftcphoenix.fw.spatial.References;
import edu.ftcphoenix.fw.task.Task;

/**
 * Convenience factory methods that build go-to-pose behaviors using the
 * {@link DriveGuidancePlan} infrastructure.
 *
 * <p>This keeps the “go to pose” concept (very common in autonomous) while reusing
 * the same aim/translate/resolveWith/tuning machinery as TeleOp DriveGuidance overlays.</p>
 */
public final class GoToPoseTasks {

    private GoToPoseTasks() {
        // utility
    }

    /**
     * Drive to an absolute pose in the field frame.
     */
    public static Task goToPoseFieldRelative(
            AbsolutePoseEstimator poseEstimator,
            DriveCommandSink drivebase,
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
                .faceTo()
                .fieldHeadingRad(targetFieldPose.headingRad)
                .doneFaceTo()
                .resolveWith()
                .localizationOnly()
                .localization(poseEstimator)
                .doneResolveWith()
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
            AbsolutePoseEstimator poseEstimator,
            DriveCommandSink drivebase,
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

        ReferenceFrame2d target = References.relativeToTagFrame(
                tagId,
                forwardInches,
                leftInches,
                Math.PI + headingOffsetRad
        );

        DriveGuidancePlan plan = DriveGuidance.plan()
                .translateTo()
                .point(References.framePoint(target))
                .doneTranslateTo()
                .faceTo()
                .frameHeading(target)
                .doneFaceTo()
                .resolveWith()
                .localizationOnly()
                .localization(poseEstimator)
                .fixedAprilTagLayout(tagLayout)
                .doneResolveWith()
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
            AbsolutePoseEstimator poseEstimator,
            DriveCommandSink drivebase,
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
                .faceTo()
                .fieldHeadingRad(targetFieldHeadingRad)
                .doneFaceTo()
                .resolveWith()
                .localizationOnly()
                .localization(poseEstimator)
                .doneResolveWith()
                .tuning(tuning)
                .build();

        return new DriveGuidanceTask(drivebase, plan, taskCfg);
    }

    /**
     * Turn to face a field heading without holding position.
     */
    public static Task aimOnlyFieldHeading(
            AbsolutePoseEstimator poseEstimator,
            DriveCommandSink drivebase,
            double targetFieldHeadingRad,
            DriveGuidancePlan.Tuning tuning,
            DriveGuidanceTask.Config taskCfg) {

        Objects.requireNonNull(poseEstimator, "poseEstimator");
        Objects.requireNonNull(drivebase, "drivebase");
        Objects.requireNonNull(tuning, "tuning");

        DriveGuidancePlan plan = DriveGuidance.plan()
                .faceTo()
                .fieldHeadingRad(targetFieldHeadingRad)
                .doneFaceTo()
                .resolveWith()
                .localizationOnly()
                .localization(poseEstimator)
                .doneResolveWith()
                .tuning(tuning)
                .build();

        DriveGuidanceTask.Config cfg = taskCfg != null ? taskCfg : new DriveGuidanceTask.Config();
        return new DriveGuidanceTask(drivebase, plan, cfg);
    }
}
