package edu.ftcsushi.robots.examples.tagalignment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ftcsushi.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.ftc.vision.AprilTagVision;

/** Held direct-tag alignment, deliberately disabled until this robot's configuration is reviewed. */
@TeleOp(name = "FW Direct Tag Align", group = "FW Examples")
@Disabled
public final class TagAlignmentTeleOp extends FtcRobotOpMode {
    @Override protected void configure(RobotProgram program) {
        TagAlignmentProfile profile = TagAlignmentProfile.example();
        profile.requireMotionAllowed();
        TagAlignmentCamera camera = program.service(new TagAlignmentCamera(hardwareMap, profile.camera));
        AprilTagVision tags = camera.tags();
        DriveGuidancePlan plan = TagAlignment.plan(profile, tags.tagSensor(), tags.cameraMountConfig());
        TagAlignmentControls controls = new TagAlignmentControls(new GamepadDevice(gamepad1), plan);
        program.drive(controls.driveSource(), FtcDrives.mecanum(hardwareMap, profile.drive));
        program.presenter((clock, telemetry) -> {
            telemetry.addData("alignment", controls.status());
            telemetry.addData("target", "Tag " + profile.tagId + "; no field localization");
        });
    }
}
