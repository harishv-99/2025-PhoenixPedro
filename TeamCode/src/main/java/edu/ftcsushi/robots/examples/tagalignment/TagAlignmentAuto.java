package edu.ftcsushi.robots.examples.tagalignment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.MecanumDrivebase;
import edu.ftcsushi.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.ftc.vision.AprilTagVision;

/** One bounded direct-tag approach; TIMEOUT is not success and never starts another action here. */
@Autonomous(name = "FW Direct Tag Align Auto", group = "FW Examples")
@Disabled
public final class TagAlignmentAuto extends FtcRobotOpMode {
    @Override protected void configure(RobotProgram program) {
        TagAlignmentProfile profile = TagAlignmentProfile.example();
        profile.requireMotionAllowed();
        TagAlignmentCamera camera = program.service(new TagAlignmentCamera(hardwareMap, profile.camera));
        AprilTagVision tags = camera.tags();
        DriveGuidancePlan plan = TagAlignment.plan(profile, tags.tagSensor(), tags.cameraMountConfig());
        AutoDrive drive = program.service(new AutoDrive(hardwareMap, profile.drive));
        DriveGuidanceTask approach = plan.task(drive.sink, profile.auto); // Fresh for this run.
        program.rootTask(approach);
        program.presenter((clock, telemetry) -> {
            telemetry.addData("alignment.outcome", approach.getOutcome());
            telemetry.addData("alignment.loss", "Unavailable tag requests zero; timeout is not arrival");
        });
    }

    /** Owns drive lifecycle; the Task is the only command writer. No idle output overwrites it. */
    private static final class AutoDrive implements RobotProgram.Service {
        final MecanumDrivebase sink;
        private boolean stopped;

        AutoDrive(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap,
                  FtcDrives.MecanumConfig config) {
            sink = FtcDrives.mecanum(hardwareMap, config);
        }

        @Override public void update(LoopClock clock) {
            // This direct mecanum sink has no independent follower heartbeat.
        }

        @Override public void stop() {
            if (stopped) return;
            stopped = true;
            sink.stop();
        }
    }
}
