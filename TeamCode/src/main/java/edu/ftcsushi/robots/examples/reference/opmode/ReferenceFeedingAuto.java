package edu.ftcsushi.robots.examples.reference.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.robots.examples.reference.capability.launcher.ReferenceLauncher;
import edu.ftcsushi.robots.examples.reference.capability.launcher.ReferenceLauncherMechanism;

/** Disabled and separately locked Auto that runs one feed, without retries or automatic recovery. */
@Autonomous(name = "FW Reference: One Feedback-confirmed Feed", group = "FW Examples")
@Disabled
public final class ReferenceFeedingAuto extends FtcRobotOpMode {
    private static final boolean MOTION_REVIEWED = false;

    @Override
    protected void configure(RobotProgram program) {
        if (!MOTION_REVIEWED) {
            throw new IllegalStateException("Reference feed Auto is locked. Review device identity, "
                    + "staged-sensor placement, directions, servo endpoints, loaded-wheel behavior, "
                    + "interruption/retraction, containment and emergency STOP before setting "
                    + "MOTION_REVIEWED=true and removing @Disabled. START requests one attempt.");
        }

        ReferenceLauncherMechanism.Config config = ReferenceLauncherMechanism.Config.defaults();
        ReferenceLauncherMechanism launcher = program.output(
                new ReferenceLauncherMechanism(hardwareMap, config));
        Task feed = launcher.feedOne();
        program.rootTask(feed);

        program.presenter((clock, telemetry) -> {
            ReferenceLauncher.Status status = launcher.status();
            telemetry.addData("feed.outcome", feed.getOutcome());
            telemetry.addData("feed.phase", status.phase());
            telemetry.addData("feed.reason", status.reason());
            telemetry.addData("feed.recoveryRequired", status.recoveryRequired());
            telemetry.addLine("One attempt at START; no retry or automatic recovery.");
        });
    }
}
