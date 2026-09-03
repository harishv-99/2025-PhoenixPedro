package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.task.Task;

/** Claw-only Auto showing that a standard-servo Task requests intent without claiming arrival. */
@Autonomous(name = "FW Basic: Claw Auto", group = "FW Examples")
@Disabled
public final class BasicClawAuto extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        BasicClawProfile profile = BasicClawProfile.current();
        BasicClawProfile.requireMotionAllowed(profile, "Basic Claw Auto");

        BasicClawMechanism claw = program.output(
                new BasicClawMechanism(hardwareMap, profile.claw));
        Task requestOpen = claw.setStateTask(BasicClaw.State.OPEN);
        program.rootTask(requestOpen);
        program.presenter((clock, telemetry) -> {
            BasicClaw.Status status = claw.status();
            telemetry.addData("claw.request", status.requestedState());
            telemetry.addData("claw.appliedCoordinate", "%.2f", status.appliedCoordinate());
            telemetry.addLine("Task success means the OPEN request was published, not measured");
        });
    }
}
