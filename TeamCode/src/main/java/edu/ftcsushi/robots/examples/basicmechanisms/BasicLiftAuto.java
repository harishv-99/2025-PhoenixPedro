package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.Tasks;

/** Lift-only Auto whose later moves start only after reference and feedback success. */
@Autonomous(name = "FW Basic: Lift Auto", group = "FW Examples")
@Disabled
public final class BasicLiftAuto extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        BasicLiftProfile profile = BasicLiftProfile.current();
        BasicLiftProfile.requireMotionAllowed(profile, "Basic Lift Auto");

        BasicLiftMechanism lift = program.output(
                new BasicLiftMechanism(hardwareMap, profile.lift));
        Task auto = Tasks.sequence(
                lift.home(),
                lift.moveTo(BasicLift.Height.HIGH),
                lift.moveTo(BasicLift.Height.STOWED));
        program.rootTask(auto);
        program.presenter((clock, telemetry) -> {
            BasicLift.Status status = lift.status();
            telemetry.addData("lift.request", status.requestedHeight());
            telemetry.addData("lift.positionIn", "%.2f / %.2f",
                    status.measuredPositionIn(), status.requestedPositionIn());
            telemetry.addData("lift.referenced", status.referenced());
            telemetry.addData("auto.outcome", auto.getOutcome());
        });
    }
}
