package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.task.Task;

/** Mechanism-only Auto host used to learn sequencing, parallel work, and success gates. */
@Autonomous(name = "FW Basic: Mechanisms Auto", group = "FW Examples")
@Disabled
public final class BasicMechanismsAuto extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        BasicLiftProfile liftProfile = BasicLiftProfile.current();
        BasicClawProfile clawProfile = BasicClawProfile.current();
        BasicLiftProfile.requireMotionAllowed(liftProfile, "Basic Mechanisms Auto");
        BasicClawProfile.requireMotionAllowed(clawProfile, "Basic Mechanisms Auto");

        // Register each owner immediately so a later construction failure still receives STOP.
        BasicLiftMechanism lift = program.output(
                new BasicLiftMechanism(hardwareMap, liftProfile.lift));
        BasicClawMechanism claw = program.output(
                new BasicClawMechanism(hardwareMap, clawProfile.claw));

        Task auto = BasicAutoRoutines.guide(lift, claw);
        program.rootTask(auto);
        program.presenter((clock, telemetry) -> {
            BasicLift.Status liftStatus = lift.status();
            BasicClaw.Status clawStatus = claw.status();
            telemetry.addData("lift", "%s %.2f/%.2f in referenced=%s atTarget=%s",
                    liftStatus.requestedHeight(),
                    liftStatus.measuredPositionIn(),
                    liftStatus.requestedPositionIn(),
                    liftStatus.referenced(),
                    liftStatus.atTarget());
            telemetry.addData("claw", "%s coordinate=%.2f",
                    clawStatus.requestedState(), clawStatus.appliedCoordinate());
            telemetry.addData("auto.complete", auto.isComplete());
            telemetry.addData("auto.outcome", auto.getOutcome());
        });
    }
}
