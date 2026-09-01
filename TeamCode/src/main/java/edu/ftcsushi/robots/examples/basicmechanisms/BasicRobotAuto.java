package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import edu.ftcsushi.fw.core.lifecycle.CleanupActions;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.task.Task;

/** Complete basic Auto host with one exclusive fixed-time drive phase. */
@Autonomous(name = "FW Basic 7: Robot Auto", group = "FW Examples")
@Disabled
public final class BasicRobotAuto extends FtcRobotOpMode {

    @Override
    protected void configure(RobotProgram program) {
        BasicDriveProfile driveProfile = BasicDriveProfile.current();
        BasicLiftProfile liftProfile = BasicLiftProfile.current();
        BasicClawProfile clawProfile = BasicClawProfile.current();
        BasicDriveProfile.requireMotionAllowed(driveProfile, "Basic Robot Auto");
        BasicLiftProfile.requireMotionAllowed(liftProfile, "Basic Robot Auto");
        BasicClawProfile.requireMotionAllowed(clawProfile, "Basic Robot Auto");
        BasicHardwareOwnership.requireDistinctDriveAndLiftMotors(
                driveProfile.drive, liftProfile.lift, "Basic Robot Auto");

        BasicLiftMechanism lift = program.output(
                new BasicLiftMechanism(hardwareMap, liftProfile.lift));
        BasicClawMechanism claw = program.output(
                new BasicClawMechanism(hardwareMap, clawProfile.claw));

        // This sink belongs only to DriveTasks; do not also pass it to program.drive(...).
        DriveCommandSink autoDrive = FtcDrives.mecanum(hardwareMap, driveProfile.drive);
        BasicDriveStopOwner driveStopOwner = new BasicDriveStopOwner(autoDrive);
        try {
            // A stop-only owner covers STOP even if a lift prerequisite ends before drive starts.
            program.output(driveStopOwner);
            Task auto = BasicRobotAutoRoutines.complete(lift, claw, autoDrive);
            program.rootTask(auto);
            program.presenter((clock, telemetry) -> {
                BasicLift.Status liftStatus = lift.status();
                BasicClaw.Status clawStatus = claw.status();
                telemetry.addData("lift", "%s %.2f/%.2f in referenced=%s atTarget=%s",
                        liftStatus.requestedHeight,
                        liftStatus.measuredPositionIn,
                        liftStatus.requestedPositionIn,
                        liftStatus.referenced,
                        liftStatus.atTarget);
                telemetry.addData("claw", "%s command=%.2f",
                        clawStatus.requestedState, clawStatus.appliedPosition);
                telemetry.addData("auto.complete", auto.isComplete());
                telemetry.addData("auto.outcome", auto.getOutcome());
            });
        } catch (RuntimeException failure) {
            // Preserve the failure while still attempting the acquired sink's software STOP.
            throw CleanupActions.attemptAllAfterFailure(failure, driveStopOwner::stop);
        }
    }
}
