package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import edu.ftcsushi.fw.core.lifecycle.CleanupActions;
import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.drive.DriveTasks;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.task.Task;

/**
 * Bounded drive-only Auto with one exclusive Task command path and a stop-only lifecycle owner.
 */
@Autonomous(name = "FW Basic 6: Drive Auto", group = "FW Examples")
@Disabled
public final class BasicDriveAuto extends FtcRobotOpMode {

    /** Robot-centric axial request before the profile's {@code maxAxial} mixer scaling. */
    private static final double FORWARD_REQUEST = 0.20;

    /** Positive duration for which the request remains observable, in seconds. */
    private static final double FORWARD_DURATION_SEC = 0.75;

    @Override
    protected void configure(RobotProgram program) {
        BasicDriveProfile driveProfile = BasicDriveProfile.current();
        BasicDriveProfile.requireMotionAllowed(driveProfile, "Basic Drive Auto");

        // This sink belongs only to DriveTasks; do not also pass it to program.drive(...).
        DriveCommandSink autoDrive = FtcDrives.mecanum(hardwareMap, driveProfile.drive);
        BasicDriveStopOwner driveStopOwner = new BasicDriveStopOwner(autoDrive);
        try {
            // A stop-only owner is registered before the root Task can fail to be constructed.
            program.output(driveStopOwner);
            Task auto = DriveTasks.driveExclusivelyForSeconds(
                    autoDrive,
                    new DriveSignal(FORWARD_REQUEST, 0.0, 0.0),
                    FORWARD_DURATION_SEC);
            program.rootTask(auto);
            program.presenter((clock, telemetry) -> {
                telemetry.addData("auto.complete", auto.isComplete());
                telemetry.addData("auto.outcome", auto.getOutcome());
            });
        } catch (RuntimeException failure) {
            throw CleanupActions.attemptAllAfterFailure(failure, driveStopOwner::stop);
        }
    }
}
