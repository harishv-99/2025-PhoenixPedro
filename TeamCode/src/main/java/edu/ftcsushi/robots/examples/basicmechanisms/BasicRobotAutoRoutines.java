package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.drive.DriveSignal;
import edu.ftcsushi.fw.drive.DriveTasks;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.Tasks;

/** Full-robot Auto policy composed from semantic mechanisms and one exclusive drive sink. */
public final class BasicRobotAutoRoutines {

    /** Robot-centric axial request before {@link BasicDriveProfile} mixer scaling. */
    private static final double FORWARD_REQUEST = 0.20;

    /** Positive duration for which the axial request remains observable, in seconds. */
    private static final double FORWARD_DURATION_SEC = 0.75;

    private BasicRobotAutoRoutines() {
        // Static robot-owned routine factories.
    }

    /**
     * Builds a complete basic Auto that homes, carries, drives forward briefly, releases, and
     * stows.
     *
     * <p>The direct drive sink is exclusive to the timed {@link DriveTasks} phase; callers must
     * not also register it through {@code program.drive(...)}. Lift feedback gates every later
     * motion. The fixed graph calls these side-effect-free capability Task factories eagerly, but
     * normal cancellation stops the active task graph without starting a later phase.</p>
     *
     * @return fresh single-use full-robot root Task
     */
    public static Task complete(BasicLift lift,
                                BasicClaw claw,
                                DriveCommandSink driveSink) {
        BasicLift requiredLift = Objects.requireNonNull(lift, "lift");
        BasicClaw requiredClaw = Objects.requireNonNull(claw, "claw");
        DriveCommandSink requiredDrive = Objects.requireNonNull(driveSink, "driveSink");

        return Tasks.sequence(
                requiredLift.home(),
                // The lift deadline preserves its exact outcome; CLOSED publishes concurrently
                // and remains held after its immediate Task succeeds.
                Tasks.parallelDeadline(
                        requiredLift.moveTo(BasicLift.Height.HIGH),
                        requiredClaw.setStateTask(BasicClaw.State.CLOSED)),
                DriveTasks.driveExclusivelyForSeconds(
                        requiredDrive,
                        new DriveSignal(FORWARD_REQUEST, 0.0, 0.0),
                        FORWARD_DURATION_SEC),
                Tasks.parallelDeadline(
                        requiredLift.moveTo(BasicLift.Height.STOWED),
                        requiredClaw.setStateTask(BasicClaw.State.OPEN)));
    }
}
