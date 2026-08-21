package edu.ftcphoenix.robots.examples.reference.autonomous;

import java.util.Objects;

import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.robots.examples.reference.capability.launcher.ReferenceLauncher;
import edu.ftcphoenix.robots.examples.reference.capability.lift.ReferenceLift;
import edu.ftcphoenix.robots.examples.reference.robot.ReferenceCapabilities;

/** Autonomous policy composed from the same Reference capabilities used by TeleOp. */
public final class ReferenceAutoRoutines {
    private ReferenceAutoRoutines() {
        // Static robot-owned routine factories.
    }

    /**
     * Homes, waits for LOW, then launches one object.
     *
     * <p>Each phase must report {@link TaskOutcome#SUCCESS} before the next motion starts. A
     * timeout requests launcher cleanup while the enclosing sequence retains that timeout. Direct
     * active cancellation remains terminal and never constructs a later phase or cleanup branch;
     * the managed program then stops every declared output owner.</p>
     *
     * @param capabilities non-null Reference capability handoff
     * @return fresh single-use root Task
     */
    public static Task homeMoveLowThenLaunch(ReferenceCapabilities capabilities) {
        ReferenceCapabilities c = Objects.requireNonNull(capabilities, "capabilities");
        ReferenceLift lift = Objects.requireNonNull(c.lift(), "capabilities.lift()");
        ReferenceLauncher launcher = Objects.requireNonNull(
                c.launcher(), "capabilities.launcher()");

        Task home = lift.home();
        return Tasks.sequence(
                home,
                Tasks.buildAtStart("Reference Auto after home", () ->
                        home.getOutcome() == TaskOutcome.SUCCESS
                                ? moveLowThenLaunch(lift, launcher)
                                : abortLaunches(launcher)));
    }

    private static Task moveLowThenLaunch(ReferenceLift lift,
                                          ReferenceLauncher launcher) {
        Task moveLow = lift.moveTo(ReferenceLift.Height.LOW);
        return Tasks.sequence(
                moveLow,
                Tasks.buildAtStart("Reference Auto after LOW", () ->
                        moveLow.getOutcome() == TaskOutcome.SUCCESS
                                ? launchThenCheck(launcher)
                                : abortLaunches(launcher)));
    }

    private static Task launchThenCheck(ReferenceLauncher launcher) {
        Task launch = launcher.launchOne();
        return Tasks.sequence(
                launch,
                Tasks.buildAtStart("Reference Auto after launch", () ->
                        launch.getOutcome() == TaskOutcome.SUCCESS
                                ? Tasks.noop()
                                : abortLaunches(launcher)));
    }

    private static Task abortLaunches(ReferenceLauncher launcher) {
        return Tasks.runOnce(launcher::abortLaunches);
    }
}
