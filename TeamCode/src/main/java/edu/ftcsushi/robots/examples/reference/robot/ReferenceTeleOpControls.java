package edu.ftcsushi.robots.examples.reference.robot;

import java.util.Objects;

import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.drive.source.GamepadDriveSource;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.input.binding.CallbackBindings;
import edu.ftcsushi.fw.task.TaskBindings;
import edu.ftcsushi.robots.examples.reference.capability.launcher.ReferenceLauncher;
import edu.ftcsushi.robots.examples.reference.capability.lift.ReferenceLift;

/** Keeps operator meanings separate from the reference mechanisms that realize them. */
final class ReferenceTeleOpControls {
    private final GamepadDevice gamepad;
    private final DriveSource drive;
    private boolean bindAttempted;

    ReferenceTeleOpControls(GamepadDevice gamepad) {
        this.gamepad = Objects.requireNonNull(gamepad, "gamepad");
        drive = new GamepadDriveSource(
                gamepad.leftX(), gamepad.leftY(), gamepad.rightX(),
                GamepadDriveSource.Config.defaults());
    }

    void bind(CallbackBindings callbacks,
              TaskBindings tasks,
              ReferenceLift lift,
              ReferenceLauncher launcher) {
        CallbackBindings requiredCallbacks = Objects.requireNonNull(callbacks, "callbacks");
        TaskBindings requiredTasks = Objects.requireNonNull(tasks, "tasks");
        ReferenceLift requiredLift = Objects.requireNonNull(lift, "lift");
        ReferenceLauncher requiredLauncher = Objects.requireNonNull(launcher, "launcher");
        claimBind();

        requiredCallbacks.onRise(gamepad.dpadDown(),
                () -> requiredLift.setHeight(ReferenceLift.Height.STOWED));
        requiredCallbacks.onRise(gamepad.dpadLeft(),
                () -> requiredLift.setHeight(ReferenceLift.Height.LOW));
        requiredCallbacks.onRise(gamepad.dpadUp(),
                () -> requiredLift.setHeight(ReferenceLift.Height.HIGH));
        requiredTasks.onRise(gamepad.x(), requiredLift::home);

        // TaskBindings delegates to the same ordered callback graph. Declare Y first so a
        // simultaneous B rise invalidates the just-created launch before the Task runner phase.
        requiredTasks.onRise(gamepad.y(), requiredLauncher::launchOne);
        requiredCallbacks.onRise(gamepad.b(), requiredLauncher::abortLaunches);
    }

    /** Consume the single declaration attempt before the first callback registration. */
    private void claimBind() {
        if (bindAttempted) {
            throw new IllegalStateException(
                    "ReferenceTeleOpControls.bind(...) may be called only once; create a fresh "
                            + "controls owner for another callback graph");
        }
        bindAttempted = true;
    }

    DriveSource driveSource() {
        return drive;
    }
}
