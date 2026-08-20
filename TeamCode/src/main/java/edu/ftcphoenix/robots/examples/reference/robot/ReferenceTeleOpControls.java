package edu.ftcphoenix.robots.examples.reference.robot;

import java.util.Objects;

import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.ftc.input.GamepadDevice;
import edu.ftcphoenix.fw.input.binding.CallbackBindings;
import edu.ftcphoenix.fw.task.TaskBindings;
import edu.ftcphoenix.robots.examples.reference.capability.launcher.ReferenceLauncher;
import edu.ftcphoenix.robots.examples.reference.capability.lift.ReferenceLift;

/** Keeps operator meanings separate from the reference mechanisms that realize them. */
final class ReferenceTeleOpControls {
    private final GamepadDevice gamepad;
    private final DriveSource drive;

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
        Objects.requireNonNull(callbacks, "callbacks");
        Objects.requireNonNull(tasks, "tasks");
        Objects.requireNonNull(lift, "lift");
        Objects.requireNonNull(launcher, "launcher");

        callbacks.onRise(gamepad.dpadDown(),
                () -> lift.setHeight(ReferenceLift.Height.STOWED));
        callbacks.onRise(gamepad.dpadLeft(),
                () -> lift.setHeight(ReferenceLift.Height.LOW));
        callbacks.onRise(gamepad.dpadUp(),
                () -> lift.setHeight(ReferenceLift.Height.HIGH));
        tasks.onRise(gamepad.x(), lift::home);

        callbacks.onRise(gamepad.b(), launcher::idle);
        tasks.onRise(gamepad.y(), launcher::launchOne);
    }

    DriveSource driveSource() {
        return drive;
    }
}
