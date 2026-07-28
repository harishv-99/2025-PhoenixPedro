package edu.ftcphoenix.robots.examples.starter;

import java.util.Objects;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveSource;
import edu.ftcphoenix.fw.drive.source.GamepadDriveSource;
import edu.ftcphoenix.fw.input.GamepadDevice;
import edu.ftcphoenix.fw.input.binding.Bindings;

/** Owns every gamepad meaning used by the starter TeleOp. */
final class StarterTeleOpControls {

    private static final double SLOW_TRANSLATE_SCALE = 0.35;
    private static final double SLOW_OMEGA_SCALE = 0.20;

    private final Bindings bindings = new Bindings();
    private final DriveSource driveSource;

    StarterTeleOpControls(GamepadDevice driver, StarterIntake intake) {
        GamepadDevice requiredDriver = Objects.requireNonNull(driver, "driver");
        StarterIntake requiredIntake = Objects.requireNonNull(intake, "intake");

        driveSource = new GamepadDriveSource(
                requiredDriver.leftX(),
                requiredDriver.leftY(),
                requiredDriver.rightX(),
                GamepadDriveSource.Config.defaults()
        ).scaledWhen(requiredDriver.rightBumper(), SLOW_TRANSLATE_SCALE, SLOW_OMEGA_SCALE);

        bindings.onRise(
                requiredDriver.a(),
                () -> requiredIntake.setMode(StarterIntake.Mode.COLLECT));
        bindings.onRise(
                requiredDriver.b(),
                () -> requiredIntake.setMode(StarterIntake.Mode.EJECT));
        bindings.onRise(
                requiredDriver.x(),
                () -> requiredIntake.setMode(StarterIntake.Mode.STOPPED));
    }

    DriveSource driveSource() {
        return driveSource;
    }

    void update(LoopClock clock) {
        bindings.update(clock);
    }
}
