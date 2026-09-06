package edu.ftcsushi.robots.examples.starter.robot;

import java.util.Objects;

import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.drive.source.GamepadDriveSource;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.input.binding.CallbackBindings;
import edu.ftcsushi.robots.examples.starter.capability.intake.StarterIntake;

/** Combines the shared intake controls with the Starter TeleOp's drive meanings. */
final class StarterTeleOpControls {

    static final double SLOW_TRANSLATE_SCALE = 0.35;
    static final double SLOW_OMEGA_SCALE = 0.20;

    private final StarterIntakeControls intakeControls;
    private final DriveSource driveSource;

    /** Builds stable drive sources and the shared intake owner without registering callbacks. */
    StarterTeleOpControls(GamepadDevice driver) {
        GamepadDevice requiredDriver = Objects.requireNonNull(driver, "driver");
        intakeControls = new StarterIntakeControls(requiredDriver);

        driveSource = new GamepadDriveSource(
                requiredDriver.leftX(),
                requiredDriver.leftY(),
                requiredDriver.rightX(),
                GamepadDriveSource.Config.defaults()
        ).scaledWhen(requiredDriver.rightBumper(), SLOW_TRANSLATE_SCALE, SLOW_OMEGA_SCALE);
    }

    /**
     * Declare this controls owner's callback mappings exactly once.
     *
     * @param callbackBindings managed callback surface; validated before the bind is claimed
     * @param intake semantic intake capability; validated before the bind is claimed
     * @throws NullPointerException if either argument is {@code null}; this does not consume the
     *                              bind opportunity
     * @throws IllegalStateException if a bind was already attempted, including one whose callback
     *                               registration failed partway through
     */
    void bind(CallbackBindings callbackBindings, StarterIntake intake) {
        intakeControls.bind(callbackBindings, intake);
    }

    /** Returns the stable shaped drive source, including right-bumper slow mode. */
    DriveSource driveSource() {
        return driveSource;
    }
}
