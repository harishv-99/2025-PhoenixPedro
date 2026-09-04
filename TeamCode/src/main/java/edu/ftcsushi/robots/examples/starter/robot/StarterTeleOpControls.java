package edu.ftcsushi.robots.examples.starter.robot;

import java.util.Objects;

import edu.ftcsushi.fw.drive.DriveSource;
import edu.ftcsushi.fw.drive.source.GamepadDriveSource;
import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.input.binding.CallbackBindings;
import edu.ftcsushi.robots.examples.starter.capability.intake.StarterIntake;

/** Owns every gamepad meaning used by the starter TeleOp. */
final class StarterTeleOpControls {

    static final double SLOW_TRANSLATE_SCALE = 0.35;
    static final double SLOW_OMEGA_SCALE = 0.20;

    private final GamepadDevice driver;
    private final DriveSource driveSource;
    private boolean bindAttempted;

    StarterTeleOpControls(GamepadDevice driver) {
        this.driver = Objects.requireNonNull(driver, "driver");

        driveSource = new GamepadDriveSource(
                this.driver.leftX(),
                this.driver.leftY(),
                this.driver.rightX(),
                GamepadDriveSource.Config.defaults()
        ).scaledWhen(this.driver.rightBumper(), SLOW_TRANSLATE_SCALE, SLOW_OMEGA_SCALE);
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
        CallbackBindings requiredCallbacks = Objects.requireNonNull(
                callbackBindings,
                "callbackBindings"
        );
        StarterIntake requiredIntake = Objects.requireNonNull(intake, "intake");
        claimBind();

        requiredCallbacks.onRise(
                driver.a(),
                () -> requiredIntake.setMode(StarterIntake.Mode.COLLECT));
        requiredCallbacks.onRise(
                driver.b(),
                () -> requiredIntake.setMode(StarterIntake.Mode.EJECT));
        requiredCallbacks.onRise(
                driver.x(),
                () -> requiredIntake.setMode(StarterIntake.Mode.STOPPED));
    }

    private void claimBind() {
        if (bindAttempted) {
            throw new IllegalStateException(
                    "StarterTeleOpControls.bind(...) may be called only once; "
                            + "create a fresh controls owner for another callback graph"
            );
        }
        bindAttempted = true;
    }

    DriveSource driveSource() {
        return driveSource;
    }
}
