package edu.ftcsushi.robots.examples.starter.robot;

import java.util.Objects;

import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.input.binding.CallbackBindings;
import edu.ftcsushi.robots.examples.starter.capability.intake.StarterIntake;

/** Owns the A/B/X intake meanings shared by the focused and combined Starter TeleOps. */
final class StarterIntakeControls {

    private final GamepadDevice driver;
    private boolean bindAttempted;

    /** Retains the stable gamepad input adapter without registering callbacks. */
    StarterIntakeControls(GamepadDevice driver) {
        this.driver = Objects.requireNonNull(driver, "driver");
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

    /** Claims the one registration attempt before any callback graph can be mutated. */
    private void claimBind() {
        if (bindAttempted) {
            throw new IllegalStateException(
                    "StarterIntakeControls.bind(...) may be called only once; "
                            + "create a fresh controls owner for another callback graph"
            );
        }
        bindAttempted = true;
    }
}
