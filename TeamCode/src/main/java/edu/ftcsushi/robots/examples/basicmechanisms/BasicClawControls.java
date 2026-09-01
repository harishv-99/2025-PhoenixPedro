package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.input.binding.CallbackBindings;

/** Owns the driver's named claw meanings, independent of drivetrain and lift controls. */
final class BasicClawControls {

    private final GamepadDevice driver;
    private boolean bindAttempted;

    BasicClawControls(GamepadDevice driver) {
        this.driver = Objects.requireNonNull(driver, "driver");
    }

    /** Declares the claw controls exactly once for one callback graph. */
    void bind(CallbackBindings callbacks, BasicClaw claw) {
        CallbackBindings requiredCallbacks = Objects.requireNonNull(callbacks, "callbacks");
        BasicClaw requiredClaw = Objects.requireNonNull(claw, "claw");
        claimBind();

        requiredCallbacks.onRise(
                driver.a(),
                () -> requiredClaw.setState(BasicClaw.State.CLOSED));
        requiredCallbacks.onRise(
                driver.b(),
                () -> requiredClaw.setState(BasicClaw.State.OPEN));
    }

    private void claimBind() {
        if (bindAttempted) {
            throw new IllegalStateException(
                    "BasicClawControls may be bound only once; create a fresh controls owner for "
                            + "another callback graph");
        }
        bindAttempted = true;
    }
}
