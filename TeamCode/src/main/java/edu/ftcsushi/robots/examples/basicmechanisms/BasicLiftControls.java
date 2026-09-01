package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.input.binding.CallbackBindings;
import edu.ftcsushi.fw.task.TaskBindings;

/** Owns the driver's named lift meanings, independent of drivetrain and claw controls. */
final class BasicLiftControls {

    private final GamepadDevice driver;
    private boolean bindAttempted;

    BasicLiftControls(GamepadDevice driver) {
        this.driver = Objects.requireNonNull(driver, "driver");
    }

    /** Declares the lift controls exactly once for one callback and Task graph. */
    void bind(CallbackBindings callbacks, TaskBindings tasks, BasicLift lift) {
        CallbackBindings requiredCallbacks = Objects.requireNonNull(callbacks, "callbacks");
        TaskBindings requiredTasks = Objects.requireNonNull(tasks, "tasks");
        BasicLift requiredLift = Objects.requireNonNull(lift, "lift");
        claimBind();

        requiredCallbacks.onRise(
                driver.dpadDown(),
                () -> requiredLift.setHeight(BasicLift.Height.STOWED));
        requiredCallbacks.onRise(
                driver.dpadLeft(),
                () -> requiredLift.setHeight(BasicLift.Height.LOW));
        requiredCallbacks.onRise(
                driver.dpadUp(),
                () -> requiredLift.setHeight(BasicLift.Height.HIGH));

        // A Supplier creates a fresh single-use homing Task on every X-button rise.
        requiredTasks.onRise(driver.x(), requiredLift::home);
    }

    private void claimBind() {
        if (bindAttempted) {
            throw new IllegalStateException(
                    "BasicLiftControls may be bound only once; create a fresh controls owner for "
                            + "another callback graph");
        }
        bindAttempted = true;
    }
}
