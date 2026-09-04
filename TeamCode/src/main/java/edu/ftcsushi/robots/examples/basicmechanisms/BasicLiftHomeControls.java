package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.task.TaskBindings;

/** Owns the driver's single X-to-home meaning for the isolated reference lesson. */
final class BasicLiftHomeControls {

    private final GamepadDevice driver;
    private boolean bindAttempted;

    BasicLiftHomeControls(GamepadDevice driver) {
        this.driver = Objects.requireNonNull(driver, "driver");
    }

    /** Declares X as a fresh home Task exactly once for one Task binding graph. */
    void bind(TaskBindings tasks, BasicLift lift) {
        TaskBindings requiredTasks = Objects.requireNonNull(tasks, "tasks");
        BasicLift requiredLift = Objects.requireNonNull(lift, "lift");
        claimBind();

        requiredTasks.onRise(driver.x(), requiredLift::home);
    }

    private void claimBind() {
        if (bindAttempted) {
            throw new IllegalStateException(
                    "BasicLiftHomeControls may be bound only once; create a fresh controls owner "
                            + "for another Task binding graph");
        }
        bindAttempted = true;
    }
}
