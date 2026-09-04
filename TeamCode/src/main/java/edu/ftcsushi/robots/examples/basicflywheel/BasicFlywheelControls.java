package edu.ftcsushi.robots.examples.basicflywheel;

import java.util.Objects;

import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.input.binding.CallbackBindings;

/** Owns the operator's flywheel meanings for the isolated velocity lesson. */
final class BasicFlywheelControls {
    private final GamepadDevice operator;
    private final double candidateVelocityTicksPerSec;
    private boolean bindAttempted;

    /** Creates controls for one explicitly chosen positive candidate velocity. */
    BasicFlywheelControls(GamepadDevice operator, double candidateVelocityTicksPerSec) {
        this.operator = Objects.requireNonNull(operator, "operator");
        if (!Double.isFinite(candidateVelocityTicksPerSec)
                || candidateVelocityTicksPerSec <= 0.0) {
            throw new IllegalArgumentException(
                    "candidateVelocityTicksPerSec must be finite and > 0, got "
                            + candidateVelocityTicksPerSec);
        }
        this.candidateVelocityTicksPerSec = candidateVelocityTicksPerSec;
    }

    /** Declares the direct test-fixture meanings exactly once for one binding graph. */
    void bind(CallbackBindings callbacks, BasicFlywheel flywheel) {
        CallbackBindings requiredCallbacks = Objects.requireNonNull(callbacks, "callbacks");
        BasicFlywheel requiredFlywheel = Objects.requireNonNull(flywheel, "flywheel");
        claimBind();

        requiredCallbacks.onRise(
                operator.a(),
                () -> requiredFlywheel.setVelocityTicksPerSec(candidateVelocityTicksPerSec));
        requiredCallbacks.onRise(
                operator.b(),
                () -> requiredFlywheel.setVelocityTicksPerSec(0.0));
    }

    private void claimBind() {
        if (bindAttempted) {
            throw new IllegalStateException(
                    "BasicFlywheelControls may be bound only once; create a fresh controls owner "
                            + "for another callback graph");
        }
        bindAttempted = true;
    }
}
