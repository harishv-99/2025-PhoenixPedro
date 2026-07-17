# Software PIDF Tuning Workflow

Use this workflow for a Phoenix software-regulated Plant built with
`ScalarRegulators.pidf(...)`. FTC device-managed `.velocityPidf(...)` configures a motor
controller instead and has a different tuning path.

## The one rule

Tune in a dedicated tuning/tester OpMode. Production TeleOp and Auto start only from checked-in
profile values and never read mutable tuning-UI state.

A live value is not production configuration merely because the mechanism ran successfully. It
becomes production configuration only after a student copies it into the robot profile, reviews and
commits the source, and starts a fresh production OpMode that loads that checked-in snapshot.

## Before tuning

Before enabling a mechanism:

1. Secure the robot and keep people and loose objects clear of moving hardware.
2. Start from the current checked-in four-gain tuple.
3. Configure the Plant's target range and the complete regulator's output range before tuning.
4. Start with a zero or disabled target and require a deliberate arm action.
5. Make the tuning mode's stop and failure paths request zero and stop the owned mechanism.
6. Retain both the standard `PidfRegulator` and its outermost regulator composition in the robot
   realization.

Finite gains are not automatically safe gains. The robot owner still chooses conservative test
targets, output bounds, arming policy, and physical precautions.

## Retain the two correct handles

The inner handle owns the standard four PIDF gains. The outer handle owns the complete composition
that may also include voltage compensation, output limiting, or another stateful decorator:

```java
private final PidfRegulator flywheelPidf;
private final ScalarRegulator flywheelRegulator;

FlywheelRealization(double kP,
                    double kI,
                    double kD,
                    double kF,
                    double maximumPower) {
    flywheelPidf = ScalarRegulators.pidf(kP, kI, kD, kF)
            .setIntegralLimits(-0.15, 0.15)
            .setPidOutputLimits(-1.0, 1.0);

    flywheelRegulator =
            ScalarRegulators.outputLimited(flywheelPidf, 0.0, maximumPower);
}
```

Keep those fields inside realization. Normal capability methods and match OpModes should continue
to express mechanism intent, such as selected velocity and enabled state; they should not expose or
manipulate controller pieces.

## Apply one complete candidate

The robot realization needs only this local method:

```java
void applyPidfCandidate(double kP, double kI, double kD, double kF) {
    flywheelPidf.setGains(kP, kI, kD, kF);
    try {
        flywheelRegulator.reset();
    } catch (RuntimeException resetFailure) {
        throw new IllegalStateException(
                "PIDF gains changed, but outer regulator reset failed",
                resetFailure);
    }
}
```

`setGains(...)` validates all four finite values before changing any applied gain. An invalid
candidate leaves the previous four gains in place. Reset the outermost composition so Phoenix's
built-in nested decorators and controllers clear together; any custom stateful decorator must honor
and propagate `reset()`. Reset does not itself command or stop the actuator. The next ordinary
Plant update uses the new control law.

The PIDF object cannot discover wrappers around itself, which is why the realization that built the
composition owns the reset. Wrapping a reset failure as shown prevents it from being mistaken for
gain rejection after the gains have already changed.

## Use one explicit apply event

The safest default is to accept a new candidate only while the mechanism is disarmed. A robot may
deliberately allow active tuning only after it defines conservative targets, output bounds, and
failure behavior for that mechanism.

The following disarmed-apply example shows the loop shape. The `ui`, `requestedRpm`, and mechanism
methods are robot-local placeholders, not additional Phoenix APIs:

```java
clock.update(getRuntime());

try {
    if (ui.consumeArmRequest()) {
        armed = tuningSafetyAllowsArm();
    }
    if (ui.consumeDisarmRequest()) {
        armed = false; // disarm wins if both events arrive together
    }

    if (ui.consumeApplyRequest()) {
        if (armed) {
            applyMessage = "Disarm before applying gains";
        } else {
            // Read each candidate field exactly once at this OpMode-loop boundary.
            double candidateKP = ui.kP();
            double candidateKI = ui.kI();
            double candidateKD = ui.kD();
            double candidateKF = ui.kF();

            try {
                flywheel.applyPidfCandidate(
                        candidateKP, candidateKI, candidateKD, candidateKF);
                applyMessage = "Applied";
            } catch (IllegalArgumentException rejected) {
                // Only setGains(...) can reach this catch; the applied tuple is unchanged.
                applyMessage = rejected.getMessage();
            }
        }
    }

    requestedRpm.set(armed ? boundedTestRpm : 0.0);
    flywheel.update(clock); // one normal owner update, after any accepted change
} catch (RuntimeException lifecycleFailure) {
    armed = false;
    try {
        requestedRpm.set(0.0);
    } catch (RuntimeException zeroFailure) {
        if (zeroFailure != lifecycleFailure) {
            lifecycleFailure.addSuppressed(zeroFailure);
        }
    }
    try {
        flywheel.stopOwnedMechanism();
    } catch (RuntimeException stopFailure) {
        if (stopFailure != lifecycleFailure) {
            lifecycleFailure.addSuppressed(stopFailure);
        }
    }
    throw lifecycleFailure;
}
```

The UI must publish the four candidate fields coherently before it publishes the apply event.
Reading each field once prevents repeated reads inside one attempt, but Phoenix does not make four
independently changing fields atomic. Do not call controller, Plant, or hardware methods from a
background or UI callback; apply on the owned OpMode loop.

Do not apply automatically whenever any one field changes. An explicit apply event lets the student
finish editing a complete candidate and makes every controller reset deliberate.

## Report accepted values

After a successful apply, use `getKP()`, `getKI()`, `getKD()`, and `getKF()` from the retained
`PidfRegulator` as the applied truth. Telemetry should print copyable assignments for the robot's
actual profile fields, for example:

```java
telemetry.addLine("profile.shooter.velocityKp = "
        + Double.toString(flywheelPidf.getKP()) + ";");
telemetry.addLine("profile.shooter.velocityKi = "
        + Double.toString(flywheelPidf.getKI()) + ";");
telemetry.addLine("profile.shooter.velocityKd = "
        + Double.toString(flywheelPidf.getKD()) + ";");
telemetry.addLine("profile.shooter.velocityKf = "
        + Double.toString(flywheelPidf.getKF()) + ";");
```

Those names are an example; the robot owns its profile shape. Report the applied getters rather
than echoing the UI fields so a rejected candidate cannot look accepted. `Double.toString(...)`
uses a locale-independent decimal point and enough precision to round-trip the accepted value into
a Java `double`; do not round the copyable assignments for display.

## Rollback and failure behavior

- **Invalid candidate:** report the validation error. The previously applied gains remain in use.
- **Valid but poor candidate:** disarm first, then explicitly reapply the last known-good or
  checked-in four-gain tuple through the same method.
- **Apply, reset, or update failure:** request zero and stop the mechanism owned by the tuning mode.
  Do not claim an automatic transaction across arbitrary decorators or physical hardware.
- **OpMode stop:** request zero, cancel any robot-owned transient work, and stop the Plant or
  mechanism owner normally.

Restoring four numbers cannot restore controller history or undo a physical command. Fail-stop and
explicit reapply are more truthful than a generic automatic rollback promise.

## Record and restart

Finish every accepted tuning session with this checklist:

1. Display the applied four gains from the retained PIDF getters.
2. Copy them into the checked-in robot profile.
3. Review and commit the source change.
4. Stop the tuning OpMode.
5. Start a fresh production TeleOp or Auto.
6. Confirm that production reports the same gains without reading tuning-UI state.

Runtime code cannot prove that source was copied or committed. Production readiness comes from
keeping the live UI out of production modes, not from a global "tuning values saved" flag.

## What Phoenix does and does not guarantee

| Phoenix provides | The robot or tuning UI still owns |
|---|---|
| All-or-nothing validation of four finite gains in `setGains(...)` | Publication of one coherent candidate |
| Applied-gain getters | Safe mechanism-specific gains and test targets |
| A reset lifecycle; built-in regulator decorators propagate it inward | Retaining and resetting the correct outer composition |
| Normal Plant target and output defenses | Arming, stop, and failure policy |
| One shared OpMode-loop boundary | Copying and committing profile values, plus physical confirmation |

## Avoid these patterns

- Reading mutable tuning fields continuously from production TeleOp or Auto.
- Applying whenever any one field changes instead of requiring one explicit apply event.
- Calling `setGains(...)`, `reset()`, Plant methods, or hardware from a UI callback.
- Reading the same candidate field several times during one apply attempt.
- Rebuilding a regulator, Plant, subsystem, or robot for every edit.
- Resetting only the inner PIDF when stateful wrappers exist.
- Letting a tuner write motors directly or bypass the source-driven Plant.
- Treating finite gains as safe without explicit target and complete-output bounds.
- Swallowing a lifecycle failure while leaving the mechanism enabled.
- Treating displayed values as checked in or using a mutable global flag to claim match readiness.
- Automatically rewriting robot profile source on the Robot Controller.

## Related reading

- [`FTC Actuators & Plants`](<../ftc-boundary/FTC Actuators & Plants.md#13-velocity-bounds-mapping-and-tuning>)
- [`Layered Shooter Example`](<../examples/Layered Shooter Example.md#5-layer-3-realization>)
- [`Recommended Robot Design`](<../design/Recommended Robot Design.md#keep-software-pidf-inside-realization>)
