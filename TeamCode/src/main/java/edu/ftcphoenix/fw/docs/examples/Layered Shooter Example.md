# Layered Shooter Example

This walkthrough is the architecture-focused companion to
[`TeleOp_09_LayeredShooterMechanism`](<../../tools/examples/TeleOp_09_LayeredShooterMechanism.java>).

If the earlier shooter docs answer “how do AprilTag aim and range estimation fit in?”, this guide
answers a different question:

> **what should one mechanism owner look like when you write the three layers explicitly?**

The three layers are:

1. <b>Requests</b> — caller-owned memory
2. <b>Behavior</b> — robot-owned execution and timing
3. <b>Realization</b> — plant-owned final target application

> **Advanced construction exception:** this one-file lesson deliberately exposes target-graph
> assembly in its composition root and passes completed Plants into the nested `Realization` so the
> three relationships can be studied separately. Copy the layer responsibilities, not that
> construction seam. In ordinary robot code, follow the
> [`Modern Starter Robot`](<Modern Starter Robot.md>): call
> `new Mechanism(hardwareMap, profile.mechanism)`, and let that mechanism construct and privately
> own its Plants. A completed-Plant constructor is appropriate only when it is explicitly kept as a
> hardware-neutral test, custom-adapter, or advanced assembly seam.

---

## 1. Why this example exists

Examples 07 and 08 already hint at the layering philosophy, but they intentionally keep the code
lightweight:

- Example 07 uses a supervisor + subsystem split.
- Example 08 keeps behavior almost trivial.

Example 09 is the first one that says the quiet part out loud by naming the three layers directly.
Copy its layer responsibilities when your mechanism gets “just complicated enough” to need
structure; keep the ordinary starter construction boundary unless you intentionally need the
advanced seam called out above.

---

## 2. The mechanism in one sentence

The mechanism has a flywheel and a feeder.

The driver can:

- keep the flywheel spun up with a held request,
- nudge the selected flywheel speed,
- command manual feeder power each loop,
- queue one or more shots,
- and cancel queued/active transient actions.

That makes it a good teaching example because it combines the three common request shapes in one
place.

---

## 3. Layer 1: Requests

The request layer in Example 09 stores four things:

- `HeldValue<Boolean> flywheelHeld`
- `HeldValue<Double> selectedVelocityNative`
- `FrameValue<Double> manualFeedPower`
- `RequestCounter shotRequests`

That gives a compact summary of the three request-memory shapes.

### 3.1 Held requests

Held values remember the last explicitly written value until something changes them.

In this example, held state is used for:

- “keep the flywheel enabled”
- “use this selected flywheel velocity”

Held values are the right fit when the driver or higher-level caller means “stay like this until I
say otherwise.”

### 3.2 Frame requests

Frame values are for commands that should disappear unless they are refreshed every loop.

In this example, manual feed power is a frame value. That is the right match for a stick-driven
manual override because letting go of the stick should naturally return the command to zero.

The wiring is intentionally direct:

```java
bindings.copyEachCycle(
        gamepads.p1().rightY().deadbandNormalized(0.08, -1.0, 1.0),
        shooter::commandManualFeedPower
);
```

That line centralizes the gamepad mapping in `Bindings`, while `FrameValue` handles the lifetime.

### 3.3 Pending requests

`RequestCounter` holds queued shot requests.

That is different from a held boolean:

- tapping A means “queue one unit of work”
- tapping B means “queue three units of work”
- the request stays pending until behavior consumes it

That is why `RequestCounter` belongs in layer 1 but is allowed to be consumed by layer 2.

---

## 4. Layer 2: Behavior

Behavior is where the interesting logic lives.

Example 09 makes this explicit by having a `Behavior` owner with its own state and enums.

Its job is to answer questions like:

- Is a shot request blocked by manual feed?
- Is the flywheel ready for the selected velocity?
- Should a pending shot turn into an active pulse right now?
- Should the feed be in `IDLE`, `MANUAL`, or `PULSE` mode?
- Should the flywheel keep spinning because there is still work pending?

### 4.1 Why the pulse lives here

The feed pulse is not part of the request layer.

A shot request is only the statement “I want one shot.” The request layer should not know how long
a shot pulse lasts.

Likewise, the pulse is not a plant concern. The feeder plant should not know why it is being driven
for 0.15 seconds.

So the pulse lives in behavior, where the robot owns timing and execution.

### 4.2 Why behavior reads realization readback

Behavior needs to know whether the flywheel is ready before it consumes a pending shot.

That is why realization exports a small `Readback` snapshot containing:

- whether the flywheel is at target,
- what flywheel target the plant was actually holding,
- and the measured flywheel velocity.

Behavior uses that readback to answer the specific question it cares about:

> is the flywheel ready for the currently selected shot velocity?

That keeps plant details out of the request layer while still allowing behavior to make decisions.

### 4.3 Priority in this example

The example intentionally uses a simple priority rule:

1. the queued `PULSE` output wins while active
2. otherwise the baseline feed target can be `MANUAL`
3. otherwise the baseline is `IDLE`

That means:

- an active shot pulse overrides the feeder baseline until it ends,
- manual feed power works whenever no pulse is active,
- and queued shots do not start while manual feed is active.

The code expresses that priority in two places with two different responsibilities:

- `Behavior` owns the `OutputTaskRunner` and decides when to enqueue a pulse.
- In this lesson's explicitly advanced one-file seam, the composition root assembles the feeder
  Plant from the final `PlantTargetResolver` created by `PlantTargets.overlay(...)`; `Realization`
  owns that completed Plant's lifecycle and accesses its graph-owned command through the Plant when
  it needs to write it.

That is the important source-driven lesson: behavior proposes temporary outputs; the final target
resolver arbitrates; the Plant consumes one target.

---

## 5. Layer 3: Realization

Realization is where final targets hit real hardware.

In Example 09, realization owns two Plants:

- a velocity plant for the flywheel
- a power plant for the feeder

At the advanced construction seam identified at the top of this guide, the composition root passes
those completed Plants into realization without also passing their already-bound command targets.
`plant.commandTarget()` returns the same stable command identity without sampling hardware or
changing state, so realization may retrieve it at each write site. The feeder's named base target
still exists while its overlay graph is assembled; it is not repeated as a peer constructor
dependency after the Plant exists.

The flywheel's simple command can be created inline with `targetFromNewCommand(0.0)`, so
realization retains only the Plant and writes through `flywheelPlant.commandTarget()`. Its Plant can
use either FTC device-managed velocity control or a Phoenix-regulated velocity loop. A standard
power-based flywheel declares its setpoint, PID, typed feedforward, voltage compensation, and final
power policy in one Plant recipe:

```java
flywheelPlant = FtcActuators.plant(hardwareMap)
        .motor(config.flywheelName, config.flywheelDirection)
        .velocity()
        .regulated()
            .internalEncoder()
        .bounded(0.0, config.maximumRpm)
        .scaleToNative(config.ticksPerRpm)
        .velocityTolerance(config.toleranceRpm)
        .setpointFromAccelerationLimitedProfile(config.maximumRpmPerSec)
        .feedbackFromPid(config.kP, config.kI, config.kD)
        .feedbackIntegralLimitedTo(-0.15, 0.15)
        .feedforwardFromMotion(config.kS, config.kV, config.kA)
        .voltageCompensationFrom(batteryVoltage, 13.0, 9.0, 1.4)
        .outputPowerLimitedTo(0.0, config.maximumPower)
        .targetFromNewCommand(0.0)
        .build();
```

The final policy covers PID, feedforward, and voltage compensation; target bounds remain a
different Plant-unit fact. Standard control owns saturation-aware integral behavior and profile-
settled completion inside the Plant lifecycle. Production TeleOp and Auto use checked-in config and
continue to talk only in selected velocity targets and readiness.

FTC `deviceManagedWithOverrides().velocityPidf(...)` instead configures the motor controller and
uses the ready-made `FtcPanelsTuners.velocityPidf(...)` workflow. See the
[`PIDF tuning workflow`](<../testing-calibration/PIDF Tuning Workflow.md>) for the standard software
tuning order and the distinct FTC live workflow. A nonlinear or table-driven complete law uses the
explicit advanced `controlFromCustomRegulator(...)` exit; it does not become a parallel ordinary
PID/feedforward recipe.

The feeder uses a richer final target resolver:

```java
PlantTargetResolver finalFeederTarget = PlantTargets.overlay(feederBaseTarget)
        .add("feedPulse", feederPulseQueue.activeSource(), feederPulseQueue)
        .build();
```

The feeder Plant is then built with that final target resolver. Because the overlay base is the
`ScalarTarget` named `feederBaseTarget`, the graph automatically carries it as the command target
that `ScalarTasks.set(...)` can write if needed. Conditional layers never replace that command
identity:

```java
Plant feederPlant = FtcActuators.plant(hardwareMap)
        .crServo("transferLeftServo", Direction.FORWARD)
        .andCrServo("transferRightServo", Direction.REVERSE)
        .power()
        .targetFromResolver(finalFeederTarget)
        .build();
```

Its loop code is intentionally small:

1. receive the behavior output
2. update the command baseline targets
3. update the plants, letting each Plant invoke its final target resolver
4. export readback for the next loop

That smallness is the whole point. The target-resolver ownership rule becomes obvious because only
realization has the Plant references and accesses their graph-owned commands, and the feeder's
pulse-vs-baseline priority is expressed in the resolver graph instead of as hidden Plant state.

---

## 6. What the loop looks like

The loop order in Example 09 is intentionally straightforward:

1. update `LoopClock`
2. update `Bindings`
3. call `shooter.update(clock)`
4. print grouped telemetry

Inside `shooter.update(clock)`, the layered flow is:

1. read last-cycle plant state from `Realization.readback()`
2. let `Behavior` consume requests and choose outputs
3. let `Realization` apply those outputs to the plants

That one-way flow is the pattern to copy into richer robot code.

---

## 7. How to use this pattern on a real robot

When a real robot mechanism starts to accumulate:

- remembered selections,
- manual overrides,
- queued work,
- timing,
- readiness gates,
- and more than one plant,

this is the point where it helps to stop writing “just one more boolean” and instead split the
mechanism into the three explicit owners.

A good rule of thumb is:

- if the value is written by caller-facing methods, it probably belongs in <b>Requests</b>
- if the value is written by runtime decision logic, it probably belongs in <b>Behavior</b>
- if the value is about a plant target, measurement, or controller, it probably belongs in
  <b>Realization</b>

That is the main lesson of Example 09.

---

## 8. Read next

- [`Examples Progression & Layered Mechanisms.md`](<Examples Progression & Layered Mechanisms.md>)
- [`Shooter Case Study & Examples Walkthrough.md`](<Shooter Case Study & Examples Walkthrough.md>)
- [`../design/Supervisors & Pipelines.md`](<../design/Supervisors & Pipelines.md>)
- [`../design/Robot Capabilities & Mode Clients.md`](<../design/Robot Capabilities & Mode Clients.md>)
