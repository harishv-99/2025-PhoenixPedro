# Behavior Lanes

Phoenix supports several different kinds of robot behavior. The framework stays simplest when you
pick the lane that matches the problem instead of forcing everything into one abstraction.

The quick decision rule is:

1. **Am I commanding a local actuator setpoint directly?** Use a `Plant`.
2. **Am I regulating one measured scalar toward a setpoint?** Use a scalar controller source.
3. **Am I reacting to an event or classification?** Use `BooleanSource` / `Source<T>` with a
   supervisor or task.
4. **Am I solving a 2D frame-to-target relation?** Use drive guidance / spatial logic.
5. **Am I following an external route package?** Adapt it into Phoenix tasks and status, do not
   duplicate the planner.

---

## Lane 1: local setpoint

Use this when the mechanism already has a meaningful local command mode:

- servo position
- motor position
- motor velocity
- raw motor / CR servo power

```java
Plant wrist = Actuators.plant(hardwareMap)
        .servo("wristServo", Direction.FORWARD)
        .position()
        .build();

wrist.setTarget(0.72);
wrist.update(clock.dtSec());
```

This is the simplest lane. Do not add a controller or a queue unless you really need one.

---

## Lane 2: scalar regulation

Use this when you have one measured variable and want to drive its error toward zero:

- lift height from a potentiometer or distance sensor
- arm angle from an analog sensor
- turret local angle hold from an encoder

Pattern:

```java
ScalarSource measuredHeight = ...;
ScalarSource desiredHeight = ...;
Pid pid = Pid.withGains(0.12, 0.0, 0.0).setOutputLimits(-0.55, 0.55);

ScalarSource liftPower = ScalarControllers.pid(desiredHeight, measuredHeight, pid);

liftPlant.setTarget(liftPower.getAsDouble(clock));
liftPlant.update(clock.dtSec());
```

This is still a **local scalar** problem. It is not spatial guidance.

See also: `tools/examples/TeleOp_08_LiftExternalSensorControl.java`.

---

## Lane 3: event / classification supervision

Use this when the sensor is telling you that something happened, not “how far away” you are:

- touch sensor for homing
- beam break for game-piece present
- color sensor classifying an object
- current spike meaning a hard stop or jam

Pattern:

```java
BooleanSource homePressed = FtcSensors.touchPressed(hardwareMap, "liftHome");

OutputTask seekHome = Tasks.gatedOutputUntil(
        "seekHome",
        BooleanSource.constant(true),
        homePressed,
        ScalarSource.constant(-0.25),
        0.0,
        0.0,
        1.0,
        0.0
);

runner.enqueue(seekHome);
```

Or for a supervisor-style intake gate:

```java
Source<Rgba> rgba = FtcSensors.rgba(hardwareMap, "intakeColor");
BooleanSource pieceSeen = rgba
        .mapToBoolean(c -> c.alpha > 250)
        .debouncedOnOff(0.03, 0.03);
```

This lane relies on `BooleanSource`, `Source<T>`, supervisors, and tasks. It is not a PID lane.

---

## Lane 4: spatial relation guidance

Use this when the thing you care about lives in 2D robot/field space:

- drivetrain goes to a field point
- drivetrain faces a target while translating
- turret faces a tag
- end effector lines up to a tag-relative point

Today Phoenix exposes the mature version of this lane through **DriveGuidance**.

```java
DriveGuidancePlan plan = DriveGuidance.plan()
        .translateTo()
            .fieldPointInches(48, 24)
            .doneTranslateTo()
        .aimTo()
            .fieldHeadingRad(Math.PI / 2.0)
            .doneAimTo()
        .feedback()
            .fieldPose(poseEstimator)
            .doneFeedback()
        .tuning(DriveGuidancePlan.Tuning.defaults())
        .build();
```

The shared idea here is a **frame-to-target relation**, but Phoenix does not yet publish a generic
`SpatialSpec2d` because DriveGuidance is currently the only mature framework consumer. That
extraction should wait until a second, real non-drive consumer proves out the public API.

---

## Lane 5: external route integration

Use this when Road Runner, Pedro Pathing, or another route package already owns the path logic.

Phoenix should not reimplement those planners. The integration pattern is:

- route package owns path following
- Phoenix wraps route execution in a `Task`
- Phoenix uses `Task.cancel()` / `TaskRunner.clearAndCancel()` for interruption
- Phoenix keeps mechanism control, supervision, and telemetry around the route task

The seam for Phoenix-owned drivetrain helpers is now `DriveCommandSink`, not `MecanumDrivebase`
directly. That keeps autonomous helpers reusable even when the robot later swaps in an external
follower stack.

---

## Phoenix example

In the updated Phoenix robot code:

- `Shooter` is the lane-1 subsystem that owns plants and the feed queue.
- `ShooterSupervisor` is the lane-3 supervisor that owns hold-to-shoot, intake/eject priority, and queue buffering.
- `PhoenixRobot` wires the bindings and lane-4 drive guidance / auto-aim together at the robot-container level.

That keeps gamepad intent, subsystem control, and drive guidance in separate places without introducing a large number of public objects.

## Why these lanes exist

These lanes line up with how successful robot code is commonly organized:

- one subsystem owns each hardware group
- commands/tasks can be cancelled or interrupted cleanly
- event-driven behaviors are separate from continuous feedback loops
- route libraries are integrated through adapters instead of duplicated planners

Phoenix keeps the framework small by only adding reusable pieces where they genuinely repeat across
robots.
