# Pedro integration

This folder holds the optional Phoenix framework bridge for Pedro Pathing.

- Keep `fw` core packages free of `com.pedropathing.*` imports.
- Keep reusable Pedro bridge code here, such as `PedroPathingDriveAdapter`.
- Construct the Pedro `Follower` on the robot side (typically `Constants.createFollower(hardwareMap)`) so team-specific setup stays out of the framework.
- Keep robot-specific Pedro autos on the robot side (`phoenix/.../autonomous/pedro/`).
- Later, this folder can become its own source set or module with minimal code motion.

## Lifecycle contract

Robot-side Auto setup constructs the team-configured Pedro `Follower`, wraps it once, and supplies
the adapter to the robot composition root through the backend-neutral drive seam:

```java
Follower follower = Constants.createFollower(hardwareMap);
PedroPathingDriveAdapter adapter = new PedroPathingDriveAdapter(follower);

PhoenixRobot robot = new PhoenixRobot(hardwareMap, telemetry, gamepad1, gamepad2, profile);
robot.initAuto(adapter);
```

After `initAuto(adapter)` accepts the adapter, `PhoenixRobot` owns its Auto lifecycle. It calls
`adapter.update(clock)` every Auto loop after Phoenix localization/targeting and before the Auto
`TaskRunner`, then calls `adapter.stop()` as part of `PhoenixRobot.stop()`. The OpMode should not add
a second raw `follower.update()`, adapter update, or adapter stop.

`PedroPathingDriveAdapter` still implements both `RouteFollower<PathChain>` and
`DriveCommandSink`, so route and guidance Tasks use the same short APIs as other integrations.
Those generic Tasks may call the adapter's update hook too; the adapter deduplicates by
`LoopClock.cycle()` so the one Pedro `Follower` advances at most once in a Phoenix cycle. Tasks
select route or manual behavior, while the composition root owns the recurring heartbeat. This
keeps pose and hold-end control alive during mechanism/wait Tasks and gives shutdown one final,
immediate stopped-output owner.

This lifecycle bridge does not choose routes, configure one team's drivetrain/localizer, or decide
whether Pedro or Phoenix owns pose correction. Those drivetrain, localization, and pose-authority
decisions remain deliberately deferred to `PEDRO-02` in the framework improvement tracker.


## Related docs

- [`../../docs/README.md`](<../../docs/README.md>)
- [`../../docs/design/Recommended Robot Design.md`](<../../docs/design/Recommended Robot Design.md>)
- [`../../docs/drive-vision/Drive Guidance.md`](<../../docs/drive-vision/Drive Guidance.md>)
- [`../../Framework Principles.md`](<../../Framework Principles.md>)
