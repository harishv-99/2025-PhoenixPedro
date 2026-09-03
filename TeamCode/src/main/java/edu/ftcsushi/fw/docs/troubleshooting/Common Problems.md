---
tags:
  - Test & Tune
---

# Common Problems

Sushi fails early when it can identify an invalid configuration or lifecycle. Read the complete
message first: it normally names the invalid value, expected domain, or owner that must change.

## Quick symptom map

| Symptom | Check first |
|---|---|
| Project does not build | Android Studio Gradle/JDK setup and the first compiler error |
| OpMode is missing on Driver Station | `@Disabled`, annotation, build, and deployment |
| Panels tester stops after losing its client | Reconnect Panels, then start a fresh Panels tester OpMode |
| A dedicated Panels tuner rejects the connection or an edit does not apply | Match its declared client-count policy; Update All, then press A |
| Hardware name error during INIT | Robot Configuration spelling, case, and duplicate group names |
| START remains blocked | Always-on readiness telemetry and the named calibration/config fact |
| Motor or servo moves the wrong way | Configured `Direction`, then `HW: Actuator Bring-up` |
| A button does nothing or fires repeatedly | `onRise`, `mirrorOnChange`, or `copyEachCycle` semantics |
| A macro works once | A started Task object was reused instead of rebuilt |
| Feedback move is rejected | The Plant is open-loop or the Task names the wrong command target |
| Plant target is wrong or rejected | Plant units versus native units; finite value and declared range |
| Mechanism target changes but hardware does not | Mechanism registration and its one Plant update owner |
| Drive fights, jitters, or ignores assist | More than one final drive writer or an incorrectly composed source |
| Camera is ready but sees no target | Readiness and target visibility are separate facts |
| Pedro becomes idle but Auto does not report success | Inspect the retained route status, not only vendor busy state |

## The project does not build

1. Start with the first compiler or Gradle error, not the later cascade.
2. In Android Studio, use the bundled JDK configured for this project.
3. Confirm that edited files are under `TeamCode/src/main/java` and their `package` declaration
   matches the folder.
4. Compare the call with the [`Sushi Cheat Sheet`](<../reference/Sushi Cheat Sheet.md>) and the
   method's Javadocs. Sushi builders are staged; a missing conceptual answer changes which methods
   are available next.
5. If the failure is a broken documentation link or fence, run the focused documentation test from
   the repository root:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.fw.docs.DocumentationLinksTest
```

## The OpMode does not appear on Driver Station

Check all of these:

- the class has `@TeleOp` or `@Autonomous`;
- `@Disabled` has been removed only after its hardware/config values are reviewed;
- the class compiled successfully and the new Robot Controller app was deployed;
- the Driver Station is connected to that Robot Controller; and
- the annotation name is not being mistaken for the Java class name.

Do not enable a starter until the active slice's hardware names, intended directions, power limits,
drive brake choice, clear-space plan, and operator-on-STOP plan have been reviewed. Set only its
`allowIntakeMotion` or `allowDriveMotion` permission; TeleOp requires both, while the intake-only
Auto deliberately leaves drive permission false. These booleans acknowledge human review but do not
prove any physical fact. Then use the first enabled, supervised, unloaded test to verify physical
directions, response, and immediate STOP behavior. Follow
[`Build and run Sushi`](<../getting-started/Build and Run.md>).

## INIT reports a hardware name problem

FTC hardware names are case-sensitive. Sushi trims surrounding whitespace and requires nonblank,
distinct names inside one homogeneous actuator or mecanum group before it resolves fresh hardware.

1. Open the active Robot Configuration on the Robot Controller.
2. Compare every name character-for-character with the profile.
3. Check that grouped members do not repeat the same configured name.
4. Correct the data-only profile or Robot Configuration; do not catch and ignore the exception.

A group-local duplicate check does not prove physical device identity and does not prevent a
separate read-only feedback adapter from intentionally using the same configured device.

## INIT reports that starter motion is not allowed

`StarterProfile.current()` returns software-valid example Configs with both permissions false.
Review only the slice needed by the mode, replace its example facts, then set its permission. Auto
requires only `allowIntakeMotion`; TeleOp requires both that permission and `allowDriveMotion`.
Clear the corresponding permission whenever you edit a slice.

## INIT reports an owner-qualified starter Config error

The starter has no aggregate issue collector. Its root checks mode permissions and TeleOp's
cross-owner name collision before every hardware lookup. Each active owner then copies and validates
its own Config before that owner's lookup. Correct the exact field named by the intake or drive
owner; do not treat compiling defaults or a true permission as hardware evidence.

A later drive error can be discovered after the intake was registered and controls were bound.
Managed `RuntimeException` cleanup clears those bindings and stops the registered intake before
rethrowing the original failure. This fail-stop behavior is not a claim that earlier SDK
configuration effects were transactionally rolled back.

## The Panels tester loses its client

**FW: Testers (Panels)** accepts only Panels virtual-gamepad input. If no client is connected, its
last client disconnects, or input cannot be sampled, that tester run terminally fail-stops and
best-effort stops its active tester. Later reconnecting cannot rearm the same OpMode instance.

Stop the OpMode, reconnect Panels, verify its controls are neutral, then begin a fresh INIT/start.
Use **FW: Testers (Driver Station)** instead when physical gamepads should own input; the two
entries never merge or switch input owners. Both entries mirror tester telemetry to Driver Station
and Panels.

Browser STOP is not a physical emergency stop. Keep immediate access to robot power during every
powered tester run, and validate disconnect and stop behavior on the actual robot before relying on
the Panels workflow. Sushi sees only Panels' total client count. If another client remains after
the input view closes, the host relies on the pinned Panels transport aging unattended controls to
neutral; a stalled OpMode loop still cannot apply a new neutral command.

A dedicated powered-tuning OpMode may deliberately require exactly one connected client. Close
extra Panels tabs/devices before INIT. If the declared count policy is not satisfied or changes
while running, the tuning session ends closed; correct the connection count and start a fresh
OpMode instance.

Panels **Update All** changes only Configurables draft fields. It never applies hardware. In
the direct flywheel workflow, finish editing every field, press Update All, then press A on the
virtual gamepad to capture one stable complete candidate. If telemetry reports an invalid or
unstable draft, correct the whole candidate and repeat; a running segment intentionally remains
unchanged. See the [`control tuning workflow`](<../testing-calibration/Control Tuning Workflow.md>).

## FTC START remains blocked

A `RobotProgram.Prestart` may return `BLOCKED` when a complete mode is not ready to operate safely.
The framework then keeps services, bindings, Tasks, drive, and outputs inert while presenters remain
visible.

Read the always-on telemetry. Correct the exact missing calibration, field fact, route maturity, or
configuration acknowledgement it names, stop the OpMode, and initialize it again. Do not turn a
blocker into a warning merely to reach START.

## A mechanism moves in the wrong direction

Stop immediately. Do not compensate by changing every target sign.

1. Confirm which physical member moved.
2. Check its configured `Direction` at the FTC boundary.
3. For a group, verify each member independently before testing the group.
4. Run either **FW: Testers (Driver Station) → HW: Actuator Bring-up** or **FW: Testers (Panels) →
   HW: Actuator Bring-up** with the device isolated and robot safely supported.
5. Only after direction is correct, tune scale, bounds, gains, or mechanism targets.

Use the [`actuator bring-up runbook`](<../testing-calibration/Actuator Bring-up.md>) and then
[`Robot Calibration Tutorials`](<../testing-calibration/Robot Calibration Tutorials.md>) for the
ordered integration checks. A captured endpoint or passing software range check does not prove
wiring, mounting, load, overshoot, or safe motion.

## A button action does not behave as expected

Choose the binding by meaning:

- `onRise(...)` — once per accepted press;
- `onFall(...)` — once per accepted release;
- `mirrorOnChange(...)` — publish a persistent held state when it changes;
- `whileHigh(...)` / `whileLow(...)` — run a quick action on every eligible loop at that level;
- `copyEachCycle(...)` — refresh one continuous scalar command each loop; or
- `program.taskBindings().onRise(...)` — construct and enqueue one fresh Task per press; the shared
  FIFO runner starts it when it reaches the queue head.

The first edge sample establishes a baseline, so starting the OpMode with a button already held does
not invent a press. A control context may also require a neutral sample before it rearms.

## A Task or macro runs only once

Task instances are single-use. Do not save one Task and enqueue or start it again.

```java
// Correct: the method creates a fresh Task for every accepted press.
program.taskBindings().onRise(driver.y(), mechanism::createScoreTask);
```

Also make sure a composite does not contain the same child object identity twice. Build a fresh
child for every position in the graph. See
[`Tasks and Macros`](<../design/Tasks & Macros Quickstart.md>).

## A feedback move is rejected

`ScalarTasks.set(target, value).untilReachedBy(plant)` requires:

1. a feedback-capable Plant; and
2. the exact `ScalarTarget` that the Plant follows as its command target.

Direct power and set-and-hold servo Plants are open-loop; they cannot truthfully report physical
arrival. Use a bounded timed command or an independent sensor condition. For a feedback Plant, use
`plant.commandTarget()` rather than constructing an unrelated target beside it, then choose the
move's explicit active-cancellation policy.

`SemanticScalarTasks.set(command, request).untilReachedBy(plant)` likewise requires feedback and a
Plant whose resolver carries that exact semantic command. Keep the Plant argument: one command may
feed multiple observers, and only the selected Plant supplies the completion evidence.

## A Plant target is rejected or looks scaled incorrectly

First name the domain of each number:

- `bounded(...)`, `periodic(...)`, target commands, references, and position/velocity tolerances use
  **plant units**;
- only methods whose names contain `Native` or a controller unit such as `Ticks` use native units;
- direct motor and CR-servo power is `[-1, +1]`; and
- standard-servo native position is `[0, 1]`.

Then trace the value before clamping. `NaN` and infinities are configuration or controller failures,
not targets to hide at a bound. For grouped outputs, check the complete mapped image of every child,
not only the shared command. Use
[`FTC Actuators & Plants`](<../ftc-boundary/FTC Actuators & Plants.md>).

## The command target changes but the mechanism does not move

Setting a `ScalarTarget` stores intent; it does not write hardware. Verify that:

1. the mechanism implements `RobotProgram.Output`;
2. the composition root declares that complete mechanism with `program.output(...)`;
3. the mechanism's `update(clock)` calls each private Plant once in its intended order;
4. no Task or controls class calls `plant.update(clock)` itself; and
5. the Plant's final resolver, guards, reference state, and bounds allow the request.

At STOP, the mechanism must cancel independently owned transient work and terminally stop its owned
Plants/resources. A Plant's natural stop acts immediately and later updates are inert, so shutdown
does not also rewrite its target graph. The program manages when that output lifecycle runs; the
mechanism remains the one hardware realization owner.

## Cancelling a Task does not produce the motion I expected

Cancellation is cooperative and active-only. It asks the active Task to release or change the
requests that Task owns. It does not bypass the Plant source graph or directly stop hardware.

For a feedback move, choose either `cancelTo(...)` or `leaveRequestOnCancel()`. For a timed numeric
or semantic write, choose `.then(...)` or `.leaveThere()`. Timed work owns and reasserts its request
for the interval; feedback work publishes once and yields to superseding requests. For a queued
output, use the total-abort API that cancels active work and clears pending work. Robot-level STOP
must still cancel independently owned coordinated work and terminally stop the final owners; it
need not neutralize every stopped Plant's resolver graph.

## Drive commands fight or an assist has no effect

Ordinary TeleOp has one final path:

```text
controls and guidance -> one composed DriveSource -> program.drive(...) -> one DriveCommandSink
```

Do not also run an imperative drive Task against that sink. `DriveTasks.driveExclusivelyForSeconds(...)`
is for simple Auto/test intervals where the Task is the sole behavior-command writer. Verify overlay
activation, loss behavior, frames, and units before changing gains.

See [`Drive Guidance`](<../drive-vision/Drive Guidance.md>) and
[`Loop Structure`](<../core-concepts/Loop Structure.md>).

## Vision is ready but no target is selected

Readiness means the configured camera/lane can provide results. It does not mean a target is in view.
Check separately:

- current target visibility;
- observation freshness;
- enabled processor or confirmed Limelight pipeline;
- candidate/eligible tag IDs;
- fixed-layout membership when localization is required;
- camera mount calibration; and
- selection/loss policy.

Do not make a cached vendor result fresh by recomputing its timestamp. The acquisition owner anchors
one `LoopTimestamp` to the vendor result identity.

See [`AprilTag Localization & Fixed Layouts`](<../drive-vision/AprilTag Localization & Fixed Layouts.md>)
and [`AprilTag Practice Setup`](<../drive-vision/AprilTag Practice Setup.md>).

## A camera cannot reopen after leaving a tester

The concrete webcam or Limelight lane owns the resource. BACK, failure, and STOP must close that
owner, not only a borrowed sensor/processor view. A retry constructs a fresh owner; webcam portal
processors must also be fresh. If cleanup itself fails, stop and restart the OpMode rather than
opening a competing camera owner.

## Pedro is idle, but the route did not succeed

Idle/not-busy is not proof that the intended endpoint completed. Before termination, a route may be
`NOT_STARTED` or `ACTIVE`. Inspect the retained per-start terminal `RouteStatus`: `COMPLETED`,
`FOLLOWER_TIMEOUT_OR_STALL`, `INTERRUPTED`, `REPLACED`, `TASK_TIMEOUT`, `CANCELLED`, `FAILED`, or
`UNKNOWN_TERMINAL`. Robot routine policy must explicitly decide whether each non-normal result
continues, starts a selected fallback, or aborts.

The adapter still needs one stable managed heartbeat throughout the active OpMode, including loops
where no route Task is active. See [`Follow one Pedro route`](<../build/First Pedro Auto.md>)
and [`Pedro integration contract`](<../../integrations/pedro/README.md>).

## The problem is still unclear

Reduce it to one owner and one checkpoint:

1. What was the last expected observable result?
2. Which owner should have produced it?
3. Did that owner's input/status snapshot contain the expected value?
4. Did the next documented phase run?
5. What exact exception, outcome, readiness, or route status was retained?

Use required telemetry for driver-visible state and a selected `debugDump(...)` for internal state.
Presenters and debug output must not sample new hardware, advance a filter, mutate behavior, clear
telemetry, or commit the frame.

[Troubleshooting index](<README.md>) · [Cheat sheet](<../reference/Sushi Cheat Sheet.md>) · [Docs home](<../README.md>)
