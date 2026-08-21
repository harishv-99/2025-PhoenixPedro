# From requirement to robot

**Question:** When the team identifies a robot need, which code owner should change?

**Reading time:** about 20 minutes

Start with a sentence about robot behavior, not a framework class. Then give each decision one
owner. The result is simple client code because hardware construction, timing, and evidence remain
behind the capability vocabulary.

This chapter provides a design exercise you may use on paper or in a code review. It does not
require filling in the worksheet, editing the examples, or moving hardware.

## Source map

- [`ReferenceLift.java`](<../../../../robots/examples/reference/capability/lift/ReferenceLift.java>)
  shows a small mode-neutral capability with persistent intent, a Task factory, and cached status.
- [`ReferenceLauncher.java`](<../../../../robots/examples/reference/capability/launcher/ReferenceLauncher.java>)
  keeps a multi-device action simple for both mode clients.
- [`ReferenceTeleOpControls.java`](<../../../../robots/examples/reference/robot/ReferenceTeleOpControls.java>)
  owns operator meanings without owning hardware.
- [`ReferenceRobot.java`](<../../../../robots/examples/reference/robot/ReferenceRobot.java>) wires the
  owners and hands the declared capabilities to its mode client.
- [`ReferenceAutoRoutines.java`](<../../../../robots/examples/reference/autonomous/ReferenceAutoRoutines.java>)
  composes the same capabilities as fresh autonomous strategy.

## The repeatable path

```text
robot requirement
    -> mode-neutral capability intent and status
    -> data-only physical/configuration answers
    -> mechanism or subsystem owning the final Plant graph
    -> controls and Auto as parallel capability clients
    -> cached evidence and presenter
    -> software proof of configuration, mapping, and Task outcomes
    -> a separate subsystem experiment when physical proof is needed
```

Use this order for one subsystem at a time:

1. **Name the robot meaning.** Write what TeleOp and Auto need to request or observe without motor
   names: “select HIGH,” “launch one,” or “is the flywheel ready?”
2. **Choose one capability owner.** Put the shared verbs and immutable status in a small,
   mode-neutral interface.
3. **Author configuration facts.** Record hardware names, directions, units, bounds, tolerances,
   and time budgets in the mechanism's data-only `Config`.
4. **Build one realization path.** The mechanism receives `HardwareMap` plus that configuration,
   privately owns its final Plants/resolvers, and owns update and stop.
5. **Connect mode clients.** Controls give buttons operator meaning; Auto composes fresh capability
   Tasks. Neither client reaches into raw Plants.
6. **Publish only useful evidence.** The mechanism caches status; a presenter formats the snapshot.
7. **Prove the software contract.** Before motion, test configuration rejection, semantic mapping,
   fresh Task outcomes, cleanup, and status with fakes or hardware-neutral seams.
8. **Separate physical validation.** If software must calculate an answer, print it from an
   experiment. Let the operator record directly visible outcomes.

## Worked example: a lift with named levels

The shared vocabulary stays small:

```java
    /** Selects a persistent semantic height. */
    void setHeight(Height height);

    /** Builds a fresh feedback-aware move that waits for the selected height. */
    Task moveTo(Height height);

    /** Builds a fresh non-blocking bottom-switch reference search. */
    Task home();

    /** Returns cached command and feedback evidence. */
    Status status();
```

Source: [`ReferenceLift.java`](<../../../../robots/examples/reference/capability/lift/ReferenceLift.java>)

That interface says nothing about FTC motors, switch polarity, encoder scaling, or homing power.
Those are realization facts owned by `ReferenceLiftMechanism.Config` and the mechanism. Controls
map D-pad meanings and X to the capability. A TeleOp callback uses `setHeight(...)` when replacing
the persistent request is sufficient. Auto uses the fresh `moveTo(...)` Task when the next action
must wait for feedback. Status reports the requested semantic height, requested position,
measurement, reference state, and at-target state.

If the team changes from a bottom switch to a different reference cue, the mechanism and its
configuration change. TeleOp and Auto should not need a new vocabulary if “home the lift” still
means the same robot behavior.

## Worked example: launch one object

The launcher's requirement crosses several physical devices, but the clients still ask for one
robot meaning:

```java
    /** Replaces the persistent flywheel request in encoder ticks per second. */
    void setTargetVelocityTicksPerSec(double targetVelocityTicksPerSec);

    /** Builds a fresh outcome-aware spin-up, feed, and retraction-request Task. */
    Task launchOne();

    /** Invalidates older launch Tasks and restores active-match launcher requests. */
    void abortLaunches();

    /** Returns the latest cached controller and sensor evidence. */
    Status status();
```

Source: [`ReferenceLauncher.java`](<../../../../robots/examples/reference/capability/launcher/ReferenceLauncher.java>)

`ReferenceLauncherMechanism` owns the paired velocity Plant, per-wheel evidence, transfer overlay
queue, release Plant, object sensor conditioning, Task factory, update order, and stop behavior.
That is one mechanism owner, not a requirement for each motor or servo to become a public
subsystem. Temporary transfer/release plumbing stays private; mode clients request `launchOne()`.

The launcher also shows where robot-specific policy belongs. `launchOne()` decides that a spin-up
timeout clears its temporary requests without feeding. Its object sensor remains status-only. If
BIOBUZZ later requires object-gated feeding, the team must make that new policy explicit rather
than assuming the example already implements it.

## Put the owners in predictable packages

A team robot can omit any package it does not need, but this handoff keeps each decision easy to
find:

```text
<robot>/
  capability/<family>/   mode-neutral intent/status plus the mechanism realization
  robot/                 profile, composition root, and controls
  opmode/                thin FTC TeleOp, Auto, and tester entries
  autonomous/            routines, routes, and autonomous policy
  tester/                optional robot-specific criteria, experiments, and menu factory
```

The directory does not decide ownership by itself. A composition root still wires rather than
scripts; Auto policy still belongs in `autonomous/`; and a tester never shares a production
mechanism instance.

## Optional: apply it to team code

When it is useful, fill one row for a real requirement before writing code:

| Decision | Team answer |
|---|---|
| Mode-neutral request | What should both TeleOp and Auto be able to say? |
| Status evidence | Which requested, applied, measured, ready, or fault facts can software know? |
| Capability owner | Which small family owns that vocabulary? |
| Configuration | Which wiring, direction, unit, range, tolerance, and timing facts are required? |
| Realization owner | Which mechanism/subsystem owns the final resolver, Plants, update, and stop? |
| Controls client | Which operator action requests the capability? |
| Auto client | Which fresh Tasks compose the same capability? |
| Coordination policy | Does a supervisor/service need to arbitrate multiple capabilities? |
| Experiment | What computed result must be printed, and what result can an operator observe? |

Do not create a service or supervisor merely because the table has a row for one. Add it only when
the robot has continuing coordination policy that does not belong to a single mechanism or mode
client.

## Prove software before asking hardware

For each new subsystem, add the narrowest software checks before a physical experiment:

- invalid or colliding configuration fails before hardware lookup;
- controls map an edge to the intended capability meaning exactly once;
- every repeatable macro returns a fresh Task;
- success, timeout, active cancellation, and terminal cleanup retain the promised outcomes and
  requests; and
- status names distinguish requested, applied, measured, ready, and externally observed facts.

These checks cannot prove direction, tuning, clearance, or game-piece performance. They make the
later physical experiment smaller and easier to interpret. Use the copyable
[`Subsystem experiments`](<../../examples/Subsystem Experiments.md>) card for that separate phase.

## Debug one boundary at a time

When behavior is wrong, trace the first fact that stops matching:

```text
input source
  -> binding and semantic capability request
  -> requested Plant target
  -> applied target after bounds/guards
  -> measurement and readiness evidence
  -> Task outcome and Auto policy
```

For example, if the button source rises but the requested height does not change, inspect the
binding/capability boundary. If the request changes but the applied target does not, inspect
reference, bounds, and guards. If both targets look correct but the measurement does not, stop and
inspect hardware/evidence rather than changing Auto sequencing blindly.

## Where specialized needs go

The Reference robot deliberately does not demonstrate every integration:

- Field-relative drive, `RobotProgram.Service`, and Prestart selection stay in the
  [field-relative focused example](<../../examples/Field-relative Drive.md>).
- Parking geometry stays in `ReferenceParkingPlan`, a controller-free policy helper that Reference
  Auto does not execute.
- Pedro follower lifecycle and route policy stay in the
  [Pedro autonomous reference](<../../examples/Pedro Autonomous Reference.md>).
- AprilTags, localization, and guidance stay in the
  [drive and vision guides](<../../drive-vision/README.md>).
- Haptics and cross-capability supervisors are optional guide, Javadoc, or production-reference
  topics until the robot has that concrete need.

These are satellites around the common ownership model, not missing layers every beginner robot
must add.

## Trace it

1. **A new button should send the lift HIGH. Where does the button meaning belong?**

   In the controls owner, calling the existing `ReferenceLift.setHeight(HIGH)` capability. The
   mechanism should not read the gamepad.

2. **Auto must wait until a lift physically arrives before launching. Is `setHeight(HIGH)` enough?**

   No. That method makes a persistent request. Use a fresh `moveTo(HIGH)` and sequence its
   successful completion before launch.

3. **A beam break should prevent feeding. Is publishing it in status enough?**

   No. Status makes evidence observable. The launcher or a robot-owned supervisor must explicitly
   use that evidence in the feed policy.

4. **The team wants to follow a Pedro route. Should the follower move into `ReferenceRobot`?**

   No. Start from the focused Pedro integration, then let the team's Auto routine call its existing
   mode-neutral capabilities.

## Predict it

The team adds a sensor that can detect when its real collector is full. Should TeleOp read that
sensor and stop the motor directly?

No. The collector mechanism should own and condition its sensor beside the Plant it realizes, then
publish the semantic evidence through its capability status. Collector-owned policy or a distinct
robot supervisor should decide how that evidence changes collection intent. TeleOp continues to
request robot meanings rather than becoming a second sensor or motor owner.

## Definition of a clear subsystem

Before integration, another student should be able to answer:

- What can TeleOp and Auto request without knowing hardware details?
- Who owns every final actuator write and STOP behavior?
- Which status fields are requests, applied targets, measurements, or derived readiness?
- Which Task factory returns a fresh single-use graph each time?
- What happens on success, timeout, active cancellation, and terminal stop?
- Which configuration values are merely software-valid, and which have physical evidence?
- What experiment proves computed performance, and what must an operator observe?

If any answer has two owners, rename or redraw the boundary before adding more code.

## Copy, adapt, and leave alone

- **Copy:** the owner relationships and the small capability-facing client code.
- **Adapt:** every capability noun, configuration fact, Task outcome policy, and evidence field to
  the team's real design.
- **Do not copy wholesale:** the Reference robot's devices, example values, motion permissions, or
  speculative BIOBUZZ mechanisms.
- **Leave framework lifecycle managed:** ordinary OpModes stay thin and declare roles through
  `RobotProgram`.

**Previous:** [Evidence and experiments](<Evidence and Experiments.md>)

**Continue by role:** [Role paths](<Role Paths.md>)
