---
tags:
  - Learn
---

# From requirement to robot

**Learning mode:** Architecture reference

Use this page to assign requirements to owners, then
follow a linked buildable module to author the selected slice.

**Question:** When the team identifies a robot need, which code owner should change?

**Reading time:** about 9 minutes

Begin with a sentence about robot behavior, not a framework class. First trace the smallest Starter
need, then add roles only when a larger requirement demands them. You may follow this example only
by reading; no worksheet, code edit, or hardware run is required.

## Start with one Starter requirement

> TeleOp and Auto must be able to collect, eject, and stop one intake.

[`StarterIntake`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntake.html>) names
those shared robot meanings. [`StarterProfile`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/robot/StarterProfile.html>)
owns the motor name, direction, powers, and motion permission.
[`StarterIntakeMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.html>)
privately owns the Plant that realizes them. Controls decide which buttons request them;
[`StarterAuto`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/starter/opmode/StarterAuto.html>) composes a fresh timed
Task through the same capability; status separates the semantic request from the cached applied
target.

That complete path needs no supervisor, Service, feedback controller, or capability bundle. Change
a button in controls, a power or motor name in configuration, and hardware realization in the
mechanism. Prove software contracts first and physical behavior separately.

## Scale when the requirement needs feedback: a periodic turret

### Critical code

Suppose the team decides:

> TeleOp and Auto must request a turret angle. Auto must be able to wait for arrival, and the
> mechanism must choose the nearest legal full-turn equivalent inside reviewed cable bounds.

Work from that meaning toward hardware:

| Step | One owner | Turret answer |
|---|---|---|
| Requirement | Team design | Request a logical angle, choose a safe representative, and wait when needed. |
| Capability | `ReferencePeriodicTurretMechanism` public methods | Expose the complete numeric request and cached status without exposing the Plant. |
| Configuration | `ReferencePeriodicTurretMechanism.Config` | Hold the motor name, direction, radian scale, physical bounds, tolerance, and initial hold. |
| Realization | `ReferencePeriodicTurretMechanism` | Privately own the command, equivalent-position resolver, Plant update, and stop. |
| Mode clients | Controls and Auto routine | Give buttons meaning or compose fresh Tasks through the same mechanism API. |
| Evidence | Cached `Status` and presenter | Distinguish logical request, selected representative, applied target, measurement, and arrival. |
| Proof | Tests, then experiment | Prove software contracts separately from physical direction, clearance, and performance. |

The mode-neutral vocabulary stays small:

Abbreviated shape (omissions shown):

<!-- teaching-shape -->
```java
// ...
void setAngleRad(double angleRad);
Task setAngleTask(double angleRad, double timeoutSec);
Status status();
// ...
```

**What to notice**

- A numeric angle is the complete request here, so no extra semantic wrapper is needed.
- TeleOp may replace the persistent request; Auto may create fresh work that waits for feedback.

**Key APIs:** `Task` represents non-blocking work; capability `Status` is the shared read-only
evidence vocabulary.

[`ReferencePeriodicTurretMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/targeting/ReferencePeriodicTurretMechanism.html>)
does not expose its FTC motor or Plant. Hardware identity, encoder scaling, cable bounds, tolerance,
and the already-referenced assumption remain explicit configuration and realization facts.

Controls call `setAngleRad(...)` when replacing the persistent request is enough. Auto builds a
fresh `setAngleTask(...)` when its next action must wait for command-correlated arrival. Neither
client reaches into the Plant. If the cable bounds change, the mechanism can select a different
physical representative while the logical angle request stays the same.

## Prove only what each boundary knows

Before motion, focused software tests can prove that:

- invalid or colliding configuration fails before hardware lookup;
- one input edge maps to the intended capability meaning once;
- every repeatable macro returns a fresh, single-use Task;
- success, timeout, active cancellation, and terminal cleanup preserve their contracts; and
- status names keep requested, applied, measured, ready, and external facts distinct.

Those checks cannot prove motor direction, safe bounds under load, tuning, clearance, or game-piece
performance. A subsystem experiment should print facts software must calculate, such as measured
position or time to reach a target. The operator records facts available by observation, such as a
collision, jam, vibration, or successful score. Use the full
[Subsystem Experiments](<../../examples/Subsystem Experiments.md>) card before supervised motion.

When debugging, follow the same boundaries in order:

```text
input -> capability request -> requested Plant target -> applied target
      -> measurement/readiness -> Task outcome and Auto policy
```

Stop at the first mismatch. If requested and applied targets look correct but measurement does not,
inspect hardware and evidence instead of rewriting Auto sequencing.

## Scale only when the requirement demands it

The Reference launcher shows the same path applied to paired flywheels, a transfer overlay, a
release Plant, and outcome-aware launch policy. Its public client still asks for `launchOne()` and
reads status; the mechanism keeps the multi-device realization private. Study
[`ReferenceLauncher`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/launcher/ReferenceLauncher.html>)
only when that additional coordination helps your robot.

The [examples index](<../../examples/README.md>) routes concrete robot needs to maintained examples
without claiming game rules, hardware design, or physical success criteria.

## Check your understanding

**A beam break appears in launcher status, and the team now requires it to prevent feeding. Is the
status field enough?**

No. Status only publishes evidence. The launcher owner or a distinct robot supervisor must
explicitly use that evidence in feed policy; controls and Auto should keep requesting the same
mode-neutral behavior.

## Go deeper when needed

- Capability/client ownership: [Robot Capabilities and Mode Clients](<../../design/Robot Capabilities & Mode Clients.md>)
- Focused paired-velocity capability: [`ReferenceFlywheels`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/flywheel/ReferenceFlywheels.html>)
- Focused periodic-position realization: [`ReferencePeriodicTurretMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/targeting/ReferencePeriodicTurretMechanism.html>)
- Topic-specific routes: [Choose a Sushi topic](<../Beginner's Guide.md>)
- [Choose another Sushi topic](<../Beginner's Guide.md>)
- [Return to the Basic Mechanisms course](<../Basic Mechanisms Robot.md>)
