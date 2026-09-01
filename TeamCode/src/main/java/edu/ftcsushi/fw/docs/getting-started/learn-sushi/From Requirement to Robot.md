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

[`StarterIntake`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntake.java>) names
those shared robot meanings. [`StarterProfile`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterProfile.java>)
owns the motor name, direction, powers, and motion permission.
[`StarterIntakeMechanism`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java>)
privately owns the Plant that realizes them. Controls decide which buttons request them;
[`StarterAuto`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/opmode/StarterAuto.java>) composes a fresh timed
Task through the same capability; status separates the semantic request from the cached applied
target.

That complete path needs no supervisor, Service, feedback controller, or capability bundle. Change
a button in controls, a power or motor name in configuration, and hardware realization in the
mechanism. Prove software contracts first and physical behavior separately.

## Scale when the requirement needs feedback: a lift

### Critical code

Suppose the team decides:

> TeleOp and Auto must select LOW and HIGH. Auto must be able to wait for arrival, and the lift must
> establish its bottom reference without blocking the robot.

Work from that meaning toward hardware:

| Step | One owner | Lift answer |
|---|---|---|
| Requirement | Team design | Select named heights, wait when needed, and home safely. |
| Capability | `ReferenceLift` | Expose shared intent and status without motor details. |
| Configuration | `ReferenceLiftMechanism.Config` | Hold the hardware name, direction, units, bounds, powers, tolerances, and time budgets. |
| Realization | `ReferenceLiftMechanism` | Privately own the final Plant graph, bottom-switch conditioning, update order, and stop. |
| Mode clients | Controls and Auto routine | Give buttons meaning or compose fresh Tasks through the capability. |
| Evidence | Cached `Status` and presenter | Distinguish requested height/position, measurement, reference state, and at-target state. |
| Proof | Tests, then experiment | Prove software contracts separately from physical direction, clearance, and performance. |

The mode-neutral vocabulary stays small:

Abbreviated shape (omissions shown):

<!-- teaching-shape -->
```java
// ...
void setHeight(Height height);
Task moveTo(Height height);
Task home();
Status status();
// ...
```

**What to notice**

- The capability names robot meanings and evidence, not FTC device details.
- TeleOp may replace a persistent request; Auto may create fresh work that waits for feedback.

**Key APIs:** `Task` represents non-blocking work; capability `Status` is the shared read-only
evidence vocabulary.

[`ReferenceLift.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/capability/lift/ReferenceLift.java>)
does not mention an FTC motor, switch polarity, encoder scaling, or homing power. Those physical and
realization details belong to the Config and mechanism.

Controls call `setHeight(...)` when replacing the persistent request is enough. Auto builds a fresh
`moveTo(...)` when its next action must wait for feedback. Neither client reaches into the Plant.
If the team later changes the reference sensor, the mechanism and configuration can change while
“home the lift” remains the same robot meaning.

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
[`ReferenceLauncher.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/capability/launcher/ReferenceLauncher.java>)
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
- Complete Reference composition: [`ReferenceRobot.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/robot/ReferenceRobot.java>)
- Topic-specific routes: [Choose a Sushi topic](<../Beginner's Guide.md>)
- [Choose another Sushi topic](<../Beginner's Guide.md>)
- [Return to the Basic Mechanisms course](<../Basic Mechanisms Robot.md>)
