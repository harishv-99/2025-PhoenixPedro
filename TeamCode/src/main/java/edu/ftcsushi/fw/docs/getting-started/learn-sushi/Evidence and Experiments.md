# Evidence and experiments

**Learning mode:** Architecture reference

This page explains what each evidence level proves;
the linked scenario and experiment modules contain the code-authoring work.

**Question:** What does a boolean, status row, or experiment result actually prove?

**Reading time:** about 8 minutes

Sushi separates a request, an electrical observation, a measurement, and a physical conclusion.
This page requires no sensor, code edit, or robot run.

## Climb the evidence ladder

Use the lowest-cost evidence that can truthfully answer the current question, then move upward only
when the next fact requires it:

```text
semantic test
    -> software device scenario
    -> optional modeled simulation
    -> supervised bring-up
    -> robot experiment
```

1. A **semantic test** replaces a capability with a small recording object. It proves controls,
   Tasks, or policy requested the right robot meaning without constructing FTC devices.
2. A **software device scenario** constructs the real production mechanism and Plants with a
   test-only `HardwareMap`. The scenario explicitly injects readings and records actuator commands.
3. An **optional modeled simulation** evolves state with an authored dynamics model. Use that name
   only when its assumptions and fidelity are explicit; many mechanism questions do not need it.
4. **Supervised bring-up** establishes one physical device fact such as wiring, direction, encoder
   sign, or STOP response with conservative commands.
5. A **robot experiment** evaluates a complete physical subsystem question against team-authored
   success criteria over controlled trials.

The arrow shows increasing kinds of evidence, not permission to skip safety review. A green lower
step remains valuable, but it cannot make a physical claim from software alone.

In a software device scenario, commanded power, velocity, or position must never be copied
automatically into encoder or velocity feedback. The test supplies each observation independently;
otherwise a broken feedback loop can pass by reading back its own command. The checked-in
[Starter mechanism lesson](<../Test a Mechanism Without Hardware.md>) and optional
[Reference scenarios](<../../examples/Hardware-free Reference Scenarios.md>) show the boundary.

The probes are passive, but a typed Java scenario can still be reactive. Request an action, run its
Task and output phases, and assert the command that production code actually issued. Only then name
and inject the next external fact; advance the one shared clock once and inspect status or the Task
outcome. This preserves causality without asking students to predict future commands in a per-cycle
input file. A small fixture may own setup and the Task-before-output cycle helper, but the request,
command assertion, and observation remain visible in the test. Plain Java is enough for that
software question.

## Start with the Starter status

### Critical code

The Starter presenter reads one cached snapshot:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterRobot.java -->
```java
StarterIntake.Status status = intake.status();
telemetry.addData("intake.mode", status.mode());
telemetry.addData("intake.appliedTargetPower", status.appliedTargetPower());
```

**What to notice**

- The presenter formats one already-computed snapshot; it does not sample hardware.
- Requested mode and applied target are shown as different facts.

**Key APIs:** capability `status()` publishes a cached semantic snapshot; `Telemetry.addData(...)`
formats evidence without owning decisions.

`mode` describes the current semantic command. `appliedTargetPower` is the Plant's cached final
target after its resolver and guards. It is **not** motor feedback and does not prove that the motor
moved. The presenter formats existing facts; it does not resample hardware or make policy.

## A semantic boolean is not an electrical level

### Critical code

`gamepad.y()` is already semantic: it is `true` while Y is pressed. A gamepad trigger is different
again—it is a `ScalarSource` from `0.0` released to `1.0` fully pressed.

An FTC `DigitalChannel` reports an electrical level. Sushi makes the chosen polarity visible:

- `FtcSensors.digitalHigh(...)` is true while the pin is HIGH.
- `FtcSensors.digitalLow(...)` is true while the pin is LOW.

The Reference lift deliberately selects and conditions active-low input:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/capability/lift/ReferenceLiftMechanism.java -->
```java
bottomSwitch = FtcSensors.digitalLow(map, c.bottomSwitchName)
        .debouncedOnOff(0.02, 0.02);
```

**What to notice**

- Electrical polarity is selected once at the mechanism boundary.
- Debounce conditions the chosen meaning; it does not discover or invert that meaning.

**Key APIs:** `FtcSensors.digitalLow(...)` adapts an active-low FTC channel;
`BooleanSource.debouncedOnOff(...)` conditions the semantic fact over shared-clock time.

For its reviewed wiring, released is HIGH/false and depressed is LOW/true. `digitalLow` does not
discover the circuit or mechanical state; the robot author chooses it because LOW means “bottom
switch depressed” for that circuit. Debounce then requires the selected value to remain stable for
20 ms before changing. It neither inverts the value nor supplies its meaning.

## Scale to measured readiness

The Reference launcher caches requested velocity, independent left/right measured velocities,
per-wheel at-target facts, aggregate readiness, conditioned object presence, and whether a temporary
transfer pulse is active. `ready` is true only for a positive target when both finite measurements
meet tolerance. It does not prove that an object launched or scored.

`objectPresent` is status-only in the Reference mechanism. `launchOne()` does not use it to permit
feeding. A team that needs object-gated feeding must add that policy explicitly rather than treating
a telemetry row as an interlock.

The optional `ReferenceInventoryStatusService` shows the next step without adding inventory to the
framework. Three separately debounced active-low inputs become one immutable cached snapshot. Its
`conditionedOccupiedPositionCount` counts asserted sensor positions, not proven physical objects;
`OrderIssue` describes an observation outside the example's ordered-fill pattern without diagnosing
a broken sensor. A presenter reads the full snapshot once, while an Auto Task may observe the same
cached `fullSource()`. The software scenario proves that derivation for authored levels, not the
sensor placement, capacity, or collection result on a robot.

## Print computed evidence; observe visible evidence

The locked Reference flywheel experiment prints what software must calculate: trial number and
state, target velocity, both measured velocities, and elapsed time. When a trial ends, it freezes
time and measurements before requesting zero, so later coast-down cannot relabel the result. The
operator records directly visible or audible facts—direction, vibration, sound, damage, clearance,
and STOP response.

The trial changes only flywheel velocity; it never requests a transfer or release pulse. The whole
production mechanism still updates its normal idle outputs, so the release servo may move to its
configured retracted position and still needs reviewed clearance. A separate, team-authored loaded
launch experiment could record score or miss; this spin-up trial cannot.

Checked-in criteria keep `reviewedForMotion` false. Their target and powered-run cap are placeholders,
not physical permission or success criteria. The cap is a cooperative boundary checked once per
active loop: zero is applied on the first observed loop at or after the limit, so the safety plan
must allow for worst-case loop delay. `TARGET_REACHED` means only that the computed wheel condition
was met; it is not an overall `PASS`.

Before any supervised run, the team must author the complete question, safe range, success threshold,
procedure, stop conditions, and external observations in the full experiment card.

## Check your understanding

**The trial reports `TARGET_REACHED`, but the operator observes unacceptable vibration. Did it pass?**

No. Software may retain its computed timing result while the operator records the vibration and the
team rejects the configuration. Neither observation should be converted silently into the other.

## Go deeper when needed

- Electrical conditioning: [`ReferenceLiftMechanism.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/capability/lift/ReferenceLiftMechanism.java>)
- Cached launcher evidence: [`ReferenceLauncherMechanism.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/capability/launcher/ReferenceLauncherMechanism.java>)
- Robot-owned multi-sensor evidence: [`ReferenceInventoryStatusService.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/capability/inventory/ReferenceInventoryStatusService.java>)
- Hardware-free feedback cases: [Hardware-free Reference scenarios](<../../examples/Hardware-free Reference Scenarios.md>)
- Safe experiment card and workflow: [Subsystem Experiments](<../../examples/Subsystem Experiments.md>)
- Evidence vocabulary: [Glossary](<../../reference/Glossary.md#evidence>)
- [Choose another Sushi topic](<../Beginner's Guide.md>)
