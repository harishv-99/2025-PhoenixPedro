---
tags:
  - Learn
---

# Evidence and experiments

**Learning mode:** Architecture reference

This page explains what each evidence level proves;
the linked scenario and experiment modules contain the code-authoring work.

**Question:** What does a boolean, status row, or experiment result actually prove?

**Reading time:** about 8 minutes

Sushi separates a request, an electrical observation, a measurement, and a physical conclusion.
This page requires no sensor, code edit, or robot run.

## Climb the canonical evidence ladder

Use the lowest-cost level that can truthfully answer the current question, then widen the boundary
only when the next fact requires it. This is the same five-level ladder defined in
[How to test a Sushi component](<../../testing-calibration/How to test a Sushi component.md#the-five-evidence-levels>):

```text
1. semantic intent
    -> 2. software-device scenario
    -> 3. supplied managed slice
    -> 4. maintainer regression
    -> 5. physical bring-up, calibration, or experiment
```

1. **Semantic intent** keeps the control, Task, or policy decision real and replaces the capability
   with a small recorder.
2. A **software-device scenario** constructs the production mechanism and Plants with a test-only
   `HardwareMap`, injects observations explicitly, and records actuator commands.
3. A **supplied managed slice** keeps enough production lifecycle to establish phase order and
   cleanup without claiming robot-specific response.
4. A **maintainer regression** protects exhaustive edge cases or structural contracts that a
   beginner may run without treating its fixtures as robot-code templates.
5. **Physical bring-up, calibration, or experiment** observes the assembled robot under stated
   conditions. Bring-up establishes one device fact; calibration records robot facts; an experiment
   evaluates a written subsystem question and criterion.

The arrow widens evidence scope; it is not permission to skip safety review. A green software level
remains valuable, but it cannot make a physical claim.

### Test shapes are not extra evidence levels

A reactive Java scenario, modeled simulation, managed-lifecycle slice, or broad regression suite is
a **test shape** chosen inside that ladder, not another ladder. An optional modeled simulation adds
an authored dynamics model to a software question; state its assumptions and fidelity explicitly.
It may support a level-2 mechanism scenario or a level-3 managed slice, but modeled motion never
becomes level-5 physical evidence. Likewise, “managed slice” and “regression” describe scope and
audience; neither word upgrades what the test actually observed.

In a software device scenario, commanded power, velocity, or position must never be copied
automatically into encoder or velocity feedback. The test supplies each observation independently;
otherwise a broken feedback loop can pass by reading back its own command. The checked-in
[focused software checkpoints](<../../build/README.md>) and
[software-device scenarios](<../../examples/Hardware-free Reference Scenarios.md>) show the boundary.

The probes are passive, but a typed Java scenario can still be reactive. Request an action, run its
Task and output phases, and assert the command that production code actually issued. Only then name
and inject the next external fact; advance the one shared clock once and inspect status or the Task
outcome. This preserves causality without asking students to predict future commands in a per-cycle
input file. A small fixture may own setup and the Task-before-output cycle helper, but the request,
command assertion, and observation remain visible in the test. Plain Java is enough for that
software question.

## Start with the Starter status

### Critical code

The Starter presenter asks for one new capability-shaped capture of cached facts:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/robot/StarterRobot.java -->
```java
StarterIntake.Status status = intake.status();
telemetry.addData("intake.mode", status.mode());
telemetry.addData("intake.appliedTargetPower", status.appliedPower());
```

**What to notice**

- The presenter formats one new capture of already-computed cached facts; it does not sample
  hardware.
- Requested mode and applied target are shown as different facts.

**Key APIs:** `status()` returns a new capability view over cached facts;
`Telemetry.addData(...)` formats it without owning decisions. Generic command and Plant snapshots
stay behind that boundary.

`mode()` names semantic intent. `appliedPower()` is the resolver's cached final target, not feedback
or proof of motion.

## A semantic boolean is not an electrical level

### Critical code

`gamepad.y()` is already semantic: it is `true` while Y is pressed. A gamepad trigger is different
again—it is a `ScalarSource` from `0.0` released to `1.0` fully pressed.

An FTC `DigitalChannel` reports an electrical level. Sushi makes the chosen polarity visible:

- `FtcSensors.digitalHigh(...)` is true while the pin is HIGH.
- `FtcSensors.digitalLow(...)` is true while the pin is LOW.

The Reference inventory service deliberately selects and conditions three active-low inputs:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/capability/inventory/ReferenceInventoryStatusService.java -->
```java
firstOccupied = FtcSensors.digitalLow(firstChannel)
        .debouncedOnOff(occupiedDelay, vacatedDelay);
secondOccupied = FtcSensors.digitalLow(secondChannel)
        .debouncedOnOff(occupiedDelay, vacatedDelay);
thirdOccupied = FtcSensors.digitalLow(thirdChannel)
        .debouncedOnOff(occupiedDelay, vacatedDelay);
```

**What to notice**

- Electrical polarity is selected once at the mechanism boundary.
- Debounce conditions the chosen meaning; it does not discover or invert that meaning.

**Key APIs:** `FtcSensors.digitalLow(...)` adapts an active-low FTC channel;
`BooleanSource.debouncedOnOff(...)` conditions the semantic fact over shared-clock time.

For its reviewed wiring, an empty position is HIGH/false and an occupied position is LOW/true.
`digitalLow` does not discover the circuit or physical object; the robot author chooses it because
LOW means “position occupied” for that circuit. Debounce then requires the selected value to remain
stable for the configured interval before changing. It neither inverts the value nor supplies its
meaning.

## Scale to measured readiness

Each successful paired-flywheel update publishes one immutable
[`ReferenceFlywheels.Status`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/flywheel/ReferenceFlywheels.Status.html>):
the grouped Plant snapshot plus two independent wheel measurements. Its requested/selected/applied
methods avoid generic navigation. `ready()` requires one positive value selected and applied
without fallback plus both wheels in tolerance; it does not prove a launch or score. The launcher
composes this complete value as `status().flywheels()` rather than mirroring its fields.

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
state, target velocity, both measured velocities, and elapsed time. When a trial ends, one immutable
terminal result freezes the flywheel Status, authored target, and elapsed time before requesting
zero, so later coast-down cannot relabel the result. The operator records directly visible or
audible facts—direction, vibration, sound, damage, clearance, and STOP response.

The trial owns only the focused paired-flywheel mechanism; it has no transfer, release, or object
sensor to move accidentally. A separate, team-authored loaded-launch experiment could record score
or miss; this spin-up trial cannot.

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

- Electrical conditioning: [`ReferenceInventoryStatusService`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/inventory/ReferenceInventoryStatusService.html>)
- All-or-nothing launcher publication: [`ReferenceLauncherMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/launcher/ReferenceLauncherMechanism.html>)
- Robot-owned multi-sensor evidence: [`ReferenceInventoryStatusService`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/inventory/ReferenceInventoryStatusService.html>)
- Hardware-free feedback cases: [Hardware-free Reference scenarios](<../../examples/Hardware-free Reference Scenarios.md>)
- Safe experiment card and workflow: [Subsystem Experiments](<../../examples/Subsystem Experiments.md>)
- Evidence vocabulary: [Glossary](<../../reference/Glossary.md#evidence>)
- [Choose another Sushi topic](<../Beginner's Guide.md>)
