---
tags:
  - Test & Tune
---

# Actuator bring-up: direction and safe endpoints

**Learning mode:** Operational runbook

Use **HW: Actuator Bring-up** before enabling a new motor or servo in a physical mechanism OpMode. Reading,
software authoring, and software tests need no hardware. This is the
one ordinary Sushi tool for answering two hardware questions:

1. Which FTC `Direction` makes positive command mean what the mechanism owner expects?
2. If travel is limited, which commands or encoder readings correspond to points deliberately
   short of the physical obstructions?

**Bounded** means the mechanism has allowed lower and upper targets. **Native** means the numbers
the FTC device API uses; the mechanism can give those numbers another meaning, such as inches above
the bottom or `0 = closed`, `1 = open`. Its endpoints are those chosen minimum/maximum meanings,
not automatically the numerically smallest/largest device reading.

The tool establishes configuration evidence. It does not automatically find a hard stop, tune a
controller, edit a profile, or prove that a mechanism is safe under production load.

## Before starting the OpMode

- Remove game pieces and support the robot so the selected device cannot drive it away.
- Keep people, hair, tools, and wires outside every possible motion path.
- Begin unloaded. Do not use the generic tool on an unsupported gravity-loaded arm or lift.
- Make the FTC Robot Configuration containing the intended device name active before INIT; the
  picker shows only configured hardware.
- Decide what **positive** motion should mean before testing direction.
- Know how to stop the selected tester entry and assign one person to do it. When using Panels,
  browser STOP is not a physical emergency stop; keep immediate access to robot power.
- For a bounded mechanism, mark a conservative point short of each physical obstruction. Capture
  these safe points, not a stall or collision.

Run either **FW: Testers (Driver Station)** with physical gamepad input or
**FW: Testers (Panels)** with Panels virtual-gamepad input, then open **HW: Actuator Bring-up**.
Choose one input source for the run: the Driver Station entry ignores Panels controls, and the
Panels entry ignores physical gamepads; inputs are never merged. Both entries use this same tester
and show the same telemetry on the Driver Station and Panels. For launch, connection, navigation,
and restart instructions, see [Using the tester console](<Using the Tester Console.md>). The picker
lists configured hardware by type and name, for example:

```text
[DC motor] lift
[CR servo] turret
[Servo] claw
```

Selecting an entry during INIT only inspects it. The tester does not command an actuator until you
start the OpMode and the controls have returned to neutral.

## One control grammar

| Control | Meaning |
|---|---|
| A | Arm after the OpMode starts and a neutral sample; for a standard servo reporting unknown-state `NaN`, one fresh press acknowledges the warning, releasing A separates the presses, and a second fresh press submits `0.5` and arms |
| B | Disarm; write motor/CR-servo power zero, or stop standard-servo jog updates and retain its last request |
| X | After A has prepared the device and while disarmed, test the other temporary FTC `Direction`; captures are cleared |
| D-pad up/down | While disarmed, change motor/CR-servo power by `0.05` within `0.05..0.30`, or standard-servo jog rate by `0.01` native command units/second within `0.01..0.25`; both start at `0.05` |
| Hold left bumper | Move in the negative direction while armed |
| Hold right bumper | Move in the positive direction while armed |
| D-pad left | After A has prepared the device and while disarmed, capture `nativeAtPlantMin` |
| D-pad right | After A has prepared the device and while disarmed, capture `nativeAtPlantMax` |
| gamepad START | While disarmed, clear both captures |
| Y | While disarmed, finish and print one candidate result after a successful nonzero jog under the current `Direction` |
| BACK | Return to the picker; attempt to restore temporary Direction/settings and command motor/CR-servo zero, while a standard servo retains its last request |

Exactly one bumper must be held. Releasing both, holding both, pressing B, leaving the device,
stopping the OpMode, or encountering a failure immediately requests or best-effort attempts zero
motor/CR-servo power. A hardware-write or cleanup failure means physical state is uncertain: use
FTC Driver Station STOP and robot power rather than trusting telemetry. For a standard servo, those
actions stop new jog commands and retain the last request; the shaft may still be traveling toward
it. After a selection or direction change, return every movement control to neutral before arming
again.

Endpoint names are semantic. `nativeAtPlantMin` means “the hardware fact that should represent the
minimum of my chosen Plant coordinate”; it does not mean the numerically smaller value. Reversed
servo endpoints are valid and are never sorted.

## DC motor: direction and optional travel span

The tester temporarily uses `RUN_WITHOUT_ENCODER`. Power magnitude starts at `0.05`; while
disarmed, D-pad up/down changes it in `0.05` steps within the inclusive `0.05..0.30` range. It
snapshots and later restores the motor's original mode, direction, and zero-power behavior. It
never resets the encoder.

For direction-only devices such as an intake or flywheel:

1. Start the OpMode, neutralize the controls, and press A.
2. Hold the right bumper briefly to test positive command; use the left bumper only to cross-check
   negative command.
3. Press B and confirm the positive direction means what you decided.
4. Press X while disarmed and repeat only if the direction is wrong.
5. Press Y with no endpoint captures to print the tested `Direction`.

For a bounded encoder mechanism:

1. Approach one edge slowly.
2. Back away until the mechanism has deliberate clearance.
3. Press B so motor power is zero.
4. Capture that position as Plant minimum or maximum.
5. Repeat at the other edge, then press Y.

The result reports the direction-adjusted SDK encoder readings, signed span, and absolute span. A
simple relative mechanism should have a positive signed span under the selected direction. If it
does not, correct direction or reconsider which physical pose is the semantic Plant minimum.

An incremental encoder's raw values are not durable endpoints. If the safe captures are `7342` and
`11452`, the useful fact is the `4110`-tick travel. A later homing/reference action establishes
where Plant `0.0` is for that run.

### Critical code

Using ticks as Plant units:

Abbreviated shape (omissions shown):

<!-- teaching-shape -->
```java
// ...begin the position-Plant recipe...
.bounded(0.0, 4110.0)
.nativeUnits()
.needsReference("lift not homed")
```

Using meaningful units, such as an 18-inch lift:

Abbreviated shape (omissions shown):

<!-- teaching-shape -->
```java
// ...begin the position-Plant recipe...
.bounded(0.0, 18.0)
.scaleToNative(4110.0 / 18.0)
.needsReference("lift not homed")
```

**What to notice**

- `bounded(...)` declares the legal public target interval; it is not a hardware-limit detector.
- The unit mapping and runtime reference answer different questions and stay explicit.
- The captured span is useful, while an incremental encoder's arbitrary startup count is not a durable zero.

**Key APIs**

- `bounded(min, max)` — declares the Plant-coordinate target range.
- `nativeUnits()` / `scaleToNative(...)` — selects how Plant coordinates convert to native units.
- `needsReference(reason)` — prevents position use until a runtime reference is established.

Do not paste the arbitrary raw minimum as a permanent zero. Use a limit switch, index, absolute
sensor, or deliberately chosen startup pose through the position-reference APIs described in
[`Robot Calibration Tutorials`](<Robot Calibration Tutorials.md>).

## Continuous-rotation servo: direction only

A CR servo accepts power but supplies no position feedback. Its dead-man power magnitude starts at
`0.05`; while disarmed, D-pad up/down changes it in `0.05` steps within `0.05..0.30`. The generic
tester cannot discover positional bounds.

A rotating plate on a CR servo may be:

- an unbounded power mechanism;
- a periodic position mechanism with an external encoder/index; or
- a bounded position mechanism with external feedback and mechanism-specific homing.

Those are different robot designs. Do not infer one from elapsed run time or a CR servo's power
command.

## Standard servo: native endpoints without choosing Plant units

The standard-servo jog rate starts at `0.05` logical native command units/second. While disarmed,
D-pad up/down changes it in `0.01` steps within `0.01..0.25` units/second. During a jog, each loop
may change the submitted logical command by at most `0.005`, even if the elapsed loop time would
produce a larger step. These are command-space limits, not measured shaft speed or angle.

The hardware servo programmer and the FTC SDK solve different problems:

- A **hardware servo programmer** may set electronic limits inside the servo, such as a nominal
  270-degree range.
- The FTC SDK commands a **logical native position from `0.0` to `1.0`**. The configured FTC servo
  type/controller maps that value to PWM, and the servo's own programming and mechanics determine
  the resulting physical travel.
- Your Plant chooses meaningful public units and safe bounds for the attached mechanism.

If `Servo.getPosition()` reports a finite known command state, arming performs no servo write and the
tester jogs gradually from that value. If it explicitly returns the Servo API's documented unknown-
state `NaN`, the conservative first-use bootstrap requires **two fresh A presses**: remove the
horn/linkage, or prove every possible position under the current PWM mapping and servo programming
is clear; press A once to acknowledge without writing; release A; then press A again to submit
logical command `0.5` and arm. `0.5` is only a command-space midpoint, not a promise of a centered
shaft angle. The API cannot prove that a finite reported value was physically delivered or that it
reflects the shaft.

Releasing the bumper or pressing B stops new jog updates and retains the last request; a standard
servo has no power-zero position and may still be traveling toward the request.
`Servo.getPosition()` is SDK command state, not shaft-angle feedback, so visual inspection and safe
clearance are still required.

Before capturing either servo endpoint, release the bumper, wait for the linkage to become visibly
stationary, inspect clearance from the physical obstruction, then press B and capture the backed-off
command while disarmed. The captured number proves which command was submitted; your observation is
the evidence that the mechanism arrived and retained safe clearance.

Suppose a 270-degree programmed servo is mechanically allowed to move a plate through only 180
degrees. You capture native commands `0.17` and `0.83`, then choose degrees as Plant units:

### Critical code

Replace the demonstration endpoints with backed-off captures from your mechanism.

Abbreviated shape (omissions shown):

<!-- teaching-shape -->
```java
// ...begin the servo position-Plant recipe...
.bounded(0.0, 180.0)
.rangeMapsToNative(0.17, 0.83)
```

For a claw whose public coordinate is simply closed-to-open:

Abbreviated shape (omissions shown):

<!-- teaching-shape -->
```java
// ...begin the servo position-Plant recipe...
.bounded(0.0, 1.0)
.rangeMapsToNative(0.32, 0.76)
```

A reversed linkage may truthfully be:

Abbreviated shape (omissions shown):

<!-- teaching-shape -->
```java
// ...begin the servo position-Plant recipe...
.bounded(0.0, 1.0)
.rangeMapsToNative(0.76, 0.32)
```

**What to notice**

- Plant bounds use mechanism meaning; native endpoints remain FTC logical servo commands.
- Reversed native endpoints are valid and must not be sorted.
- The affine mapping commands the servo but does not measure shaft angle or arrival.

**Key APIs**

- `bounded(min, max)` — declares legal targets in the mechanism's public units.
- `rangeMapsToNative(nativeAtMin, nativeAtMax)` — maps those two bounds to logical servo commands.

`rangeMapsToNative(...)` performs an **affine mapping**, a straight-line conversion between two
endpoint pairs, from the two Plant bounds to the
two native endpoints. It does not reprogram the servo, measure angle, linearize a nonlinear
linkage, or prove arrival.

## Keep the five domains separate

| Term | What it means |
|---|---|
| Servo/controller mapping | FTC servo type maps logical command to controller PWM; servo programming and mechanics turn PWM into motion |
| Native domain | FTC command/measurement units: motor ticks, power, or logical servo `0.0..1.0` |
| Physical safe interval | Motion the real mechanism can tolerate with deliberate clearance |
| Plant bounds | Legal targets expressed in the mechanism's chosen public units |
| Mapping/reference | Conversion to native units, and the run-time anchor that establishes position zero |

Plant bounds limit command values, not physical safety. A finite request outside the range may be
clamped to a bound and reported as `CLAMPED_TO_RANGE`; invalid configuration is rejected, and an
unavailable/non-finite runtime target has its own status. Bounds cannot prove that inertia, controller overshoot,
load, flex, or linkage geometry stays inside the physical interval.

## What to copy, and what to verify next

The copy-ready candidate remains on the final Driver Station/Panels screen until BACK and is also
written to Logcat under the `SushiActuatorBringUp` tag. Copy it from either location into the
data-only robot configuration or mechanism builder; the tool does not modify either one.

Then verify the production owner:

1. Rebuild with the copied direction/mapping. Recapture servo endpoints after changing FTC servo
   type/PWM configuration, `Direction`, servo programming, horn geometry, or linkage.
2. Run the robot-specific mechanism at conservative limits.
3. Confirm semantic commands, hold/stop behavior, sensors, guards, and homing.
4. Repeat under expected load before increasing speed or power.
5. For a drivetrain, finish with the robot-specific configured-drivetrain and raised-wheel mecanum
   checks.

## Use a different workflow when

Do not use this generic wizard to discover limits by collision or stall, coordinate coupled
actuators, hold an unsafe gravity load, combine a CR servo with a separately owned sensor, calibrate
a periodic mechanism, define mechanism-specific homing, or tune PIDF. Those jobs require the real
mechanism owner and its safety policy. Closed-loop control tuning is covered separately in
[`Control Tuning Workflow`](<Control Tuning Workflow.md>).

[Testing and calibration](<README.md>) · [FTC actuators and Plants](<../ftc-boundary/FTC Actuators & Plants.md>) · [Common problems](<../troubleshooting/Common Problems.md>)
