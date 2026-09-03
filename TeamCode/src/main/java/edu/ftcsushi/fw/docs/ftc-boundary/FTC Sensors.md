---
tags:
  - Advanced
---

# FTC Sensors

Use this guide alongside [`Recommended Robot Design`](<../design/Recommended Robot Design.md>) when deciding whether a sensor belongs in a local control loop, an event/classification supervisor, or a spatial-guidance stack.

This guide explains the recommended way to read FTC sensors in Sushi.

Sushi keeps FTC SDK specifics in the **boundary layer** (`edu.ftcsushi.fw.ftc`).
For sensor inputs, the boundary adapters are in:

* `edu.ftcsushi.fw.ftc.FtcSensors`

The output types are framework-native:

* `ScalarSource` – produces a `double` each loop
* `Source<T>` – produces a value object each loop
* `BooleanSource` – produces a `boolean` each loop (typically derived via thresholds + filters)

The goal is to make sensor reads participate in Sushi's **one loop, one heartbeat** design:

> Define sources once, sample them using the current `LoopClock`.

---

## Battery voltage

Battery voltage is represented as a `ScalarSource` in volts. This is useful for debug telemetry and
for framework-regulated mechanisms that intentionally compensate power commands as battery voltage
sags.

```java
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.ftc.FtcSensors;

// Memoized per loop by default.
ScalarSource batteryVoltage = FtcSensors.batteryVoltage(hardwareMap);
```

Notes:

* `FtcSensors.batteryVoltage(hardwareMap)` samples the FTC `VoltageSensor` entries exposed by the
  hardware map and returns the lowest positive finite reading. That is the conservative value when
  multiple hubs report voltage.
* Invalid readings (`NaN`, infinity, zero, or negative values) are ignored. If no usable reading is
  available, the source returns `Double.NaN` instead of inventing a voltage.
* Keep compensation policy outside the raw sensor adapter. A standard regulated Plant answers
  `voltageCompensationFrom(...)` after PID/feedforward; an advanced custom regulator may instead
  use the generic regulator decorator. Both keep voltage sampling out of robot-loop arithmetic.

---

## Motor and incremental-encoder measurements

FTC motor ports expose encoder position and direct velocity through different SDK paths:

```java
ScalarSource positionTicks = FtcSensors.motorPositionTicks(hardwareMap, "armMotor");
ScalarSource directVelocityTicksPerSec =
        FtcSensors.motorVelocityTicksPerSec(hardwareMap, "armMotor");
```

Both helpers publish one successful hardware read per `LoopClock.cycle()`. If an SDK read throws,
the adapter does not cache zero or the preceding loop's value as a success; a later same-cycle call
may retry. Position is the SDK-observed signed 32-bit count. Direct velocity is the SDK's
motor-velocity reading; converting its units does not make its underlying controller representation
wider.

For ordinary built-in motor encoders, the regulated motor-velocity builder's
`.internalEncoder(...)` answer uses direct SDK velocity. For a physically external incremental
encoder, use `.externalEncoder(name[, direction])`; that path maintains continuous signed-32-bit
position at the FTC boundary, then uses the generic `ScalarSource.ratePerSecond()` transform to
produce native ticks/second:

The following Plant builder is an excerpt from the owning mechanism constructor. In a structured
robot, the composition root passes that constructor `HardwareMap` plus its active data-only config.
The mechanism snapshots and validates that config before its own hardware lookup rather than asking
the root to duplicate owner rules or construct this Plant itself.

```java
// Inside the mechanism constructor; flywheel is a private field.
this.flywheel = FtcActuators.plant(hardwareMap)
        .motor("flywheel", Direction.FORWARD)
        .velocity()
        .regulated()
            .externalEncoder("flywheelEncoder")
        .bounded(0.0, MAX_FLYWHEEL_RPM)
        .scaleToNative(TICKS_PER_REV / 60.0)
        .velocityTolerance(75.0)
        .setpointFromAccelerationLimitedProfile(MAX_RPM_PER_SEC)
        .feedbackFromPid(kP, kI, kD)
        .feedforwardFromMotion(kS, kV, kA)
        .outputPowerLimitedTo(0.0, MAXIMUM_POWER)
        .targetFromNewCommand(0.0)
        .build();
```

The robot still owns physical facts: which encoder is attached, its counts per revolution,
direction, wiring, and appropriate operating range. Configuration metadata cannot prove that the
named motor port contains a particular encoder or that the hub captured every electrical edge. For
a high-count-rate REV Through Bore encoder, use a hardware-counted Control/Expansion Hub encoder
port 0 or 3 and validate the exact hardware/firmware/loop stack before claiming reliable
maximum-speed feedback.

Keep optional smoothing, outlier handling, and semantic unit conversion outside the raw FTC adapter.
`ratePerSecond()` assumes its input is a linear unwrapped position; an absolute-angle source must be
unwrapped by a layer that knows its period first.

---

## Motor current

Use `motorCurrentAmps(...)` for a raw per-motor current source in explicit amps:

```java
// Ordinary named construction resolves this DcMotorEx once, during construction.
ScalarSource intakeCurrentAmps =
        FtcSensors.motorCurrentAmps(hardwareMap, "intakeMotor");

// If the owner already resolved the exact device, use the direct overload instead.
// ScalarSource intakeCurrentAmps = FtcSensors.motorCurrentAmps(intakeMotor);
```

Named construction performs one eager `DcMotorEx` lookup. The adapter does not call `getCurrent(...)`
during construction; any device initialization caused by lookup remains SDK `HardwareMap` behavior.

Choose one construction form, construct the source once in the owning mechanism, and share that
source with every consumer. Memoization belongs to the source instance, so independently wrapping
the same motor can cause independent SDK reads. Between resets, the first normally returning read
in a non-null `LoopClock.cycle()` publishes its exact `double`; same-cycle consumers reuse it. If
the SDK read throws a `RuntimeException`, the failure escapes without being cached and a later
nonrecursive same-cycle call may retry. `reset()` clears the software memo after propagating the
ordinary upstream reset, permitting a fresh read even in the same cycle; it does not reset,
configure, or probe the motor.

The SDK current interface returns an unvalidated `double`, so Sushi forwards every normally
returned value unchanged, including zero, a negative finite value, `NaN`, or either infinity. This
software contract does not claim that real FTC hardware normally produces every such value. The
pinned FTC SDK's Lynx controller implementation can catch some current-read failures and return an
indistinguishable placeholder `0.0`, so Sushi cannot classify a normally returned zero as either
a real observation or an unavailable reading.

This adapter is read-only acquisition. It does not configure SDK current alerts, filter or
aggregate samples, create a current manager, or choose thresholds, hysteresis, dwell, homing, jam
recovery, derating, subsystem priorities, or another response. The adopting robot owns those
decisions. Before using current for behavior or protection, validate the configured device and
controller, wiring, firmware/cache behavior, accuracy and sign, calibration, resolution, noise,
freshness, cadence, and loop read cost on the actual hardware. Also validate disconnected,
disabled, startup, stall, and fault behavior plus every mechanism-specific threshold, filter,
timeout, recovery, derating, priority, and safety choice.

---

## Distance sensors

Distance sensors are represented as a `ScalarSource` in your chosen unit.

```java
import com.qualcomm.robotcore.hardware.DistanceSensor;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.ftc.FtcSensors;

DistanceSensor gateSensor = hardwareMap.get(DistanceSensor.class, "gateDistance");

// Memoized per loop by default.
ScalarSource gateDistanceCm = FtcSensors.distanceCm(gateSensor);

// Ball present at gate: hysteresis + debounce.
BooleanSource ballAtGate = gateDistanceCm
        .hysteresisBelow(6.0, 7.0)     // ON <= 6cm, OFF >= 7cm
        .debouncedOnOff(0.05, 0.05);   // 50ms stability
```

Notes:

* `FtcSensors.distance*(...)` sources publish **one successful sample per loop**. Repeated reads
  reuse that result; a failed hardware read publishes no substitute and remains eligible for a
  same-cycle retry.
* For stable boolean gates, prefer **hysteresis** (threshold-domain stability) and then
  **debounce** (time-domain stability).

---

## Color sensors

Sushi supports two useful ways to read FTC color sensors:

* `FtcSensors.rgba(...)` → raw `Source<Rgba>` channel counts in sensor-native units
* `FtcSensors.normalizedRgba(...)` → normalized `Source<NormalizedRgba>` values (usually `0..1`)

For simple close-range classification, **normalized colors are usually the better starting point**
because they behave more consistently across lighting changes and sensor gain.

### Raw channels

```java
import com.qualcomm.robotcore.hardware.ColorSensor;
import edu.ftcsushi.fw.core.color.Rgba;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.ftc.FtcSensors;

ColorSensor loaderColor = hardwareMap.get(ColorSensor.class, "loaderColor");

// Memoized per loop by default.
Source<Rgba> rgba = FtcSensors.rgba(loaderColor);

BooleanSource looksGreen = rgba
        .mapToBoolean(c -> c.gRatio() > 0.45 && c.rRatio() < 0.35)
        .debouncedOnOff(0.05, 0.05);
```

`Rgba` intentionally does not assume a specific numeric range for channel values. Many sensors
report 0–255, but others report larger values. Sushi encourages **ratio-based** logic (e.g.
`gRatio()`), which tends to transfer better between sensors.

### Normalized channels (recommended for classification)

```java
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import edu.ftcsushi.fw.core.color.NormalizedRgba;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.ftc.FtcSensors;

enum BallColor { RED, GREEN, PURPLE, UNKNOWN }

NormalizedColorSensor loaderColor = hardwareMap.get(NormalizedColorSensor.class, "loaderColor");
loaderColor.setGain(6.0f);   // tune on your robot

Source<NormalizedRgba> color = FtcSensors.normalizedRgba(loaderColor);

Source<BallColor> rawBallColor = color.map(c -> {
    // Reject dim / weak-color readings before trusting classification.
    if (c.a < 0.06 || c.chroma() < 0.08) return BallColor.UNKNOWN;

    if (c.rRatio() > 0.48 && c.gRatio() < 0.28 && c.bRatio() < 0.28) return BallColor.RED;
    if (c.gRatio() > 0.45 && c.rRatio() < 0.32 && c.bRatio() < 0.28) return BallColor.GREEN;
    if (c.gRatio() < 0.26 && c.rRatio() > 0.28 && c.bRatio() > 0.28) return BallColor.PURPLE;
    return BallColor.UNKNOWN;
});
```

Notes:

* Prefer **ratios + confidence gates** over hue-only logic.
* `Rgba` and `NormalizedRgba` intentionally expose a parallel helper surface:
  `sumRgb()`, `maxChannel()`, `minChannel()`, `chroma()`, channel ratios, and simple
  HSV-style helpers (`hueDeg()`, `saturation()`, `value()`).
* For object classification, only trust the reading when the object is **close enough** or the
  brightness / alpha channel is strong enough.
* Treat HSV as **secondary debug telemetry**, not the first thing you threshold on.
  Hue gets noisy when chroma is low.
* If your sensor supports a built-in light or gain, set those during initialization; `FtcSensors`
  only adapts the reading into Sushi sources.
* The framework hardware menu includes `HW: Color Sensor (Normalized)`, which prints
  normalized RGBA, ratios, alpha/chroma, HSV, and optional raw RGBA detail while you tune gain.

### Reset-driven slot / window memory

Sometimes the sensor should classify what it sees **right now**, but your robot should remember that
result until a separate boundary event begins the next observation window. For example, an encoder
pulse might mark the start of the next spinner slot. That memory belongs in `Source<T>`
composition, not inside `FtcSensors`.

```java
enum SlotColor { EMPTY, GREEN, PURPLE, UNKNOWN }

BooleanSource slotBoundaryPulse = encoderSlotBoundaryPulse;

Source<SlotColor> slotColor = rawBallColor.accumulateUntil(
        slotBoundaryPulse,
        MyRobot::updateSlotColor,
        SlotColor.EMPTY
);
```

This keeps the FTC boundary adapter focused on **reading the sensor**, while the robot-specific
reducer controls which samples should win, merge, or be ignored.

Use `accumulateUntil(...)` when the boundary pulse is already part of the loop graph. If a
supervisor, tester, or other owner object defines the window lifecycle imperatively, keep the
memory as `accumulate(...)` and call `reset()` from that owner instead.

---

## Where classification belongs

Sushi separates:

1. **Sensor reads** (FTC SDK boundary) → Sources (`FtcSensors`)
2. **Signal conditioning** (debounce/hysteresis/memo/edges) → Sources (`ScalarSource`, `BooleanSource`)
3. **Robot policy** (what to do with colors / balls) → your robot code

For more advanced cases (specific ball colors, rejecting wrong colors, slot-local memory, etc.),
Sushi can provide generic helpers (filters, hold-last-valid patterns, reset-driven accumulation,
action queues), but the *meaning* of colors, reset boundaries, and thresholds are typically
robot-specific.

---

## Touch sensors

Touch / limit switches are a natural `BooleanSource`.

```java
import com.qualcomm.robotcore.hardware.TouchSensor;
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.ftc.FtcSensors;

TouchSensor limit = hardwareMap.get(TouchSensor.class, "armLimit");

// Memoized per loop by default.
BooleanSource atLimit = FtcSensors.touchPressed(limit)
        .debouncedOnOff(0.02, 0.02); // 20ms stability to ignore switch bounce

BooleanSource justHitLimit = atLimit.risingEdge();
```

Notes:

* Use `risingEdge()` / `fallingEdge()` for event-style behavior (one-loop pulses).
* Use `debouncedOnOff(...)` if the physical switch bounces.

---

## Digital inputs

Digital sensors (beam breaks, hall sensors, endstops) are commonly wired as FTC `DigitalChannel` devices.

Sushi provides two explicit adapters:

* `FtcSensors.digitalHigh(...)` – true when the pin is **HIGH**
* `FtcSensors.digitalLow(...)` – true when the pin is **LOW** (common for active-low sensors)

```java
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.ftc.FtcSensors;

// Example: active-low beam break (LOW means "beam broken").
BooleanSource beamBroken = FtcSensors.digitalLow(hardwareMap, "beamBreak")
        .debouncedOnOff(0.01, 0.02);

BooleanSource beamJustBroken = beamBroken.risingEdge();
BooleanSource beamJustCleared = beamBroken.fallingEdge();
```

Notes:

* The adapter explicitly sets the channel to `INPUT` mode.
* For stability, prefer `hysteresis...` on analog values and `debounced...` on boolean gates.

---

## Analog inputs

Analog sensors (potentiometers, distance sensors with analog output, custom voltage dividers) are exposed as a `ScalarSource`.

```java
import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.ftc.FtcSensors;

ScalarSource potVolts = FtcSensors.analogVoltage(hardwareMap, "armPot");

// Example conversion: 0..3.3V -> 0..270 degrees (your robot will differ!)
ScalarSource armDeg = potVolts.mapToDouble(v -> (v / 3.3) * 270.0);

// Stable "at top" gate: hysteresis (threshold stability) + debounce (time stability).
BooleanSource atTop = armDeg
        .hysteresisAbove(250.0, 245.0)
        .debouncedOnOff(0.05, 0.05);
```

Notes:

* `ScalarSource` has built-in helpers like `clamped(...)`, `deadband(...)`, `hysteresisAbove(...)`, and `holdLastFinite(...)`.
* Use `holdLastFinite(...)` when a sensor sometimes returns NaN/Inf (common in some vision pipelines).
