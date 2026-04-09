# FTC Sensors

Use this guide alongside [`Recommended Robot Design`](<../design/Recommended Robot Design.md>) when deciding whether a sensor belongs in a local control loop, an event/classification supervisor, or a spatial-guidance stack.

This guide explains the recommended way to read FTC sensors in Phoenix.

Phoenix keeps FTC SDK specifics in the **boundary layer** (`edu.ftcphoenix.fw.ftc`).
For sensor inputs, the boundary adapters are in:

* `edu.ftcphoenix.fw.ftc.FtcSensors`

The output types are framework-native:

* `ScalarSource` – produces a `double` each loop
* `Source<T>` – produces a value object each loop
* `BooleanSource` – produces a `boolean` each loop (typically derived via thresholds + filters)

The goal is to make sensor reads participate in Phoenix's **one loop, one heartbeat** design:

> Define sources once, sample them using the current `LoopClock`.

---

## Distance sensors

Distance sensors are represented as a `ScalarSource` in your chosen unit.

```java
import com.qualcomm.robotcore.hardware.DistanceSensor;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.ftc.FtcSensors;

DistanceSensor gateSensor = hardwareMap.get(DistanceSensor.class, "gateDistance");

// Memoized per loop by default.
ScalarSource gateDistanceCm = FtcSensors.distanceCm(gateSensor);

// Ball present at gate: hysteresis + debounce.
BooleanSource ballAtGate = gateDistanceCm
        .hysteresisBelow(6.0, 7.0)     // ON <= 6cm, OFF >= 7cm
        .debouncedOnOff(0.05, 0.05);   // 50ms stability
```

Notes:

* `FtcSensors.distance*(...)` sources are **memoized per loop** so the hardware sensor is sampled
  at most once per `LoopClock.cycle()`.
* For stable boolean gates, prefer **hysteresis** (threshold-domain stability) and then
  **debounce** (time-domain stability).

---

## Color sensors

Phoenix supports two useful ways to read FTC color sensors:

* `FtcSensors.rgba(...)` → raw `Source<Rgba>` channel counts in sensor-native units
* `FtcSensors.normalizedRgba(...)` → normalized `Source<NormalizedRgba>` values (usually `0..1`)

For simple close-range classification, **normalized colors are usually the better starting point**
because they behave more consistently across lighting changes and sensor gain.

### Raw channels

```java
import com.qualcomm.robotcore.hardware.ColorSensor;
import edu.ftcphoenix.fw.core.color.Rgba;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.ftc.FtcSensors;

ColorSensor loaderColor = hardwareMap.get(ColorSensor.class, "loaderColor");

// Memoized per loop by default.
Source<Rgba> rgba = FtcSensors.rgba(loaderColor);

BooleanSource looksGreen = rgba
        .mapToBoolean(c -> c.gRatio() > 0.45 && c.rRatio() < 0.35)
        .debouncedOnOff(0.05, 0.05);
```

`Rgba` intentionally does not assume a specific numeric range for channel values. Many sensors
report 0–255, but others report larger values. Phoenix encourages **ratio-based** logic (e.g.
`gRatio()`), which tends to transfer better between sensors.

### Normalized channels (recommended for classification)

```java
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import edu.ftcphoenix.fw.core.color.NormalizedRgba;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.ftc.FtcSensors;

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
  only adapts the reading into Phoenix sources.
* The framework hardware menu now includes `HW: Color Sensor (Normalized)`, which prints
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


---

## Where classification belongs

Phoenix separates:

1. **Sensor reads** (FTC SDK boundary) → Sources (`FtcSensors`)
2. **Signal conditioning** (debounce/hysteresis/memo/edges) → Sources (`ScalarSource`, `BooleanSource`)
3. **Robot policy** (what to do with colors / balls) → your robot code

For more advanced cases (specific ball colors, rejecting wrong colors, slot-local memory, etc.),
Phoenix can provide generic helpers (filters, hold-last-valid patterns, reset-driven accumulation,
action queues), but the *meaning* of colors, reset boundaries, and thresholds are typically
robot-specific.

---

## Touch sensors

Touch / limit switches are a natural `BooleanSource`.

```java
import com.qualcomm.robotcore.hardware.TouchSensor;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.ftc.FtcSensors;

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

Phoenix provides two explicit adapters:

* `FtcSensors.digitalHigh(...)` – true when the pin is **HIGH**
* `FtcSensors.digitalLow(...)` – true when the pin is **LOW** (common for active-low sensors)

```java
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.ftc.FtcSensors;

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
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.ftc.FtcSensors;

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
