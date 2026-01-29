# FTC Sensors

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

Color sensors are exposed as a `Source<Rgba>` (a small framework-owned value object).

```java
import com.qualcomm.robotcore.hardware.ColorSensor;
import edu.ftcphoenix.fw.core.color.Rgba;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.ftc.FtcSensors;

ColorSensor loaderColor = hardwareMap.get(ColorSensor.class, "loaderColor");

// Memoized per loop by default.
Source<Rgba> rgba = FtcSensors.rgba(loaderColor);

// Example: compute a simple "green-ish" boolean gate.
// (Real classification is usually done with tuned ratios / thresholds.)
BooleanSource looksGreen = rgba
        .mapToBoolean(c -> c.gRatio() > 0.45 && c.rRatio() < 0.35)
        .debouncedOnOff(0.05, 0.05);
```

`Rgba` intentionally does not assume a specific numeric range for channel values. Many sensors
report 0–255, but others report larger values. Phoenix encourages **ratio-based** logic (e.g.
`gRatio()`), which tends to transfer better between sensors.

---

## Where classification belongs

Phoenix separates:

1. **Sensor reads** (FTC SDK boundary) → Sources (`FtcSensors`)
2. **Signal conditioning** (debounce/hysteresis/memo/edges) → Sources (`ScalarSource`, `BooleanSource`)
3. **Robot policy** (what to do with colors / balls) → your robot code

For more advanced cases (specific ball colors, rejecting wrong colors, etc.), Phoenix can provide
generic helpers (filters, hold-last-valid patterns, action queues), but the *meaning* of colors
and the thresholds are typically robot-specific.
