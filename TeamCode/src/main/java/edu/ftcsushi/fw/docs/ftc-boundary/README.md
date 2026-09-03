---
tags:
  - Advanced
---

# FTC boundary reference

Sushi keeps FTC SDK hardware and UI details at this explicit boundary. Ordinary robot mechanisms
usually enter through `FtcActuators`, while the rest of their policy stays in robot-owned classes.

FTC gamepad input enters through `edu.ftcsushi.fw.ftc.input`. Construct `GamepadDevice` or the
two-controller `Gamepads` aggregate with the same direct calls used in robot code:

```java
import edu.ftcsushi.fw.ftc.input.GamepadDevice;
import edu.ftcsushi.fw.ftc.input.Gamepads;

GamepadDevice driver = new GamepadDevice(gamepad1);
Gamepads gamepads = Gamepads.create(gamepad1, gamepad2);
```

These adapters expose Sushi `ScalarSource` and `BooleanSource` values; core input, binding, and
drive code never imports the raw SDK `Gamepad` type.

- [`FTC Actuators & Plants`](<FTC Actuators & Plants.md>) — staged construction for motors, servos,
  CR servos, grouped outputs, units, references, feedback, control strategy, and the narrow advanced
  FTC velocity-PIDF configuration handle used by the framework tuning workflow.
- [`FTC Sensors`](<FTC Sensors.md>) — battery, motor-current, encoder, distance, color, touch,
  digital, and analog sources.
- [`FTC Manual Bulk Caching`](<FTC Manual Bulk Caching.md>) — advanced opt-in, first-service
  lifecycle ownership for a robot that has deliberately selected module-wide `MANUAL` caching.
- [`FTC UI Helpers`](<FTC UI Helpers.md>) — selection menus, navigation, summaries, and hardware
  name pickers.
- [`FTC Auto-to-TeleOp handoff`](<FTC Auto-to-TeleOp Handoff.md>) — advanced process-local,
  cleanup-gated state transfer.

For the ordinary Plant shape, start with
[`Plants and hardware`](<../getting-started/learn-sushi/Plants and Hardware.md>)
or the [`Sushi Cheat Sheet`](<../reference/Sushi Cheat Sheet.md>).

[Back to the Sushi docs home](<../README.md>)
