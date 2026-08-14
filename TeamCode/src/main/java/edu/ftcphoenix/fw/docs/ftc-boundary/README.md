# FTC boundary reference

Phoenix keeps FTC SDK hardware and UI details at this explicit boundary. Ordinary robot mechanisms
usually enter through `FtcActuators`, while the rest of their policy stays in robot-owned classes.

FTC gamepad input enters through `edu.ftcphoenix.fw.ftc.input`. Construct `GamepadDevice` or the
two-controller `Gamepads` aggregate with the same direct calls used in robot code:

```java
import edu.ftcphoenix.fw.ftc.input.GamepadDevice;
import edu.ftcphoenix.fw.ftc.input.Gamepads;

GamepadDevice driver = new GamepadDevice(gamepad1);
Gamepads gamepads = Gamepads.create(gamepad1, gamepad2);
```

These adapters expose Phoenix `ScalarSource` and `BooleanSource` values; core input, binding, and
drive code never imports the raw SDK `Gamepad` type.

- [`FTC Actuators & Plants`](<FTC Actuators & Plants.md>) — staged construction for motors, servos,
  CR servos, grouped outputs, units, references, feedback, control strategy, and the narrow advanced
  FTC velocity-PIDF configuration handle used by the framework tuning workflow.
- [`FTC Sensors`](<FTC Sensors.md>) — battery, encoder, distance, color, touch, digital, and analog
  sources.
- [`FTC UI Helpers`](<FTC UI Helpers.md>) — selection menus, navigation, summaries, and hardware
  name pickers.
- [`FTC Auto-to-TeleOp handoff`](<FTC Auto-to-TeleOp Handoff.md>) — advanced process-local,
  cleanup-gated state transfer.

For the ordinary Plant shape, start with [`Your first mechanism`](<../getting-started/First Mechanism.md>)
or the [`Phoenix Cheat Sheet`](<../reference/Phoenix Cheat Sheet.md>).

[Back to the Phoenix docs home](<../README.md>)
