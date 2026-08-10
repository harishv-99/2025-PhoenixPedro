# FTC boundary reference

Phoenix keeps FTC SDK hardware and UI details at this explicit boundary. Ordinary robot mechanisms
usually enter through `FtcActuators`, while the rest of their policy stays in robot-owned classes.

- [`FTC Actuators & Plants`](<FTC Actuators & Plants.md>) — staged construction for motors, servos,
  CR servos, grouped outputs, units, references, feedback, and control strategy.
- [`FTC Sensors`](<FTC Sensors.md>) — battery, encoder, distance, color, touch, digital, and analog
  sources.
- [`FTC UI Helpers`](<FTC UI Helpers.md>) — selection menus, navigation, summaries, and hardware
  name pickers.
- [`FTC Auto-to-TeleOp handoff`](<FTC Auto-to-TeleOp Handoff.md>) — advanced process-local,
  cleanup-gated state transfer.

For the ordinary Plant shape, start with [`Your first mechanism`](<../getting-started/First Mechanism.md>)
or the [`Phoenix Cheat Sheet`](<../reference/Phoenix Cheat Sheet.md>).

[Back to the Phoenix docs home](<../README.md>)
