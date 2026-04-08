# Pedro integration

This folder holds the optional Phoenix framework bridge for Pedro Pathing.

- Keep `fw` core packages free of `com.pedropathing.*` imports.
- Keep robot-specific Pedro autos on the robot side (`phoenix/.../autonomous/pedro/`).
- Later, this folder can become its own source set or module with minimal code motion.
