# Pedro integration

This folder holds the optional Phoenix framework bridge for Pedro Pathing.

- Keep `fw` core packages free of `com.pedropathing.*` imports.
- Keep reusable Pedro bridge code here, such as `PedroPathingDriveAdapter`.
- Construct the Pedro `Follower` on the robot side (typically `Constants.createFollower(hardwareMap)`) so team-specific setup stays out of the framework.
- Keep robot-specific Pedro autos on the robot side (`phoenix/.../autonomous/pedro/`).
- Later, this folder can become its own source set or module with minimal code motion.


## Related docs

- [`../../docs/README.md`](<../../docs/README.md>)
- [`../../docs/design/Recommended Robot Design.md`](<../../docs/design/Recommended Robot Design.md>)
- [`../../docs/drive-vision/Drive Guidance.md`](<../../docs/drive-vision/Drive Guidance.md>)
- [`../../Framework Principles.md`](<../../Framework Principles.md>)
