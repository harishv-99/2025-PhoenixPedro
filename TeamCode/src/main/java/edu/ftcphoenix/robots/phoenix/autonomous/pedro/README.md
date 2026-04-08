# Phoenix Pedro autonomous examples

This folder keeps Pedro-specific robot code out of Phoenix core.

- `PhoenixPedroAutoTestOpMode` is the integration test auto.
- The OpMode intentionally calls `Constants.createFollower(hardwareMap)` directly. If your team keeps the Pedro constants class somewhere else, change that import or the local `createPedroFollower(...)` method.
- Later, this folder can move into a Pedro-specific source set or module.
