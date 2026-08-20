# BIOBUZZ capability map

This helper is intentionally provisional. It maps plausible robot needs to stable framework
lessons; it does not assert unreleased field geometry, scoring rules, strategy, inventory, or a
physical mechanism design.

| Possible BIOBUZZ need | Start with | Confidence |
|---|---|---|
| Driver-controlled collection or transfer | [Modern Starter Robot](<Modern Starter Robot.md>) direct-power mechanism | Plausible; final hardware and game-piece interaction are team evidence. |
| Raise or place something at named levels | `ReferenceLift` and `ReferenceLiftMechanism` | Plausible; travel, reference method, and safe heights are robot-owned facts. |
| Accelerate and release an object | `ReferenceLauncher` and `ReferenceLauncherMechanism` | Plausible; required speed, timing, and whether a flywheel is appropriate are unknown. |
| Repeat a multi-step action safely | `launchOne()` and [Tasks & Macros Quickstart](<../design/Tasks & Macros Quickstart.md>) | High for any timed behavior: use fresh, non-blocking, outcome-aware Tasks. |
| Maintain alignment while the driver translates | [Drive Guidance](<../drive-vision/Drive Guidance.md>) | Plausible; the relevant observation and target remain season policy. |
| Drive relative to the field | [Field-relative Drive](<Field-relative Drive.md>) | Optional driver preference; alliance may select station “up” before START. |
| Detect a known field target | [Drive Guidance](<../drive-vision/Drive Guidance.md>) | Plausible; keep tag layout separate from camera ownership and robot policy. |
| Enter a known-clear parking area | [Drive Guidance](<../drive-vision/Drive Guidance.md>) | Plausible; only a reviewed full-box target permits automatic guidance. |
| Follow an autonomous route | [Pedro Autonomous Reference](<Pedro Autonomous Reference.md>) | Optional integration after route geometry and completion criteria are known. |
| Evaluate a mechanism before competition code depends on it | [Subsystem Experiments](<Subsystem Experiments.md>) | High; the team supplies the physical question and success criteria. |

When the official game and the robot design are known, create BIOBUZZ-owned capabilities that use
these lessons. Do not copy this table into mechanism code or promote speculation into framework API.

[Back to the examples index](<README.md>)
