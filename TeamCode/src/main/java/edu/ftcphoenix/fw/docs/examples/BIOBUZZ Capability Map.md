# BIOBUZZ capability map

This helper is intentionally provisional. It maps plausible robot needs to stable framework
lessons; it does not assert unreleased field geometry, scoring rules, strategy, inventory, or a
physical mechanism design.

| Possible BIOBUZZ need | Start with | Confidence |
|---|---|---|
| Driver-controlled collection or transfer | [Controls and intent](<../getting-started/learn-phoenix/Controls and Intent.md>) and the Starter intake | Plausible; final hardware and game-piece interaction are team evidence. |
| Raise or place something at named levels | [Plants and hardware](<../getting-started/learn-phoenix/Plants and Hardware.md>) and the Reference lift | Plausible; travel, reference method, and safe heights are robot-owned facts. |
| Accelerate and release an object | [Plants and hardware](<../getting-started/learn-phoenix/Plants and Hardware.md>) and the Reference launcher | Plausible; required speed, timing, and whether a flywheel is appropriate are unknown. |
| Repeat a multi-step action safely | [Tasks and autonomous](<../getting-started/learn-phoenix/Tasks and Autonomous.md>) and `launchOne()` | High for any timed behavior: use fresh, non-blocking, outcome-aware Tasks. |
| Maintain alignment while the driver translates | [Drive Guidance](<../drive-vision/Drive Guidance.md>) | Plausible; the relevant observation and target remain season policy. |
| Drive relative to the field | [Field-relative Drive](<Field-relative Drive.md>) | Optional driver preference; the named station selects its separately authored “up” before START. |
| Detect a known field target | [Drive Guidance](<../drive-vision/Drive Guidance.md>) | Plausible; keep tag layout separate from camera ownership and robot policy. |
| Enter a known-clear parking area | [Drive Guidance](<../drive-vision/Drive Guidance.md>) | Plausible; only a reviewed full-box target permits automatic guidance. |
| Follow an autonomous route | [Pedro Autonomous Reference](<Pedro Autonomous Reference.md>) | Optional integration after route geometry and completion criteria are known. |
| Exercise proposed intake, lift, or launcher software before matching hardware exists | [Test a mechanism without hardware](<../getting-started/Test a Mechanism Without Hardware.md>) and [Hardware-free Reference scenarios](<Hardware-free Reference Scenarios.md>) | High for software behavior only; direct input injection and recorded commands do not select or validate a physical design. |
| Evaluate a mechanism before competition code depends on it | [Evidence and experiments](<../getting-started/learn-phoenix/Evidence and Experiments.md>) and [Subsystem Experiments](<Subsystem Experiments.md>) | High; the team supplies the physical question and success criteria. |

When the official game and the robot design are known, create BIOBUZZ-owned capabilities that use
these lessons. Do not copy this table into mechanism code or promote speculation into framework API.

[Back to the examples index](<README.md>)
