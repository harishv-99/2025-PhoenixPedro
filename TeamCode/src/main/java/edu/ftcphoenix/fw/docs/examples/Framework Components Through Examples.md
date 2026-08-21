# Framework components through examples

This is a season-spanning lookup, not a second learning path. Start with
[Phoenix in one picture](<../getting-started/Framework Overview.md>) for the complete Starter
execution story, then [choose a Phoenix topic](<../getting-started/Beginner's Guide.md>) when one
robot problem needs a deeper explanation. Return here for its canonical example or guide.

## Package structure to copy

Every managed example uses the same scalable role vocabulary, while omitting roles it does not
need:

```text
example/
  robot/                 profile, composition root, and package-private controls
  opmode/                thin FTC entries only
  capability/<family>/   mode-neutral intent/status and its hardware realization
  autonomous/            routes, routines, outcome policy, and autonomous spatial policy
  tester/                robot-specific experiment criteria and testers
```

Package placement does not merge ownership: controls still own operator meanings, capabilities
still own mode-neutral intent, mechanisms still own Plants, and the composition root only wires
them. A small robot simply leaves out unused packages.

## The progression

| Concept | Canonical example | Student takeaway |
|---|---|---|
| Managed host, profile, root, output, presenter | `starter/opmode/StarterTeleOp`, `starter/robot/StarterProfile`, `StarterRobot` | The OpMode chooses a profile and declares roles; owners do the work. |
| Sources and callback bindings | `StarterTeleOpControls` | Map gamepad meanings to semantic capability calls. |
| Robot-centric mecanum | `StarterTeleOpControls.driveSource()` | A `DriveSource` describes intent; one sink realizes it. |
| Direct-power Plant and timed macro | `StarterIntakeMechanism` and `StarterAuto` | A mechanism owns its Plant; repeated behavior returns a fresh Task. |
| Hardware-free mechanism evidence | [Test a mechanism without hardware](<../getting-started/Test a Mechanism Without Hardware.md>) and [Reference scenarios](<Hardware-free Reference Scenarios.md>) | Construct the unchanged production mechanism, inject device inputs explicitly, and record outputs without claiming physics. |
| Field-relative composition | `fieldrelative/opmode/FieldRelativeDriveExample` | Define station “up” explicitly and rotate intent before the normal mecanum sink. |
| Referenced position, feedback moves, and homing | `ReferenceLift`, `ReferenceLiftMechanism` | Use a persistent request for immediate intent and a fresh feedback-aware Task when later work must wait for arrival. |
| Paired velocity wheels and readiness | `reference/capability/launcher/ReferenceLauncherMechanism` | One Plant owns paired commands while each wheel supplies readiness evidence; controller readiness is not proof that a game piece scored. |
| Temporary output override | `ReferenceLauncherMechanism` transfer queue | Overlay a short feed pulse on a persistent zero target without adding a competing writer. |
| Outcome-aware Tasks | `ReferenceLauncherMechanism.launchOne()` | Branch after spin-up so timeout cannot accidentally feed. |
| Shared capability vocabulary | `ReferenceLift`, `ReferenceLauncher`, `ReferenceCapabilities` | TeleOp and Auto call the same mode-neutral robot meanings; an aggregate is useful only for a real multi-family client. |
| Sensors and status | reference lift/launcher status | Mechanisms condition sensors and publish cached evidence; presenters only format it. |
| Haptics | `HapticSink` Javadocs and `FtcHaptics.gamepad(...)` | Bind a semantic edge to a short cue and stop the sink during total cleanup. |
| AprilTag observation and offsets | [Drive Guidance](<../drive-vision/Drive Guidance.md>) | Keep camera ownership, field facts, tag selection, and aim policy distinct. |
| Localization and field geometry | [AprilTag Localization & Fixed Layouts](<../drive-vision/AprilTag Localization & Fixed Layouts.md>) | Pose evidence belongs upstream of guidance; frames and units stay explicit. |
| Guided parking and conservative footprint | [Drive Guidance](<../drive-vision/Drive Guidance.md>) | A reviewed target pose may guide into a known-clear full box; corner-inside is literal status only. |
| Pedro route Tasks | `BasicPedroAutoPaths`, `BasicPedroAutoRoutine` | The example builds fixed geometry eagerly; use the named start-time factory only for genuinely live geometry, and preserve truthful route outcomes. |
| Controller tuning and calibration | [Control Tuning Workflow](<../testing-calibration/Control Tuning Workflow.md>) | Use supported tools to establish controller evidence before robot behavior depends on it. |

The Reference robot does not wire field-relative drive, Prestart, services, parking guidance,
Pedro, AprilTags, localization, haptics, or supervisors. Rows for those concepts deliberately point
to focused examples, guides, Javadocs, or production references instead of pretending one example
owns every capability.

## What stays advanced

Custom Task state machines, regulators, vendor adapters, vision processors, dynamic frames,
contextual/endgame controls, periodic-equivalent planning, Auto-to-TeleOp handoff, and raw host
internals are reference topics. Introduce one only when the robot has a concrete need that the
ordinary factories and owners cannot express.

## Flywheel checklist

The launcher example deliberately covers the concepts needed by a typical flywheel mechanism:
paired velocity actuation, a bounded target, measured velocity, tolerance-based readiness, spin-up
timeout, outcome-aware feeding, a temporary transfer pulse, positional release, object sensing,
status presentation, reusable active idle, and terminal stop. Range-to-speed interpolation and aiming
are separate policy lessons; add them upstream without moving Plant ownership out of the launcher.

[Back to the examples index](<README.md>)
