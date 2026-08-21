# Role paths

The common [Learn Phoenix](<../Beginner's Guide.md>) path gives the team one shared vocabulary. Use
this page afterward to follow only the code and guides relevant to your current job. These are
reading paths, not assignments to run the example robot.

## Controls and drive

Read in this order:

1. [Controls and intent](<Controls and Intent.md>) for semantic sources, callbacks, Task suppliers,
   and the final robot-centric `DriveSource`.
2. [`ReferenceTeleOpControls.java`](<../../../../robots/examples/reference/robot/ReferenceTeleOpControls.java>)
   to see operator meanings kept out of mechanisms.
3. [Sources and Signals](<../../core-concepts/Sources and Signals.md>) for edges, debounce,
   hysteresis, and composition.
4. [Framework Lanes and Robot Controls](<../../design/Framework Lanes & Robot Controls.md>) when a
   robot needs contextual or coordinated control policy.
5. [Field-relative Drive](<../../examples/Field-relative Drive.md>) only if drivers want that
   optional input transformation. Its Service and Prestart roles belong to that focused
   composition; field-relative behavior is not built into Phoenix or required by mecanum drive.

Keep button meaning in controls, mode-neutral requests in capabilities, and the one final drive
write in the declared drive sink.

## Mechanisms and Plants

Read in this order:

1. [Plants and hardware](<Plants and Hardware.md>) for the ordinary intent-to-hardware trace.
2. [`StarterIntakeMechanism.java`](<../../../../robots/examples/starter/capability/intake/StarterIntakeMechanism.java>)
   for the smallest direct-power owner.
3. [`ReferenceLiftMechanism.java`](<../../../../robots/examples/reference/capability/lift/ReferenceLiftMechanism.java>)
   for bounded referenced position and homing.
4. [`ReferenceLauncherMechanism.java`](<../../../../robots/examples/reference/capability/launcher/ReferenceLauncherMechanism.java>)
   for paired velocity wheels, readiness, an overlay queue, and multiple Plant update order.
5. [FTC Actuators and Plants](<../../ftc-boundary/FTC Actuators & Plants.md>) and
   [Output Tasks and Queues](<../../design/Output Tasks & Queues.md>) for the detailed construction
   grammar.

Ordinary mechanisms receive `HardwareMap` plus data-only configuration, privately construct their
final resolver/Plant graphs, and own update and stop. Resolver internals and advanced assembly seams
are reference material, not extra objects beginners must expose.

## Tasks and autonomous

Read in this order:

1. [Tasks and autonomous](<Tasks and Autonomous.md>) for single-use Tasks, outcomes, cancellation,
   and the Reference Auto trace.
2. [`ReferenceRobot.java`](<../../../../robots/examples/reference/robot/ReferenceRobot.java>) for
   declaration and its `ReferenceCapabilities` handoff, then
   [`ReferenceAutoRoutines.java`](<../../../../robots/examples/reference/autonomous/ReferenceAutoRoutines.java>)
   for the Auto-owned sequence.
3. [Tasks and Macros](<../../design/Tasks & Macros Quickstart.md>) for sequences, parallel groups,
   timeouts, feedback moves, and start-time construction.
4. [Pedro Autonomous Reference](<../../examples/Pedro Autonomous Reference.md>) and
   [First Pedro Auto](<../First Pedro Auto.md>) only when the robot needs that third-party follower.

`ReferenceParkingPlan` is a separate, controller-free geometry-policy helper. It validates parking
facts but is not called by `ReferenceAuto`; use the [Drive Guidance](<../../drive-vision/Drive Guidance.md>)
guide before building a real guided parking Task.

## Vision, localization, and guidance

Read in this order:

1. [Drive and Vision](<../../drive-vision/README.md>) for the topic map.
2. [AprilTag Localization and Fixed Layouts](<../../drive-vision/AprilTag Localization & Fixed Layouts.md>)
   for camera observation, fixed field facts, and pose evidence.
3. [Spatial Queries](<../../drive-vision/Spatial Queries.md>) for frame-explicit predicates.
4. [Drive Guidance](<../../drive-vision/Drive Guidance.md>) for robot-owned guidance policy and
   bounded Tasks.

These topics use canonical guides and production references because the Reference robot does not
own a camera, localizer, or guidance service. Do not add those objects merely to make a beginner
robot resemble a complete competition stack.

## Testing, calibration, and experiments

Read in this order:

1. [Evidence and experiments](<Evidence and Experiments.md>) for truthful booleans, cached status,
   and computed versus operator-observed evidence.
2. [Subsystem Experiments](<../../examples/Subsystem Experiments.md>) for experiment design and
   success criteria.
3. [Testing and Calibration](<../../testing-calibration/README.md>) for the supported physical
   workflow.
4. [Actuator Bring-up](<../../testing-calibration/Actuator Bring-up.md>) before supervised motion,
   and [Control Tuning Workflow](<../../testing-calibration/Control Tuning Workflow.md>) when a
   controller needs measured tuning evidence.

[`ReferenceTestersOpMode.java`](<../../../../robots/examples/reference/opmode/ReferenceTestersOpMode.java>)
uses the specialized `FtcTeleOpTesterOpMode` lifecycle. It is not a `RobotProgram` role and must not
share a production mechanism instance. The checked-in flywheel experiment remains locked until an
adopting team authors and reviews its physical criteria.

The concrete experiment is
[`ReferenceFlywheelSpinUpExperiment.java`](<../../../../robots/examples/reference/tester/ReferenceFlywheelSpinUpExperiment.java>).
Its trial action changes only flywheel velocity, correlates retained evidence with a trial number,
and never requests a release or transfer pulse. The production mechanism still realizes its normal
active idle targets.

## Concept-to-source map

| Concept | Learning location | Coverage kind |
|---|---|---|
| Thin managed OpMode, profile, composition root | [Robot roles](<Robot Roles.md>) with Starter and Reference | Common walkthrough |
| Semantic controls and robot-centric drive | [Controls and intent](<Controls and Intent.md>) | Common walkthrough |
| Private Plants and final realization | [Plants and hardware](<Plants and Hardware.md>) | Common walkthrough |
| Fresh Tasks, outcomes, cancellation | [Tasks and autonomous](<Tasks and Autonomous.md>) | Common walkthrough |
| Electrical polarity, status, experiments | [Evidence and experiments](<Evidence and Experiments.md>) | Common walkthrough |
| Team subsystem workflow | [From requirement to robot](<From Requirement to Robot.md>) | Common walkthrough |
| Service, Prestart, field-relative transform | [Field-relative Drive](<../../examples/Field-relative Drive.md>) | Focused example |
| Parking geometry policy | [`ReferenceParkingPlan.java`](<../../../../robots/examples/reference/autonomous/ReferenceParkingPlan.java>) | Focused helper; not Reference Auto behavior |
| Pedro route lifecycle and outcomes | [Pedro Autonomous Reference](<../../examples/Pedro Autonomous Reference.md>) | Focused example and integration guide |
| AprilTags, localization, spatial policy, guidance | [Drive and Vision](<../../drive-vision/README.md>) | Canonical guides and production references |
| Tester lifecycle and locked experiments | [Subsystem Experiments](<../../examples/Subsystem Experiments.md>) | Focused example and tool guide |
| Haptics | [`HapticSink`](<../../../haptic/HapticSink.java>) and [`FtcHaptics`](<../../../ftc/FtcHaptics.java>) Javadocs | Optional API reference |
| Supervisors and cross-capability policy | [Supervisors and Pipelines](<../../design/Supervisors & Pipelines.md>) | Advanced guide/production reference |
| Custom Task state machines and custom regulators | Detailed design guides and Javadocs | Advanced; only for a concrete unmet need |

“Advanced” does not mean more correct. It means the ordinary owners and factories cannot express a
specific, demonstrated requirement. Begin with one obvious path and add a satellite only when the
robot needs its distinct capability.

## BIOBUZZ lookup

The [BIOBUZZ capability map](<../../examples/BIOBUZZ Capability Map.md>) points possible season
needs into these lessons. It is intentionally provisional: it does not establish game rules,
hardware design, physical success criteria, or a new framework API.

**Return to:** [Learn Phoenix](<../Beginner's Guide.md>)
