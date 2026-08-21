# Hardware-free Reference scenarios

**Audience:** Students whose mechanism needs feedback beyond the Starter intake

**Purpose:** Show how the same test-only hardware registry scales to sensors, encoders, and paired
velocity motors without changing production robot construction.

Complete [Test a mechanism without hardware](<../getting-started/Test a Mechanism Without Hardware.md>)
first. These are optional Reference case studies, not a second beginner path and not code to copy
wholesale.

## The scenario boundary

Each scenario constructs the real production mechanism with its ordinary `HardwareMap + Config`
constructor. `FtcTestHardware` supplies software device probes at that existing FTC boundary:

- a scenario **injects inputs** such as encoder ticks, measured velocity in ticks per second, or an
  electrical HIGH/LOW level;
- the production mechanism and Plants compute normally; and
- the scenario **records outputs** such as motor power, target ticks, commanded velocity, CR-servo
  power, or servo position.

Inputs never change merely because an output was commanded. These direct-setter scenarios contain
no motor, mechanism, battery, response-time, or game-piece physics. Call them software device
scenarios, not simulations.

## Referenced lift: position and active-low input

The
[`ReferenceLiftSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcphoenix/robots/examples/reference/capability/lift/ReferenceLiftSoftwareScenarioTest.java>)
adds one motor and one digital input using the names from `ReferenceLiftMechanism.Config.defaults()`.
It then constructs the unchanged `ReferenceLiftMechanism`.

The motor's injected encoder ticks are measurement input. The bottom switch's injected electrical
level is a separate input: HIGH means the active-low semantic source is false; LOW means it is
true. The scenario must advance enough explicit `ManualLoopClock` cycles for the configured
debounce before the homing Task can establish a reference. Recorded power and target ticks remain
outputs; neither one automatically moves the encoder or presses the switch.

This is useful for proving software questions such as:

- Does LOW, not HIGH, satisfy this chosen bottom-switch polarity?
- Does the homing Task retain success only after conditioned switch evidence?
- Does the position request scale inches into the expected target ticks?

It cannot prove that the switch is actually wired active-low, the lift reaches bottom safely, or
the encoder scale is physically correct.

## Launcher: independent wheel measurements

The
[`ReferenceLauncherSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcphoenix/robots/examples/reference/capability/launcher/ReferenceLauncherSoftwareScenarioTest.java>)
registers both flywheel motors plus the transfer CR servo, release servo, and object input before
constructing the unchanged `ReferenceLauncherMechanism`.

One velocity request is recorded on both motors, but left and right measured velocities are
injected independently. The mechanism publishes readiness only when each finite measurement is
inside its configured tolerance. A command of 1000 ticks per second therefore does not manufacture
two 1000-tick-per-second measurements or claim that either wheel spun.

The compact scenario exposes a one-sided or averaged readiness error. The same pattern can support
separate focused cases for swapped names, a missing wheel, non-finite measurements, or an incorrect
inclusive tolerance. It does not predict spin-up time, voltage sag, load recovery, vibration,
launch result, or safe wheel speed.

## Adapt the pattern to one team subsystem

1. Start from the subsystem's data-only production configuration.
2. Register only the software devices named by that configuration.
3. Construct the same production mechanism used by the robot.
4. Request one semantic action.
5. Inject only the input facts owned by the scenario.
6. Advance one shared `ManualLoopClock` cycle and update each owner once.
7. Assert the smallest status and recorded-output facts needed for the question.

Keep the setup behind a small test fixture when it distracts from the question. Prefer explicit
units such as `currentPositionTicks` and `measuredVelocityTicksPerSec`. If future work introduces a
real dynamics model, label its assumptions and fidelity separately; a list of setter values alone
is still a software device scenario.

A later trace reader could parse an input file and call the same probe setters before each clock
cycle. The current teaching surface deliberately defines no trace format, playback engine,
timeline, physics, or public model interface.

### Proves

The real production mechanism interprets explicitly supplied device observations and realizes its
documented software outputs for the tested cycles.

### Does not prove

The probes do not model physics or establish physical wiring, polarity, calibration, timing under
load, safety, reliability, or game performance.

### Next gate

Use supervised [actuator bring-up](<../testing-calibration/Actuator Bring-up.md>) for the physical
device, then create a team-owned [subsystem experiment](<Subsystem Experiments.md>) with explicit
success criteria for the facts that only the robot can establish.

[Back to the examples index](<README.md>)
