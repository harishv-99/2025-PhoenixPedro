# Subsystem experiments

An experiment is a robot-specific use of existing capabilities and tester infrastructure, not a
new framework abstraction. Its purpose is to answer one decision before ordinary robot code relies
on the answer.

## Lab card

Write this card before enabling motion:

1. **Question:** one falsifiable subsystem question.
2. **Configuration:** reviewed hardware names, directions, bounds, limits, and safe STOP behavior.
3. **Trial:** exact starting condition, command, duration or terminal condition, repetitions, and
   emergency-stop owner.
4. **Computed evidence:** values the code must calculate, such as target, measurement, error,
   elapsed time, timeout, and Task outcome.
5. **Observed evidence:** facts a person must see, such as whether an object entered a basket. The
   operator records these in the experiment spreadsheet.
6. **Success criteria:** numeric or observable thresholds supplied and reviewed by the adopting
   team. Checked-in examples do not invent them.
7. **Decision:** accept, revise, or reject the configuration, with the evidence location.

## Telemetry rule

Driver Station telemetry prints only information known by the program: trial identity, command,
measurement, timing, error, and outcome. It must not print `PASS` merely because a controller was
at target when the real question was whether a physical scoring action succeeded. Existing Panels
tuning tools may publish the controller evidence they explicitly support. Do not persist trial
results on the Robot Controller.

## Suggested subsystem trials

| Subsystem | Computed telemetry | Operator record |
|---|---|---|
| Drivetrain | command, pose/distance if measured, elapsed time, final error | tracking, obstruction, wheel slip, stop behavior |
| Intake/transfer | requested mode, sensor edges, cycle time, timeout | acquisition/release outcome and jams |
| Referenced lift | reference state, requested/measured position, error, move time, timeout | clearance, repeatability marks, unwanted contact |
| Flywheel/launcher | target/measured velocity, spin-up time, droop, recovery time, Task outcome | launch result, consistency, game-piece damage |
| Vision/localization | observation age, selected ID, residual/innovation, accepted/rejected status | lighting, occlusion, field setup notes |
| Parking/guidance | frozen target pose, pose evidence age, final pose error, literal footprint status | full containment and shared-area safety observations |

Use `StandardTesters` for actuator bring-up and the existing calibration/localization tools. A robot
may add a `TesterSuite` entry that constructs its production subsystem and prints the card's
computed fields. Keep every motion entry locked until the team has supplied and reviewed that
card's physical criteria.

[Back to the examples index](<README.md>)
