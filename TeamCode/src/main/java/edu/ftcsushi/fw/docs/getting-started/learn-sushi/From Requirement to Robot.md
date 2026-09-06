---
tags:
  - Learn
---

# From requirement to robot

**Question:** A robot requirement changed; which owner should change?

This is an on-demand design reference. Use the [Build course](<../../build/README.md>) for complete
worked slices. Reading the following decisions requires no installation, code edit, test run, or
robot hardware.

## Start with the behavior the team needs

Use [Robot roles](<Robot Roles.md>) for the ownership vocabulary and
[Plants and hardware](<Plants and Hardware.md#plant>) for request, applied target, and measurement.
These short references are knowledge prerequisites, not hardware assignments.

“TeleOp and Auto must collect, eject, and stop the intake” names a shared capability.
The [intake lesson](<../../build/Continuous Intake.md>) follows those names all the way to a private
Plant. Button choices belong to controls, configured motor facts belong to data, and the mechanism
realizes the same capability for both modes.

| Requirement changed | Owner to inspect | Question to preserve |
| --- | --- | --- |
| B should collect instead of eject | controls | Does one accepted press request the intended meaning? |
| Collection needs a different reviewed power | mechanism configuration | Is the new finite command within the permitted range? |
| Show whether the switch is pressed | observation owner and presenter | Is the displayed fact cached, and is its physical meaning stated? |
| Wait for a requested position | mechanism's feedback Task | Does fresh evidence belong to that exact request? |
| Abort or continue after a timeout | Auto routine or robot policy owner | Which outcome permits the next action? |
| Add a second independent mechanism | composition root | Does each resource still have one owner and one managed update/stop path? |

Observation does not automatically become policy. A cached `pressed` or `objectPresent` field
does not stop an intake or permit feeding by itself. The mechanism or an explicitly named robot
policy owner must use that fact when the requirement calls for that behavior.

## Trace the first mismatch

Follow the facts in the direction they are produced:

```text
input -> capability request -> requested target -> applied target
      -> measurement/readiness -> Task outcome -> Auto decision
```

If the button does not change the request, inspect input meaning and bindings. If the request is
correct but the applied target differs, inspect bounds and guards. If a command is correct but
feedback is absent, inspect the observation boundary before changing Auto sequencing. A displayed
software command alone cannot establish physical movement.

Use the [ownership map](<Robot Roles.md>) when several objects appear responsible for the same
fact. Add a role only when it owns a new responsibility; an extra forwarding class does not make
a design clearer.

## Reuse the smallest relevant example

For ordinary robot authorship, use the [package guidance](<../../build/README.md#author-in-your-robot>)
and adapt only the focused capability your robot needs. Keep your robot's policy and hardware
facts in its own package beside the maintained examples.

When a new requirement needs additional target or coordination policy, choose the corresponding
reference individually:

- [Periodic turret position](<../../advanced/Periodic Turret Position.md>) explains an angle with
  several legal full-turn representatives.
- [Paired flywheel velocity](<../../advanced/Paired Flywheel Velocity.md>) separates a shared
  command from two independent readiness measurements.
- [Robot capabilities and mode clients](<../../design/Robot Capabilities & Mode Clients.md>) explains
  larger capability families shared by TeleOp and Auto.
- [Supervisors and pipelines](<../../design/Supervisors & Pipelines.md>) explains robot-specific
  coordination beyond one mechanism.
- [Subsystem experiments](<../../examples/Subsystem Experiments.md>) supplies the procedure for a
  team-authored physical question.

Those topics extend an understood requirement. They do not form another mandatory learning path.
