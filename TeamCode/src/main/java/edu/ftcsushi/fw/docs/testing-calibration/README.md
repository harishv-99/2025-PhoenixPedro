---
tags:
  - Test & Tune
---

# Test and tune one fact at a time

Begin with [How to test a Sushi component](<How to test a Sushi component.md>). It explains how to
choose the owner under test, preserve the production heartbeat, replace only the outside world, and
avoid claiming evidence the test cannot provide.

## Choose the evidence you need

| Current question | Use | Stop when you know |
|---|---|---|
| Does a button request the intended meaning? | a small semantic-intent test | the callback or fresh Task request is correct |
| Does a real mechanism owner produce the expected command? | a [software-device scenario](<../examples/Hardware-free Reference Scenarios.md>) | the request/evidence/heartbeat decision is correct |
| Does the managed lifecycle call owners in the right order? | a supplied managed-slice test | the tested phase order and cleanup are correct |
| Which direction and backed-off range are safe? | [Actuator bring-up](<Actuator Bring-up.md>) | recorded robot evidence supports the profile values |
| Which robot facts must be established next? | [Robot calibration tutorials](<Robot Calibration Tutorials.md>) | each required fact has an owner and recorded result |
| Do controller gains meet a stated criterion? | [Control tuning workflow](<Control Tuning Workflow.md>) | the bounded experiment meets the criterion |
| How should a guided team procedure be assembled? | [Guided calibration walkthroughs](<Guided Calibration Walkthroughs.md>) | the steps expose one fact at a time |
| Something is already failing | [Common problems](<../troubleshooting/Common Problems.md>) | the observed symptom has one evidence-backed cause |

## The handoff from software to hardware

Software tests may establish mappings, lifecycle, cached status, Task outcomes, and decisions made
from explicitly injected measurements. They do not establish wiring, motor or servo direction,
mechanism clearance, safe travel, switch placement, encoder scale, controller response, route
accuracy, or tuning.

Before running any mechanism OpMode:

1. Keep motion examples `@Disabled` and every motion-permission flag false.
2. Inspect the assembled mechanism and establish an immediate stop plan.
3. Use conservative commands and the device-focused bring-up tool.
4. Record the direction and backed-off endpoints or operating range in the robot profile.
5. Run the mechanism-only TeleOp before connecting drivetrain or multi-mechanism behavior.
6. Re-run the software suite after recording the reviewed configuration.

Each Build recipe links its one next physical gate so a student does not have to infer which
procedure applies.
