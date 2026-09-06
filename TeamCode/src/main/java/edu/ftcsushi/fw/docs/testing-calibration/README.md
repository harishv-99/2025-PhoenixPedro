---
tags:
  - Test & Tune
---

# Test and tune one fact at a time

Start without hardware, then cross to one controlled robot question. A successful build or software
test can prove a request, heartbeat, or recorded command; it cannot prove wiring, motion, clearance,
safe travel, calibration, or tuning.

**Before running software checks:** complete [Build and Run](<../getting-started/Build and Run.md>) so
the repository builds from its root. Reading the experiment and its expected results needs no setup.
Before opening a hardware tester, deploy the app and make the FTC Robot
Configuration containing the intended device names active; an unconfigured device cannot appear in
the picker.

## Start here

1. Read the maintained starter-mechanism experiment from
   [Hardware-free Reference Scenarios](<../examples/Hardware-free Reference Scenarios.md>). Read its
   request → ordinary heartbeat → recorded command chain and name the physical fact it cannot prove.
   Optionally run its software check after setup.
2. Before any on-robot tool, learn the two ready OpModes, connection order, menu controls, and stop
   boundary in [Using the tester console](<Using the Tester Console.md>).
3. Choose one physical question below. Do not combine bring-up, calibration, and tuning into one
   first run.

## Choose the next question

| Current question | Use | Stop when you know |
| --- | --- | --- |
| Which direction and backed-off range are safe candidates? | [Actuator bring-up](<Actuator Bring-up.md>) | recorded robot evidence supports profile values to verify |
| Which camera, odometry, or localization fact is missing? | [Robot calibration](<Robot Calibration Tutorials.md>) | one fact is recorded in the robot profile and rechecked by its configured owner |
| Do controller gains meet a written criterion? | [Control tuning](<Control Tuning Workflow.md>) | one bounded experiment meets the criterion |
| How do I design another focused test? | [Testing philosophy](<How to test a Sushi component.md>) | its owner, replacement, observation, limit, and next gate are explicit |
| How should a team-specific procedure be assembled? | [Guided calibration](<Guided Calibration Walkthroughs.md>) | each stage exposes one fact and handoff |
| Something is already failing | [Common problems](<../troubleshooting/Common Problems.md>) | the observed symptom has one evidence-backed cause |

Before hardware motion, keep unrelated motion disabled, secure the mechanism, use conservative
commands, and assign one person to FTC Driver Station STOP. Record each result in robot-owned
configuration, rebuild, and verify it through the configured mechanism or subsystem; a generic
tester does not save or adopt that result for you.
