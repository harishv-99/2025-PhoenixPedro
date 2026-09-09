---
tags:
  - Test & Tune
---

# Test and tune one fact at a time

Start without hardware, then test one controlled robot fact. A passing software test proves code
behavior, not wiring, physical motion, clearance, calibration or tuning.

**Before software checks:** complete [Build and Run](<../getting-started/Build and Run.md>).
Reading needs no setup. Before hardware testing, deploy the app and activate the FTC Robot
Configuration containing the intended devices; unconfigured devices cannot appear in the picker.

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
| How do I keep a completed result on my laptop? | [Download experiment results](<Download and Inspect Experiment Results.md>) | the file is saved and joined to the existing lab card |
| How do I design another focused test? | [Testing philosophy](<How to test a Sushi component.md>) | its owner, replacement, observation, limit, and next gate are explicit |
| How do I connect testers to my robot's settings? | [Add calibration testers to your robot](<Add Calibration Testers to Your Robot.md>) | fresh testers receive the same checked-in facts as the robot; cameras and powered checks stay optional |
| How should a team-specific procedure be assembled? | [Guided calibration](<Guided Calibration Walkthroughs.md>) | each stage exposes one fact and handoff |
| Something is already failing | [Common problems](<../troubleshooting/Common Problems.md>) | the observed symptom has one evidence-backed cause |

Before motion, secure the mechanism, disable unrelated motion, use conservative commands, and
assign a person to FTC Driver Station STOP. A generic tester does not save configuration or adopt
results: review candidate values, rebuild, and verify them through the configured mechanism.
