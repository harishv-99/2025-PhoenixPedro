# Testing and calibration

Calibrate one physical fact at a time. Keep the robot clear of people, begin with conservative
commands, know how to stop the OpMode, and do not treat a successful build as proof of safe motion.

If you copied the framework into another project, the ready-made Driver Station OpMode `FW:
Testers` opens the framework tester tree. This repository's production Phoenix robot has its own
ordered [`Phoenix calibration guide`](<../../../robots/phoenix/Phoenix Calibration Guide.md>).

## Student runbooks

1. [`Actuator bring-up`](<Actuator Bring-up.md>) — the one ordinary device-first workflow for
   motor/servo direction and optional safe endpoint evidence.
2. [`Robot Calibration Tutorials`](<Robot Calibration Tutorials.md>) — mechanism references,
   drivetrain integration, encoders, camera mount, AprilTags, Pinpoint, and corrected localization.
3. [`AprilTag Practice Setup`](<../drive-vision/AprilTag Practice Setup.md>) — a known small test
   area when a complete field is unavailable.
4. [`Software PIDF Tuning Workflow`](<Software PIDF Tuning Workflow.md>) — a separate
   mechanism-owned safe-candidate, apply, observe, record, and restart workflow.

## Mentor and tester-author reference

- [`Guided Calibration Walkthroughs`](<Guided Calibration Walkthroughs.md>) explains how to create
  ordered, checkpoint-based tester menus.
- [`FTC Sensors`](<../ftc-boundary/FTC Sensors.md>) documents the boundary sources used by testers.

For an immediate symptom, start with [`Common Problems`](<../troubleshooting/Common Problems.md>).

[Back to the Phoenix docs home](<../README.md>)
