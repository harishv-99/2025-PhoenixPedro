# Testing and calibration

Calibrate one physical fact at a time. Keep the robot clear of people, begin with conservative
commands, know how to stop the OpMode, and do not treat a successful build as proof of safe motion.

If hardware is unavailable, stay in software: [Test a mechanism without hardware](<../getting-started/Test a Mechanism Without Hardware.md>)
uses the real Starter mechanism and Plant with a recording test device. The optional
[Reference scenarios](<../examples/Hardware-free Reference Scenarios.md>) add explicit switch,
encoder, and independent flywheel-velocity inputs. These scenarios can establish software behavior
before a robot exists, but they do not model physics or authorize motion.

If you copied the framework into another project, choose exactly one ready-made entry for a
general-purpose tester run:

| Entry | Sole input owner |
|---|---|
| **FW: Testers (Driver Station)** | Physical Driver Station gamepads |
| **FW: Testers (Panels)** | Panels virtual gamepads |

Both entries open the same tester suite, use the same D-pad/button/bumper controls, and mirror the
same row-oriented telemetry to both Driver Station and Panels. The entry name chooses who may
command the tester: inputs are never merged or switched during a run. Stop the current OpMode and
start the other entry to change input owners.

For a supervised bench session without a Driver Hub or other Driver Station device, use Panels'
OpMode controls to select, initialize, and start **FW: Testers (Panels)**, then use its virtual
gamepads for the unchanged tester controls.

1. Connect the laptop or tablet to the Robot Controller's Wi-Fi network and open Panels.
2. Open **OpModes Control**, choose **FW: Testers (Panels)**, and initialize it.
3. Keep **Telemetry** visible for the same menu, warnings, and evidence shown on Driver Station.
4. Use the default **Combined Gamepad** widget: gamepad 1 owns the ordinary tester controls, and
   gamepad 2 is used only when a specific tester documents it. This widget contains the two Panels
   virtual gamepads; it does not merge physical and browser input.
5. Return both virtual gamepads to neutral before starting the OpMode or arming an actuator.

The Panels entry requires a connected Panels client. No connected client, loss of the last client,
or an input-sampling failure terminally fail-stops that tester run and best-effort stops its active
tester. Reconnecting does not rearm the same OpMode instance; stop it, reconnect, and begin a fresh
INIT/start. Browser STOP is not a physical emergency stop, so powered testing still requires clear
access to robot power and a person ready to remove it.

Sushi can observe only the total Panels client count exposed by the pinned integration. If one
Panels view closes while another client remains connected, the host cannot identify that view as
the input owner; it then relies on Panels aging unattended virtual controls back to neutral. Treat
that as transport behavior to validate on the robot, never as an emergency-stop guarantee. A
stalled OpMode loop cannot apply a new neutral command.

This repository's production Phoenix robot has its own ordered
[`Phoenix calibration guide`](<../../../robots/phoenix/Phoenix Calibration Guide.md>).

## When hardware is available, choose one question

Hardware work is not required by either software lesson. When a supervised robot is available,
start with the one physical question that must be answered rather than running every tool:

| Question | Start here |
|---|---|
| Does this named actuator move in the intended direction and stop when commanded? | [Actuator bring-up](<Actuator Bring-up.md>) |
| Are the mechanism reference, encoder scale, drivetrain response, or sensor axes correct? | [Robot Calibration Tutorials](<Robot Calibration Tutorials.md>) |
| Which reviewed controller settings meet the mechanism's measured response goal? | [Control Tuning Workflow](<Control Tuning Workflow.md>) |
| Does the whole subsystem meet a team-authored success criterion over repeated trials? | [Subsystem Experiments](<../examples/Subsystem Experiments.md>) |

Stop after collecting the evidence needed for that question. A later question begins a separate
reviewed test; one successful result does not validate unrelated hardware or behavior.

## Student runbooks

1. [`Actuator bring-up`](<Actuator Bring-up.md>) — the one ordinary device-first workflow for
   motor/servo direction and optional safe endpoint evidence.
2. [`Robot Calibration Tutorials`](<Robot Calibration Tutorials.md>) — mechanism references,
   drivetrain integration, encoders, camera mount, AprilTags, Pinpoint, and corrected localization.
3. [`AprilTag Practice Setup`](<../drive-vision/AprilTag Practice Setup.md>) — a known small test
   area when a complete field is unavailable.
4. [`Control Tuning Workflow`](<Control Tuning Workflow.md>) — ready-made velocity and position
   experiments for supported FTC device-managed and Sushi standard controllers.

Robot code calls `FtcPanelsTuners.velocityControl(...)` or `positionControl(...)` and supplies a
factory for one fresh Plant built from its production owner's canonical recipe. The completed
Plant selects the supported controller topology. The framework owns draft capture, the controller
session, segments, metrics, history, restoration, and cleanup. This is not a generic raw-actuator
editor: the Plant remains the sole actuation path and robot code declares the finite physical
experiment envelope.

The production Phoenix robot provides the concrete **Phoenix: Tuning (Panels)** entry. It opens the
flywheel workflow directly with no tester menu. Powered tuning requires exactly one Panels client
rather than the general tester entry's at-least-one rule. Panels **Update All** publishes only a
draft; A captures and applies one complete candidate on the OpMode loop. Read the
[`control tuning runbook`](<Control Tuning Workflow.md>)
before enabling it.

## Mentor and tester-author reference

- [`Guided Calibration Walkthroughs`](<Guided Calibration Walkthroughs.md>) explains how to create
  ordered, checkpoint-based tester menus.
- [`FTC Sensors`](<../ftc-boundary/FTC Sensors.md>) documents the boundary sources used by testers.

For an immediate symptom, start with [`Common Problems`](<../troubleshooting/Common Problems.md>).

[Back to the Sushi docs home](<../README.md>)
