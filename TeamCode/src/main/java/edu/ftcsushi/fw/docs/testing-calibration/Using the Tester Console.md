---
tags:
  - Test & Tune
---

# Using the tester console

**Learning mode:** Operational runbook

**Outcome:** open the ready Sushi tester home, use exactly one control source, and know how to stop
or recover before any hardware experiment. No Java source lookup or custom tester OpMode is needed.

**Before this page:** run the first
[hardware-free software experiment](<../examples/Hardware-free Reference Scenarios.md>). Before
pressing START for a hardware tester, also read the runbook for that one experiment.

## Choose one input owner

The Driver Station lists two ready OpModes. Their input ownership is fixed and mutually exclusive
for the whole run:

| Select on the FTC Driver Station | The tester accepts | Telemetry appears on |
| --- | --- | --- |
| **FW: Testers (Driver Station)** | physical FTC gamepads only; gamepad 1 navigates ordinary tester menus | Driver Station and Panels |
| **FW: Testers (Panels)** | Panels virtual gamepads only; virtual gamepad 1 navigates ordinary tester menus | Driver Station and Panels |

The inactive control source is ignored; inputs are never merged and cannot switch owners while the
OpMode is running. Mirrored telemetry does not give Panels control of the Driver Station-owned
entry, or the physical gamepads control of the Panels-owned entry.

## Connect Panels before INIT

Skip this connection step when using only the Driver Station-owned entry. For Panels input:

1. Connect the browser device to the robot network.
2. Open the address for the Robot Controller:
   - phone Robot Controller: `http://192.168.49.1:8001`
   - Control Hub: `http://192.168.43.1:8001`
3. In Panels, open **Telemetry** and the default **Combined Gamepad** widget. Use Combined Gamepad 1
   for the controls on this page.
4. Confirm that a Panels client is connected **before pressing INIT** on the Driver Station.

Use exactly one Panels client for a student run. The ready OpMode technically accepts one or more,
but all connected clients share one Panels input state; Sushi cannot identify that the intended
operator disconnected while another client remains.

The [official current Panels access instructions](<https://panels.bylazar.com/docs/com.bylazar.docs/Accessing%20Panels/>)
cover Panels installation and network access. The Sushi-specific OpMode choice, controls, menu,
failure response, and stop boundary are all described on this page.

## Navigate during INIT

Select the chosen OpMode on the FTC Driver Station and press INIT. The exact standard home is
**Framework Tester Home**, in this order:

1. **HW: Actuator Bring-up**
2. **Framework: Calibration & Localization**
3. **Advanced: Hardware Diagnostics**

Gamepad 1 uses **Dpad Up/Down** to highlight, **A** to enter, and **BACK** to go back. A child may
handle BACK inside its own screens; otherwise BACK stops that child and returns to its parent menu.
Each child screen owns its other controls, so follow that screen and its runbook.

You may enter menus and choose a device during INIT. Hardware motion remains locked until the FTC
Driver Station START transition; press Driver Station START only after the mechanism is secured,
the controls are neutral, and the selected experiment's stop plan is ready.

Panels uses PlayStation-style face labels. Sushi translates them to the gamepad names printed by
tester screens:

| Panels virtual control | Tester gamepad name |
| --- | --- |
| Cross | A |
| Circle | B |
| Square | X |
| Triangle | Y |
| Options | START |
| Share | BACK |

Options is the tester's virtual gamepad START button; it does not perform the FTC OpMode START
transition. Use the physical Driver Station for INIT, START, and STOP.

## Stop and reconnect safely

!!! danger "Danger: FTC Driver Station STOP is the emergency stop"

    Keep one person ready at FTC Driver Station STOP and with access to robot power. A browser
    control, virtual-gamepad button, BACK, or closing the browser is not an emergency stop.

If the last Panels client disconnects, the client count becomes invalid, or input sampling fails,
the tester terminally fail-stops. Reconnecting the browser does not rearm that OpMode. Press FTC
Driver Station STOP, reconnect Panels, then select and INIT a fresh **FW: Testers (Panels)** OpMode
before starting again.

If another client remains connected, the ready host cannot detect that the intended operator left;
stale-input neutralization then belongs to Panels transport. This is another reason to use one
client, and never a reason to treat browser state as an emergency-stop guarantee.

## Checkpoint and next choice

Before motion, you should be able to see **Framework Tester Home** in both telemetry views and move
its highlight from only the input source named by the selected OpMode. That proves the console path
and input owner are correct; it proves nothing about wiring, direction, clearance, or safe travel.

Continue with one question:

- establish direction or backed-off endpoints with [Actuator bring-up](<Actuator Bring-up.md>);
- establish one camera, odometry, or localization fact with
  [Robot calibration](<Robot Calibration Tutorials.md>); or
- after bring-up, evaluate one controller with the
  [Control tuning workflow](<Control Tuning Workflow.md>).
