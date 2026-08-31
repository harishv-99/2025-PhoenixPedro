# FTC UI Helpers

Sushi's FTC UI helpers are small telemetry-screen building blocks for places where an FTC
telemetry console needs to show choices during `init_loop()` or a tester run.

The important layering rule is:

- **framework UI owns display and navigation mechanics**
- **robot/tester code owns what the choices mean**

That means a menu can show an autonomous strategy, a hardware name, or a calibration step, but the menu does not know how to build a robot, resolve a route, or run a tester. It simply reports the selected value back to the caller.

## Package

Reusable FTC telemetry UI helpers live in:

```java
edu.ftcsushi.fw.ftc.ui
```

This package is FTC-facing because it renders to `Telemetry` and some helpers enumerate `HardwareMap`. It should not be used by FTC-independent framework core classes.

## Tester console ownership

The framework exposes exactly two ready general-purpose tester entries:
**FW: Testers (Driver Station)** and
**FW: Testers (Panels)**. Both use these same menu helpers, tester suite, and controls, and both mirror
their row-oriented telemetry to Driver Station and Panels. The named entry is the sole input owner
for its lifetime: the Driver Station entry reads only physical gamepads, while the Panels entry
reads only Panels virtual gamepads. Stop and start the other entry instead of merging or switching
input sources.

The Panels entry terminally fail-stops when it has no connected client, loses its last client, or
cannot sample input. Reconnection does not rearm that OpMode instance; reconnect and start a fresh
run. Browser STOP is not a physical emergency stop, so powered tests still require immediate access
to robot power. These UI helpers remain console-independent; tester code does not create a second
Panels-specific menu or control grammar.

A dedicated powered-tuning OpMode can reuse this fixed-owner lifecycle, mirrored telemetry, and
virtual-gamepad grammar while opening one framework-created tuner directly. It may tighten the
connection policy to exactly one Panels client because another unattended client would make input
ownership ambiguous; zero or multiple clients then fail that run closed.

Panels Configurables and Graph are presentation/transport facilities, not a second controller.
**Update All** edits only the active synchronized draft map. `FtcPanelsTuners.velocityControl(...)`
and `positionControl(...)` capture one stable complete candidate and apply it deliberately on the
OpMode loop; Graph consumes numeric telemetry from the retained segment. The workflow owns a fresh
Plant from the robot's canonical recipe, while UI callbacks never write Plants or FTC devices. See
the [`control tuning workflow`](<../testing-calibration/Control Tuning Workflow.md>) for the concrete
edit/Update All/A contract, experiment history, and cleanup behavior.

## `SelectionMenu<T>`

`SelectionMenu<T>` is a **single-screen list**. It owns:

- item rows
- selected index
- wrap/non-wrap movement
- scrolling for long lists
- selected-item help text
- compact tags such as `OK`, `TODO`, `WARN`, or `DEFAULT`
- disabled rows with an explanation
- stable item ids so selection can survive row rebuilds

It intentionally does **not** own nested navigation. Use `MenuNavigator` for hierarchy.

Basic example:

```java
SelectionMenu<MyStrategy> menu = new SelectionMenu<MyStrategy>()
        .setTitle("Auto Strategy")
        .setHelp("Dpad: select | A: choose | B: back");

menu.addItem("safe", "Safe Preload", "Low-risk route.", "DEFAULT", true, null, MyStrategy.SAFE);
menu.addItem("cycle", "Partner-Aware Cycle", "Avoids the lane partner owns.", "WARN", true, null, MyStrategy.CYCLE);
menu.addItem("five", "Five-Cycle Test", "Not ready for match use.", "TODO", false, "Practice-only route.", MyStrategy.FIVE_CYCLE);
```

When a menu is rebuilt from changing data, prefer `setItemsPreserveSelectionById(...)` over clearing and re-adding rows. That lets the highlighted item stay put when the same stable id still exists.

## `MenuItem<T>`

`MenuItem<T>` is the immutable row model used by `SelectionMenu`.

The two most important fields are:

- `label`: the human-facing text shown in telemetry
- `id`: the stable identity used to preserve selection across refreshes

For a hardware picker, the hardware name is a natural id. For an autonomous selector, an enum name or explicit strategy id is usually better than the visible label.

## `MenuNavigator`

`MenuNavigator` owns a stack of `MenuScreen`s. It is the right tool when a UI has levels:

```text
Auto Setup > Red > Audience > Strategy
```

The navigator:

- dispatches one shared set of bindings to the current screen
- supports push/pop/home
- supports `setRoot(...)` and `replaceTop(...)` for wizard-style flows
- renders breadcrumb path and nesting level
- avoids stale bindings from screens that are no longer active

`SelectionMenu` remains useful inside the navigator because it implements `MenuScreen`.

Bind navigation once when Up/Down/Select keep the same meaning while the current screen changes.
For example, selecting an alliance can push a start-position picker, whose selection pushes a
strategy picker; the navigator redirects the same continuously sampled controls to the current
screen. A held selection button does not become a fresh press merely because a new picker appears.

Do not create one control context per picker for that flow. Use a `Bindings.ControlContext` when a
whole navigator is optional, or when controls actually change meaning or eligibility—for example,
switching from picker navigation to live hardware controls. UI registration helpers accept either
root `Bindings` or a context through `CallbackBindings`; the root still owns the only
`bindings.update(clock)` call.


## `SelectionMenus`

`SelectionMenus` contains convenience factories for common menu shapes. The first general helper is
`forEnum(...)`, which turns an enum into one `SelectionMenu` row per value. This is especially useful
for pre-start setup screens such as alliance, start position, or strategy.

```java
SelectionMenu<Alliance> alliance = SelectionMenus.forEnum("Alliance", Alliance.class);
```

The helper can also accept a custom display adapter for labels, help text, tags, and disabled rows.
The selected enum value still belongs to robot code; the framework only builds the list.

## `ConfirmationScreen`

`ConfirmationScreen` is a simple summary page for review-before-action flows. It renders labeled rows
and treats standard UI actions consistently:

```text
A: confirm
B/BACK: cancel/back
Y: home/reset
```

Example:

```java
ConfirmationScreen confirm = ConfirmationScreen.builder("Reset Calibration?")
        .row("Sensor", selectedSensorName)
        .warning("This clears the saved reference.")
        .onConfirm(() -> resetSelectedCalibration())
        .build();
```

Use it in a tester or another explicit-action flow before applying a calibration reset or similar
operation. An ordinary managed `RobotProgram.Prestart` remains data-only and never defers hardware
graph construction behind this screen.

Keep it for flows that genuinely need a distinct confirm/cancel action. An Auto selector with a
read-only summary can instead remain editable through a separate action and use FTC START as its
sole freeze boundary.

## `SummaryScreen`

`SummaryScreen` is a read-only status or review page. It is useful after a flow has produced a
result, and also before an external lifecycle boundary such as FTC START applies a currently
selected value. Unlike `ConfirmationScreen`, pressing `A` does not imply an apply action.

Example:

```java
SummaryScreen review = SummaryScreen.builder("Autonomous Selection")
        .status("READY", "FTC START will freeze this setup.")
        .row("Alliance", spec.alliance)
        .row("Start", spec.startPosition)
        .row("Strategy", spec.strategy)
        .controls("START: freeze selection | X: edit selection")
        .onSecondary(() -> navigator.setRoot(allianceScreen()))
        .consumeBack(true)
        .consumeHome(true)
        .build();

navigator.setRoot(review);
```

An Auto selector can use this shape after the strategy choice. Hardware is already owned, the
summary performs no confirmation, and `X` returns to the selector. The one
`RobotProgram.Prestart.freezeForStart()` boundary freezes the visible data and decides whether
behavior may start. A locked post-action page remains another valid use; consume back/home when the
caller must prevent visible state from drifting away from already-applied state.

## `UiControls`

`UiControls` packages standard menu controls so screens do not each invent their own meanings.

Default gamepad mapping:

```text
Dpad Up/Down: move selection
Dpad Left/Right: page or adjust a side value
A: choose / confirm
B or BACK: back / cancel
X: secondary action, commonly refresh/details
Y: home/root
```

Tester suites still avoid using `B` as the global active-tester back button because many tester screens use `B` for local actions such as zero, center, reset, or abort. Hardware picker refresh moved to `X` so `B` can remain available for back/cancel in richer menu flows.

## `HardwareNamePicker`

`HardwareNamePicker` is a framework-level picker for configured FTC device names. It uses `SelectionMenu` internally and preserves the highlighted hardware name by stable id after refresh.

Typical controls:

```text
Dpad: highlight | A: choose | X: refresh
```

A picker should be used while the robot is in a selection screen, usually during `init()` / `initLoop()` or a tester picker state. Do not repeatedly enumerate hardware while actively commanding actuators.

The picker owns only selection UI, not the selected device or tester lifecycle. Its caller must
retain any resource created from the selected name before further setup and close or stop it before
showing the picker again. If that cleanup throws, keep replacement disabled and direct the operator
to restart the OpMode; opening another selection would create competing ownership while the old
state is unknown.

## Design rule

Keep these responsibilities separate:

```text
SelectionMenu:
  one visible list screen

MenuNavigator:
  levels, breadcrumbs, back/home, wizard root/top replacement

SelectionMenus:
  common row-building helpers such as enum-backed menus

ConfirmationScreen:
  final review before confirm/cancel flows

SummaryScreen:
  read-only review, status, or locked-result pages; no implied A-to-apply action

HardwareNamePicker:
  FTC hardware-name enumeration

Robot/tester/auto code:
  what a selected value actually does
```

This split supports tester menus and pre-start autonomous selectors without turning the framework
into a robot-specific menu system.
