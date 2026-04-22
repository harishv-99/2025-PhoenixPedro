# FTC UI Helpers

Phoenix's FTC UI helpers are small telemetry-screen building blocks for places where the Driver Station needs to show choices during `init_loop()` or a tester run.

The important layering rule is:

- **framework UI owns display and navigation mechanics**
- **robot/tester code owns what the choices mean**

That means a menu can show an autonomous strategy, a hardware name, or a calibration step, but the menu does not know how to build a robot, resolve a route, or run a tester. It simply reports the selected value back to the caller.

## Package

Reusable FTC telemetry UI helpers live in:

```java
edu.ftcphoenix.fw.ftc.ui
```

This package is FTC-facing because it renders to `Telemetry` and some helpers enumerate `HardwareMap`. It should not be used by FTC-independent framework core classes.

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
Auto Setup > Red > Audience > Partner Plan > Strategy
```

The navigator:

- dispatches one shared set of bindings to the current screen
- supports push/pop/home
- supports `setRoot(...)` and `replaceTop(...)` for wizard-style flows
- renders breadcrumb path and nesting level
- avoids stale bindings from screens that are no longer active

`SelectionMenu` remains useful inside the navigator because it implements `MenuScreen`.


## `SelectionMenus`

`SelectionMenus` contains convenience factories for common menu shapes. The first general helper is
`forEnum(...)`, which turns an enum into one `SelectionMenu` row per value. This is especially useful
for pre-start setup screens such as alliance, start position, partner plan, or strategy.

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
ConfirmationScreen confirm = ConfirmationScreen.builder("Confirm Auto")
        .row("Alliance", "RED")
        .row("Start", "AUDIENCE")
        .row("Strategy", "Safe Preload")
        .warning("Verify this before pressing START.")
        .onConfirm(() -> buildSelectedAuto())
        .build();
```

Use it after a multi-step selector, before building hardware-heavy runtime objects or applying a
calibration reset.

## `SummaryScreen`

`SummaryScreen` is a read-only status page for after a setup flow has already produced or applied
its result. It is useful when the visible UI must stop changing after the runtime object graph has
been built.

Example:

```java
SummaryScreen locked = SummaryScreen.builder("Phoenix Auto Locked")
        .status("LOCKED", "Phoenix + Pedro are initialized for this spec.")
        .row("Alliance", spec.alliance)
        .row("Start", spec.startPosition)
        .row("Strategy", spec.strategy)
        .consumeBack(true)
        .consumeHome(true)
        .controls("START: run selected Auto | B/Y: locked after initialization")
        .build();

navigator.setRoot(locked);
```

Use it after confirmation when changing the previous choices would no longer rebuild the underlying
robot runtime. That prevents a Driver Station menu from showing one setup while the already-created
robot and route queue are using another.

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
  read-only status or locked-result pages after a setup flow has already applied

HardwareNamePicker:
  FTC hardware-name enumeration

Robot/tester/auto code:
  what a selected value actually does
```

This split is meant to support both current tester menus and future pre-start autonomous selectors without turning the framework into a robot-specific menu system.
