# Shooter Case Study & Examples Walkthrough

This document walks through the **mecanum + shooter** examples in
`edu.ftcphoenix.fw.examples` and explains how they build on each other:

1. Example 01 – `TeleOp_01_MecanumBasic`
2. Example 02 – `TeleOp_02_ShooterBasic`
3. Example 03 – `TeleOp_03_ShooterMacro`
4. Example 04 – `TeleOp_04_ShooterInterpolated`
5. Example 05 – `TeleOp_05_ShooterTagAimVision`
6. Example 06 – `TeleOp_06_ShooterTagAimMacroVision`

All six examples share the same **loop shape**, and each adds a new idea:

* Plants and Actuators
* Bindings and Tasks
* PlantTasks helpers and macros
* Interpolated shooter speed vs distance
* TagAim and vision‑based alignment

The goal of this case study is to show how these pieces fit together using
**real code from the examples**. When you’re writing your own robot code, the
recommended order of tools is:

1. Use the **factory helpers** first:

    * `Tasks.*` for general tasks.
    * `PlantTasks.*` for mechanism‑related tasks.
    * `DriveTasks.*` for drive‑related tasks.
2. Drop down to the core Task classes (`InstantTask`, `RunForSecondsTask`,
   `SequenceTask`, `ParallelAllTask`, ...) only when you need extra
   customization or when you’re building new helper factories.

---

## 1. Shared loop shape

Every example uses the same high‑level loop:

```java
@Override
public void start() {
    clock.reset(getRuntime());
}

@Override
public void loop() {
    // 1) Clock
    clock.update(getRuntime());
    double dtSec = clock.dtSec();

    // 2) Inputs + bindings
    gamepads.update(dtSec);
    bindings.update(dtSec);

    // 3) Macros (if any)
    macroRunner.update(clock);   // used in Examples 03 & 06

    // 4) Drive
    DriveSignal driveCmd = stickDrive.get(clock).clamped();
    drivebase.drive(driveCmd);
    drivebase.update(clock);

    // 5) Mechanism plants
    shooter.update(dtSec);
    transfer.update(dtSec);
    pusher.update(dtSec);

    // 6) Telemetry
    sendTelemetry();
}
```

Key points:

* **Tasks/macros never directly drive motors.** They only call
  `Plant.setTarget(...)` (and drive code calls `drivebase.drive(...)`).
* The main loop is responsible for calling `update(...)` on everything
  once per cycle.
* You can drop in more tasks, more bindings, or more drive logic without
  changing this loop shape.

The rest of this walkthrough explains how each example adds behavior on
top of this pattern.

---

## 2. Example 01 – `TeleOp_01_MecanumBasic`

**Goal:** drive a mecanum robot with gamepad sticks.

This example introduces:

* `Loops` and `LoopClock`.
* `Gamepads` and `Bindings`.
* `DriveSource` (`GamepadDriveSource`) to convert sticks → `DriveSignal`.
* `MecanumDrivebase` created via `Drives.mecanum(...)`.

There is **no shooter** yet. The important lesson is the shape of the loop
and how the drivebase is updated independently of anything else.

---

## 3. Example 02 – `TeleOp_02_ShooterBasic`

**Goal:** add a shooter mechanism (shooter + transfer + pusher) using:

* `Actuators.plant(...)` to turn hardware into `Plant`s.
* `Bindings` to map buttons to shooter modes.

### 3.1 Hardware assumptions

* Drive: same mecanum configuration as Example 01.
* Shooter motors: `"shooterLeftMotor"`, `"shooterRightMotor"`.
* Transfer CR servos: `"transferLeftServo"`, `"transferRightServo"`.
* Pusher servo: `"pusherServo"`.

The example defines constants like:

```java
private static final String HW_SHOOTER_LEFT  = "shooterLeftMotor";
private static final String HW_SHOOTER_RIGHT = "shooterRightMotor";

private static final String HW_TRANSFER_LEFT  = "transferLeftServo";
private static final String HW_TRANSFER_RIGHT = "transferRightServo";

private static final String HW_PUSHER = "pusherServo";
```

### 3.2 Creating plants with `Actuators.plant(...)`

Shooter plants are created in a compact builder style. For example:

```java
// Shooter: two motors, velocity‑controlled pair.
shooter = Actuators.plant(hardwareMap)
        .motorPair(HW_SHOOTER_LEFT, false,
                   HW_SHOOTER_RIGHT, true)
        .velocity(SHOOTER_VELOCITY_TOLERANCE_NATIVE)
        .build();

// Transfer: two CR servos, power‑controlled pair.
transfer = Actuators.plant(hardwareMap)
        .crServoPair(HW_TRANSFER_LEFT, false,
                     HW_TRANSFER_RIGHT, true)
        .power()
        .build();

// Pusher: single positional servo, 0..1 position plant.
pusher = Actuators.plant(hardwareMap)
        .servo(HW_PUSHER, false)
        .position()   // servo set‑and‑hold, no feedback
        .build();
```

Interpretation:

* `motorPair(...).velocity(...)` → encoder‑backed **velocity plant** with
  feedback and a tolerance, so `hasFeedback() == true` and `atSetpoint()`
  has meaning.
* `crServoPair(...).power()` → open‑loop power plant (no feedback).
* `servo(...).position()` → servo position plant; this is open‑loop
  "set‑and‑hold" (no feedback), so `atSetpoint()` is always true.

### 3.3 Shooter modes via `Bindings`

Example 02 uses an `enum` for the shooter mode and a small state machine
in the main loop to convert modes → plant targets.

Controls (Example 02):

* Drive: same sticks + RB slow mode as Example 01.
* P1 right bumper: toggle shooter on/off.
* P1 A: LOAD (gentle transfer, pusher load position).
* P1 B: SHOOT (faster transfer, pusher shoot position).
* P1 X: RETRACT (stop transfer, retract pusher).

The important point is that **no tasks/macros** are used yet. Everything
is purely “mode → plant target” based on the current button state.

---

## 4. Example 03 – `TeleOp_03_ShooterMacro`

**Goal:** build a "shoot one ball" macro using the Task system.

This example adds:

* A `TaskRunner` for shooter macros.
* `PlantTasks` helpers for common plant behaviors.
* A single macro that spins up, feeds one ball, then spins down.

### 4.1 Adding a TaskRunner for shooter macros

The OpMode has a `TaskRunner` field:

```java
private final TaskRunner macroRunner = new TaskRunner();
```

The main loop now has:

```java
// --- 3) Macros (shooter/transfer/pusher) ---
macroRunner.update(clock);
```

Bindings connect buttons to macros:

```java
// Y: enqueue "shoot one ball" macro.
bindings.onPress(
        gamepads.p1().y(),
        this::enqueueShootOneBallMacro
);

// B: cancel macro and stop shooter.
bindings.onPress(
        gamepads.p1().b(),
        this::cancelShootMacros
);
```

Inside `enqueueShootOneBallMacro()` the code simply does:

```java
private void enqueueShootOneBallMacro() {
    macroRunner.enqueue(buildShootOneBallMacro());
}
```

### 4.2 Safe defaults when no macro is running

When no macro is active, the example applies a “safe idle” behavior
for the shooter, transfer, and pusher.

Roughly:

```java
if (!macroRunner.isBusy()) {
    shooter.setTarget(0.0);
    transfer.setTarget(0.0);
    pusher.setTarget(PUSHER_POS_RETRACT);
}
```

This ensures that if you cancel the macro (or it completes), the
mechanism goes back to a known safe state.

### 4.3 Using `PlantTasks` to build the shooter macro

The heart of Example 03 is `buildShootOneBallMacro()`, which uses
`PlantTasks` helpers to construct a macro from **small, reusable tasks**.

```java
private Task buildShootOneBallMacro() {
    // Step 1: set shooter target and wait for atSetpoint() or timeout.
    Task spinUp = PlantTasks.moveTo(
            shooter,
            SHOOTER_VELOCITY_NATIVE,
            SHOOTER_SPINUP_TIMEOUT_SEC
    );

    // Step 2: feed one ball.
    //   - Transfer runs at shoot power for TRANSFER_PULSE_SEC, then stops.
    //   - Pusher steps through LOAD → SHOOT → RETRACT.
    Task feedTransfer = PlantTasks.holdFor(
            transfer,
            TRANSFER_POWER_SHOOT,
            TRANSFER_PULSE_SEC
    );

    Task pusherLoad = PlantTasks.holdFor(
            pusher,
            PUSHER_POS_LOAD,
            PUSHER_STAGE_SEC
    );

    Task pusherShoot = PlantTasks.holdFor(
            pusher,
            PUSHER_POS_SHOOT,
            PUSHER_STAGE_SEC
    );

    Task pusherRetract = PlantTasks.holdFor(
            pusher,
            PUSHER_POS_RETRACT,
            PUSHER_STAGE_SEC
    );

    Task feedBoth = ParallelAllTask.of(
            feedTransfer,
            SequenceTask.of(pusherLoad, pusherShoot, pusherRetract)
    );

    // Step 3: hold shooter up to speed briefly before spinning down.
    Task holdBeforeSpinDown = PlantTasks.holdFor(
            shooter,
            SHOOTER_VELOCITY_NATIVE,
            SHOOTER_SPINDOWN_HOLD_SEC
    );

    Task spinDown = PlantTasks.setInstant(shooter, 0.0);

    return SequenceTask.of(
            spinUp,
            feedBoth,
            holdBeforeSpinDown,
            spinDown
    );
}
```

You could rewrite this using only factories:

```java
Task spinUp = PlantTasks.moveTo(
        shooter,
        SHOOTER_VELOCITY_NATIVE,
        SHOOTER_SPINUP_TIMEOUT_SEC
);

Task feedTransfer = PlantTasks.holdFor(
        transfer,
        TRANSFER_POWER_SHOOT,
        TRANSFER_PULSE_SEC
);

Task feedBoth = Tasks.parallelAll(
        feedTransfer,
        Tasks.sequence(pusherLoad, pusherShoot, pusherRetract)
);

Task macro = Tasks.sequence(
        spinUp,
        feedBoth,
        holdBeforeSpinDown,
        PlantTasks.setInstant(shooter, 0.0)
);
```

In your own code, we recommend:

* Prefer **`Tasks.sequence`** / **`Tasks.parallelAll`** and the
  `PlantTasks` helpers for common patterns.
* Use `SequenceTask.of` / `ParallelAllTask.of` mainly when you’re building
  new helper factories or want very explicit control.

Controls (Example 03):

* Drive: same as Example 01.
* P1 Y: enqueue the “shoot one ball” macro.
* P1 B: cancel macro and stop shooter.

---

## 5. Example 04 – `TeleOp_04_ShooterInterpolated`

**Goal:** choose shooter speed automatically based on distance.

This example introduces:

* `InterpolatingTable1D` – a 1D lookup table with interpolation.
* Manual distance selection via the D‑pad.
* A single source of truth for mapping distance → shooter velocity.

### 5.1 The interpolation table

The code builds a table of `(distanceInches, shooterVelocity)` points:

```java
private InterpolatingTable1D shooterTable;

private void initShooterTable() {
    shooterTable = new InterpolatingTable1D();
    shooterTable.put(24.0, 2100.0);  // near shot
    shooterTable.put(36.0, 2200.0);  // mid shot
    shooterTable.put(48.0, 2350.0);  // far shot
}
```

Later, the TeleOp picks a distance (from D‑pad input) and uses the table
for the shooter target:

```java
double distanceInches = manualDistance;        // from D‑pad
double shooterTarget  = shooterTable.interpolate(distanceInches);
shooter.setTarget(shooterTarget);
```

You can think of this as **decoupling**:

* “How far away is the goal?”
* “What shooter velocity do we want for that distance?”

Once you have this table, you can plug in a different distance source
without changing the mapping logic.

---

## 6. Example 05 – `TeleOp_05_ShooterTagAimVision`

**Goal:** use AprilTags to aim the robot at the goal and estimate distance.

This example adds:

* A vision pipeline that estimates robot pose from AprilTags.
* A `TagAimDriveSource` that turns the AprilTag pose into drive signals to
  center and align the robot.
* A distance estimate from the same pose, which can be fed into the shooter
  interpolation table.

The key idea is that **drive aiming** is separated from **shooter control**:

* Drive uses `TagAimDriveSource` to rotate/translate the robot so the tag
  is centered and the robot is square to the goal.
* Shooter uses the distance estimate to pick a velocity from `shooterTable`.

Because both are just ordinary `DriveSource` / `Plant` users, they plug
into the same loop shape as before.

---

## 7. Example 06 – `TeleOp_06_ShooterTagAimMacroVision`

**Goal:** combine everything:

* Mecanum drive.
* Vision‑based TagAim alignment.
* Distance‑based shooter interpolation.
* A shooter macro built with `PlantTasks` and `Tasks`.

This OpMode looks a lot like Example 03, but:

* Uses vision distance instead of manual distance.
* Uses TagAim when a button is held to keep the robot aligned.
* Reuses the same `buildShootOneBallMacro()` idea from Example 03.

The message is that **Tasks + Plants + Drive sources compose cleanly**:

* You can plug in TagAim where the drive signal is computed.
* You can plug in interpolated shooter speeds where you set the shooter target.
* You can reuse the same macros whether distance is manual or vision‑based.

---

## 8. Recommended patterns going forward

When you build your own mechanisms and macros, this is the suggested
approach:

1. **Model hardware as Plants** using `Actuators.plant(...)`.

    * Use `motor(...)` / `motorPair(...)` + `.velocity(...)` or
      `.position(tolerance)` for feedback‑capable plants.
    * Use `servo(...)` + `.position()` or `crServo(...)` + `.power()` for
      simpler open‑loop plants.
2. **Use `PlantTasks` for common behaviors**:

    * Time‑based holds: `holdFor(...)`, `holdForThen(...)`.
    * Feedback‑based moves: `moveTo(...)`, `moveTo(..., timeout)`,
      `moveToThen(...)`.
    * Instant target changes: `setInstant(...)`.
3. **Use `Tasks` factories to assemble macros**:

    * `Tasks.sequence(...)` and `Tasks.parallelAll(...)`.
    * `Tasks.waitForSeconds(...)`, `Tasks.waitUntil(...)`, `Tasks.instant(...)`.
4. **Drop down to raw Task classes** (`InstantTask`, `RunForSecondsTask`,
   `SequenceTask`, `ParallelAllTask`, ...) when you:

    * Are building new helper factories.
    * Need special behavior not covered by the factories yet.

If your code starts to feel repetitive or complex at the Task level,
that’s a sign you should extract a new helper into `PlantTasks`,
`DriveTasks`, or your own `MyRobotTasks` class so the next student can
call a single method instead of re‑creating the pattern.

This shooter case study is meant to be a template: you can swap in your
own mechanisms and goals, but keep the same building blocks and loop
shape for predictable, non‑blocking robot code.
