# Your first mechanism

## Goal

Configure one intake motor, understand its Plant-backed owner, and run the starter's one-motor Auto
checkpoint before adding a drivetrain.

**Time:** 30–45 minutes, including a secured, unloaded direction test.

**Prerequisites:**

- the successful build and test checkpoint from [`Build and Run`](<Build and Run.md>);
- one motor in the active Robot Controller configuration; and
- a mechanism that can turn safely at low power without hitting a hard stop.

**Files for this lesson:**

- [`StarterProfile.java`](<../../../robots/examples/starter/StarterProfile.java>) — intake name,
  direction, and powers;
- [`StarterIntake.java`](<../../../robots/examples/starter/StarterIntake.java>) — robot meanings;
- [`StarterIntakeMechanism.java`](<../../../robots/examples/starter/StarterIntakeMechanism.java>) —
  private Plant and safe stop;
- [`StarterRobot.java`](<../../../robots/examples/starter/StarterRobot.java>) — Auto declarations;
- [`StarterAuto.java`](<../../../robots/examples/starter/StarterAuto.java>) — disabled FTC entry.

## Safety

- Disconnect or remove game pieces before the first motor test.
- Secure loose wires, clothing, hair, and tools away from the mechanism.
- Secure the robot itself, but leave the unloaded mechanism free to rotate. Do not hold or stall a
  powered shaft, wheel, roller, or gearbox by hand.
- Use a mechanism that can run continuously in either direction. Do not use this power example for
  an arm, lift, or other bounded mechanism.
- Begin with a low nonzero power, keep one operator ready to press STOP, and verify that STOP
  immediately removes power.
- Set `hardwareConfigurationReviewed = true` only after reviewing the values used by this Auto.

## 1. Configure only the intake facts

`StarterRobot.declareAuto(...)` calls `profile.requireReadyForAuto()`. That check requires the
shared intake fields but does not require the unused mecanum configuration.

Edit `StarterProfile.current()`:

```java
public static StarterProfile current() {
    StarterProfile profile = new StarterProfile();

    // Leave the checked-in drive placeholders unchanged for this one-motor Auto.

    profile.intake.motorName = "YOUR_INTAKE_NAME";
    profile.intake.direction = Direction.FORWARD;
    profile.intake.collectPower = 0.20;
    profile.intake.ejectPower = -0.20;
    profile.hardwareConfigurationReviewed = true;
    return profile;
}
```

Replace the name with the exact case-sensitive Robot Controller configuration name. The direction
and powers above show the required value types; verify the signs on your hardware. The two powers
must be finite, nonzero, different, and inside `[-1.0, +1.0]`.

## 2. Read the capability before the motor code

`StarterIntake` gives TeleOp and Auto one robot vocabulary:

```java
enum Mode {
    STOPPED,
    COLLECT,
    EJECT
}

void setMode(Mode mode);
Task collectForSeconds(double durationSec);
Status status();
```

Callers ask for `COLLECT`; they do not know the motor name or power. That keeps the same behavior
available to both modes.

## 3. Follow the ownership chain

The mechanism receives `HardwareMap` plus its data-only config, copies the config, and privately
builds one normalized-power Plant:

```java
plant = FtcActuators.plant(hardwareMap)
        .motor(snapshot.motorName, snapshot.direction)
        .power()
        .targetFromNewCommand(STOPPED_POWER)
        .build();
```

Read the chain from left to right:

1. use FTC hardware from this `HardwareMap`;
2. select the configured motor and direction;
3. command normalized power;
4. create its persistent command with an initial target of `0.0`; and
5. build the Plant.

The same class implements `RobotProgram.Output`:

```java
@Override
public void update(LoopClock clock) {
    plant.update(clock);
}

@Override
public void stop() {
    plant.stop();
}
```

The mechanism is the only owner that updates and stops this Plant. Controls and Tasks change its
request; they never add another hardware update. This final `stop()` immediately applies the
power Plant's natural zero-power stop and makes the Plant terminal. It does not need to rewrite the
command target: every later `update(...)` is inert. Request `STOPPED` through `setMode(...)` when
the mechanism should idle during the active match and remain usable afterward.

## 4. See how Auto declares the mechanism

`StarterRobot.declareAuto(...)` transfers the completed mechanism to the managed program, asks the
capability for one behavior, and declares that behavior as the root:

```java
StarterIntakeMechanism intake = program.output(
        new StarterIntakeMechanism(hardwareMap, profile.intake));
Task root = intake.collectForSeconds(collectDurationSec);
program.rootTask(root);
```

The next Task lesson explains how the timed behavior works. For this checkpoint, observe its visible
contract: collect for the requested duration, then return to stopped.

## 5. Enable and run the one-motor Auto

Remove `@Disabled` from `StarterAuto`. Leave `StarterTeleOp` and the Pedro example disabled.

Compile and deploy:

```powershell
.\gradlew.bat --console=plain :TeamCode:compileDebugJavaWithJavac
```

With the robot secured and the unloaded mechanism clear and free to rotate:

1. Select **FW Starter: Auto**.
2. Press INIT. The motor must remain stopped.
3. Press START while ready to press STOP.
4. Observe collection power for 0.75 seconds.
5. Verify the motor returns to zero without a blocking delay.
6. Press STOP and verify zero again.

If the mechanism moves opposite the meaning of `COLLECT`, press STOP and correct
`profile.intake.direction` or the reviewed power signs. Keep the semantic meanings in the
capability unchanged.

## Expected checkpoint

- Auto initializes without requiring drive motor configuration.
- INIT produces no motor motion.
- START commands the reviewed collection direction for 0.75 seconds.
- The mechanism returns to `STOPPED` automatically.
- FTC STOP immediately applies the mechanism's safe stop path.
- You can point to the capability, private Plant, program output declaration, and hardware owner.

## Common problems

**Auto reports that `StarterProfile` is not ready.**

Read the complete issue list. For this mode it identifies `hardwareConfigurationReviewed` or an
intake name, direction, or power. Drive placeholders do not block this Auto.

**The motor name cannot be found.**

Compare the exact name and case with the active Robot Controller configuration. Check for spaces and
confirm that the expected configuration is selected.

**The motor never moves.**

Press STOP and remove robot power before touching a cable, motor port, or hub. Then confirm the
configured power is nonzero, the mechanism is connected to the configured port, and the hub can be
powered normally. Restore power only after hands and tools are clear, then repeat the controlled
test. Do not increase power until those facts are checked.

**The motor keeps running after 0.75 seconds or STOP.**

Press STOP, power down the mechanism, and inspect local changes before continuing. The checked-in
Task ends at zero and the mechanism's terminal `stop()` immediately submits the Plant's natural
zero-power stop.

**Can a lift use this power Plant?**

Not as a copy-paste replacement. A lift has travel bounds and normally needs position feedback and
safety policy. Finish the course with the free-running intake, then use the actuator and
calibration guides for the lift's real requirements.

**Next:** [`Your first TeleOp`](<First TeleOp.md>)
