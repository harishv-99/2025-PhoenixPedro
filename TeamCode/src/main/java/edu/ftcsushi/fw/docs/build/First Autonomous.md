---
tags:
  - Build
---

# Sequence proven mechanism Tasks in Auto

**Outcome:** build a fresh autonomous Task from already-proven mechanisms, with later steps starting
only after their prerequisites succeed.

**Prerequisites:** every mechanism used by the selected Auto has passed its software checkpoint and
isolated hardware gate. Both mechanism recipes are required only for the optional combined capstone.

## Critical production idea

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoRoutines.java -->
```java
return Tasks.sequence(
        requiredLift.home(),
        requiredLift.moveTo(BasicLift.Height.HIGH),
        // The lift is the deadline, while the write-once claw request starts concurrently
        // and remains held by the mechanism after that companion Task succeeds.
        Tasks.parallelDeadline(
                requiredLift.moveTo(BasicLift.Height.LOW),
                requiredClaw.setStateTask(BasicClaw.State.CLOSED)),
        Tasks.waitForSeconds(GUIDE_HOLD_SEC),
        Tasks.parallelDeadline(
                requiredLift.moveTo(BasicLift.Height.STOWED),
                requiredClaw.setStateTask(BasicClaw.State.OPEN)));
```

Notice:

- [`Tasks.sequence`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/task/Tasks.html>)
  preserves a failed prerequisite outcome and does not start later work.
- `parallelDeadline` starts both children together, but the feedback-aware lift move defines when
  that phase ends; the claw's write-once Task leaves its request held.
- `guide(...)` is a factory: every call creates a fresh single-use Task graph.

## Files in this checkpoint

**Main:**

- [`BasicClawAuto`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawAuto.html>) — smallest claw-only request host.
  [Complete source: `BasicClawAuto.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicClawAuto.java>)
- [`BasicLiftAuto`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftAuto.html>) — lift-only home and feedback-move sequence.
  [Complete source: `BasicLiftAuto.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftAuto.java>)
- [`BasicAutoRoutines`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoRoutines.html>) — robot-owned sequence policy.
- [Complete source: `BasicAutoRoutines.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoRoutines.java>)
- [`BasicMechanismsAuto`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicMechanismsAuto.html>) — optional lift-and-claw composition root.
- [Complete source: `BasicMechanismsAuto.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicMechanismsAuto.java>)

Start with the single-mechanism host that matches the hardware already proven. The combined host and
routine are the optional capstone; they reuse the exact `BasicLift*` and `BasicClaw*` files from the
prerequisite recipes rather than introducing alternate mechanism implementations.

**Test:**

- **Beginner scenario:** [Complete source: `BasicAutoSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoSoftwareScenarioTest.java>) — one successful path with framework-built recording Tasks.
- **Supplied maintainer evidence:** [Complete source: `BasicAutoRoutinesTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoRoutinesTest.java>) — the broader timeout, cancellation, and fresh-Task contract suite. Run it; do not treat its controllable Task double as beginner robot code.

## Software checkpoint: success admits the next work

- **Question:** Does each successful prerequisite admit exactly the next authored work?
- **Keep real:** `BasicAutoRoutines` and Sushi's sequence/outcome composition.
- **Replace:** mechanism capabilities with small recorders whose Tasks succeed after test-controlled time.
- **Observe:** the order in which home, lift moves, and claw requests start, plus the root outcome.
- **Cannot conclude:** real timing, clearance between mechanisms, current limits, or safe motion.

The recording lift uses only `Tasks.sequence`, `Tasks.runOnce`, and `Tasks.waitForSeconds`; there is
no invented Task state machine to reverse-engineer. One test second stands in for successful lift
feedback so the test can make each admission decision visible. The recording claw merely records
the write-once semantic request.

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoSoftwareScenarioTest.java -->
```java
// REQUEST: starting the real routine admits only its first prerequisite: homing.
auto.start(time.clock());
assertEquals(Arrays.asList("home"), events);

// HEARTBEAT: successful home admits HIGH; successful HIGH admits LOW and CLOSED together.
auto.update(time.nextCycle(STEP_SEC));
auto.update(time.nextCycle(STEP_SEC));
assertEquals(
        Arrays.asList("home", "lift HIGH", "lift LOW", "claw CLOSED"),
        events);
```

Run:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.basicmechanisms.BasicAutoSoftwareScenarioTest
```

**Read the causal chain:** start admits only homing; its successful test boundary admits HIGH; HIGH
success starts LOW and CLOSED together; LOW success admits the authored hold; the hold starts STOWED
and OPEN together; final lift success makes the root outcome `SUCCESS`.

**Proves:** the real routine preserves the authored success gates, parallel starts, and successful
root outcome without requiring hardware or a custom test Task implementation.

**Does not prove:** the combined robot has mechanical clearance or completes the sequence on time.

After this story is clear, run the supplied `BasicAutoRoutinesTest` maintainer suite to check the
same policy's timeout preservation, active cancellation, and fresh-Task guarantees.

## Isolated hardware gate

Keep the selected Auto disabled. Before enabling it, draw the complete Task timeline, including the
evidence required to advance each step, and review every motion gate. Reconfirm every included
mechanism's STOP plan, reduce first-run energy, and appoint an immediate STOP operator. Only then
enable that host and its required motion gates for one supervised run. Use
`BasicMechanismsAuto` only after both mechanisms are proven together; a timeout is a failed run,
not permission to continue.

**Next gate:** add at most one already-proven behavior to that timeline, then repeat the software
outcome checkpoint before another physical run.
