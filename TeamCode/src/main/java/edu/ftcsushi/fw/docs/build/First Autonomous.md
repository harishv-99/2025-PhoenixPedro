---
tags:
  - Build
---

# Sequence proven lift Tasks in Auto

**Outcome:** run a fresh lift-only Auto in which `home`, `HIGH`, and `STOWED` begin in order only
after the preceding feedback-aware Task succeeds.

**Prerequisites:** first complete [one timed Auto](<Run One Timed Auto.md>) so the managed root Task
and START lifecycle are familiar. Then complete [reference a lift](<Referenced Lift.md>) and
[move a referenced lift](<Move a Referenced Lift.md>), including their software checkpoints and
isolated hardware gates; those two fixtures supply the capability evidence needed by this sequence.
No claw is required.

## Critical production idea

The robot-owned routine factory composes the same capability Tasks already used in focused lift
work:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoRoutines.java -->
```java
public static Task liftOnly(BasicLift lift) {
    BasicLift requiredLift = Objects.requireNonNull(lift, "lift");
    return Tasks.sequence(
            requiredLift.home(),
            requiredLift.moveTo(BasicLift.Height.HIGH),
            requiredLift.moveTo(BasicLift.Height.STOWED));
}
```

[`Tasks.sequence(...)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/task/Tasks.html>)
constructs the fixed child graph eagerly, so each capability method must be a side-effect-free
factory. The returned children do not start eagerly. At FTC START, only `home()` starts. Exact
`SUCCESS` admits `HIGH`; exact `SUCCESS` from `HIGH` admits `STOWED`. `TIMEOUT`, `CANCELLED`, or
`UNKNOWN` becomes the root outcome and suppresses every later child.

The lift-only host registers the mechanism output first, builds one fresh graph, and declares that
graph as the program's one root:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftAuto.java -->
```java
BasicLiftMechanism lift = program.output(
        new BasicLiftMechanism(hardwareMap, profile.lift));
Task auto = BasicAutoRoutines.liftOnly(lift);
program.rootTask(auto);
```

`program.rootTask(auto)` delegates START, per-cycle updates, active cancellation, and cleanup to the
managed host. The sequence never sleeps and never calls the lift's output update. Its Tasks publish
semantic requests or evaluate cached feedback; the later managed output phase advances the private
Plant and writes hardware.

Success is evidence, not elapsed time alone. The production `home()` succeeds only after its
reference cue is established, and `moveTo(...)` succeeds only from the selected request's cached
arrival evidence. A timeout reports that evidence did not arrive within the configured budget; it
does not grant permission to continue or start `STOWED`. `BasicLift.moveTo(...)` deliberately uses
`leaveRequestOnCancel()`, however, so `HIGH` remains the persistent request after its timeout or a
direct active cancellation. The managed output may keep realizing or holding that request. A
different recovery target must be explicit robot policy. FTC STOP both cancels the active child and
terminally stops the Plant immediately, without needing another loop.

### Optional capstone: add the proven claw

Only after the claw and lift have each passed their separate gates, `BasicAutoRoutines.guide(...)`
and `BasicMechanismsAuto` may be used as a combined capstone. Its
`Tasks.parallelDeadline(liftMove, clawRequest)` phases start a feedback-aware lift move and a
write-once claw request together. The lift is explicitly the deadline: its completion and outcome
end that phase, while the claw's persistent request remains owned by the claw mechanism. This is a
later coordination decision, not a prerequisite for learning the first sequence.

Notice:

- `liftOnly(...)` is a factory; each call builds a fresh single-use root and fresh child Tasks.
- Ordinary `sequence(...)` is success-gated, so failed evidence cannot silently start a later Task.
- Task policy requests behavior; the lift mechanism remains the only Plant updater and motor writer.

## Files in this checkpoint

**Main:**

- [`BasicAutoRoutines`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoRoutines.html>) — lift-only sequence and optional combined capstone factories.
  [Complete source: `BasicAutoRoutines.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoRoutines.java>)
- [`BasicLiftAuto`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftAuto.html>) — first lift-only managed host.
  [Complete source: `BasicLiftAuto.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicLiftAuto.java>)
- [`BasicMechanismsAuto`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/basicmechanisms/BasicMechanismsAuto.html>) — optional lift-and-claw capstone host.
  [Complete source: `BasicMechanismsAuto.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicMechanismsAuto.java>)

**Test:**

- [Complete source: `BasicAutoSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoSoftwareScenarioTest.java>) — student-facing success, timeout, and cancellation evidence using framework Tasks.
- [Complete source: `BasicAutoRoutinesTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoRoutinesTest.java>) — supplied maintainer coverage for the optional parallel capstone.

## Software checkpoint: success admits the next work

- **Question:** Does the maintained lift-only routine start only the admitted work, retain exact
  success/timeout/cancel outcomes, and suppress later Task starts after non-success?
- **Keep real:** `BasicAutoRoutines.liftOnly(...)` and Sushi's Task factories, timing, sequence,
  outcomes, and cancellation.
- **Replace:** the physical lift with a recording capability whose `home()` and `moveTo(...)`
  return only framework-built Tasks.
- **Observe:** which semantic step starts and the exact root `TaskOutcome` after supplied evidence,
  timeout, or cancellation.
- **Cannot conclude:** encoder direction, switch truth, real travel time, clearance, current draw,
  or physical stopping distance.

The successful path makes every admission decision visible. One test second stands in for each
piece of successful lift evidence; it is not a mechanism model:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoSoftwareScenarioTest.java -->
```java
auto.start(time.clock());
assertEquals(Arrays.asList("home"), events);

// INJECT EVIDENCE: each successful boundary admits exactly the next move.
auto.update(time.nextCycle(STEP_SEC));
assertEquals(Arrays.asList("home", "lift HIGH"), events);
auto.update(time.nextCycle(STEP_SEC));
assertEquals(Arrays.asList("home", "lift HIGH", "lift STOWED"), events);
assertFalse(auto.isComplete());
```

The second method first injects a timeout while `HIGH` is active and observes no `STOWED` event:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoSoftwareScenarioTest.java -->
```java
timed.start(timeoutTime.clock());
timed.update(timeoutTime.nextCycle(STEP_SEC));
timed.update(timeoutTime.nextCycle(STEP_SEC));
assertEquals(TaskOutcome.TIMEOUT, timed.getOutcome());
assertEquals(TaskOutcome.TIMEOUT, timedLift.highTask.getOutcome());
assertEquals(Arrays.asList("home", "lift HIGH"), timedEvents);
```

It then starts a fresh graph, cancels while `HIGH` is active, repeats cancellation to prove
idempotence, and again observes no `STOWED` event:

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/basicmechanisms/BasicAutoSoftwareScenarioTest.java -->
```java
cancelled.start(cancelTime.clock());
cancelled.update(cancelTime.nextCycle(STEP_SEC));
cancelled.cancel();
cancelled.cancel();
assertEquals(TaskOutcome.CANCELLED, cancelled.getOutcome());
assertEquals(TaskOutcome.CANCELLED, cancelledLift.highTask.getOutcome());
assertEquals(Arrays.asList("home", "lift HIGH"), cancelledEvents);
```

Run:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.robots.examples.basicmechanisms.BasicAutoSoftwareScenarioTest
```

**Read the causal chain:** START admits only `home`; each successful child admits exactly one next
move; final success becomes root `SUCCESS`. When `HIGH` instead times out or the root is actively
cancelled, the same exact non-success reaches the active `HIGH` Task and `STOWED` never starts.
Because the lift chose `leaveRequestOnCancel()`, suppressing `STOWED` does not erase the persistent
`HIGH` request. Only another explicit request changes that held command; terminal Plant stop
separately ends hardware realization without rewriting the request. Separate factory calls produce
separate Task identities.

**Proves:** the production lift-only policy is cooperative, success-gated, outcome-preserving,
propagates active cancellation to its current child, and is fresh for each requested run.

**Does not prove:** the real lift produces correct evidence, reaches either height, or stops safely.

## Isolated hardware gate

Keep `BasicLiftAuto` disabled and its motion permission false. Draw the three-step timeline and
write down the reference cue, arrival evidence, timeout, and STOP response expected at each step.
Re-run the lift's isolated home and move gates, reduce first-run energy where applicable, clear the
full travel envelope, and appoint an immediate STOP operator. Only then enable the lift-only host
and permission for one supervised run. Treat any timeout as a failed run; do not continue manually
into the next step without diagnosing the missing evidence, and use FTC STOP immediately because
the last selected height request remains persistent.

**Next gate:** after the lift-only sequence is proven, add one already-proven claw through the
optional `guide(...)` capstone and repeat the same software outcome review before physical motion.
