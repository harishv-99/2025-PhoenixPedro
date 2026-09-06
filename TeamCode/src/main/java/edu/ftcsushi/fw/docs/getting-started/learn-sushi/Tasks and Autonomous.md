---
tags:
  - Learn
---

# Tasks and autonomous

**Question:** How does unfinished work advance, finish, or cancel across loops?

This is a concept reference. [Run one timed Auto](<../../build/Run One Timed Auto.md>) teaches the
first complete path, including the expected START and STOP observations. Reading requires no
installation, code changes, test execution, or hardware.

## A Task remembers progress

A [Task](<../Framework Overview.md#task>) is a bookmark for unfinished robot work. Each FTC loop
advances it a little and returns so the other owners can update. It is not a thread and does not
need `sleep()` or a private long-running loop.

A timed intake request illustrates the lifetime choice inside its mechanism:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/starter/capability/intake/StarterIntakeMechanism.java -->
```java
return SemanticScalarTasks.set(modeCommand, Mode.COLLECT)
        .forSeconds(durationSec)
        .then(Mode.STOPPED)
        .build();
```

The builder prepares work without starting it. When the Task starts, it selects `COLLECT` through
the mechanism's semantic command. The first loop that observes the duration has elapsed selects
`STOPPED`; active cancellation also selects that ending request. The mechanism's ordinary output
phase still owns the Plant and motor write.

In the maintained first Auto, `program.rootTask(...)` saves one fresh root during INIT. FTC START
starts it and performs the first downstream output update. The interval is `0.75` seconds from
that Task's own start, not time spent waiting in INIT. The
[full timed lesson](<../../build/Run One Timed Auto.md>) shows the declaration and exact values.

## Direct request or feedback wait?

`lift.setHeight(LOW)` changes a persistent request and returns. A fresh `lift.moveTo(LOW)` Task
makes that request when started and waits for matching cached feedback. The two calls answer
different questions: “request LOW” and “request LOW, then tell me when the completion condition
holds.” Neither directly writes the motor.

A `Task` is single-use. Call `collectForSeconds(...)`, `home()`, or `moveTo(...)` again to build
new work for a new run. A Java method reference such as `lift::home` saves a factory invocation,
not an old Task. Repeatable `TaskBindings.onRise(...)` therefore receives a factory.

## Keep outcomes and continuation explicit

| Outcome | Meaning |
| --- | --- |
| `NOT_DONE` | work is still active |
| `SUCCESS` | the documented software completion condition was met |
| `TIMEOUT` | the time budget elapsed first |
| `CANCELLED` | active work was cancelled or deliberately failed closed |
| `UNKNOWN` | no more specific terminal result is available |

Ordinary `Tasks.sequence(...)` starts its next child only after exact `SUCCESS`. A timeout
reports missing completion evidence; it does not choose recovery or grant permission to continue.
Fixed child Tasks are constructed eagerly but do not all start at once. The
[lift-sequence lesson](<../../build/First Autonomous.md>) makes success and abnormal outcomes
observable without adding route behavior.

Cancellation before start has no effect. Active cancellation is terminal and idempotent; repeated
or terminal cancellation does nothing. Direct cancellation starts no later child. A mechanism must
state what happens to its persistent request: the timed intake selects `STOPPED`, while the
[feedback lift](<../../build/Move a Referenced Lift.md>) explicitly uses
`leaveRequestOnCancel()`. Ending a Task does not bypass the source graph. FTC STOP separately
cancels work and stops the output owners.

## Exact contracts and optional composition

**Key APIs:** [`Task`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/task/Task.html>)
owns one cooperative lifetime;
[`Tasks`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/task/Tasks.html>)
composes lifetimes and outcomes;
[`SemanticScalarTasks`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/SemanticScalarTasks.html>)
writes named requests through their semantic owner.

Parallel lifetimes, explicit repair after timeout, `sequenceOnCompletion(...)`, and
`branchOnOutcome(...)` belong in [Tasks and Macros](<../../design/Tasks & Macros Quickstart.md>)
when those are the robot's actual requirements. They are not prerequisites for one timed Auto.
The exceptional `sequenceOnCompletion(...)` form can continue after a valid natural terminal
outcome while retaining the first non-success result; use it for an explicit continuation or
repair policy, not a prerequisite that must succeed. Direct cancellation still starts no later
child.
Return to [the concept index](<../Beginner's Guide.md>) for another question.
