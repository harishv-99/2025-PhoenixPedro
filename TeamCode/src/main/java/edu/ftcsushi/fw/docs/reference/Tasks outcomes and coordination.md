---
tags:
  - Reference
---

# Tasks, outcomes, and coordination quick reference

## Ordinary entry points

| Need | API |
|---|---|
| represent cooperative work over loop cycles | [`Task`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/task/Task.html>) |
| compose waits, sequences, branches, timeouts, and parallel work | [`Tasks`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/task/Tasks.html>) |
| run one short owner-chosen ending action on completion, active cancellation, or lifecycle failure | [`Tasks.withCleanup(child, cleanup)`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/task/Tasks.html#withCleanup(edu.ftcsushi.fw.task.Task,java.lang.Runnable)>) |
| inspect success, timeout, cancellation, unknown, or still-running state | [`TaskOutcome`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/task/TaskOutcome.html>) |
| set or move a complete numeric capability | [`ScalarTasks`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/ScalarTasks.html>) |
| set or move named mechanism intent through its owner | [`SemanticScalarTasks`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/SemanticScalarTasks.html>) |
| run queued Tasks outside the managed host seam | [`TaskRunner`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/task/TaskRunner.html>) |

## Remember

Every Task instance is single-use. A macro method or `Supplier<Task>` creates a fresh graph each
time. `Tasks.sequence(...)` is the ordinary exact-success chain; abnormal continuation and cleanup
must be explicit. Timeouts report outcomes rather than inventing recovery policy.

Timed Task families share one guarded lifetime: start once, advance once per clock cycle, and
retain lifecycle failures instead of repeating uncertain effects. The first update may share the
start cycle. An active recursive update does no additional work; hardware/controller reentry
rules are unchanged. A second same-cycle call also cannot reclaim a numeric or named timed request
that another owner superseded after the first update. Reclamation waits for the next cycle.

Cleanup means restoring the caller's chosen request or releasing its temporary state in the same
lifecycle call, not launching another timed operation. `withCleanup(...)` returns a `Task`, arms
cleanup after validating its start clock immediately before child start, and does nothing on
pre-start cancellation. Successful cleanup preserves the child's exact natural outcome.
A lifecycle or cleanup `RuntimeException`
remains a retained failure: later `getOutcome()` and `update(...)` rethrow it, rather than reporting
an ordinary `CANCELLED` result that could allow recovery to continue. Cleanup uses normal capability
setters; it neither owns physical stop nor proves rollback.

See [Restore a request when work ends](<../design/Tasks & Macros Quickstart.md#35-restore-a-request-when-work-ends>)
for the example and the narrow reentrant ending/inspection rules. A trailing sequence child is not
equivalent: direct cancellation and lifecycle failure skip it.

Read [Tasks and Macros](<../design/Tasks & Macros Quickstart.md>) and
[Tasks and autonomous](<../getting-started/learn-sushi/Tasks and Autonomous.md>).

## Advanced Task implementations

This section is for authors of a genuinely new behavior that existing factories cannot express.
[`AbstractTask`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/task/AbstractTask.html>)
is an implementation base, not a builder. A subclass uses `super(debugName)` to name its diagnostic
state, then supplies protected **hooks**: methods the base invokes at defined lifecycle points.
Its final public lifecycle methods cannot be replaced by the subclass. Ordinary robot code keeps
using `Tasks`, scalar builders, output factories, and route/guidance helpers.

| Implementation choice | Protected contract |
| --- | --- |
| Begin and advance behavior | `onStart(clock)` and `onUpdate(clock)` receive the owner's shared clock; no hook advances that clock |
| Choose active cancellation | Required `onCancel()` runs after ending is claimed; an intentional no-op must be explicit |
| Preserve externally completed evidence before cancelling | Optional `onBeforeCancel()` may observe the exact owned execution and call `complete(outcome)` |
| Classify an exceptional abort differently | Optional `onFailure(failure)` defaults to the cancellation policy; it does not turn the exception into a valid result |
| Restore state on every started ending | Optional `onFinish()` runs once after cancellation/failure actions; absent ending work requires no supplied cleanup action |
| Declare natural completion | Call `complete(outcome)` inside a lifecycle or `observe(...)` hook with a terminal value, never `null` or `NOT_DONE` |
| Read externally updated typed status | `observe(action)` guards a non-advancing observation independently of the update-cycle allowance; `requireOutcomeAvailable()` protects final result consumption |
| Explain cached state | `debugState(sink, prefix)` adds domain rows without polling behavior or querying the owning Task's pending/failed outcome |

The base consumes the start attempt before validation, but arms hooks only after a non-null start
clock. It claims an update cycle before invoking behavior. A repeated start rejects before effects;
an active recursive update is inert. Use `isActive()` after an external callback before issuing more
commands: cancellation can happen inside that callback. `isStarted()`, `hasFailure()` and
`checkFailure()` expose the corresponding guarded facts without another lifecycle owner.

An ending makes `isComplete()` true immediately. That prevents repeated cancellation, but does not
yet promise a consumable result. `getOutcome()` and protected result guards remain unavailable until
ending actions and the outermost synchronous callback return. Reading a pending result is itself a
retained failure, even if a callback catches it. This prevents either sequence policy from advancing
while a supposedly finished child's callback can still fail.

A lifecycle `RuntimeException` is retained before best-effort abort and finish actions. Later
failures are suppressed on that first exception; subsequent updates and outcome reads rethrow it.
Ending actions are not repeated. Diagnostic-only `RuntimeException`s are reported as unavailable
diagnostics without replacing the lifecycle result. Java `Error` is not caught or treated as a
recoverable Task failure.

Resource acquisition needs one additional owner check. If acquiring a route execution or temporary
calibration search returns normally after cancellation happened inside that call, the subclass
must classify or release that exact returned resource before its start callback returns. It must
not publish another ordinary command or rerun the whole ending policy. The narrow
`completeAfterAcquisition(outcome)` helper allows an already-terminal returned execution to refine
only a still-pending start cancellation; it cannot rewrite a settled result. The base does not
guess resource ownership or undo arbitrary callback writes after cancellation.

The shared mechanics do not unify domain policy: a wait checks its condition before its timeout,
a gated pulse times its RUN phase separately from WAIT and cooldown, and a route retains exact
execution status and hold-end behavior. An `OutputTask` still exposes its scalar proposal through
the separate, non-advancing and retryable `getOutput()` contract. Public result shapes and ordinary
construction calls remain specific to those capabilities.
