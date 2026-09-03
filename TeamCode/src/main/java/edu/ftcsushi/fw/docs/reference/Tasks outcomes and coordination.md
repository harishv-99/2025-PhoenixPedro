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
| inspect success, timeout, cancellation, unknown, or still-running state | [`TaskOutcome`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/task/TaskOutcome.html>) |
| set or move a complete numeric capability | [`ScalarTasks`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/ScalarTasks.html>) |
| set or move named mechanism intent through its owner | [`SemanticScalarTasks`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/actuation/SemanticScalarTasks.html>) |
| run queued Tasks outside the managed host seam | [`TaskRunner`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/task/TaskRunner.html>) |

## Remember

Every Task instance is single-use. A macro method or `Supplier<Task>` creates a fresh graph each
time. `Tasks.sequence(...)` is the ordinary exact-success chain; abnormal continuation and cleanup
must be explicit. Timeouts report outcomes rather than inventing recovery policy.

Read [Tasks and Macros](<../design/Tasks & Macros Quickstart.md>) and
[Tasks and autonomous](<../getting-started/learn-sushi/Tasks and Autonomous.md>).
