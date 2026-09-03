---
tags:
  - Reference
---

# Values, sources, and bindings quick reference

## Ordinary entry points

| Need | API |
|---|---|
| expose or transform any loop value | [`Source<T>`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/core/source/Source.html>) |
| expose a numeric value | [`ScalarSource`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/core/source/ScalarSource.html>) |
| expose and combine a boolean | [`BooleanSource`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/core/source/BooleanSource.html>) |
| retain observation age and timestamp | [`TimeAwareSource<T>`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/core/source/TimeAwareSource.html>) |
| attach synchronous control meanings | [`CallbackBindings`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/input/binding/CallbackBindings.html>) |
| queue a fresh Task from an input | [`TaskBindings`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/task/TaskBindings.html>) |
| adapt FTC gamepads into stable sources | [`GamepadDevice`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/input/GamepadDevice.html>) |

## Remember

Use `Source.of(...)`, `ScalarSource.of(...)`, or `BooleanSource.of(...)` for ordinary adapters.
Bindings own the human meaning; capability setters own robot intent. Stateful sources publish at
most one successful observation for one clock cycle.

Read [Sources and Signals](<../core-concepts/Sources and Signals.md>) and
[Controls and intent](<../getting-started/learn-sushi/Controls and Intent.md>).
