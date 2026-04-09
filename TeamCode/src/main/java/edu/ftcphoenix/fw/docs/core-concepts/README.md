# Core concepts

This section covers the Phoenix loop contract and the generic signal model.

## Read in this order

1. [`Loop Structure.md`](<Loop Structure.md>)
2. [`Sources and Signals.md`](<Sources and Signals.md>)

## Use this section when

- you are composing `Source<T>`, `ScalarSource`, or `BooleanSource` graphs
- you need to reason about memoization, debounce, hysteresis, edges, or hold-last behavior
- you are deciding between reset-by-signal (`accumulateUntil(...)`) and lifecycle reset (`reset()`)

## Pair with

- [`../design/Recommended Robot Design.md`](<../design/Recommended Robot Design.md>)
- [`../ftc-boundary/FTC Sensors.md`](<../ftc-boundary/FTC Sensors.md>)
- [`../../Framework Principles.md`](<../../Framework Principles.md>)
