# Maintainer Notes

This document is for mentors and framework maintainers.
Most students should not need anything here to write robot code.

For the student-facing architecture, start with [`Framework Overview`](<../getting-started/Framework Overview.md>) and [`Recommended Robot Design`](<../design/Recommended Robot Design.md>).

---

## 1. Advanced notes

### 1.1 Multiple `TaskRunner`s

The recommended starting point is **one `TaskRunner` per robot**.

Use multiple runners only when it genuinely reduces complexity, for example:

- one Runner A: a long, non-interruptible sequence (like an endgame climb)
- Runner B: small, interruptible TeleOp macros

If you do this, be explicit about **ownership**:

- Which runner is allowed to command which plants/drive behaviors
- What happens when a macro is interrupted (what targets are left behind)

### 1.2 Idle behavior and safety

Phoenix does not enforce a single global notion of "safe idle".

Instead, keep it explicit in your robot logic:

- define what each mechanism should do when no macro is running
- reset targets intentionally

When it helps clarity, use plant wrappers:

- `RateLimitedPlant` to smooth target changes
- `InterlockPlant` to enforce simple safety rules

### 1.3 FTC boundary rule (for maintainers)

As a best practice, keep FTC SDK types (`com.qualcomm.*`) inside:

- `edu.ftcphoenix.fw.ftc.*` (the adapter/boundary layer)
- `edu.ftcphoenix.fw.tools.*` (testers/examples that necessarily depend on OpModes)

This keeps the student-facing building blocks (`actuation/drive/input/task/...`) easier to reason about and easier to test in isolation.
