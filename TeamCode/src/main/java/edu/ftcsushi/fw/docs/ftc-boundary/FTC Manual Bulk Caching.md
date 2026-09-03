---
tags:
  - Advanced
---

# FTC manual bulk caching

`FtcBulkCaching.manual(hardwareMap)` is an advanced, opt-in FTC-boundary owner for a robot that has
already decided to use REV/Lynx `MANUAL` bulk caching. It owns module discovery, cache-mode changes,
deduplicated Sushi-cycle invalidation attempts, and terminal cleanup inside the managed
`FtcRobotOpMode` lifecycle.

This owner is not installed by default. An ordinary program leaves the SDK-selected cache mode
untouched (`OFF` after the FTC SDK 11.1 OpMode reset). Installing the owner changes module-wide read
and failure semantics; it does not by itself prove lower latency, fewer transactions, fresher
observations, or any other performance benefit.

## Declare the one owner first

Register the returned service at the top of `configure(...)`, before any helper or constructor that
could declare another service:

```java
import edu.ftcsushi.fw.ftc.FtcBulkCaching;

@Override
protected void configure(RobotProgram program) {
    program.service(FtcBulkCaching.manual(hardwareMap)); // declare this service first

    // Declare sensing, localization, targeting, and other services afterward.
}
```

`RobotProgram` starts and updates services in declaration order and stops them in reverse order.
Literal first placement lets the cache owner select and invalidate `MANUAL` mode before dependent
services read sensors, then attempt mode restoration only after those services stop. The current
registration API cannot inspect or reserve the first slot, so this is an explicit composition-root
contract, not a runtime-enforced priority.

The factory discovers all configured `LynxModule` instances during `configure(...)`/INIT and copies
their returned order. Discovery must produce at least one module. Construction performs no mode
read, mode write, cache clear, or bulk fetch; this owner remains cache-inert until START.
The SDK does not promise a discovery order, but the owner preserves the order it receives. A
discovery `RuntimeException` escapes before a service exists or any cache operation occurs.

Install exactly one owner for the discovered module set. While it is active, no other code may call
`setBulkCachingMode(...)`, `clearBulkCache()`, or direct `getBulkData()` on those modules. Ordinary
eligible hardware getters are expected consumers of the shared packet; cache-management calls are
the conflicting operations.

## Managed lifecycle contract

The service is single-use and belongs to one `LoopClock`. It never advances or resets that clock.

| Managed state | Owner behavior |
| --- | --- |
| INIT/configuration | Discovers and copies the unique modules. It does not inspect or mutate cache state. |
| READY START | Captures every module's actual prior mode before any mutation. It then walks modules in order to set `MANUAL`, halting on the first runtime failure; only after all sets succeed does it walk them in order for the initial clears. A fully successful START owns that START cycle, so a same-cycle update does not clear again. |
| ACTIVE update | For each new, non-regressing cycle on the same clock, claims the cycle before effects and makes at most one owner-issued clear attempt per module. A successful pass makes exactly one such attempt for every module. |
| BLOCKED | The managed host does not start services. Its later stop therefore performs no cache operation. |
| STOP or managed lifecycle failure | After mutation was attempted, first attempts a final clear on every captured module, then attempts to restore every module's exact captured mode. The two global passes continue after runtime failures. |

The per-cycle bound applies only to the periodic START/update ownership pass. Terminal cleanup is a
separate final-invalidation pass and can add another owner-issued clear in that same clock cycle.
SDK internals and a forbidden competing cache caller could also clear independently, so the owner
never states a total-clear count.

START mode capture is all-or-nothing: if a mode read throws or returns null, no module is mutated,
the start attempt is consumed, and stop has no cache effect. Once a complete snapshot exists and a
mutation is attempted, failure cleanup covers every captured module, including modules not yet
mutated when the failure became visible.

An update pass is best-effort across all modules. Its first `RuntimeException` remains primary and
later failures are suppressed in encounter order. Repeating update after success in the same cycle
is a no-op; repeating it after a same-cycle failure rethrows the same failure without replaying a
clear. A later cycle may attempt again. A different clock, a regressing cycle, update before START,
duplicate START, and update after STOP are rejected before another ordinary cache effect.

Stop claims terminal state before cleanup and is idempotent. Stop before START performs no module
operation and terminalizes the single-use service. A reentrant stop during the pre-mutation START
snapshot performs no cleanup effect, and the outer START issues no later module callback. If stop is
invoked reentrantly after mutation cleanup has been claimed, cleanup is deferred until the in-flight
module callback returns, and the interrupted ordinary pass does not resume after restoration. This
prevents a later callback from mutating cache state after the prior modes have been restored. An
escaping callback failure remains primary; otherwise a cleanup failure is primary, or a named
lifecycle error reports the reentrant stop after successful cleanup. Framework-standard `Error`
behavior is not caught or relabeled. Use this owner only from the managed heartbeat; it makes no
cross-thread synchronization guarantee.

The final clear intentionally invalidates the owner's retained packet and local cache histories.
Restoring the exact prior `OFF`, `AUTO`, or `MANUAL` mode does **not** restore the packet or command
history that existed at START. It restores the mode choice only.

There is no runtime toggle, boolean enable flag, selected-module overload, or `AUTO`/`OFF` sibling.
Leave the service uninstalled when the robot has not selected this policy; STOP is the installed
owner's one lifecycle point for attempting final invalidation and restoration of the captured prior
modes.

## What the SDK packet can affect

For the FTC SDK 11.1 behavior reviewed for this boundary, the shared Lynx bulk packet backs these
eligible observations:

- digital-input and touch state;
- analog-input voltage;
- motor encoder position;
- motor velocity;
- motor busy/at-target state; and
- motor over-current status.

The following reads are specifically excluded from that packet:

- motor electrical current, which uses a separate ADC command; and
- battery voltage, which also uses a separate ADC command.

Manual bulk caching therefore does not cache or alter the motor-current acquisition used by the FTC
motor-current sources. Any effect on surrounding bus timing is unproven. I2C, distance, color,
vision, vendor, and other reads not listed as eligible above must not be assumed to share this
packet. Hardware writes do not use it.

The SDK modes and direct cache APIs have their own semantics:

- `OFF` bypasses the shared cache.
- `MANUAL` can reuse one retained module packet until invalidation.
- `AUTO` invalidates after a repeated identical SDK-tagged command, not at a Sushi cycle boundary.
- `clearBulkCache()` discards the retained packet and local command histories.
- `getBulkData()` performs its own local clear and attempts a bulk transaction in every mode; a
  normally returned real or fake response replaces the shared cache.
- Selecting `OFF` clears the cache, while a transition between `MANUAL` and `AUTO` can preserve an
  older packet and history.

Consequently, the Sushi owner promises only its own ordered invalidation attempts. It does not
promise one total SDK clear, one transaction, or even a transaction in every cycle. Another cache
API caller would invalidate those ownership statements as well as the intended packet lifetime.

## Observation and failure truth

Bulk caching is not a validity layer or a coherent physical-sampling transaction. Under SDK 11.1,
ordinary bulk NACK, interruption, and non-`ForceStopException` runtime failures can be handled inside
the SDK by installing a fake all-zero/false packet until the next invalidation. High-level hardware
getters do not expose the fake marker. With such a packet:

- position, velocity, and analog observations report zero;
- digital-input and over-current observations report false; and
- motor busy can report true because cached `isBusy()` uses the packet's `!atTarget` value.

`ForceStopException` instead escapes after the SDK's internal clear without installing a packet.
Even without a bus failure, cached `isBusy()` evaluates bulk `!atTarget` before the `OFF` path's
`RUN_TO_POSITION` guard, so selecting caching can change an observable result.

Writes neither consume nor invalidate the retained packet. A read after a write can therefore still
report pre-write cached state until invalidation and a later eligible read replaces the packet.
Neither this service nor a successful getter establishes freshness, validity, physical coherence,
device success, transaction count, loop timing, or correctness for a robot decision.

This module-level cache is also separate from Sushi
[`Source` memoization](<../core-concepts/Sources and Signals.md#rev-bulk-caching-is-a-separate-ftc-boundary>).
Source memoization publishes one successful source result per `LoopClock` cycle; it does not manage
an SDK packet or repair any fake/stale value the SDK returned.

## Keep explicit-snapshot diagnostics separate

The advanced motor-power/encoder diagnostic calls `getBulkData()` to obtain and decode one explicit
module snapshot. That operation clears the same shared cache before its transaction attempt and, on
a normal real or fake return, replaces it. It is therefore incompatible with this lifecycle owner.
Run the diagnostic as a separate tester OpMode with no installed manual owner; do not use it to
validate coexistence. See
[`Robot calibration tutorials`](<../testing-calibration/Robot Calibration Tutorials.md#high-resolution-external-encoder-velocity-comparison>).

Raw OpModes, `FtcTeleOpTesterOpMode`, legacy/vendor tuning code, and FTC samples do not participate
in `RobotProgram`'s ordered service phase. They cannot gain this lifecycle contract merely by
calling the factory, and they are not templates for a second cache owner. Audit opaque vendor code
for direct cache-management calls before adoption.

## Validate an adopting robot

Deterministic software tests can establish discovery, lifecycle ordering, owner-issued effects,
failure propagation, and final-invalidation/prior-mode-restoration attempts. They cannot establish
that manual caching is useful on a particular robot.

Before adoption, measure the actual module set, firmware and vendor interactions, eligible read mix,
loop timing, and bus traffic. Exercise stale-after-write cases, fake-packet and fault behavior, and
every affected sensor/control decision. Compare those results with leaving the SDK-selected mode
untouched. Do not describe the owner as faster, fresher, safer, or transaction-reducing unless that
claim is supported by the adopting robot's evidence.

[Back to the FTC boundary reference](<README.md>)
