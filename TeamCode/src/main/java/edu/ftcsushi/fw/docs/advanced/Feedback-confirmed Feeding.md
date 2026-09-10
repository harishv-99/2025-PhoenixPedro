---
tags:
  - Advanced
---

# What evidence permits and ends one feed?

A wheel-speed request does not prove that both wheels reached speed. A feed command does not prove
that an object moved. This optional lesson connects those two decisions without calling either one
a scored shot.

**Before this page:** read [paired flywheel velocity](<Paired Flywheel Velocity.md>) for independent
wheel readiness, [Read a switch](<../build/Read a Switch.md>) for sampled sensor conditioning, and
[one timed Auto](<../build/Run One Timed Auto.md>) for fresh Tasks and the managed output phase.
Reading requires no hardware. The complete fixture adds two flywheel motors, a transfer CR servo,
a release servo, and three digital position sensors; its FTC hosts are disabled and motion-locked.

**Completion evidence:** explain why the attempt waits, which later sensor observation can confirm
departure, and why an uncertain attempt cannot automatically retry. Supplied software scenarios
check that policy; they do not validate a physical shooter.

## Start with one staged object

The **staged position** is the fixture's first inventory sensor: the place an object waits before
feeding. The other two sensors describe positions farther back. Its **ordered-fill assumption**
expects the first position to fill before the second, and the second before the third. A gap before
feeding makes the observation unsuitable for this policy; it is not a universal jam diagnosis.
Gaps during movement are expected possibilities and do not, by themselves, cancel a feed.

**Settling** means that several eligible software samples keep meeting the prerequisites across a
chosen interval. Here those prerequisites are a consistent inventory with the staged position
occupied and both wheels independently ready for this attempt's requested speed. A single ready
sample, or a good average of one fast and one slow wheel, is insufficient.

The launcher privately owns its inventory sampler, paired-wheel owner, release Plant, and transfer
Plant. Register only the launcher as the program's output. Do not separately register or update
those children. The managed loop makes decisions in Tasks, then realizes requests and publishes
observations in Outputs. A Task normally reads the previous output cycle's evidence, subject to
the configured age limit; it does not force a new hardware read.

## Make the evidence rules visible

The ordinary construction is one
[`ReferenceLauncherMechanism`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/launcher/ReferenceLauncherMechanism.html>)
with `HardwareMap` and its data-only `Config`. The constructor copies and validates that
configuration. These are the effective **illustrative software defaults**, not recommended physical
settings. Change the named fields on the team's configuration before constructing its owner.

| Field | Default | What the example means by it |
| --- | --- | --- |
| `feedVelocityTicksPerSec` | `3000.0` | One requested encoder speed for both wheels. |
| `flywheels.velocityToleranceTicksPerSec` | `100.0` | Each wheel must independently be within this difference from the matching request. |
| `flywheels.maximumVelocityTicksPerSec` | `5000.0` | Software command ceiling, not a tested safe speed. |
| `spinUpTimeoutSec` | `2.0` | Maximum wait for all pre-feed prerequisites, not just wheel speed. |
| `readySettlingSec` | `0.10` | Span of advancing eligible samples required before release. |
| `evidenceMaxAgeSec` | `0.10` | Maximum permitted observation age or gap; an excessive gap breaks settling before feed and ends an in-progress feed. |
| `inventory.occupiedDebounceSec`, `inventory.vacatedDebounceSec` | `0.02`, `0.02` | Sampled delays before publishing occupied or vacant. They reject short observed changes, not unobserved physical events. |
| `releaseDurationSec` | `0.15` | Bounded release-command interval. |
| `transferPower`, `transferDurationSec` | `0.25`, `0.20` | Normalized transfer command and its bounded interval. |
| `departureTimeoutSec` | `0.50` | Overall departure-confirmation budget beginning at release start. |
| `releaseRetractedNativePosition`, `releaseExtendedNativePosition` | `0.25`, `0.60` | FTC servo command endpoints mapped from the release Plant's retracted `0.0` and extended `1.0`; not shaft feedback. |

Default hardware names are `flywheelLeft`, `flywheelRight`, `transfer`, `release`, `inventoryFirst`,
`inventorySecond`, and `inventoryThird`. Wheel directions are respectively `FORWARD` and `REVERSE`;
transfer and release use `FORWARD`. Inventory inputs are active-low: an electrical low is interpreted
as occupied. These names, signs, sensor placements, endpoints, and timing decisions must match the
adopting robot. Native servo values inside `[0, 1]` are not automatically safe travel limits.

**Freshness** means the observation belongs to this clock lifetime, is recent enough, and advances
through successful output publications. Wheel evidence also belongs to a specific successful
request occurrence: requesting the same number again is still a new request. Re-reading one status,
calling update twice in a cycle, or changing away and back to the same speed cannot accumulate
settling evidence. These timestamps record software sampling, not independent new motor-controller
frames or continuous physical dwell between samples.

A different wheel request during settling discards the accumulated interval; matching fresh
evidence must establish a new one. Once feeding has begun, a changed request ends this attempt
instead of silently adapting its speed mid-feed.

## Observe departure after output realization

**Observed departure** means an eligible occupied observation at or after successful release-output
realization is followed by a strictly later eligible conditioned-vacant observation. A vacant sensor
before release is not confirmation. **Conditioned** means the sensor's sampled debounce rule has
accepted the change, as explained in the switch lesson; it does not identify or track a projectile.

![Illustrative sampled timeline: readiness settles before release; staged occupancy is observed after release realization and vacancy later; bounded command phases must also finish.](<../assets/diagrams/feedback-confirmed-feeding.svg>)

On a small screen, open the [full-size timeline](<../assets/diagrams/feedback-confirmed-feeding.svg>)
to enlarge the labels. The same sequence is explained below.

The diagram is an authored explanation, not a recording or a physics simulation. Time is relative
to the release request. Ready samples at `-0.15`, `-0.10`, and `-0.05` seconds span the `0.10`-second
settling interval. The next decision requests release at `0.00`; that cycle's output realizes it
and samples the still-occupied stage. An illustrative electrical change at `0.05` is published
as conditioned vacancy at the `0.10` sample, after the `0.02`-second sampled delay. The release
and transfer commands still run their own `0.15`- and `0.20`-second phases. A subsequent decision
can finish only after both timed phases succeed and the departure observation has been accepted.

Dots represent software observations only. Bars describe requested command phases, not continuous
physical motion. The example still checks wheel and observation eligibility while feeding;
it does not ignore a later speed drop simply because vacancy was already observed. All deadlines
are cooperative: a decision happens in an actual robot loop, not at an exact wall-clock interrupt.

The result is deliberately narrow: `TaskOutcome.SUCCESS` with reason `DEPARTURE_OBSERVED` says that
the bounded phases succeeded and the staged sensor supplied the required sequence. An object could
have moved for another reason, or failed to launch after leaving that position. Scoring remains a
different observation.

## Use the same attempt in TeleOp and Auto

[`ReferenceLauncher`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/capability/launcher/ReferenceLauncher.html>)
is the mode-neutral capability: `feedOne()` constructs one fresh Task without moving hardware;
starting that Task claims an attempt and requests speed. The caller chooses when to start work;
the launcher keeps the evidence, phase, deadline, and ending policy in one place.

### TeleOp: one press, no feed backlog

The complete TeleOp checks its false motion-review lock **before** this construction excerpt:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/opmode/ReferenceFeedingTeleOp.java -->
```java
ReferenceLauncherMechanism.Config config = ReferenceLauncherMechanism.Config.defaults();
ReferenceLauncherMechanism launcher = program.output(
        new ReferenceLauncherMechanism(hardwareMap, config));
ReferenceFeedingControls controls = new ReferenceFeedingControls(new GamepadDevice(gamepad1));
controls.bind(program.callbackBindings(), program.taskBindings(), launcher);
```

`program.output(...)` registers the one owner for later update and STOP; it does not run its
feeding behavior during setup.
[`ReferenceFeedingControls`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/reference/control/ReferenceFeedingControls.html>)
registers these later button actions through `bind(...)`:

A asks for one feed and B aborts it. X requests **recovery acknowledgement**: removing the software
block after an uncertain feed when the required evidence permits it. X commands no recovery motion,
never resumes old work, and returns an acceptance or rejection reason.

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/control/ReferenceFeedingControls.java -->
```java
tasks.onRise(operator.a(), () -> createFeedUnlessPending(launcher));
callbacks.onRise(operator.b(), launcher::abortFeedAttempts);
callbacks.onRise(operator.x(),
        () -> lastRecoveryResult = launcher.acknowledgeRecovery());
```

The `() -> ...` functions are saved during setup and called synchronously on a later button rise,
not on a new thread. `launcher::abortFeedAttempts` similarly means "call this method later."
A uses the managed Task binding; B and X are immediate, short capability calls in the binding phase.

The controls remember the last **created** feed Task, including time spent waiting in the program's
queue. Checking only the launcher's active flag would miss that pending work. The private factory is:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/control/ReferenceFeedingControls.java -->
```java
if (lastFeed != null && !lastFeed.isComplete()) {
    return Tasks.noop();
}
lastFeed = launcher.feedOne();
return lastFeed;
```

`Tasks.noop()` is an already-complete no-action Task, so an extra press creates no additional feed
attempt. The original A signal still detects the press: holding A through completion cannot
manufacture a new press. B invalidates previously constructed attempts, including pending ones;
it does not access or clear the private program runner. X never resumes old work. A new explicit
press after the prior Task ends asks for a new attempt.

### Auto: declare one bounded attempt

After the same motion gate and output construction, the Auto uses:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/opmode/ReferenceFeedingAuto.java -->
```java
Task feed = launcher.feedOne();
program.rootTask(feed);
```

Construction runs in configuration without feeding. The managed program starts this Task at FTC
START, updates it before the launcher output, and cancels it before stopping the output at FTC STOP.
This Auto attempts once: it neither retries nor acknowledges recovery automatically. Its presenter
shows `feed.getOutcome()` beside the cached launcher phase and reason. Before completion the Task
reports `NOT_DONE`; a lifecycle exception is a failure, not a normal unsuccessful result to ignore.

For a larger Auto, exact-success `Tasks.sequence(...)` can place subsequent fresh work after this
attempt; unsuccessful feeding must not silently authorize dependent actions. See
[Tasks and Macros](<../design/Tasks & Macros Quickstart.md#31-choose-how-a-sequence-advances>) for
explicit outcome branches. Deciding to retry, park, or abandon scoring remains robot strategy.

## Acknowledge uncertainty without commanding recovery motion

**Recovery acknowledgement** is an explicit request to re-arm the software after an uncertain feed.
It is not a reverse pulse, re-home, jam-clear routine, or automatic retry. This fixture requests
idle when its owned attempt ends: zero wheel velocity, zero transfer power, and retracted release
through the same source-driven output path. Requesting those values is not physical stop evidence,
and retracting halfway through a feed is not known safe for an unspecified mechanism.

| Observation or ending | Software decision | What the operator must not infer |
| --- | --- | --- |
| No stage, inconsistent inventory, fleeting readiness, or broken settling | Wait only within the prerequisite budget; otherwise `PREREQUISITE_TIMEOUT`. | A longer timer would fix the physical cause. |
| Wheel droop or loss of eligible evidence during feeding | End the attempt, request idle, and require acknowledgement. | Ordinary load-induced droop is necessarily a fault on another shooter. |
| No eligible departure before the bound | Retain `DEPARTURE_TIMEOUT`, request idle, and require acknowledgement. | The object stayed put, or a jam was cleared by idle. |
| Abort or interruption after feeding began | Invalidate old work, request idle, and require acknowledgement. | It is safe to restart immediately. |
| Acknowledgement with active work, missing fresh idle publication, or inconsistent/unavailable inventory | Reject it with an actionable `RecoveryResult`. | Pressing X always makes the system ready. |
| `ACKNOWLEDGED` | Clear the software recovery block only; a new attempt checks staging and settling again. | Wheels physically stopped, a jam cleared, or an old attempt resumed. |

Acknowledgement requires new post-failure publications of the idle command and consistent inventory.
It reads cached observations against the owner's clock; it does not poll sensors. A request change,
clock reset, abort, or STOP cannot let an older attempt command over a replacement. STOP is terminal:
the mechanism cannot restart itself from a later update.

## Check the software; keep the hardware gate separate

**Question:** does this real example refuse unsupported feeding and finish only from the declared
sampled evidence? **Keep real:** launcher, paired-wheel and inventory owners, Plants, Tasks, copied
configuration, and Task-before-output order. **Replace:** only FTC devices with independent authored
electrical and velocity readings. **Observe:** submitted outputs, request/sample identities,
attempt outcomes, retained reasons, and no-motion acknowledgement. **Cannot conclude:** physical
causation, safe endpoints or interruption, sensor placement, launch, jam clearance, or scoring.

The supplied launcher software scenarios exercise these boundaries, including opposite wheel
errors, fleeting readiness, stale evidence, missing departure, cancellation, and recovery.
Read or run them as maintainer evidence; the software devices contain no ball or flywheel physics.
The [optional software setup and test instructions](<../getting-started/Build and Run.md>) explain
how to run tests on the development computer. The complete framework test suite includes these
classes; running it does not run either disabled FTC OpMode.

One small source-backed checkpoint asks whether finished commands are enough. Its real owner has
already completed release and transfer, but the staged input remains occupied and the Task remains
`NOT_DONE`. In this separate software fixture, `s` holds the real launcher and outside device probes;
`advance` runs Task then output once per `0.03125`-second cycle. It deliberately uses `1000` ticks/s,
`0.125`-second settling and release, `0.25`-second transfer and maximum evidence age, a
`0.75`-second departure budget, and zero sensor debounce. These are authored test values, not the
defaults used to explain the diagram above.

<!-- source-excerpt: TeamCode/src/test/java/edu/ftcsushi/robots/examples/reference/capability/launcher/ReferenceLauncherSoftwareScenarioTest.java -->
```java
s.staged.setHigh(true);
s.advance(feed, 1); // Output publishes the new observation after this cycle's Task.
assertFalse(feed.isComplete());
s.advance(feed, 1); // The next Task phase consumes that later observation.
assertEquals(TaskOutcome.SUCCESS, feed.getOutcome());
assertEquals(ReferenceLauncher.Reason.DEPARTURE_OBSERVED, s.launcher.status().reason());
```

**Read the causal chain:** setting the outside electrical reading does not complete the Task.
The output first publishes that observation; only the next Task phase can consume it and finish.
`assertFalse` checks that completion has not happened yet; `assertEquals(expected, actual)` checks
the expected result against the Task or status. Zero debounce isolates publication order here;
separate regression cases check the conditioned delays and stale observations.

**Reading checkpoint:** predict the result when vacancy arrives before release, when a wheel slows
during transfer, and when X is pressed before an idle output publication. In each case, explain
which evidence is missing rather than treating an elapsed timer as proof of completion.

**Proves:** passing scenarios establish the tested software contract for independently supplied
samples. **Does not prove:** the matching physical observations will occur on the team's robot.
**Next gate:** use [Actuator bring-up](<../testing-calibration/Actuator Bring-up.md>) and a reviewed
[subsystem lab card](<../examples/Subsystem Experiments.md#copyable-lab-card-and-results-sheet>) to
establish individual devices, then separately validate staged placement, loaded-wheel behavior,
containment, safe mid-feed interruption/retraction, and emergency STOP. Leave both motion locks
false and both `@Disabled` annotations in place until that adopting-robot review authorizes them.

The existing Reference spin-up experiment and its downloadable report still answer only the paired
wheel spin-up question. They do not execute this feed, impose its settling policy, or prove departure.
Enabling that tester does not enable these two independently locked mode clients.

## Complete source

- [Complete source: `ReferenceLauncher.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/capability/launcher/ReferenceLauncher.java>)
- [Complete source: `ReferenceLauncherMechanism.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/capability/launcher/ReferenceLauncherMechanism.java>)
- [Complete source: `ReferenceFeedingControls.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/control/ReferenceFeedingControls.java>)
- [Complete source: `ReferenceFeedingTeleOp.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/opmode/ReferenceFeedingTeleOp.java>)
- [Complete source: `ReferenceFeedingAuto.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/opmode/ReferenceFeedingAuto.java>)
- [Complete source: `ReferenceLauncherSoftwareScenarioTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/reference/capability/launcher/ReferenceLauncherSoftwareScenarioTest.java>)
- [Complete source: `ReferenceLauncherMechanismTest.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/test/java/edu/ftcsushi/robots/examples/reference/capability/launcher/ReferenceLauncherMechanismTest.java>)

[Choose another advanced question](<README.md>)
