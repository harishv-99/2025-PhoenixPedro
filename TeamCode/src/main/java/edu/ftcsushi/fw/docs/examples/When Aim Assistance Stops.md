---
tags:
  - Advanced
---

# Know when aim assistance stops

**Outcome:** distinguish alignment from failed assistance, then deliberately choose when to try
again. **Before this page:** [one bounded vision pickup](<One Bounded Vision Pickup.md>) explains
the existing policy, configuration, manual drive connection, and capture evidence. This optional
lesson adds driver feedback to that same independent example. Reading needs no robot or controller.

## Not turning does not mean aligned

A robot can stop turning because it faces the target, because the camera lost the target, or
because no usable position estimate remains. The driver should not have to guess which happened.
Even a nonzero turn command cannot prove that the physical robot moved.

**Alignment** means the current accepted evidence puts the heading error within the configured
tolerance. It does not mean that a ball was captured, a shot scored, or the estimate is physically
accurate. **Fallback** means selecting the already-configured manual drive source in TeleOp,
or zero idle intent in Auto, instead of the assisted command.

| Reported state | Meaning | Driver cue |
| --- | --- | --- |
| `AIMING` | Valid evidence says further heading correction is needed | Persistent text |
| `ALIGNED` | Valid evidence places heading within the chosen tolerance; assistance remains active | One short pulse the first time this aim session aligns |
| `LOST` | Required evidence was rejected/lost, or a bounded pickup failed naturally; not successful completion | One longer pulse and a retained explanation |
| `IDLE` | No assisted action owns drive; use the supplied manual/zero source | No failure pulse |

`REQUESTED` briefly means a new aim request is waiting for its first Services-phase check, not
that alignment was established. `PICKUP` identifies the bounded pickup's drive ownership.
`STOPPED` means the example cannot be restarted; the presenter distinguishes a lifecycle failure
from normal STOP. These are named Java `enum` values: a fixed set of meanings rather than numbers
the student has to memorize.

Held aim remains active after alignment. If the accepted heading error grows, it can correct
again without another short pulse. Loss immediately replaces the aligned indication; the old
alignment is never retained as current success. The existing `headingToleranceRad` supplies the
alignment tolerance as well as pickup heading checks. The software fixture uses `0.10` radians
(about `5.7°`), not a universal recommended accuracy. The guidance deadband is separate: it is the
small error range where the controller requests no correction. An aligned indication does not
promise an exactly zero turn command.

## Let the driver choose when assistance returns

**Reacquisition** here means making a new request to use acceptable current evidence. It does not
mean proving that a newly seen unlabeled ball is the same physical ball.

Suppose aim is held, the robot aligns, and then the target disappears:

| Illustrative serviced time (seconds) | Aim input | Evidence supplied | Selected behavior and cue |
| --- | --- | --- | --- |
| `0.00` | Press | Available | Request recorded; this loop has not yet checked it |
| `0.05` | Held | Valid and within heading tolerance | Aligned, short pulse |
| `0.10` | Held | No eligible target | Manual fallback, loss explanation, long pulse |
| `0.15` | Held | Valid target returns | Still manual; returning evidence cannot authorize steering |
| `0.20` | Release | Available | Manual, no failure pulse |
| `0.25` | Press again | Available | New request; old aim command cannot be reused |
| `0.30` | Held | Valid and within tolerance | New session aligned, one short pulse |

Each row is a software observation at a serviced instant, not a claim about motion or evidence
between samples. The important sequence is **loss → manual → release/repress → new check**.
There is no automatic resume or timer that eventually takes steering back from the driver.

An initial aim request without usable evidence is also rejected, not saved until the camera
recovers. Releasing driver override while aim stays held does not count as a new press. Starting
pickup gives that attempt drive ownership and invalidates prior aim permission; completing or
cancelling it does not silently restore the old held aim.

Pickup presses retain their own permission too. A released request waiting in the managed Task
queue cannot borrow permission from a newer press. A press while overridden is ignored. Controls
still use the original three inputs; no acknowledgement button or second queue is needed.

## Choose evidence requirements explicitly

**Evidence age** answers how old the incorporated observation is. A localization **quality score**
is the estimator's rating, not a measured probability that its position is correct. Check both:
a high score does not make an old pose current, and fresh robot pose evidence does not refresh an
old target sighting.

The existing motion-disabled `VisionPickup.Config.defaults()` leaves the new minimum score unset.
Alongside the measured geometry and reviewed bounds from the prerequisite, author this required
setting before enabling motion:

```java
reviewedConfig.minPoseQuality = reviewedMinimumPoseQuality;
```

Choose a finite value from `0` to `1` for the particular estimator and action. Explicit `0` means
no positive score floor; malformed published scores are still rejected. The supplied software
scenario uses `0.50`, with maximum pose age `0.10 s` and maximum target age `0.20 s`. Change these
assignments in your robot's configuration, not the running owner's copied configuration. The
example applies the same pose gate during aim and every moving pickup phase. It honors a stricter
target selector age limit too.

Missing AprilTag corrections do not necessarily mean localization is unusable. Fresh movement
evidence can still support it. Likewise, the existing localizer may reject an implausible camera
correction and retain usable prediction. This example checks the final pose rather than inventing
a second sensor-voting policy. Its message can truthfully say “pose stale” or “quality below the
configured minimum”; it cannot diagnose a broken odometry wheel from that snapshot.

The real-Fusion software scenarios separately author predictor loss, correction loss, and a
correction exceeding a named position-jump limit. Their injected inputs explain the cause in that
experiment. A rejection count alone does not reveal its cause, and two agreeing sensors may share
the same error. The [localization guide](<../drive-vision/AprilTag Localization & Fixed Layouts.md#7-correctedglobal-localization-and-latency-compensation>)
explains those evidence boundaries.

## Add pulses and persistent text to the existing robot graph

**Haptic feedback** means vibration felt through a controller. Sushi's
[`HapticSink`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/haptic/HapticSink.html>)
accepts short pulse requests; it does not decide which robot event matters.
The example's [`VisionPickupFeedback`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/visionpickup/VisionPickupFeedback.html>)
owns that decision and the recipient's stop lifecycle. Its public example constants are:

| Constant | Value | Meaning |
| --- | --- | --- |
| `PULSE_STRENGTH` | `1.0` | Full normalized requested strength |
| `ALIGNED_PULSE_SEC` | `0.10` seconds | First observed alignment in this aim session |
| `LOST_PULSE_SEC` | `0.50` seconds | A newly reported rejected/lost assist |

These distinguishable-duration requests are illustrative policy, not a controller guarantee.
Change the constants in your own feedback owner if supervised controller checks justify other
values. Repeated unchanged status, heading-tolerance chatter, intentional release, override, and
STOP do not repeatedly buzz. If loss and alignment become visible together, the loss cue wins;
there is no delayed success buzz for that same session.
An intentional release or override that has already returned the final observed state to `IDLE`
suppresses a pending loss pulse from earlier in that loop; the display and cue agree on the handoff.

In the existing TeleOp composition root, after its pickup service, controls, intake output, and
one drive declaration, add:

```java
program.output(new VisionPickupFeedback(pickup, FtcHaptics.gamepad(gamepad1)));
program.presenter(new VisionPickupPresenter(pickup));
```

`new` constructs one owner during setup, not each loop. `FtcHaptics.gamepad(gamepad1)` chooses the
driver's existing controller as the recipient. The output registration gives the managed program
its update and STOP responsibilities. Do not also command this same recipient from a competing
feedback owner. [`VisionPickupPresenter`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/robots/examples/visionpickup/VisionPickupPresenter.html>)
formats cached state and reasons for the Driver Station; it does not sample the camera, pulse,
change requests, or commit telemetry. These two lines extend the prerequisite's robot graph;
they are not a complete hardware OpMode or permission to enable motion.

The complete relevant order is:

`Clock → camera/localization/history Services → pickup Service → Bindings → Tasks → intake/drive Outputs → feedback Output → Presenters → one telemetry commit`

There is one clock and one drive writer. Feedback observes the latest published decision after
current-loop control cancellation and Task work. Its once-per-cycle guard prevents duplicate pulse
requests. It retains an unexpected output failure instead of retrying an uncertain effect.
Managed STOP best-effort stops the drive, feedback recipient and remaining owners even if another
cleanup fails. The pickup does not stop borrowed cameras or localizers.

Auto uses the same pickup Task with zero idle drive as described in the prerequisite. The presenter
can show its bounded result without adding TeleOp controls; controller vibration is a driver-facing
option, not an Auto completion condition. The display keeps the last pickup result separate from
current aim state. A lifecycle exception is visibly a failure with no normal outcome, not an
apparently successful finish or an ordinary cancellation that can release recovery.

!!! warning "Warning: silence is not successful alignment"

    A controller may not support vibration or may not receive a requested pulse. FTC retains only
    the latest undelivered request; two immediate pulses do not reliably make a double buzz.
    Keep the state and reason text visible. Neither a returning pulse call nor silence proves
    the driver received a cue.

## Check the software, then the driver experience

**Question:** can the same real example distinguish alignment, loss, and deliberate recovery?
**Keep real:** pickup policy, selection/history/guidance, controls, Task lifecycle, feedback output
and presenter. **Replace:** physical camera/localization/capture readings, operator inputs,
drive/intake effects, controller pulses and telemetry with explicitly recorded software fixtures.
**Observe:** selected drive intent, immutable status, Task results, formatted rows and requested
pulse durations. **Cannot conclude:** actual movement, camera accuracy, capture, vibration delivery
or how clearly the driver distinguishes the two durations.

The supplied maintainer regressions cover returning targets while aim remains held, independent
pose/target freshness, rejected correction evidence, old queued gestures, callback failure,
cancellation, STOP and pulse deduplication. `VisionPickupTestRig.configured()` is the complete
synthetic setup; the [prerequisite's configuration table](<One Bounded Vision Pickup.md#software-fixture-values>)
makes its effective values visible. These tests run the real example, not a second implementation
of its recovery policy. Use the [normal software verification command](<../maintainers/Maintainer Notes.md#16-automated-framework-verification>)
or the test-class gutter in Android Studio; neither requires a connected robot.

**Complete source:** [pickup policy](<../../../robots/examples/visionpickup/VisionPickup.java>),
[controls](<../../../robots/examples/visionpickup/VisionPickupControls.java>),
[feedback output](<../../../robots/examples/visionpickup/VisionPickupFeedback.java>), and
[presenter](<../../../robots/examples/visionpickup/VisionPickupPresenter.java>).
**Complete test source:** [evidence checks](<../../../../../../../test/java/edu/ftcsushi/robots/examples/visionpickup/VisionPickupEvidenceTest.java>),
[localization scenarios](<../../../../../../../test/java/edu/ftcsushi/robots/examples/visionpickup/VisionPickupLocalizationScenarioTest.java>),
[controls](<../../../../../../../test/java/edu/ftcsushi/robots/examples/visionpickup/VisionPickupControlsTest.java>),
[failure cases](<../../../../../../../test/java/edu/ftcsushi/robots/examples/visionpickup/VisionPickupFailureTest.java>),
[feedback/display checks](<../../../../../../../test/java/edu/ftcsushi/robots/examples/visionpickup/VisionPickupFeedbackTest.java>), and
[shared software setup](<../../../../../../../test/java/edu/ftcsushi/robots/examples/visionpickup/VisionPickupTestRig.java>).

**Next gate:** first verify the two pulse durations on the actual controller without enabling robot
motion. This needs your adopting robot's reviewed stationary test setup: it must request the two
documented haptic durations and show their meanings without registering any drive or mechanism
actuation. No hardware tester is supplied here; leave this gate pending until that setup exists.
Do not enable the synthetic pickup configuration just to produce an alignment cue. The driver must
identify alignment versus loss without guessing, while another observer checks the visible
state/reason. Then use the prerequisite's supervised motion-adoption gates:
test evidence loss, manual takeover, re-press, and physical STOP with reviewed geometry and limits.
Defaults remain motion-disabled; no software test substitutes for these checks.
