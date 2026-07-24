# Framework Principles

This document explains the design principles behind the Phoenix framework: **why the APIs look the way they do**, and the usage patterns the framework is optimized for.

If you want to *get running* first, read:

1. [`Beginner’s Guide`](<docs/getting-started/Beginner's Guide.md>) – loop shape + how to wire Plants.
2. [`Tasks & Macros Quickstart`](<docs/design/Tasks & Macros Quickstart.md>) – how to build macros using factory helpers.
3. [`Shooter Case Study & Examples Walkthrough`](<docs/examples/Shooter Case Study & Examples Walkthrough.md>) – a complete example tied to real code.

---

## 1. High-level goals

Phoenix is designed around a few core goals:

1. **Non-blocking by design**

   No `sleep(...)`, no long `while (...)` loops inside TeleOp/Auto. Anything that takes time is expressed as a **Task** that advances once per loop.

2. **Clear separation of concerns**

   Robot code is written in terms of a few narrow building blocks:

    * **Drive behavior**: `DriveSource` → `DriveSignal` → `DriveCommandSink` (for example, `MecanumDrivebase`)
    * **Mechanisms**: `Plant`
    * **Behavior over time**: `Task` / `TaskRunner`

   FTC SDK specifics live in the `fw.ftc` boundary layer (and `fw.tools`), not in your robot logic.

   Most yearly robots will also define a robot-owned **capabilities** layer between their mode
   clients and their internals. That concept is documented in
   [`Robot Capabilities & Mode Clients`](<docs/design/Robot Capabilities & Mode Clients.md>). It is
   a normal robot pattern, but usually not a framework lane.

   Vision keeps backend ownership explicit. A webcam owner declares its complete processor set
   when it builds one `VisionPortal`, then enables or disables those processors. A Limelight owner
   controls one selected onboard pipeline, confirms that a post-request result belongs to that
   pipeline, and refuses stale or unconfirmed results. These are parallel lifecycle owners, not one
   artificial universal-camera interface. Robot code may put a small semantic interface above
   either realization so Auto and TeleOp ask for meanings such as `AIMING` and consume the same
   robot-owned immutable, timestamped snapshot without importing FTC or vendor result types.
   The acquisition owner also owns frame/result identity: it anchors an age-native vendor result
   to one `LoopTimestamp` once and returns that exact timestamp while the vendor identity repeats.
   Consumers never reconstruct `now - cachedAge`, and a retained pre-reset identity stays
   unavailable until the owner observes a genuinely new frame/result in the current clock epoch.

3. **Beginner-friendly, mentor-powerful**

   Students primarily use:

    * `FtcActuators.plant(hardwareMap) ... build()` (to create Plants)
    * `PlantTasks` and `Tasks` (to create ordinary Tasks)
    * `DriveTasks.driveExclusivelyForSeconds(...)` only for simple Auto/test movement where that
      Task is the sole behavior-command writer for the drive sink

   Mentors can go deeper (HAL, adapters, custom Tasks) when needed.

4. **Composable building blocks**

   Drive logic, plants, and tasks are intentionally decoupled so you can swap components:

    * Use `GamepadDriveSource` today, add a `DriveGuidance` overlay tomorrow.
    * Keep the same shooter macro while changing how distance is estimated.
    * Keep optional third-party route integrations in library-specific edge packages
      (for example `fw/integrations/pedro`) so framework core continues to depend only on
      Phoenix seams like `DriveCommandSink` and `RouteFollower`.

5. **SOLID where practical**

   Phoenix should adhere to SOLID programming principles to the extent practical.

   * Prefer cohesive abstractions with a single clear job.
   * Prefer interface segregation and dependency inversion when they make code easier to understand, test, and compose.
   * Do **not** split APIs into tiny capability interfaces just to satisfy SOLID mechanically if that makes the common path harder to learn or use.
   * Lifecycle hooks on core interfaces are acceptable when they are genuinely part of the same abstraction. For normal runtime behavior, prefer explicit signals / events in the source graph over out-of-band imperative calls when practical.


6. **One loop, one heartbeat**

   Phoenix assumes a single loop heartbeat (`LoopClock`) that everything else uses.
   This enables robust button edge detection, predictable task timing, and consistent rate limiting.
   Treat `cycle()` as a per-cycle identity, not an elapsed-loop count. It remains monotonic for one
   `LoopClock` lifetime and advances immediately at an explicit reset so post-reset reads cannot
   alias pre-reset caches. A captured measurement that crosses component boundaries carries one
   clock-created `LoopTimestamp`, not independently passable timestamp and epoch scalars. The value
   owns clock identity, reset-epoch invalidation, freshness, and pairwise comparisons; consumers do
   not reconstruct those rules.
   When a third-party follower's supported lifecycle requires updates beyond active route/guidance
   Tasks, it also needs one stable composition-root heartbeat every relevant OpMode loop; those
   Tasks may select behavior but must not become its only lifecycle owner. If both layers can reach
   the same update hook, the adapter must deduplicate by `clock.cycle()` and count any vendor-hidden
   update as that cycle's one heartbeat.

   A route start also owns one execution identity. Its status and cancellation stay attached to
   that exact start even if a newer route replaces it. A vendor's idle or not-busy flag is not proof
   of success: the integration boundary must classify and retain normal endpoint completion,
   follower timeout/stall, interruption, replacement, failure, and unknown terminal state while the
   evidence is still observable.

   The integration reports those route facts; the robot routine owns what they mean for strategy.
   Generic Task sequences do not silently short-circuit or choose recovery. Gate position-dependent
   work on the retained route result, and state whether each non-normal result continues, starts an
   explicitly selected fallback, or aborts. A conservative routine may fallback after timeout while
   aborting on cancellation-like results; an intentional interruption-to-continue policy must be just
   as explicit. Direct active cancellation never starts a fallback. Mechanism cleanup associated
   with a route/scoring failure changes only the capability requests that the cancelled or failed
   phase owns, not unrelated persistent requests or hardware directly.

   Build fixed route geometry eagerly. When geometry depends on live pose, vision, or another
   current-cycle fact, use an explicitly named start-time route factory that resolves exactly once
   at the Route Task's own start boundary. Keep that factory quick and non-blocking, keep sensor and
   strategy interpretation in robot-owned path code, and pass only the resulting concrete route to
   the integration. Construction failure must fail closed without starting the follower.

7. **Docs are part of the API**

   Phoenix treats documentation as a first-class feature: Javadocs should be “mouse-over quality,”
   and the Markdown guides should stay in sync with the real APIs and examples.

   Phoenix also treats **debuggability** as a first-class feature: a good `debugDump(...)` (and in some cases a good `toString()`) is **live documentation**.

   **Rule of thumb:**

   * Use **`toString()`** for small, mostly-immutable *value objects* and configs where a compact one-line representation is helpful (examples: `Pose2d`, `Pose3d`, `DriveSignal`, `ChassisSpeeds`, `CameraMountConfig`, small `Owner.Config` objects).
   * Use **`debugDump(DebugSink dbg, String prefix)`** for loop-updated and/or stateful objects (things that have `update(...)`, own hardware, or own other objects): `Plant`, `AbsolutePoseEstimator`, `DriveSource` / `DriveOverlay`, `Task`s, controllers, subsystems.

   **When both exist:**

   * `toString()` should answer “what is this object?” (identity + key config).
   * `debugDump()` should answer “what is this object doing right now?” (live state: current targets, errors, timers, modes, enable flags).

   **`debugDump(...)` conventions:**

   * Accept a nullable sink and do nothing if `dbg == null`.
   * Use stable keys: `prefix + ".fieldName"` (avoid changing key names).
   * Keep it safe to call every loop: no exceptions, no blocking, no expensive work.
   * Prefer delegating: call `child.debugDump(dbg, prefix + ".child")` for owned components.

   **`toString()` conventions:**

   * Keep it short (single line).
   * Avoid huge nested output; if it wants to be multi-line or you want nested structure, prefer `debugDump(...)`.

   **Debug vs required telemetry:**

   * `debugDump(...)` is for diagnostics and understanding internal state. It should be safe to disable (for performance or to reduce telemetry noise).
   * Driver-facing / “required for normal operation” telemetry (mode, safety warnings, high-level state the drivers rely on) should **not** depend on the debug pipeline.
     In FTC OpModes, print that information via the FTC `Telemetry` API directly (or via a dedicated always-on status channel).

8. **Fail fast with actionable errors**

   When something is misconfigured, Phoenix should throw early (often at build-time) with an
   error message that tells a student what to change. Avoid silent no-ops.

   At the FTC boundary, every homogeneous actuator or mecanum command group must use nonblank,
   distinct configured names under the FTC SDK's trimmed, case-sensitive lookup identity. Validate
   each name before accepting it into a group and preflight the complete group before fresh hardware
   resolution or configuration. Rejecting a bad addition through a retained staged-builder alias
   must cause no additional hardware effects. This is a group-local configuration invariant, not
   proof of physical device identity or a global ownership registry: feedback may intentionally read
   the same configured device, and separate owners may reuse a name when their lifecycles permit it.

   Component construction and complete-behavior readiness are different facts. A component should
   validate the configuration it owns; when safe operation also depends on several robot-owned
   facts, combine those facts once at the mode boundary in a small immutable robot-specific
   readiness result. Do not add a second generic validation language, scatter exceptions through
   OpModes, or let a successful constructor claim that an entire Auto is armed.

   * A blocking result must prevent hardware behavior from being installed or started, including
     any START-without-confirmation path. Warnings and blockers required by drivers belong in
     always-on telemetry with a concrete remediation, not only in `debugDump(...)`.
   * Gate only the behavior that depends on a missing fact when a safe partial mode exists. For
     example, incomplete localization calibration may disable pose-dependent TeleOp assists while
     leaving manual drive and mechanisms available.
   * Treat test-only route geometry and other maturity distinctions as typed facts owned by their
     producer. A production-looking label, successful construction, or vendor idle state is not a
     substitute for deliberate match readiness.
   * A software pose rebase proves that coordinate owners agree; it does not measure the robot's
     physical field placement. Display the required physical start for an operator check, and do
     not describe a post-rebase comparison as an independent safety observation.

9. **Guided builders ask one conceptual question at a time**

   Staged builders should guide users through the small set of meaningful answers for the next
   required decision. This is especially important at FTC boundaries where students are choosing
   hardware, target domain, control strategy, position geometry, unit mapping, and calibration
   policy.

   * Each required conceptual question gets answered explicitly.
   * When a required question has mutually exclusive answers, the stage should expose only those
     answer methods, and each answer should return the next stage. Avoid a model where a later answer
     silently replaces an earlier answer. For example, after a target request is supplied, a planner
     preference stage may expose `nearestToMeasurement()`, `preferIncreasing()`, `preferDecreasing()`,
     and `preferRangeCenter()`, and each method should advance to the unreachable-policy question.
   * Answer each conceptual question once. Prefer direct answer methods on the current stage; do not
     make callers restate a domain already established by an earlier stage or construct a selector or
     parameter wrapper only to pass it immediately into that stage. Keep such a public type only when
     callers meaningfully store, reuse, compose, share, or independently validate it outside that
     one builder step.
   * Optional tuning only appears after the user deliberately enters a tuning branch.
   * Use a `done...()` method only for a multi-setting tuning branch where the user may reasonably
     set several options before returning. One-answer policy choices should advance immediately;
     multi-setting sections such as
     `targetGuards().maxTargetRate(...).holdLastTargetUnless(...).doneTargetGuards()` should keep
     an explicit return method.
   * Builder steps should expose only options that make sense given previous answers. For example,
     `rangeMapsToNative(...)` belongs after `bounded(...)`, not after `unbounded()`, and motor-only
     tuning methods should not appear on a standard-servo position path.
   * Default shortcuts are appropriate for optional tuning branches, such as
     `deviceManagedWithDefaults()`, but they should not hide conceptual safety decisions like
     whether a position coordinate needs a runtime reference.
   * Unit boundaries should be obvious from names. Methods without `native` in the name should
     normally use the public plant coordinate; methods that cross the plant/native boundary should
     say so explicitly.
   * Velocity mapping should stay zero-preserving. Position may need affine endpoint mapping and a
     separate reference/offset policy; velocity should keep `0.0` meaning stop in every unit
     system, so the velocity builder should expose scale-only mapping rather than offset-based
     endpoint maps.
   * Prevent invalid combinations by types when practical, not merely by throwing at `build()`.
     For example, if a request must choose between facing-derived and translation-derived spatial
     sources, the staged builder should make that choice explicit instead of allowing both with
     hidden precedence.

10. **Principle-driven evolution (breaking changes are OK)**

   Phoenix optimizes for a coherent, principle-driven API surface — not strict backwards compatibility.

   * If a better design exists, it’s okay to make a breaking change.
   * Prefer **deleting** legacy paths instead of accumulating deprecations.
     Keep deprecated code only when there is a concrete reason (external consumers, season support,
     migration cost that truly can’t be paid immediately, etc.).
   * Aim for **one obvious way** to do something. If two APIs overlap, choose one and remove the other.
   * Keep things **parallel** across similar objects:
     consistent capabilities, consistent method names, consistent argument order, and consistent nouns.
   * When similar owners share only part of an API, expose that smallest shared capability to
     reusable helpers instead of adding one overload family per owner. Keep heartbeat, clearing,
     construction, and resource cleanup on the object that actually owns them; parallel declaration
     methods must not imply parallel lifecycle ownership.
   * Choose one supported public construction layer for an API family and keep sibling operations
     parallel within that layer. A second public layer—for example, exposing both facade factories
     and concrete constructors or `of(...)` factories—needs a distinct capability, not merely
     another spelling of the same construction. Within the chosen layer, add an overload only when
     it materially simplifies a useful input or optional configuration without creating ambiguity.
     Count an immediately consumed selector or parameter wrapper as a public construction layer;
     being a parameter type does not by itself provide a distinct capability.
   * Do not let an API infer facts its inputs cannot prove. Configuration metadata is not physical
     hardware identity, and plausible runtime values are not proof of trustworthy acquisition. When
     correctness or safety materially depends on undocumented controller, firmware, vendor, or
     hardware behavior, measure the exact supported stack first and keep diagnostic tooling separate
     from the eventual production abstraction.

---

## 2. Layering: from hardware to behavior

Phoenix is built in layers. Most robot code should live near the **top**.

### 2.1 Hardware abstraction (HAL)

At the bottom is a tiny hardware abstraction layer:

* `PowerOutput` – normalized power (typically `-1..+1`).
* `PositionOutput` – position in native units (servo `0..1`, encoder ticks, etc.).
* `VelocityOutput` – velocity in native units (ticks/sec, etc.).

The FTC adapter `edu.ftcphoenix.fw.ftc.FtcHardware` wraps FTC SDK devices into these outputs.

Most robot code should **not** use these directly.

### 2.2 Plants

A **Plant** is a source-driven scalar target follower. Robot behavior does not call
`plant.setTarget(...)` every loop. Instead, every robot-facing Plant is constructed with one
`PlantTargetSource`. Simple scalar values are lifted with `PlantTargets.exact(...)`, and richer
behavior targets use `PlantTargets.overlay(...)` or `PlantTargets.plan(...)`. During
`plant.update(clock)`, the Plant resolves that target source once, applies static bounds and
plant-level hardware guards, verifies the final target is finite and still inside the declared
plant-unit range, applies that one safe mechanism target through its selected hardware/control
path, and refreshes status.

Key methods (see `edu.ftcphoenix.fw.actuation.Plant`):

* `update(LoopClock clock)` — sample the configured target source and command hardware/control.
* `getRequestedTarget()` — what behavior asked for this loop.
* `getAppliedTarget()` — the final mechanism target selected after bounds and target guards.
* `getTargetPlan()` — how the `PlantTargets` graph selected the requested target.
* `getTargetStatus()` — why requested and applied target may differ.
* `atTarget()` / `atTarget(value)` / `hasFeedback()` — feedback-based readiness.
* `getMeasurement()`, `getTargetError()`, and `getAppliedTargetError()` for feedback-capable plants.
* optional `writableTarget()`, `reset()`, and `debugDump(...)`.

A Plant may be open-loop (power, commanded servo position), device-managed closed-loop (for example FTC motor velocity/position), or framework-regulated closed-loop over a raw actuator command. The public rule is the same for all of them: **behavior shapes sources; Plants protect and apply one target per loop**.

Priority overlays keep activation and target production separate. In every Plant loop, an overlay
samples every layer's activation gate exactly once, then resolves enabled target producers lazily
from highest priority to lowest. An enabled `add(...)` layer that is unavailable blocks lower
layers; only the explicitly named `addIfAvailable(...)` form falls through. The total base resolves
only after every layer is disabled or explicitly falls through. Shadowed target producers are not
sampled merely to maintain hidden state. A measured hold therefore stays entered only across
consecutive resolution cycles in which it is actually resolved; after a resolution-cycle gap, its
next resolution captures a fresh measurement. Keep this behavior inside the source graph rather
than adding selection-reset calls or lifecycle hooks to robot code.

Behavior guards and Plant guards are intentionally parallel, but they attach at different layers:

* **Behavior target generation** happens before the Plant protects hardware. Use `PlantTargets.exact(...)`, `PlantTargets.overlay(...)`, `PlantTargets.plan(...)`, `ScalarTarget`, and output queues to answer “what target does the robot want?” Plain `ScalarSource`s are still useful number streams, but anything that will become a Plant target should be lifted into `PlantTargets`.
* **Plant target guards** live in the builder's `targetGuards()` branch and protect hardware after the requested target is resolved: max target rate, hold-last interlocks, and fallback targets. These answer “what may this hardware safely apply?” A Plant with a fixed declared range rejects a static fallback outside that range when built, and every Plant rechecks the dynamic guard result before it becomes the applied target.

Candidate freshness has one owner and one timebase. Ordinary `PlantTargetCandidate` and
`PlantTargetRequest` factories describe timeless/current robot intent. A value derived from a
sensor observation uses the parallel `observed...` factory and supplies quality plus one stable
`LoopTimestamp` created by the consuming `LoopClock`. The planner derives observation age
when it resolves the request; an optional `maxObservationAgeSec(...)` gate applies only to observed
candidates. Invalid observation metadata rejects that candidate; if no valid candidate remains, the
planner follows its explicit unavailable policy. Invalid metadata must never be silently
reclassified as timeless intent.

When an observation-derived target producer receives age rather than capture time, its reusable
source/adapter owner calls `clock.timestampSecondsAgo(sampledAgeSec)` exactly once before publishing
the immutable snapshot used for target construction, then retains the resulting timestamp. A target
request source must not repeat that conversion whenever a cached observation or request is sampled,
because doing so would make it appear newly acquired. Keep this anchoring at or before the target's
source boundary rather than adding timestamp bookkeeping to ordinary mechanism code.

For a multi-observation camera frame, publish the frame timestamp once beside the immutable geometry
list and attach that same value to every selected observation. A trustworthy processed frame with no
usable targets is distinct from no trustworthy frame; do not represent either state with a
timestamped `noTarget()` observation. Mixed frame identities fail closed at the camera boundary.

Keep static range declarations such as `bounded(min, max)` close to the Plant topology because they define the legal plant coordinate system. Direct power Plants are the simpler fixed-domain case: their normalized range is always `[-1, +1]`, so the builder does not ask students to declare it. Keep dynamic protection such as rate limits and interlocks in `targetGuards()`.

For framework-regulated Plants, keep three different bounds at their proper layers:

* A `Pid` output limit bounds that controller's own contribution. Feedforward, voltage
  compensation, or another later regulator decorator can still change the command.
* When robot policy needs a deliberately narrower range around the complete composed control law,
  place `ScalarRegulators.outputLimited(...)` outermost around that composition. The decorator is
  explicit control-law policy; putting another output-changing decorator outside it would change
  what is covered by the limit.
* Plant/output command safety is universal, not an optional policy. The normalized command boundary
  remains responsible for keeping the command finite and inside its semantic domain even when no
  `outputLimited(...)` decorator is present. A regulator limit does not replace target bounds,
  Plant guards, or final output defense.

Controller and regulator policy limits may saturate finite excursions only. They must never turn
`NaN`, infinity, or arithmetic overflow into a plausible finite boundary value, because doing so
would hide invalid control math from the final fail-stop defense.

At a framework-regulated `PowerOutput` boundary, finite commands already inside `[-1.0, +1.0]`
remain unchanged and finite excursions saturate to that inclusive range. Non-finite regulator
results are never submitted. If regulation or the output write fails, the Plant fails closed: it
best-effort stops the output, resets the regulator, and preserves the original failure as the
primary exception. A regulator reset by itself does not write hardware.

Regulated completion is also command evidence, not just matching target and measurement numbers.
Both `atTarget()` forms must remain false after reset, stop, or a failed regulated actuation until a
complete later actuation returns normally. Diagnostics distinguish the raw regulator result from
the normalized command submitted by a normally returning top-level output call. That call is a
seam-level fact, not acceptance, per-child group truth, hardware readback, or proof of motion; stop
status must likewise claim submitted zero only when every relevant top-level output stop returns
normally.

An outer output limiter cannot generically feed saturation back into an arbitrary inner controller,
so it does not claim saturation-aware anti-windup. Keep integral limits controller-specific. The
robot mechanism owner also decides when regulation is enabled, whether a zero request means coast
or active hold, and when controller history should be reset; those meanings must not be inferred by
a generic scalar decorator.

### 2.3 The beginner entrypoint: `FtcActuators.plant(...)`

`edu.ftcphoenix.fw.ftc.FtcActuators` is the recommended way to create Plants from FTC hardware. It lives in the FTC boundary because it depends directly on `HardwareMap`, FTC device classes, and FTC-specific motor tuning APIs.

```java
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.ftc.FtcActuators;

ScalarTarget shooterTarget = ScalarTarget.held(0.0);

Plant shooter = FtcActuators.plant(hardwareMap)
        .motor("shooterLeftMotor", Direction.FORWARD)
        .andMotor("shooterRightMotor", Direction.REVERSE)
        .velocity()
        .deviceManagedWithDefaults()
        .bounded(0.0, 2600.0)
        .nativeUnits()
        .velocityTolerance(100.0)
        .targetedBy(shooterTarget)
        .build();

Plant transfer = FtcActuators.plant(hardwareMap)
        .crServo("transferLeftServo", Direction.FORWARD)
        .andCrServo("transferRightServo", Direction.REVERSE)
        .power()
        .targetedByDefaultWritable(0.0)
        .build();

Plant pusher = FtcActuators.plant(hardwareMap)
        .servo("pusherServo", Direction.FORWARD)
        .position()
        .linear()
            .bounded(0.0, 1.0)
            .nativeUnits()   // servo raw 0..1 plant coordinate
        .targetedByDefaultWritable(0.0)
        .build();
```

The builder is staged on purpose:

1. **Pick hardware**: `motor` (optional `andMotor`), `servo` (optional `andServo`), `crServo` (optional `andCrServo`)
2. **Pick target domain**: `power()`, `velocity()`, `position()`
   * Power has the fixed normalized range `[-1, +1]`; out-of-range requests clamp before output.
     `CLAMPED_TO_RANGE` is the active status unless a later guard supplies a more specific status.
   * Velocity then asks loop ownership, bounds, and plant/native units.
3. **For position Plants, answer guided position questions**:
   * motor position control: `deviceManagedWithDefaults()`, `deviceManaged()...doneDeviceManaged()`, or `regulated()` followed by one direct feedback answer and `regulator(...)`
   * topology: `linear()` or `periodic(period)`
   * bounds: `bounded(min, max)` or `unbounded()`
   * mapping/reference: `nativeUnits()`, `scaleToNative(...)`, bounded-only `rangeMapsToNative(...)`, then `alreadyReferenced()`, `plantPositionMapsToNative(...)`, `assumeCurrentPositionIs(...)`, or `needsReference(...)` when a runtime reference is required
4. **Optional hardware guards**: enter `targetGuards()` for dynamic Plant-level protection such as `maxTargetRate(...)`, `holdLastTargetUnless(...)`, or `fallbackTargetUnless(...)`.
5. **Target binding**: finish with `targetedBy(ScalarTarget)`, `targetedBy(readOnlySource)`, or `targetedByDefaultWritable(initialTarget)`, then `build()`.

Configured hardware names keep FTC's own identity semantics: surrounding whitespace is trimmed for
lookup and comparison, while case remains significant. Within one motor, standard-servo, or
CR-servo command group, names must therefore be nonblank and distinct after trimming. Phoenix
checks a name before accepting it into the staged group and validates the complete group before
fresh hardware resolution or configuration, without changing the fluent call site.

That check is deliberately local to the commanded group. It does not prove that differently named
configuration entries are different physical devices, reserve a name globally, or prohibit a
regulated Plant from reading feedback through the same configured name. Separately constructed
owners may also refer to the same configured name when robot lifecycle policy makes that handoff
intentional.

Internally, Phoenix also has lower-level `Plants` factory helpers, but student code should typically prefer `FtcActuators`.

---

## 3. Drive: sources, signals, and the drivebase

Phoenix drive is split into two parts:

### 3.1 `DriveSource` produces a `DriveSignal`

A `DriveSource` converts “intent” into a robot-centric `DriveSignal`:

* manual TeleOp: `new GamepadDriveSource(driver.leftX(), driver.leftY(), driver.rightX(), GamepadDriveSource.Config.defaults())`
* assisted drive: `DriveGuidance` overlays (auto-aim, go-to-point, pose lock, etc.)
* autonomous logic: any custom `DriveSource`

### 3.2 `DriveSignal` sign conventions are a contract

`DriveSignal` is robot-centric and aligned with Phoenix pose conventions (+X forward, +Y left):

* `axial > 0` → forward
* `lateral > 0` → left
* `omega > 0` → CCW (turn left)

Driver-facing conversions (e.g., “stick right means strafe right”) happen **at the input boundary** (see `GamepadDriveSource`), not scattered through control code.

### 3.3 `MecanumDrivebase` applies the command

`MecanumDrivebase` mixes a `DriveSignal` into four wheel powers.

Drive behavior shaping belongs upstream in the source graph. For example, smooth a manual drive source with `DriveSource.rateLimited(...)` before the command reaches the drivebase:

```java
DriveSource manual = gamepadDrive
        .scaledWhen(driver.rightBumper(), 0.35, 0.20)
        .rateLimited(4.0, 4.0, 6.0);

DriveSignal s = manual.get(clock).clamped();
drivebase.drive(s);
```

Ordinary TeleOp Tasks perform decision work, update behavior sources, or request Plant targets.
After those Tasks run, one final drive phase samples the composed `DriveSource` and writes its
`DriveSignal` to the `DriveCommandSink`. Do not add a Task that imperatively competes with that final
writer.

`DriveTasks.driveExclusivelyForSeconds(...)` is a deliberately narrow exception for simple
open-loop Auto routines and drive testers. Use it only while it is the sole behavior-command writer
for that sink. It refreshes the sink and writes the requested signal on every active cycle, then
stops on completion or active cancellation. If a third-party adapter's supported lifecycle requires
updates beyond active Tasks, the composition root continues calling `update(clock)` with the shared
`LoopClock`, and the adapter deduplicates a same-cycle Task update. Timed open-loop drive is not the
normal way to run a Pedro or other external route; use the route/guidance Task helpers for that
lifecycle.

Configuration is via `MecanumDrivebase.Config` for drivebase scaling and physical-speed mapping. The drivebase makes a **defensive copy** of the config at construction time, so mutating the config object later won’t change an already-created drivebase.



### 3.4 Config objects

Phoenix uses two common patterns for configuration:

1. **Owner-scoped configs** live as a nested `Owner.Config` class.

   * Use this when the config is only meaningful to that one class.
   * Examples: `MecanumDrivebase.Config`, `FtcWebcamVisionPortalLane.Config`,
     `PinpointOdometryPredictor.Config`.
   * These are usually simple mutable data objects: start from `defaults()`, tweak the fields you care about, then pass the config into the owner.
   * Owners should defensively copy configs at construction time when later mutation would be surprising.

2. **Semantic config wrappers** are top-level `*Config` classes.

   * Use this when the config represents a real thing with meaning across multiple systems, and the type name should communicate that meaning.
   * Example: `CameraMountConfig` (robot→camera extrinsics).
   * These are often immutable value objects with `of(...)` / `identity()` factory helpers.

Rule of thumb: if you find yourself creating a top-level `XConfig` that is only ever used to configure `X`, it probably wants to be `X.Config` instead.


#### 3.4.1 Factory naming: `defaults()` vs `identity()` vs `zero()`

Phoenix uses these names intentionally:

- **`defaults()`** means “a reasonable starting configuration for a component.”
  It’s used for `Owner.Config` classes that collect tuning / hardware options.
  The values are not “correct for your robot,” they’re just safe to start from.
  Example: `PinpointOdometryPredictor.Config.defaults()`.

- **`identity()`** means “the identity transform.”
  It’s used when the object conceptually *is a transform between frames* and there is a meaningful
  identity element (“no translation, no rotation”).
  Example: `CameraMountConfig.identity()` means the camera is assumed to be at the robot origin and
  aligned with the robot axes — useful as a placeholder, but you should replace it with a calibrated
  mount for real accuracy.

- **`zero()`** is used for geometry primitives where the identity transform is also the “all zeros” value.
  Example: `Pose2d.zero()` / `Pose3d.zero()` represent “no translation, no rotation.”

Rule of thumb: if the thing you’re constructing is *behavior/tuning*, call it `defaults()`.
If it’s *pure geometry*, prefer `zero()` / `identity()` depending on what’s idiomatic for that type.


#### 3.4.2 Constructing configs: prefer `defaults()` over `new`

For **owner-scoped configs** (`Owner.Config`), Phoenix intentionally standardizes on a factory method:

- Create configs with `Owner.Config.defaults()`
- Do **not** instantiate them directly with `new Owner.Config()`

Why this matters:

- It keeps call sites uniform and easy to grep.
- It lets us change how defaults are produced later (copy-on-write, versioned defaults, etc.) without changing user code.
- It avoids the "half the code uses `new`, half uses `defaults()`" drift that makes refactors noisy.

Implementation rule:

- `Owner.Config` (and other owner-scoped nested `*Config` classes, like `FtcDrives.MecanumWiringConfig`) constructors should be `private`, and each config should provide `public static ... defaults()`.


#### 3.4.3 Naming config helpers: `withX(...)` / `withoutX()`

Most Phoenix config tuning is done by directly setting public fields:

```java
MecanumDrivebase.Config cfg = MecanumDrivebase.Config.defaults();
cfg.maxOmega = 0.8;
cfg.maxAxial = 1.0;
```

Sometimes, a config wants convenience helpers (usually when toggling a feature requires setting multiple related fields).
When Phoenix provides those helpers, the naming convention is:

- `withX(...)` to enable/apply something
- `withoutX()` to disable/clear something

Phoenix avoids `useX(...)` in config APIs. If you see a `useX(...)` method, it should be renamed to `withX(...)` for consistency.



#### 3.4.4 Boolean naming in configs

Phoenix tries to make boolean flags read clearly at the call site (especially inside `if (...)`).

**Rule:** In config objects, booleans should usually be named as *feature toggles*:

- Prefer `enableX` for “turn this behavior on/off.”
- Avoid `useX` (it's ambiguous and tends to drift into inconsistent meaning).
- Avoid negative booleans like `disableX` when possible — prefer a positive `enableX` and flip the default if needed.

Examples:

- ✅ `enableAprilTagAssist`
- ✅ `enableInitializeFromCorrection`
- ✅ `enableResetOnInit`
- ❌ `useAprilTagsAssist`
- ❌ `disableVisionInit`

If a boolean is genuinely a *permission* (“this action is allowed”), `allowX` is acceptable — but try to keep that rare and obvious.

#### 3.4.5 Live tuning is a development workflow, not production configuration

Production TeleOp and Auto must construct long-lived owners from checked-in configuration and
defensive runtime snapshots. They must not continuously read mutable fields from a tuning UI or
treat runtime values as proof that configuration was reviewed and recorded.

Use a dedicated tuning/tester mode when live edits are useful:

- Start the mechanism disabled or at a zero request, and make arming, target bounds, complete-output
  bounds, and fail-stop behavior explicit robot-owned policy.
- Accept a complete candidate at one owned OpMode-loop boundary. Read each candidate value once,
  validate and apply it through the retained control object in robot-owned realization, reset the
  robot-owned outermost composition, and let the normal Plant update perform the hardware write.
- Keep candidate publication at the UI boundary. Phoenix does not make independently changing
  fields into an atomic tuple and framework core must not depend on a vendor configuration API.
- Report the values that were actually accepted. Copy them into the robot profile, review and
  commit the source, stop the tuner, and start a fresh production mode to prove the checked-in
  snapshot.
- Do not invent a match-readiness flag that claims runtime code can prove a source edit was copied
  or committed. Production modes are independent of live tuning state by construction.

The complete standard software-PIDF workflow is in
[`Software PIDF Tuning Workflow`](<docs/testing-calibration/Software PIDF Tuning Workflow.md>).


### 3.5 Direction and naming conventions

Phoenix coordinates and sign conventions are a contract.

Importantly, Phoenix uses **multiple frames**:

* **Robot / mechanism frames (robot-centric):** Phoenix uses a right-handed frame with
  **+X forward**, **+Y left**, **+Z up**.
  * `DriveSignal` is defined in this robot-centric frame.
  * Camera mount extrinsics (`robotToCamera`) are expressed in this robot-centric frame.

* **Field frame (field-centric):** Phoenix uses the **FTC Field Coordinate System** for the
  current season.
  * The origin is the center of the field on the floor.
  * **+Z is up**.
  * The meaning of **+X and +Y is season-dependent** (diamond vs square vs inverted square)
    and is defined by FTC official docs.
  * When in doubt, use the FTC reference-frame definition: a person standing at the center
    of the Red Wall looking in — **+X is to their right** and **+Y is away from the Red Wall**.

Shared conventions (all frames):

* Distances are in **inches**.
* Angles are in **radians**.
* Heading/yaw is **CCW-positive** about **+Z**.

**Naming guideline:** prefer *directional words* (`Forward`, `Left`, `Up`, etc.) when a value is not literally a `Pose2d` component.

For example, library boundaries often use parameter names like `xOffset` / `yOffset` that are defined in that library’s local frame. In Phoenix code, prefer names like `offsetLeftInches`, `offsetForwardInches`, or `forwardPodOffsetLeftInches` to make the meaning explicit.
---

## 4. Tasks and macros

### 4.1 Cooperative tasks

A `Task` is a cooperative unit of work driven by the main loop:

* `start(LoopClock clock)` – called once
* `update(LoopClock clock)` – called each cycle while running
* `isComplete()` – true when finished
* `getOutcome()` – optional richer completion info (`TaskOutcome`)

Each `Task` **instance is single-use**: it may enter `start(clock)` once. A framework Task records
that attempt before starting children, controllers, or hardware effects and throws an actionable
error if the same object is started again. When behavior should repeat, create a fresh graph from a
macro/builder method, `Supplier<Task>`, or `OutputTaskFactory`; do not reset or re-enqueue the old
object. Composites reject obvious duplicate child identities before starting any child.

Cancellation is active-only. Calling `cancel()` before the first `start(clock)` is a
side-effect-free no-op; cancelling an active Task makes it terminal; and cancelling a terminal Task
or cancelling repeatedly is a no-op. Calling `update(clock)` directly before `start(clock)` is a
lifecycle error and framework Tasks fail with an actionable message rather than guessing whether
to start themselves. `Tasks.noop()` is the intentional exception: it is already successfully
complete when created, so direct update and cancellation are harmless no-ops.

### 4.2 The `TaskRunner`

`TaskRunner` runs tasks sequentially (FIFO). It rejects the same Task identity when that object is
already current or queued. It does not retain an ever-seen registry; framework Task start guards also
catch reuse after completion or through another runner.

A key design choice: `TaskRunner.update(clock)` is **idempotent by `clock.cycle()`**. If nested code accidentally calls `update()` twice in the same loop cycle, tasks do not advance twice.

The runner may call a newly started task's `update(clock)` in that same cycle. Because
`clock.dtSec()` describes the interval before the current loop, a Task must start its own duration,
timeout, or phase timer from `clock.nowSec()` rather than charging that preceding delta. A
positive-duration output or Plant request must remain available to its documented downstream
realization phase for at least one loop observation. The exclusive timed-drive helper instead
publishes directly when its Task starts and on each active cycle, and must make every positive
interval observable at least once. A zero-duration command may complete immediately.

`cancelAndClear()` is the normal total-abort operation: it cancels the active Task, if any, and
always discards every queued Task. Queued Tasks never receive a pre-start cancellation callback.
There is no abrupt queue-forgetting path that can silently abandon active work. If `start()`,
`update()`, `isComplete()`, or a cancellation hook throws a `RuntimeException`, the runner fails
closed: it detaches and best-effort cancels active or partially started work, clears the queue,
resets its state, and rethrows the original failure with any cleanup failure suppressed.
`OutputTaskRunner` also invalidates its source caches and returns to its configured idle output on
total-abort and fail-stop paths.

### 4.3 Prefer factory helpers

Robot code should rarely implement raw tasks directly. Prefer:

* `Tasks.*` for generic Task factories and as the public construction layer for composition
  (`sequence`, `parallelAll`, `parallelDeadline`, `withTimeout`, `waitForSeconds`, `waitUntil`, ...)
* `ScalarTasks.write(...)` for standalone `ScalarTarget`s, `PlantTasks.write(plant)` for time-based writes to a Plant's registered target, and `PlantTasks.move(plant)` for feedback-aware moves
* `Tasks.outputPulse(...)` + `OutputTaskRunner` for short output-producing pulses that are overlaid into a final Plant target source
* `DriveTasks.driveExclusivelyForSeconds(...)` for simple Auto/test drive intervals only when the
  Task is the sole behavior-command writer for the sink

Choose parallel composition by lifetime ownership. `parallelAll(...)` waits for every child.
`parallelDeadline(deadline, companions...)` lets the first argument, explicitly named `deadline`,
determine the group's completion and natural outcome, then asks every start-attempted companion to
cancel. Here “deadline” is a Task role, not necessarily a timer; a route Task is a common owner.

The deadline composite can only invoke each companion's `cancel()` hook. Every bounded companion
must therefore own cancellation-safe cleanup; `sequence(enable, wait, disable)` is unsafe because
cancelling the sequence skips the later disable step. Keep long-lived flywheel, intake, aiming, and
other mechanism requests as capability/service state instead of disguising them as forever Tasks.
Use deadline composition only when the companion is genuinely scoped to the owner's lifetime.

Use `Tasks.withTimeout(task, timeoutSec)` when the caller owns one hard elapsed-time budget around
an otherwise complete Task or composite graph. The decorator starts its clock at its own
`start(clock)` boundary and, at the deadline, reaches the child only through ordinary active
`cancel()`. A successfully cancelled child therefore lets the wrapper report `TIMEOUT`, while the
retained child may truthfully report `CANCELLED`. Put any allowed continuation outside the timed
region—for example, `sequence(withTimeout(prePark, 25.0), park)`—so an early park is not later
interrupted and restarted by a still-running timer. Direct cancellation or failed cleanup must not
start that continuation.

Keep task-local timeouts when the operation owns different semantics: phase-relative timing,
operation-specific safe targets or cleanup, or a richer terminal status such as
`RouteStatus.TASK_TIMEOUT`. Local and outer timeouts may be nested when they represent distinct
budgets; do not duplicate the same policy in both places. `waitUntil(condition, timeoutSec)` remains
the concise condition-wait API and deliberately samples its condition first at the exact boundary.
Fixed-duration waits and commands are successful timed behavior, not timeouts.

For a timed scalar or Plant write, `.then(value)` applies that final registered request after normal
completion **and active cancellation**. Omitting `.then(...)`, or selecting `.leaveThere()`, leaves
the held request in place. Choose deliberately; neither behavior is an imperative hardware stop.

### 4.4 Output pulses are source proposals, not Plant writers

Output queues follow the same source-driven philosophy as Plants. A queue owns timing and returns a scalar output, but it does not own the Plant. The usual pattern is:

```text
base PlantTargetSource
    + OutputTaskRunner layer while active
    + other behavior overrides
    ↓
PlantTargets.overlay(...) final target source
    ↓
Plant targetedBy(final target source)
```

This keeps repeated robot behaviors such as feeder pulses systematic without reintroducing multiple writers to the same mechanism.

---

## 5. Feedback vs open-loop (and why `hasFeedback()` exists)

Phoenix distinguishes:

* **Feedback-capable plants** – implement meaningful `atTarget()` / `atTarget(value)` and override `hasFeedback()` to return `true` (motor position/velocity plants).
* **Open-loop plants** – do not expose sensor-based completion and leave `hasFeedback() == false` (power plants, servo set-and-hold plants).

Important consequence:

* `PlantTasks.move(plant)` **requires** `plant.hasFeedback() == true` and will throw if used on an open-loop plant.
* Time-based helpers such as `PlantTasks.write(plant).to(...).forSeconds(...)` and compact `holdTargetFor(...)` helpers work on any Plant with a registered writable target.

A feedback move must answer its cancellation question immediately after `.to(target)`:

* `.cancelTo(cancelTarget)` writes that finite Plant-unit request once on active cancellation.
* `.leaveTargetOnCancel()` deliberately leaves the move request in place, so motion may continue.

After that required choice, robot code may add `.stableFor(...)`, `.timeout(...)`,
`.thenTarget(...)`, and `.build()`. The completion target from `.thenTarget(...)` applies to success
or timeout, not cancellation; use `.cancelTo(...)` when those outcomes need different requests.

`cancelTo(...)` only changes the registered target request. The Plant still owns the final write,
and overlays, bounds, references, and guards may mask or transform that request. Cancelling one
Task also cannot undo persistent requests left by earlier completed macro steps. Robot-owned
shutdown must still cancel queues, disable relevant overlays, and reset every related mechanism
request for coordinated or emergency cleanup.

When one owner must clean several independent resources, it first makes its terminal/idempotent
state and eligible ownership explicit, then may use `CleanupActions.attemptAll(...)` to attempt the
listed actions in their required safety order while preserving the first `RuntimeException`. Later
actions are still attempted after a `RuntimeException`; an `Error` propagates immediately. If
another operation already failed, `attemptAllAfterFailure(...)` attaches cleanup failures to that
supplied primary and returns the same exception for the caller to rethrow, wrap, retain, or report.
These helpers own exception aggregation only: they do not select resources, detach references,
choose rollback/retry policy, or authorize continuing ordinary robot commands after a failed
prerequisite.

This makes it hard to accidentally write a “wait for target” macro on a mechanism that has no feedback.

---

## 6. LoopClock: the per-cycle truth

Phoenix expects:

1. **Advance the clock once per OpMode cycle** (`clock.update(getRuntime())`).
2. **Everything else reads the clock** (dt, cycle id) but does not advance time.

Why the cycle id matters:

* Edge/toggle sources, bindings, task runners, and similar systems need a clear definition of “one loop cycle.”
* Phoenix uses `LoopClock.cycle()` as that identity.

`cycle()` is monotonic for the lifetime of one `LoopClock`. An explicit `reset(...)` advances the
cycle immediately even if the supplied time is unchanged; it never returns the cycle to zero. This
ensures every cycle-memoized owner misses its old cache at a lifecycle boundary. Reset still does
not replace explicit state resets owned by sources, Tasks, controllers, or vendor integrations.

Several core systems are **idempotent by cycle**:

* `Bindings.update(clock)`
* `TaskRunner.update(clock)`
* Stateful source wrappers like `memoized()`, `risingEdge()`, and `toggled()` (idempotent when sampled with the same clock/cycle)
* Third-party adapters whose required lifecycle lets both a composition root and active Tasks call their update hook

Idempotency prevents subtle bugs when code is layered (menus, testers, helpers) and multiple layers try to “helpfully” update the same system.

Captured observation time uses `LoopTimestamp`. Only the owning clock creates a valid value, through
`nowTimestamp()` or the explicit age-native boundary `timestampSecondsAgo(ageSec)`. The timestamp
privately keeps the clock identity, reset epoch, and seconds coordinate together. Consumers call
`ageSec(clock)`, `isFresh(clock, maxAgeSec)`, or `secondsSince(earlier)`; they do not carry a raw
epoch, expose a raw timestamp coordinate, or repeat future-time/reset checks. `unavailable()` is the
non-null no-measurement-time sentinel. Vendor timestamps remain private boundary values until the
boundary translates them once. Task-local timers may remain primitive seconds when they never
escape their one lifecycle.

For an external drive/follower integration, stopping is a separate safety invariant: `stop()` must
apply a physical stopped state immediately. Staging zero for a future heartbeat is not sufficient,
and a callback-time stop must still be the final output after the enclosing vendor update returns.

Optional loop-phase diagnostics belong at the composition root and remain observers of this one
heartbeat. `LoopPhaseProfiler` may use its hidden monotonic stopwatch to measure elapsed wall time
between explicit high-level boundaries, but it never advances `LoopClock`, supplies time to robot
behavior, sleeps, schedules work, or owns loop rate. Keep its API flat and sequential:
`startCycle(clock)`, then `finishPhase("stableName")` after each just-completed phase, then
`finishCycle(clock)`. Use stable literal phase names, not dynamic per-object names, and do not add
nested spans. Profiling is off by default; snapshots and `debugDump(...)` expose only completed
diagnostic data and must never influence a Task, controller, source, guard, or safety decision.

These measurements are elapsed wall time, including possible scheduler pauses and observer
overhead—not CPU time or the complete interval between FTC loop callbacks. If an already-used
profiler is retained across a deliberate `LoopClock.reset(...)`, reset the inactive profiler at the
same lifecycle boundary before starting another profiled cycle.

---

## 7. Nomenclature and coordinate conventions

Phoenix is designed so you can read code and know:

* what frame a value is in,
* what units it uses,
* what sign it means.

### 7.1 Frame must appear in the name

If a value depends on a frame, the frame name should appear in the identifier.

Examples:

* `fieldToRobotPose`, `fieldToRobotTargetPose`
* `robotDriveSignal`
* `cameraBearingRad`, `cameraForwardInches`, `robotLeftInches`

Avoid ambiguous names like `pose`, `targetPose`, `x`, `y`, `heading` when the frame is not obvious.

In particular, **data holders should not expose a raw `pose` field** if the frame matters. Prefer names like
`fieldToRobotPose`, `robotToCameraPose`, `cameraToTagPose`, etc.

### 7.2 Transform naming: `fromToToPose` (optionally `p`-prefixed in adapters)

For rigid transforms (`Pose2d` / `Pose3d`) that represent relationships between frames, Phoenix consistently uses:

* `cameraToTagPose`
* `robotToCameraPose`
* `fieldToRobotPose`

In **Phoenix core code**, the `p` prefix is usually unnecessary (everything is already in Phoenix framing), and it
creates noisy method APIs. Prefer names like `robotToTagPose(...)` rather than `pRobotToTag(...)`.

In **adapter code** where multiple coordinate systems coexist (FTC SDK vs Phoenix), it *can* be helpful to prefix
Phoenix-framed values with `p` (for example, `pFieldToRobotPose`) so it’s obvious which convention you’re in.

Rule of thumb: if you see `pAtoB.then(pBtoC)`, the result should be `pAtoC`.

This convention is also called out explicitly in the FTC adapter boundary (`edu.ftcphoenix.fw.ftc.FtcFrames`).

### 7.3 Units must appear in the name

Prefer suffixes like: `Inches`, `Rad`, `Deg`, `Sec`, `PerSec`, `PerSec2`.

Examples:

* `headingRad`, `omegaRadPerSec`
* `maxSpeedInchesPerSec`, `timeoutSec`

### 7.4 Signs are part of the contract

Phoenix uses right-handed conventions:

* +X forward, +Y left, +Z up
* yaw CCW-positive
* `DriveSignal.omega > 0` turns left

Convert “driver intuition” at the boundaries (for example, in `GamepadDriveSource`), not throughout the codebase.

---

## 8. Recommended usage pattern

A typical OpMode loop follows this shape:

> Clock → Sensors → Bindings → Tasks → Drive → Plants → Telemetry

In code (conceptually):

```java
clock.update(getRuntime());

// Gamepad axes/buttons are Sources; they are sampled when you call get(...).
bindings.update(clock);

macroRunner.update(clock);

drivebase.update(clock);
drivebase.drive(driveSource.get(clock).clamped());

shooter.update(clock);
transfer.update(clock);
```

The [`Loop Structure`](<docs/core-concepts/Loop Structure.md>) guide dives deeper into why this order matters.

---

## 9. Extending Phoenix

Phoenix is designed to be extended safely:

* New `DriveSource` implementations (assist drive, path following, etc.)
* New Plant types (custom control or interlocks)
* New task factories (team-specific high-level macros)

When extending:

* Keep SDK- or vendor-specific calls in adapters.
* Preserve per-cycle semantics (do not “secretly” advance time or consume edges).
* Keep interfaces narrow (`Plant`, `DriveSource`, `Task`) so systems remain composable.

---

## 10. Documentation standards

Phoenix code is meant to be read by students **during build season** under time pressure.
Javadocs aren’t “nice to have” — they are part of the API.

### 10.1 Javadocs rules of thumb

**Every class should be documented (public or not).**

- **Top-level types:** start with a one-sentence summary of what the type is for.
- **Non-public / internal types:** still add a short comment that answers “why does this exist?”
  and (if relevant) what invariant it relies on.
- If the type depends on **units** or a **coordinate frame**, say so up front.

**Every non-trivial method should be documented.**

- Start with a short verb phrase: “Create…”, “Update…”, “Return…”, “Apply…”, etc.
- Document **units and frames** for parameters and return values (use suffixes like
  `Inches`, `Rad`, `Sec`, etc. and explain when the frame is not obvious).
- Document **preconditions** and what happens when they are violated (`@throws` with a helpful message).
- Document **side effects** (what gets mutated / what gets cached).
- For per-loop methods (`update`, `get(clock)`, etc.), document **call order** and whether
  behavior is **idempotent by cycle**.

**Examples are encouraged when they prevent misuse.**

- Prefer small, copy-pastable snippets using `<pre>{@code ... }</pre>`.
- When the “right” usage is not obvious (builders, overlays, transforms), include at least one example.

**Link to the right things.**

- Use `{@link ...}` to connect concepts and reduce duplicated explanations.
- Use `{@code ...}` for code-ish names.

### 10.2 Error messages are documentation

Phoenix frequently teaches through its exceptions.

- Prefer errors like: “field heading targets require solveWith().localizationOnly(...) or solveWith().aprilTagsOnly(...)” over generic
  `NullPointerException` / “invalid state.”
- If a configuration can never work, throw **at build time** rather than silently producing
  “no output” at runtime.

### 10.3 Keep docs and code in sync

- If an API changes, update **Javadocs**, **Markdown docs**, and **examples** in the same change.
- Avoid stale comments (“TODO update later”) in student-facing areas — they become misinformation.

### 10.4 Keep the docs navigable

- Keep the repo root [`README.md`](<README.md>) as the quick orientation page.
- Keep [`docs/README.md`](<docs/README.md>) as the full docs hub.
- Keep a short `README.md` in each major docs folder with reading order and related links.
- If a doc intentionally lives next to code instead of under `docs/` (for example an optional
  integration guide), link it from the hub so it stays discoverable.
