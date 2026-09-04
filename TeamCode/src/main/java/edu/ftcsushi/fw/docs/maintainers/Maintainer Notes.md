---
tags:
  - Advanced
---

# Maintainer Notes

This document is for mentors and framework maintainers.
Most students should not need anything here to write robot code.

For all student-facing paths, start with the canonical [`Sushi docs hub`](<../README.md>).
For framework design authority, use [`Framework Principles`](<../../Framework Principles.md>).

For the fixed-tag policy itself — detector library vs field-fixed layout, selected-tag localization rules, and season bring-up guidance — see [`AprilTag Localization & Fixed Layouts`](<../drive-vision/AprilTag Localization & Fixed Layouts.md>).

---

## 1. Advanced notes

### 1.1 Advanced host ownership

Ordinary FTC robot code has one lifecycle spelling: subclass `FtcRobotOpMode` and declare roles
through `configure(RobotProgram)`. The framework-created program privately owns the clock, binding
graph, Task runner, phase order, telemetry commit, failure path, and cleanup. A different Task
duration, interruption policy, preferred update order, or desire to profile whole-program phases
is not a reason to recreate the FTC callbacks.

The `FtcTeleOpTesterOpMode` family is the specialized host for dynamic tester selection, resource
acquisition after selection, exclusive tester screens, optional alternate Panels input, and
repeated child cleanup. Use that family when those are the actual requirements. It is not an
alternate match-robot grammar and should not be folded into `RobotProgram`'s frozen declaration
graph and data-only prestart policy.

Only an explicitly advanced portable host, diagnostic, or integration whose materially different
lifecycle cannot truthfully fit either managed host may own raw FTC callbacks or replace the
managed host as the whole-program lifecycle owner. Such a replacement also owns every `LoopClock`,
`Bindings`, `TaskRunner`, phase-order decision, and telemetry commit it actually uses. Before
accepting that exception, document its concrete caller and why the managed robot and tester hosts
cannot express its lifecycle.

That restriction concerns whole-host replacement, not bounded nested ownership. Managed Prestart
selection owners and `BaseTeleOpTester` may own their local bindings, private `OutputTaskRunner`
queues may own a local runner, and deterministic tests may construct the primitive under test.
Those roles do not become competing FTC hosts: in production the established host still owns the
callbacks, one program clock, final phase order, commit, and cleanup; a deterministic test owns only
its isolated fixture lifetime.

An approved advanced host must own and verify all of these obligations:

- retain one stable resource graph and clean up every resource acquired before a partial INIT
  failure;
- define INIT, START, ACTIVE, blocked-or-not-ready, and STOP behavior, including the exact boundary
  at which timed work may begin;
- own one `LoopClock`, reset it at the documented lifecycle boundary, advance it exactly once per
  host cycle, and give every child that same clock without allowing a second heartbeat;
- make one explicit phase order that preserves the Sushi dependencies Clock → upstream Services
  → Bindings → Tasks → downstream Outputs/Drive → Presenters → one telemetry commit, omitting only
  roles that the host does not own;
- treat a `RuntimeException` from any lifecycle phase as terminal, retain the first failure as
  primary, attach later cleanup failures as suppressed, and rethrow it; do not catch Java `Error`s;
- on failure or STOP, best-effort cancel and clear Tasks, clear bindings, stop outputs including
  drive in declaration order, stop Services in reverse declaration order, and release every owned
  transport or resource; and
- make repeated or reentrant STOP inert, prevent commands after terminal cleanup begins, and never
  depend on a later loop to apply physical stop.

Ordinary robot code does not choose a runner count. It declares root and input-launched Tasks
through `RobotProgram`; different duration or interruption behavior belongs in Task composition,
capability state, or a mechanism-owned output queue. An advanced host that directly owns a
`TaskRunner` normally owns exactly one. If it genuinely needs more, every runner must have disjoint
command ownership and a documented update, cancellation, failure, and cleanup boundary.

Do not publish a callback-shaped advanced-host example as a second ordinary recipe. Keep
`RobotProgram` construction and lifecycle private to `FtcRobotOpMode`; do not expose a factory,
builder, session, or phase facade merely to give a custom host another spelling of the same
lifecycle.

### 1.2 Idle behavior and safety

Sushi does not enforce a single global notion of "safe idle".

Instead, keep it explicit in your robot logic:

- define what each mechanism should do when no macro is running
- reset targets intentionally

When it helps clarity, use plant wrappers:

- `targetGuards().maxTargetRate(...)` to smooth target changes
- `targetGuards().holdLastTargetUnless(...)` to enforce simple safety rules

### 1.3 AprilTag policy layering

Keep these three layers distinct when maintaining the framework:

- **`AprilTagLibrary`** = detector metadata (which tags the processor recognizes)
- **`TagLayout`** = field-fixed metadata (which tags Sushi trusts for localization / field-pose solving)
- **subset layouts** = role-specific views over an already-fixed layout

That separation is deliberate.

It lets Sushi:

- detect more tags than it localizes against,
- keep official-season fixed/non-fixed policy in one framework-owned place,
- and let one robot behavior use a smaller fixed-tag subset without copying tag poses or re-implementing tag filtering logic.

Season-specific fixed/non-fixed tag policy belongs in `FtcGameTagLayout`, so robot code does not
rediscover which IDs are safe to trust.

### 1.4 Production framework boundary rule

Treat the reusable framework core as every production package under `edu.ftcsushi.fw` except the
three explicit edge families below:

- `edu.ftcsushi.fw.ftc.*` owns FTC SDK and Android adapters, including the gamepad adapters in
  `edu.ftcsushi.fw.ftc.input`;
- `edu.ftcsushi.fw.integrations.*` owns narrow third-party bridges, and may touch FTC types only
  when that particular bridge must join the two boundaries; and
- `edu.ftcsushi.fw.tools.*` owns FTC-bound examples, testers, and calibration hosts.

`edu.ftcsushi.robots.*` is the application edge. It may construct SDK resources and the explicit
framework edges, then pass their small Sushi capabilities into robot policy. It is not reusable
framework core.

Dependency direction is **edge → core**. Protected core packages such as
`actuation`, `core`, `drive`, `input`, `localization`, `sensing`, `spatial`, and `task` must not
reach back into `fw.ftc`, `fw.integrations`, `fw.tools`, or `edu.ftcsushi.robots`. A protected
production source may import only `java.*`, `javax.*`, or another protected `edu.ftcsushi.fw`
package. Known fully qualified FTC, Android, vendor, edge, tool, and robot references are rejected
as well. Do not broaden an allowlist because one new import is convenient; either move the adapter
to the truthful edge or introduce the smallest Sushi contract that the core can own.

`FrameworkBoundaryTest` scans production Java sources and enforces this direction. It deliberately
does not scan `src/test`: a unit test may use an SDK stub or fake to prove an edge adapter, or span
layers to verify a cross-edge contract. That exception never permits the corresponding production
core class to acquire the dependency, and boundary-facing tests should still live with the edge
they exercise when practical.

### 1.5 Documentation topology

Keep [`../README.md`](<../README.md>) as the one canonical documentation hub. Repository,
framework-root, and section `README.md` files are short doorways into that hub, not parallel maps.
If a current reference lives beside code, give its package a short landing page and link that page
from the hub.

Use this authoring contract:

- Give each page one outcome and one primary audience; state the prerequisites before code.
- Keep the six guide areas stable: **Get Started**, **Learn**, **Build**, **Test & Tune**,
  **Advanced**, and **Reference**. Every searchable guide declares exactly one of those tags.
- Structure a Build recipe as: outcome and prerequisites, a three-to-twelve-line exact source
  excerpt, at most three observations, exact main/test file manifests, visibly labeled complete-
  source links, an explained software checkpoint, an isolated hardware gate, and one next action.
  Do not embed complete files in prose or require unrelated mechanisms to finish a recipe.
- Hold Build pages to a reconstruction test: after reading the page and its declared prerequisites,
  a student should be able to recreate the important production wiring without opening another
  source file. Full source may fill in imports, package declarations, and small mechanical details;
  it must not be the only place that constructs an owner, declares its managed heartbeat, or binds
  the concept the page claims to teach.
- A Build page explicitly labeled a **blocked software-boundary checkpoint** may stop before a
  hardware graph that cannot yet pass its conservative managed gate. It must say which wiring it
  does not teach, link the advanced integration authority, and make no physical-run claim.
- When a lesson switches to an independent fixture or later combines focused fixtures, say so.
  Show the active data-only profile and the small composition-root connections for outputs, drive,
  controls, presenters, root Tasks, and STOP that are necessary for that outcome. Explain a concept
  in plain domain language at first use before linking its exact Javadoc contract.
- Link a named framework or example class to generated Javadocs. Use repository links only when
  visibly labeled **Complete source** so API lookup and source study remain distinct actions.
- Before a displayed test, state **Question**, **Keep real**, **Replace**, **Observe**, and
  **Cannot conclude**. Make ARRANGE, REQUEST, BEFORE HEARTBEAT, HEARTBEAT, INJECT EVIDENCE, ASSERT,
  and NEXT GATE visible in Java where applicable. Follow with **Read the causal chain**,
  **Proves**, **Does not prove**, and **Next gate**.
- Make a student-facing scenario instantiate the maintained production owner and configuration it
  claims to prove. Do not duplicate axis mappings, target mappings, or safety values inside the test
  merely to obtain a convenient expected result.
- Keep one question per student-facing test method, at most two displayed methods per file, roughly
  35 executable lines per method, and about 100–120 physical lines including comments. Treat broad
  failure matrices, reflection, proxies, custom Task fixtures, and structural checks as supplied
  maintainer evidence rather than beginner-authored lessons.
- Keep search global because readers may not know the owning area. Use precise outcome headings,
  one area tag, and `search.exclude` for compatibility pointers or boilerplate that would dilute
  results. The narrative search and generated Javadoc type/member search remain separate.
- Describe the supported present state directly. Keep migration narratives, completed work logs,
  and speculative backlogs out of user documentation.
- Keep Markdown as the sole authored source for narrative guides and generated guide-site
  presentation; maintain its local links, anchors, and fences. Javadocs remain the authored exact
  method-level API contract and must stay synchronized with those guides.

### 1.6 Automated framework verification

Pure framework behavior is tested locally under `TeamCode/src/test/java`. These tests run on the
development computer and do not require a Control Hub, phone, emulator, or connected robot.

On Windows, run the complete local software check from the repository root with:

```powershell
.\gradlew.bat --console=plain :TeamCode:compileDebugJavaWithJavac :TeamCode:testDebugUnitTest
```

On macOS or Linux, use `./gradlew` in the same command; the task names and order are unchanged.
This is the local equivalent of the hosted **Verify Sushi framework** check: it compiles TeamCode
and runs the complete unit suite, including the documentation-integrity and production-boundary
tests.

In Android Studio, either use the gutter run icon beside a test class/method or open the Gradle tool
window and run **TeamCode → verification → testDebugUnitTest**.

Tests that depend on loop time should use the test-only `ManualLoopClock`, which advances the real
framework `LoopClock` deterministically. Keep tests focused on documented contracts, and do not use
Android stub default-return settings to conceal an accidental FTC/Android dependency in code that is
supposed to remain pure Java.

Mechanism-learning scenarios should use the shared test-only `FtcTestHardware` registry with the
ordinary production `HardwareMap + Config` constructor. Register only the devices needed by the
scenario, inject encoder, measured-velocity, and electrical-level inputs explicitly, and assert
recorded outputs separately. Never mirror a command into feedback. These probes provide no physics;
call the result a **software device scenario**, not a simulation. A **modeled simulation** must add
an explicit dynamics model and document its assumptions and fidelity. The current registry
intentionally defines no scenario input-file format, trace playback, timeline, physics, or public
model extension interface. A later trace reader can parse values and call the same probe setters
before each clock cycle without changing production mechanism construction.

The hosted job log contains setup, compiler, and test output; setup and compilation failures are
actionable there. When a failed unit-test execution produces reports, its run also provides the
standard JUnit XML and HTML report trees in a three-day `teamcode-test-reports` artifact on the
GitHub Actions run summary. These software checks prove only compilation and tested software
contracts. They do not prove robot wiring, motor or sensor direction, physical motion, tuning,
camera readiness, or safe stop behavior. Use the on-robot testers and calibration walkthroughs for
those facts.

### 1.7 Documentation integrity check

Run the focused Markdown link and fence check from the repository root with:

```powershell
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcsushi.fw.docs.DocumentationLinksTest
```

The check validates inline links/images, exact-case local targets, heading fragments, and balanced
fences in maintained Markdown without depending on network access. It skips generated/build trees
and the four copied FTC SDK/sample Markdown files. It does not claim that an external website is
reachable or that prose and code are semantically identical; those still require review alongside
the compiled canonical examples. Use this focused command while iterating on documentation; the
canonical full software command above already runs this test as part of the complete suite.

### 1.8 Generated documentation site

Author Markdown and Javadocs in place. The root `zensical.toml` presents those sources; it does not
create a second authored documentation tree. Generated files belong only under the ignored
`build/docs-site` directory and must never be edited or committed.

The renderer is optional maintainer tooling. Students editing or compiling robot code do not need
Python. For a local narrative preview on Windows, use Python 3.12 from the repository root:

```powershell
python -m venv build/docs-venv
.\build\docs-venv\Scripts\python.exe -m pip install --requirement requirements-docs.txt
.\build\docs-venv\Scripts\python.exe -m zensical serve
```

On macOS or Linux, activate or invoke the equivalent `build/docs-venv/bin` executables. The preview
server rebuilds the narrative pages as their source files change. Stop it before creating the exact
combined artifact, then run the strict narrative build before Javadocs so Zensical's clean step
does not remove the generated API pages:

```powershell
.\build\docs-venv\Scripts\python.exe -m zensical build --clean --strict
.\gradlew.bat --console=plain :TeamCode:sushiJavadocs
```

Serve `build/docs-site` with any local static-file server when reviewing the final combined guide
and `/api/` tree. Check navigation and both searches on desktop and a narrow mobile viewport. A
strict Zensical build checks narrative links and anchors; the standard doclet checks exact Java API
documentation. Neither replaces `DocumentationLinksTest` or Java compilation.

The checked-in workflow exposes two hosted checks for every pull request targeting `master`.
**Verify Sushi framework** runs the canonical compile-and-test contract above. After it succeeds,
**Verify documentation artifact** installs the pinned Python dependencies, checks them, builds the
strict narrative and API documentation, verifies the generated tree, and uploads the exact Pages
artifact. The trusted `master` deployment requires both jobs to succeed and deploys that same-run
artifact; it never rebuilds the site.

The `master` protection rule requires both hosted checks before merge. Keep the pull-request
workflow unfiltered so both required check identities are always reported; do not make the
master-only deployment job a required pull-request check.

This repository's live Pages source is **GitHub Actions** under **Settings > Pages > Build and
deployment**, and the `github-pages` environment permits deployment only from `master` under
**Settings > Environments**. Keep those settings aligned with the workflow's upstream-repository,
push, and live-`master` checks; do not permit arbitrary branches to deploy.

`requirements-docs.txt` is the complete version-pinned Python 3.12 dependency list, not a floating
list of minimum versions. Zensical is still a `0.0.x` package, so upgrades are deliberate:

1. choose one exact Zensical release after reviewing its changelog and supported features;
2. create a new clean Python 3.12 environment outside the checked-in source tree;
3. install only `zensical==<new-version>` and inspect `python -m pip freeze --local`;
4. replace every runtime pin in `requirements-docs.txt`, retaining platform markers such as the
   Windows-only `colorama` dependency and checking the release's tagged upstream lock;
5. compare the new release's generated default Markdown-extension map with the explicit
   `[project.markdown_extensions]` block in `zensical.toml`, then update that block and its focused
   documentation regression together; and
6. install the revised lock in clean Windows and Linux environments, then rerun the strict combined
   build, link test, search/artifact checks, and visual review before accepting the upgrade.
