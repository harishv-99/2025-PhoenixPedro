# Maintainer Notes

This document is for mentors and framework maintainers.
Most students should not need anything here to write robot code.

For all student-facing paths, start with the canonical [`Phoenix docs hub`](<../README.md>).
For framework design authority, use [`Framework Principles`](<../../Framework Principles.md>).

For the fixed-tag policy itself — detector library vs field-fixed layout, selected-tag localization rules, and season bring-up guidance — see [`AprilTag Localization & Fixed Layouts`](<../drive-vision/AprilTag Localization & Fixed Layouts.md>).

---

## 1. Advanced notes

### 1.1 `TaskRunner` in a custom host

Ordinary robot code does not choose a runner count. Its framework-created `RobotProgram` owns one
private `TaskRunner`, and robot code declares root or input-launched Tasks through the program.
Different duration or interruption policy is not a reason to construct another runner; express
that policy with Task composition, capability state, or a mechanism-owned output queue.

Only an explicitly advanced custom portable host or diagnostic with lifecycle requirements that do
not fit `FtcRobotOpMode` may own a `TaskRunner` directly. If such a host genuinely needs more than
one, every runner must have disjoint command ownership and a documented update, cancellation,
failure, and cleanup boundary. Do not present that exception as a second ordinary robot recipe.

### 1.2 Idle behavior and safety

Phoenix does not enforce a single global notion of "safe idle".

Instead, keep it explicit in your robot logic:

- define what each mechanism should do when no macro is running
- reset targets intentionally

When it helps clarity, use plant wrappers:

- `targetGuards().maxTargetRate(...)` to smooth target changes
- `targetGuards().holdLastTargetUnless(...)` to enforce simple safety rules

### 1.3 AprilTag policy layering

Keep these three layers distinct when maintaining the framework:

- **`AprilTagLibrary`** = detector metadata (which tags the processor recognizes)
- **`TagLayout`** = field-fixed metadata (which tags Phoenix trusts for localization / field-pose solving)
- **subset layouts** = role-specific views over an already-fixed layout

That separation is deliberate.

It lets Phoenix:

- detect more tags than it localizes against,
- keep official-season fixed/non-fixed policy in one framework-owned place,
- and let one robot behavior use a smaller fixed-tag subset without copying tag poses or re-implementing tag filtering logic.

Season-specific fixed/non-fixed tag policy belongs in `FtcGameTagLayout`, so robot code does not
rediscover which IDs are safe to trust.

### 1.4 Production framework boundary rule

Treat the reusable framework core as every production package under `edu.ftcphoenix.fw` except the
three explicit edge families below:

- `edu.ftcphoenix.fw.ftc.*` owns FTC SDK and Android adapters, including the gamepad adapters in
  `edu.ftcphoenix.fw.ftc.input`;
- `edu.ftcphoenix.fw.integrations.*` owns narrow third-party bridges, and may touch FTC types only
  when that particular bridge must join the two boundaries; and
- `edu.ftcphoenix.fw.tools.*` owns FTC-bound examples, testers, and calibration hosts.

`edu.ftcphoenix.robots.*` is the application edge. It may construct SDK resources and the explicit
framework edges, then pass their small Phoenix capabilities into robot policy. It is not reusable
framework core.

Dependency direction is **edge → core**. Protected core packages such as
`actuation`, `core`, `drive`, `input`, `localization`, `sensing`, `spatial`, and `task` must not
reach back into `fw.ftc`, `fw.integrations`, `fw.tools`, or `edu.ftcphoenix.robots`. A protected
production source may import only `java.*`, `javax.*`, or another protected `edu.ftcphoenix.fw`
package. Known fully qualified FTC, Android, vendor, edge, tool, and robot references are rejected
as well. Do not broaden an allowlist because one new import is convenient; either move the adapter
to the truthful edge or introduce the smallest Phoenix contract that the core can own.

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

- Give each page one purpose and one primary audience; state both near the top.
- Structure a tutorial checkpoint as: goal, prerequisites, files to inspect or edit, numbered
  steps, observable result, common problems, and one next step.
- Link to compiling source for complete programs. Keep excerpts short enough to teach one idea
  without creating a copied program that can drift.
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
This is the local equivalent of the hosted **Verify Phoenix framework** check: it compiles TeamCode
and runs the complete unit suite, including the documentation-integrity and production-boundary
tests.

In Android Studio, either use the gutter run icon beside a test class/method or open the Gradle tool
window and run **TeamCode → verification → testDebugUnitTest**.

Tests that depend on loop time should use the test-only `ManualLoopClock`, which advances the real
framework `LoopClock` deterministically. Keep tests focused on documented contracts, and do not use
Android stub default-return settings to conceal an accidental FTC/Android dependency in code that is
supposed to remain pure Java.

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
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest --tests edu.ftcphoenix.fw.docs.DocumentationLinksTest
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
.\gradlew.bat --console=plain :TeamCode:phoenixJavadocs
```

Serve `build/docs-site` with any local static-file server when reviewing the final combined guide
and `/api/` tree. Check navigation and both searches on desktop and a narrow mobile viewport. A
strict Zensical build checks narrative links and anchors; the standard doclet checks exact Java API
documentation. Neither replaces `DocumentationLinksTest` or Java compilation.

The checked-in workflow exposes two hosted checks for every pull request targeting `master`.
**Verify Phoenix framework** runs the canonical compile-and-test contract above. After it succeeds,
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
   Windows-only `colorama` dependency and checking the release's tagged upstream lock; and
5. install the revised lock in clean Windows and Linux environments, then rerun the strict combined
   build, link test, search/artifact checks, and visual review before accepting the upgrade.
