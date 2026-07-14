---
name: execute-framework-improvements
description: Execute Phoenix framework improvement items end-to-end from FRAMEWORK_IMPROVEMENT_TRACKER.md with mandatory decision gates, Framework Principles checks, one-item scope, user approvals, Android Studio review, verification, and GitHub publication. Use when asked to start, continue, review, approve, implement, merge, or move to the next tracked Phoenix framework improvement.
---

# Execute Phoenix Framework Improvements

Follow one tracker item through analysis, implementation, human review, and publication. Store the
repeatable procedure in this skill; recover item-specific state from the tracker, Git, tests, and
GitHub rather than relying on chat memory.

## Recover durable state

1. Locate the repository root and read completely:
   - `AGENTS.md`
   - `TeamCode/src/main/java/edu/ftcphoenix/fw/Framework Principles.md`
   - `FRAMEWORK_IMPROVEMENT_TRACKER.md`
   - the relevant framework guides and Javadocs
   - `Phoenix Architecture.md` when Phoenix robot code is in scope
2. Inspect `git status`, the current branch, recent commits, and the relevant remote branch or pull
   request. Preserve all user changes.
3. Treat the tracker as the durable workflow state and Git/GitHub as the publication state. Do not
   redo a completed phase after a new task, restart, or context compaction.
4. Select exactly one tracker item. Do not mix in the next item or adjacent cleanup.

## Gate 1: Decide before coding

1. Move a proposed item to `Researching` while investigating, then record its decision gate:
   - confirmed current behavior or traced failure path;
   - every relevant production, Phoenix, tool, and modern example caller;
   - for each sibling API family, every supported public construction path: facade factories,
     constructors, static `of` methods, overloads, and declared return types;
   - documentation-only, smallest-local-fix, leading-hypothesis, and other credible alternatives;
   - student call-site simplicity, concepts introduced, discoverability, error quality, ownership,
     and implementation complexity;
   - chosen design and explicit rejected designs;
   - bounded implementation scope and verification plan.
2. Record whether each supported public layer provides a distinct capability. Assess symmetry within
   each chosen supported layer; never use symmetry alone to justify duplicate APIs.
3. For a public construction layer without distinct value, explicitly choose removal and caller
   migration or record the concrete compatibility reason and cleanup gate for deferral. Do not add
   siblings to a redundant layer merely to make it look complete.
4. Compare the result against Framework Principles and the tracker hypothesis. Prefer the smallest
   robust design that leaves ordinary robot code short and obvious.
5. Update the tracker to `Ready` only after the decision record is complete.
6. Stop for user direction before implementation when:
   - the best design materially differs from the leading hypothesis;
   - a public API or major lifecycle/ownership semantic is involved, even if the hypothesis remains
     viable; or
   - new authority or a material scope expansion is required.
7. Do not edit implementation code during a required design-approval stop.

## Gate 2: Implement one approved design

1. Fetch `origin/master` and branch from that remote ref using
   `codex/<item-id-lowercase>-<short-slug>`. Do not reset, rewrite, or force-update a divergent local
   `master`; preserve it and use `origin/master` directly.
2. Mark the item `In progress` after required approval is recorded.
3. Make the smallest coherent change. Keep implementation, Javadocs, Markdown, examples, and
   callers synchronized. Do not preempt decisions deliberately assigned to later tracker items.
4. Use independent subagents when they materially improve review or speed. Give each agent
   non-overlapping file ownership, then review the shared working tree centrally.
5. Add focused regression tests for the confirmed problem, preserved behavior, error messages,
   boundary cases, and student-facing repetition or recovery paths.
6. Run adversarial reviews for correctness, Framework Principles, student simplicity, API scope,
   documentation accuracy, and test validity. For an API family, independently repeat the public
   construction-path, distinct-capability, and redundant-layer-disposition audit from Gate 1.
   Resolve findings and rerun affected checks.
7. When implementation is complete, mark the item `Verifying`, record exact automated evidence,
   and request Android Studio inspection. Tell the user which files and behaviors to inspect and
   whether robot-hardware validation is useful.
8. Stop without staging, committing, pushing, merging, or starting another item until the user
   approves the implementation.

## Gate 3: Finalize an approved implementation

Treat `<ITEM-ID> looks good` as approval to finalize, publish, and merge that item. Do not treat it
as permission to start the next item.

1. Mark the tracker item `Done` and record the user's manual verification.
2. Confirm the working tree contains only the reviewed item and rerun `git diff --check` plus any
   check affected by the final tracker edit.
3. Stage only the reviewed files and create one meaningful commit, such as
   `fix(tasks): enforce single-use task instances`.
4. Push the item branch and open a pull request whose body explains what changed, why, robot-code
   impact, deferred scope, and validation.
5. Make the approved pull request ready, merge it into remote `master`, fetch the result, and verify
   the expected head and merge tree. Do not rewrite a divergent local `master` merely to make it
   match the remote.
6. Report branch, commit, pull request, merge SHA, validation, tracker status, and any preserved local
   divergence. Stop before the next tracker item.

If the user says `looks good; move to next`, finish these publication steps first and then begin only
the next item's decision gate. Honor every major-design approval stop again.

## Verification standard

Run the narrowest focused tests while iterating. Before Android Studio review, normally run:

```powershell
$env:JAVA_HOME='C:\Program Files\Android\Android Studio\jbr'
.\gradlew.bat --console=plain :TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac
```

Also:

- count tests, failures, errors, and skips from the generated XML results;
- run `git diff --check` and a trailing-whitespace scan that includes untracked tests;
- search every affected implementation and caller instead of assuming coverage;
- distinguish existing warnings from new failures;
- state explicitly what cannot be verified without robot hardware.

## Communicate each gate

- Lead updates with the current outcome and next gate.
- Explain any skill-driven pause or approval requirement.
- At a design stop, present the recommended design, alternatives, simplicity comparison, and exact
  approval requested.
- At an implementation stop, provide concise Android Studio review instructions and the exact reply
  that will authorize finalization.
- Never imply that the skill remembers live item state; the tracker and repository do.
