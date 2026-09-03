---
tags:
  - Test & Tune
---

# How to test a Sushi component

Use this philosophy for a control meaning, mechanism, service, route, or Auto policy:

> Test the owner of one question, replace only the world outside that owner, preserve the
> production heartbeat, and claim only the evidence actually observed.

A short test is useful when its causal chain is obvious. A long test that hides the request,
heartbeat, or injected evidence can pass without teaching what happened.

## The five evidence levels

| Level | Question it answers | What it cannot establish |
|---|---|---|
| 1. Semantic intent | Did this input request the intended robot meaning? | hardware output or motion |
| 2. Software-device scenario | Did the production mechanism turn evidence and intent into the expected recorded command? | physics, wiring, or safety |
| 3. Supplied managed slice | Do lifecycle phases cooperate in production order? | robot-specific mechanism response |
| 4. Maintainer regression | Do exhaustive edge cases and structural contracts remain true? | physical behavior |
| 5. Physical bring-up, calibration, or experiment | What did this assembled robot actually do under controlled conditions? | behavior outside the observed conditions |

Start at the lowest level that owns the question. Do not jump to a whole-robot test when a control
meaning or mapping is the actual uncertainty.

## State the test before writing it

Put these five answers immediately before every test shown in a lesson:

- **Question:** the one decision this method checks.
- **Keep real:** the production owner and collaborators whose behavior matters.
- **Replace:** only the world outside that boundary—a clock, device probe, follower, or sensor input.
- **Observe:** the command, status fact, or Task outcome that answers the question.
- **Cannot conclude:** physical facts the replacement does not provide.

If you cannot answer those in a sentence each, the test probably owns too many questions.

## Make the causal chain visible in Java

Use the labels that apply; do not add a label for a step the test does not have.

```text
// ARRANGE: construct the real owner with a recorder, fake clock, or explicit sensor input.
// REQUEST: change intent; do not pretend output or motion happened yet.
// BEFORE HEARTBEAT: prove requested and applied facts are still distinct when that matters.
// HEARTBEAT: advance the real production phase once, in RobotProgram order.
// INJECT EVIDENCE: change feedback explicitly; a software probe never simulates physics.
// ASSERT: check only the command, status, or outcome supported by that evidence.
// NEXT GATE: name the wiring, safety, motion, timing, or tuning check still needed on the robot.
```

One method should answer one question. A student-facing file should normally show no more than two
methods, keep each method near 35 executable lines, and stay near 100–120 physical lines including
the explanatory comments. Comments are part of the lesson, not clutter to remove for a line budget.

## Read the test after it runs

Every scenario lesson ends with:

- **Read the causal chain:** say which request, heartbeat, evidence, and observation led to the
  assertion.
- **Proves:** name the software contract established by the observation.
- **Does not prove:** name the missing physical or integration evidence.
- **Next gate:** link one isolated follow-up, not a vague “test on the robot.”

For example, a claw probe can establish that logical `OPEN` maps through reviewed configuration to
a recorded native command after one output heartbeat. It cannot establish clearance, servo
direction, physical position, or arrival because a standard servo supplies none of that feedback.

## What students should use and what maintainers supply

Students should author small semantic and software-device scenarios using the existing test-only
clock and FTC hardware probes. Maintainers may supply broader lifecycle matrices, reflection-based
API checks, dynamic proxies, bytecode or annotation checks, and custom Task fixtures when those are
the smallest way to protect the framework. Those are regression evidence to run and inspect—not a
template a beginner must reverse engineer.

Continue with the [software-device scenario index](<../examples/Hardware-free Reference Scenarios.md>)
or cross to [actuator bring-up](<Actuator Bring-up.md>) when the software question is answered.
