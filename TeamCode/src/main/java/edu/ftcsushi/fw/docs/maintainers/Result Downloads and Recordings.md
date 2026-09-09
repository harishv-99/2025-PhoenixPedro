---
tags:
  - Advanced
---

# Result downloads and recordings

**Audience:** maintainers and authors of custom testers. Read the
[download runbook](<../testing-calibration/Download and Inspect Experiment Results.md>) first.
This reference defines ownership and the supported offline calculation, not hardware acceptance.

## Publish a custom report through the existing owner

[`ResultDownloads`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/ftc/ResultDownloads.html>)
has three operations: `publish(filename, frozenUtf8Text)` retains prepared text, `url()` returns
its current page address or null, and `clear()` invalidates it. Publication returns false when the
capability is unavailable. No operation supplies observations, interprets report fields or writes
a controller file. The producer owns its question and format; the host owns transfer/lifetime.

The independent spin-up experiment uses this exact excerpt:

<!-- source-excerpt: TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/tester/ReferenceFlywheelSpinUpExperiment.java -->
```java
String frozenReport = formatReport(terminalResult);
reportPublished = ctx.downloads.publish("spin-up-result.txt", frozenReport);
```

`formatReport` converts the experiment's already-frozen result into text. These statements run
once after the ordinary mechanism heartbeat has had the opportunity to realize terminal zero,
inside a guarded optional-export block. A later new trial clears the prior download. This adds
report publication, not a second experiment state machine, clock, telemetry owner or hardware
loop. The experiment catches an export failure without changing its outcome or cleanup.

Use [`BaseTeleOpTester`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/tools/tester/BaseTeleOpTester.html>)
inside the existing tester host. Read `ctx.downloads` from the current context: START renews its
download lease while retaining the same hardware, gamepads, telemetry and clock. Do not capture
the INIT lease in a callback that later publishes ACTIVE results. Automatically hosted direct
`TeleOpTester` implementations without that base lifecycle receive unavailable downloads.
The five-argument `TesterContext` constructor remains an offline/custom-host seam with downloads
unavailable; the six-argument advanced injection seam makes its host responsible for root reset
and STOP invalidation. No ordinary tester author constructs or registers a web server.

[Complete source: `ReferenceFlywheelSpinUpExperiment.java`](<https://github.com/harishv-99/2025-PhoenixPedro/blob/master/TeamCode/src/main/java/edu/ftcsushi/robots/examples/reference/tester/ReferenceFlywheelSpinUpExperiment.java>)
shows result freezing, deferred publication and cleanup together.

## Lifetime, bounds and trust

The SDK registrar installs fixed read-only routes on the existing FTC server. The host uses the
SDK-advertised server URL, never serialized connection information or guessed credentials. Requests
read safely published immutable bytes only; they do not access tester state, hardware or clocks.

- Keep one latest attachment, at most 512 KiB of encoded UTF-8. The standard host validates Unicode
  and safe bounded filenames before replacing a result; exact filename rules are in the API.
- Clear on the producer's documented reset/replacement and on tester exit, host reset/failure/STOP.
  Revoked child cleanup/publication cannot alter a replacement child's result.
- Admit one attachment transfer at a time; reject additional transfers without waiting. Release its
  permit on EOF/close. The old admitted bytes can finish after invalidation, but stale links cannot
  fetch replacement data. A current payload plus one old in-flight payload is at most two payloads;
  active recording and encoding storage are separately bounded, not a total JVM memory guarantee.
- The server supports no uploads, filesystem browsing, cross-origin permission or control actions.
  Attachment/no-store/nosniff headers and restrictive page policy reduce accidental interpretation;
  they do not make local HTTP encrypted or authenticated. Publish only non-secret text.

## Controller JSONL v1

The package-private control recording helper owns `sushi-control-response`, version `1`.
It records actual `ControlResponseMetrics` construction, `update`, `retainEvidence` and `finish`
inputs. An unavailable measurement uses `retainEvidence`, not `update(NaN)`. Evidence includes both
output-limiting availability and value, as well as numeric/text facts.

The file contains a START record, ordered SAMPLE/EVIDENCE_ONLY records, a FINISH record with the
original result and omissions, and a SHA-256 integrity record over the preceding encoded bytes.
Numbers preserve round-trip values with explicit nonfinite/unrecorded states. Known candidate,
initial and accepted readbacks, range/topology, session/segment and processing times remain distinct
from unavailable physical-unit mappings, original acquisition time, full robot configuration or
code revision. Never infer those missing identities from the controller tuple.

Retain at most 1,024 sample/evidence-only operations within 512 KiB. Reserve 64 KiB each for start
and finish/integrity; the body has 384 KiB. Oversized metadata makes recording unavailable.
Drop new operations at quota, retain attempted/omitted counts and omission times, and reserve the
ending. Limits do not alter control or the ordinary summary. Existing all-session summary history
is independent and is not newly bounded by this recording cap.

Freeze the original ending before target/readback changes; do not retry an unavailable ending
snapshot. Publish only after required normal zero/hold realization and only while that segment
still owns publication. STOP discards pending export without delaying cleanup. START starts fresh
recording ownership; never join a recording across clock epochs or invent a finish after reset.

The laptop-only reader runs the same real accumulators, not copied equations. It strictly bounds
bytes/records and rejects invalid UTF-8, schema, ordering, duplicates, integrity, truncation and
oversized input. Chunk boundaries cannot change decoding or decisions. Omitted required operations
produce `INCOMPLETE`, not exact replay success. The Gradle task uses existing test classes and
dependencies; the CLI/reader is not packaged into the Android app.

## Evidence and next gate

Tests cover actual tuner inputs/results, missing-feedback branches, precision/Unicode/chunking,
quotas, malformed files, stale links, concurrent/interrupted transfers, child/reset/STOP handling
and unchanged hardware polling/output ownership. Software results establish those contracts only.
Before adopting on a robot, check browser reachability, download interruption, loop cost and the
existing physical stop behavior under reviewed conditions. Neither replay nor a mock HTTP response
proves camera accuracy, controller tuning, robot motion or laptop receipt.
