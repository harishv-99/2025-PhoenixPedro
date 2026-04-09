# Phoenix Documentation Guide

Phoenix now uses a layered documentation map so the repo stays navigable whether you start from the
repo root, browse inside `docs/`, or click directly into a section folder on GitHub.

There are three main landing points:

1. `README.md` at the repo root for quick orientation
2. [`docs/README.md`](<docs/README.md>) for the full documentation hub and reading paths
3. a short `README.md` inside each major `docs/...` section for local navigation

[`Framework Principles.md`](<Framework Principles.md>) intentionally remains at the repo root. It is
Phoenix's long-lived design reference.

## Start here

- [`docs/README.md`](<docs/README.md>) — full docs hub and reading paths
- [`Framework Principles.md`](<Framework Principles.md>) — durable design reference
- [`docs/getting-started/README.md`](<docs/getting-started/README.md>) — first-time Phoenix ramp
- [`docs/design/README.md`](<docs/design/README.md>) — robot architecture, controls, and ownership
- [`docs/testing-calibration/README.md`](<docs/testing-calibration/README.md>) — bring-up and calibration order

## Common reading paths

### I am new to Phoenix

1. [`docs/getting-started/Framework Overview.md`](<docs/getting-started/Framework Overview.md>)
2. [`docs/getting-started/Beginner's Guide.md`](<docs/getting-started/Beginner's Guide.md>)
3. [`docs/design/Framework Lanes & Robot Controls.md`](<docs/design/Framework Lanes & Robot Controls.md>)
4. [`docs/core-concepts/Loop Structure.md`](<docs/core-concepts/Loop Structure.md>)
5. [`docs/design/Tasks & Macros Quickstart.md`](<docs/design/Tasks & Macros Quickstart.md>)

### I want a clean TeleOp + Auto robot design

1. [`docs/design/README.md`](<docs/design/README.md>)
2. [`docs/design/Framework Lanes & Robot Controls.md`](<docs/design/Framework Lanes & Robot Controls.md>)
3. [`docs/design/Robot Capabilities & Mode Clients.md`](<docs/design/Robot Capabilities & Mode Clients.md>)
4. [`docs/design/Recommended Robot Design.md`](<docs/design/Recommended Robot Design.md>)
5. [`docs/design/Supervisors & Pipelines.md`](<docs/design/Supervisors & Pipelines.md>)
6. [`docs/design/Output Tasks & Queues.md`](<docs/design/Output Tasks & Queues.md>)

### I am tuning sensors, localization, or calibration

1. [`docs/testing-calibration/README.md`](<docs/testing-calibration/README.md>)
2. [`docs/testing-calibration/Robot Calibration Tutorials.md`](<docs/testing-calibration/Robot Calibration Tutorials.md>)
3. [`docs/ftc-boundary/FTC Sensors.md`](<docs/ftc-boundary/FTC Sensors.md>)
4. [`docs/drive-vision/AprilTag Practice Setup.md`](<docs/drive-vision/AprilTag Practice Setup.md>)
5. [`docs/drive-vision/AprilTag Localization & Fixed Layouts.md`](<docs/drive-vision/AprilTag Localization & Fixed Layouts.md>)

### I am maintaining or extending the framework

1. [`Framework Principles.md`](<Framework Principles.md>)
2. [`docs/maintainers/README.md`](<docs/maintainers/README.md>)
3. [`docs/maintainers/Maintainer Notes.md`](<docs/maintainers/Maintainer Notes.md>)
4. [`docs/maintainers/AprilTag Localization Follow-Ups.md`](<docs/maintainers/AprilTag Localization Follow-Ups.md>)
5. [`integrations/pedro/README.md`](<integrations/pedro/README.md>)

## Section hubs

- [`docs/README.md`](<docs/README.md>)
- [`docs/getting-started/README.md`](<docs/getting-started/README.md>)
- [`docs/core-concepts/README.md`](<docs/core-concepts/README.md>)
- [`docs/design/README.md`](<docs/design/README.md>)
- [`docs/drive-vision/README.md`](<docs/drive-vision/README.md>)
- [`docs/testing-calibration/README.md`](<docs/testing-calibration/README.md>)
- [`docs/ftc-boundary/README.md`](<docs/ftc-boundary/README.md>)
- [`docs/examples/README.md`](<docs/examples/README.md>)
- [`docs/maintainers/README.md`](<docs/maintainers/README.md>)

## Integration notes

Optional integration docs may still live next to the code they describe. Right now that includes:

- [`integrations/pedro/README.md`](<integrations/pedro/README.md>) — Phoenix's optional Pedro Pathing bridge
