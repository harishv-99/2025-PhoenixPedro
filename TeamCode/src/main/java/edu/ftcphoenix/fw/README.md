# Phoenix Documentation Guide

Phoenix now organizes its documentation by **reading path** instead of keeping every guide at the
repository root. The goal is to keep the docs easier to navigate without losing the deeper rationale
and examples that make the framework usable in practice.

`Framework Principles.md` intentionally remains separate at the root. It is the durable design
reference for the framework itself.

## Start here

If you are new to Phoenix, read in this order:

1. [`docs/getting-started/Framework Overview.md`](<docs/getting-started/Framework Overview.md>)
2. [`docs/getting-started/Beginner's Guide.md`](<docs/getting-started/Beginner's Guide.md>)
3. [`docs/design/Recommended Robot Design.md`](<docs/design/Recommended Robot Design.md>)
4. [`docs/core-concepts/Loop Structure.md`](<docs/core-concepts/Loop Structure.md>)
5. [`docs/design/Tasks & Macros Quickstart.md`](<docs/design/Tasks & Macros Quickstart.md>)

## Reading paths by goal

### I want to write my first Phoenix robot

- [`docs/getting-started/Framework Overview.md`](<docs/getting-started/Framework Overview.md>)
- [`docs/getting-started/Beginner's Guide.md`](<docs/getting-started/Beginner's Guide.md>)
- [`docs/core-concepts/Loop Structure.md`](<docs/core-concepts/Loop Structure.md>)
- [`docs/design/Tasks & Macros Quickstart.md`](<docs/design/Tasks & Macros Quickstart.md>)

### I want a clean TeleOp + Auto robot design

- [`docs/design/Recommended Robot Design.md`](<docs/design/Recommended Robot Design.md>)
- [`docs/design/Supervisors & Pipelines.md`](<docs/design/Supervisors & Pipelines.md>)
- [`docs/design/Output Tasks & Queues.md`](<docs/design/Output Tasks & Queues.md>)
- [`Framework Principles.md`](<Framework Principles.md>)

### I am building drive assists, auto-aim, or route integration

- [`docs/drive-vision/Drive Guidance.md`](<docs/drive-vision/Drive Guidance.md>)
- [`docs/drive-vision/AprilTag Localization & Fixed Layouts.md`](<docs/drive-vision/AprilTag Localization & Fixed Layouts.md>)
- [`docs/design/Recommended Robot Design.md`](<docs/design/Recommended Robot Design.md>)
- [`docs/core-concepts/Sources and Signals.md`](<docs/core-concepts/Sources and Signals.md>)
- [`docs/core-concepts/Loop Structure.md`](<docs/core-concepts/Loop Structure.md>)

### I am wiring sensors or calibrating AprilTag setups

- [`docs/ftc-boundary/FTC Sensors.md`](<docs/ftc-boundary/FTC Sensors.md>)
- [`docs/drive-vision/AprilTag Practice Setup.md`](<docs/drive-vision/AprilTag Practice Setup.md>)
- [`docs/drive-vision/AprilTag Localization & Fixed Layouts.md`](<docs/drive-vision/AprilTag Localization & Fixed Layouts.md>)
- [`docs/drive-vision/Drive Guidance.md`](<docs/drive-vision/Drive Guidance.md>)

### I want to bring up and calibrate a robot from scratch

- [`docs/testing-calibration/Robot Calibration Tutorials.md`](<docs/testing-calibration/Robot Calibration Tutorials.md>)
- [`docs/testing-calibration/Guided Calibration Walkthroughs.md`](<docs/testing-calibration/Guided Calibration Walkthroughs.md>)
- [`docs/drive-vision/AprilTag Localization & Fixed Layouts.md`](<docs/drive-vision/AprilTag Localization & Fixed Layouts.md>)
- [`docs/ftc-boundary/FTC Sensors.md`](<docs/ftc-boundary/FTC Sensors.md>)

### I want a concrete end-to-end example

- [`docs/examples/Shooter Case Study & Examples Walkthrough.md`](<docs/examples/Shooter Case Study & Examples Walkthrough.md>)
- `tools/examples/*`

### I am maintaining or extending the framework

- [`Framework Principles.md`](<Framework Principles.md>)
- [`docs/maintainers/Maintainer Notes.md`](<docs/maintainers/Maintainer Notes.md>)
- [`docs/maintainers/AprilTag Localization Follow-Ups.md`](<docs/maintainers/AprilTag Localization Follow-Ups.md>)
- [`docs/drive-vision/AprilTag Localization & Fixed Layouts.md`](<docs/drive-vision/AprilTag Localization & Fixed Layouts.md>)
- [`docs/getting-started/Framework Overview.md`](<docs/getting-started/Framework Overview.md>)

## Documentation structure

- `docs/getting-started/`
  - overview, beginner ramp, and the "what exists?" map
- `docs/design/`
  - the recommended robot design, supervisors/subsystems, tasks, and output queues
- `docs/core-concepts/`
  - loop semantics and source/signal composition
- `docs/drive-vision/`
  - drive guidance, AprilTag localization policy, and practice setup
- `docs/testing-calibration/`
  - ordered calibration tutorials and walkthrough-builder guidance
- `docs/ftc-boundary/`
  - FTC-boundary sensor notes and recommendations
- `docs/examples/`
  - case studies that connect the concepts to real Phoenix examples
- `docs/maintainers/`
  - advanced notes for maintainers and framework extensions

## Notes on what changed

- The old `Behavior Lanes.md` content now lives inside
  [`docs/design/Recommended Robot Design.md`](<docs/design/Recommended Robot Design.md>), because
  it is most useful when paired with concrete robot-design recommendations and examples.
- `Notes.md` was renamed and moved to
  [`docs/maintainers/Maintainer Notes.md`](<docs/maintainers/Maintainer Notes.md>) so advanced
  maintainer guidance is still available without cluttering the main reading path.
- `Framework Principles.md` stays separate on purpose. It is the long-lived design reference and
  should remain easy to find.
