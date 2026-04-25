# Phoenix docs hub

Use this page when you want the full documentation map from inside the `docs/` tree.

For the durable framework design reference, also keep [`../Framework Principles.md`](<../Framework Principles.md>) nearby.

## Start here

If you are new to Phoenix, read in this order:

1. [`getting-started/README.md`](<getting-started/README.md>)
2. [`getting-started/Framework Overview.md`](<getting-started/Framework Overview.md>)
3. [`getting-started/Beginner's Guide.md`](<getting-started/Beginner's Guide.md>)
4. [`design/Framework Lanes & Robot Controls.md`](<design/Framework Lanes & Robot Controls.md>)
5. [`core-concepts/Loop Structure.md`](<core-concepts/Loop Structure.md>)
6. [`design/Tasks & Macros Quickstart.md`](<design/Tasks & Macros Quickstart.md>)

## Reading paths by goal

### I want to write my first Phoenix robot

- [`getting-started/README.md`](<getting-started/README.md>)
- [`getting-started/Framework Overview.md`](<getting-started/Framework Overview.md>)
- [`getting-started/Beginner's Guide.md`](<getting-started/Beginner's Guide.md>)
- [`design/Framework Lanes & Robot Controls.md`](<design/Framework Lanes & Robot Controls.md>)
- [`design/Tasks & Macros Quickstart.md`](<design/Tasks & Macros Quickstart.md>)

### I want a clean TeleOp + Auto robot design

- [`design/README.md`](<design/README.md>)
- [`design/Framework Lanes & Robot Controls.md`](<design/Framework Lanes & Robot Controls.md>)
- [`design/Robot Capabilities & Mode Clients.md`](<design/Robot Capabilities & Mode Clients.md>)
- [`design/Recommended Robot Design.md`](<design/Recommended Robot Design.md>)
- [`design/Supervisors & Pipelines.md`](<design/Supervisors & Pipelines.md>)
- [`design/Output Tasks & Queues.md`](<design/Output Tasks & Queues.md>)
- [`../Framework Principles.md`](<../Framework Principles.md>)

### I am building signal pipelines or stateful source composition

- [`core-concepts/README.md`](<core-concepts/README.md>)
- [`core-concepts/Loop Structure.md`](<core-concepts/Loop Structure.md>)
- [`core-concepts/Sources and Signals.md`](<core-concepts/Sources and Signals.md>)
- [`ftc-boundary/FTC Sensors.md`](<ftc-boundary/FTC Sensors.md>)
- [`design/Recommended Robot Design.md`](<design/Recommended Robot Design.md>)

### I am building drive assists, localization, or AprilTag tooling

- [`drive-vision/README.md`](<drive-vision/README.md>)
- [`drive-vision/Drive Guidance.md`](<drive-vision/Drive Guidance.md>)
- [`drive-vision/AprilTag Localization & Fixed Layouts.md`](<drive-vision/AprilTag Localization & Fixed Layouts.md>)
- [`drive-vision/AprilTag Practice Setup.md`](<drive-vision/AprilTag Practice Setup.md>)
- [`testing-calibration/Robot Calibration Tutorials.md`](<testing-calibration/Robot Calibration Tutorials.md>)

### I want to bring up and calibrate a robot from scratch

- [`testing-calibration/README.md`](<testing-calibration/README.md>)
- [`testing-calibration/Robot Calibration Tutorials.md`](<testing-calibration/Robot Calibration Tutorials.md>)
- [`testing-calibration/Guided Calibration Walkthroughs.md`](<testing-calibration/Guided Calibration Walkthroughs.md>)
- [`ftc-boundary/FTC Sensors.md`](<ftc-boundary/FTC Sensors.md>)
- [`drive-vision/AprilTag Practice Setup.md`](<drive-vision/AprilTag Practice Setup.md>)

### I want a concrete end-to-end example

- [`examples/README.md`](<examples/README.md>)
- [`examples/Examples Progression & Layered Mechanisms.md`](<examples/Examples Progression & Layered Mechanisms.md>)
- [`examples/Layered Shooter Example.md`](<examples/Layered Shooter Example.md>)
- [`examples/Shooter Case Study & Examples Walkthrough.md`](<examples/Shooter Case Study & Examples Walkthrough.md>)
- [`../tools/examples/`](<../tools/examples/>)

### I am maintaining or extending the framework

- [`maintainers/README.md`](<maintainers/README.md>)
- [`../Framework Principles.md`](<../Framework Principles.md>)
- [`maintainers/Maintainer Notes.md`](<maintainers/Maintainer Notes.md>)
- [`maintainers/Future Robot Boilerplate Helpers.md`](<maintainers/Future Robot Boilerplate Helpers.md>)
- [`maintainers/AprilTag Localization Follow-Ups.md`](<maintainers/AprilTag Localization Follow-Ups.md>)
- [`../integrations/pedro/README.md`](<../integrations/pedro/README.md>)

## Browse by section

- [`getting-started/README.md`](<getting-started/README.md>) — overview and beginner ramp
- [`core-concepts/README.md`](<core-concepts/README.md>) — loop semantics and source composition
- [`design/README.md`](<design/README.md>) — robot architecture, tasks, queues, supervisors
- [`drive-vision/README.md`](<drive-vision/README.md>) — drive guidance and AprilTag localization
- [`testing-calibration/README.md`](<testing-calibration/README.md>) — ordered bring-up and walkthrough guidance
- [`ftc-boundary/README.md`](<ftc-boundary/README.md>) — FTC-boundary sensor adapters and placement rules
- [`examples/README.md`](<examples/README.md>) — case studies and runnable examples
- [`maintainers/README.md`](<maintainers/README.md>) — maintainer-only references

## Notes on structure

- The repo root [`../README.md`](<../README.md>) is the quick orientation page.
- This file is the full docs hub.
- Each major docs folder has its own `README.md` so folder-level browsing still has an obvious entrypoint.
- Some docs intentionally live next to code instead of under `docs/` (for example optional integrations); when that happens, link them from here so they stay discoverable.
