---
tags:
  - Advanced
---

# Robot design and behavior

These are intermediate and advanced guides. First read
[`How Sushi runs your code`](<../getting-started/Framework Overview.md>) and the
[`Robot roles`](<../getting-started/learn-sushi/Robot Roles.md>) topic relevant to this ownership
vocabulary.

## Structure a robot

- [`Architecture Roles, Framework Lanes, and Robot Controls`](<Framework Lanes & Robot Controls.md>)
  defines primitives, lanes, field facts, controls, capability families, mechanisms, supervisors,
  services, presenters, profiles, and the composition root.
- [`Robot Capabilities and Mode Clients`](<Robot Capabilities & Mode Clients.md>) explains the
  mode-neutral vocabulary shared by TeleOp and Auto.
- [`Recommended Robot Design`](<Recommended Robot Design.md>) compares behavior patterns and shows
  larger mechanism examples.
- [`Supervisors & Pipelines`](<Supervisors & Pipelines.md>) covers robot-owned policy and final
  Plant-target arbitration.

## Build behavior over time

- [`Tasks and Macros`](<Tasks & Macros Quickstart.md>) is the main Task guide.
- [`Output Tasks & Queues`](<Output Tasks & Queues.md>) is the advanced temporary-output and queued
  pulse pattern.

The current lifecycle contract lives in [`Loop Structure`](<../core-concepts/Loop Structure.md>).
The [`Framework Principles`](<../../Framework Principles.md>) govern framework changes rather than
ordinary first-robot use.

[Back to the Sushi docs home](<../README.md>)
