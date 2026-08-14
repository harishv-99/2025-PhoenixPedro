# Phoenix documentation

Phoenix helps an FTC team build non-blocking TeleOp and Auto programs from a small set of ideas:
sources describe values, bindings turn controls into intent, Tasks coordinate behavior over time,
Plants realize mechanism targets, and one managed program owns the loop.

## New to Phoenix

Begin with two short orientation pages:

- [`Phoenix in five minutes`](<getting-started/Framework Overview.md>) — learn the mental model.
- [`Beginner course checklist`](<getting-started/Beginner's Guide.md>) — see the complete journey
  and its checkpoints.

Then follow the four core lessons in order. Each has one outcome and one observable checkpoint.

1. [`Build and run Phoenix`](<getting-started/Build and Run.md>) — prove the project builds and its
   software tests pass.
2. [`Your first mechanism`](<getting-started/First Mechanism.md>) — configure the intake and run the
   starter's one-motor Auto checkpoint.
3. [`Your first TeleOp`](<getting-started/First TeleOp.md>) — add conservative drive configuration
   and controls.
4. [`Your first Task and Auto`](<getting-started/First Task and Auto.md>) — explain and adapt the
   starter's timed behavior in an autonomous routine.

After the core course, [`Your first Pedro Auto`](<getting-started/First Pedro Auto.md>) is an
**optional software-first track**. It requires no robot motion; any later physical route test needs
a separately configured, localized, calibrated, and tuned robot. Pedro is not required to learn the
ordinary Phoenix structure.

The compiling [`modern starter robot`](<examples/Modern Starter Robot.md>) is the source authority
for the ordinary TeleOp-and-Auto structure.

## Find an answer

| Goal | Start here |
|---|---|
| Remember the ordinary API shape | [`Phoenix Cheat Sheet`](<reference/Phoenix Cheat Sheet.md>) |
| Look up a Phoenix term | [`Glossary`](<reference/Glossary.md>) |
| Fix something that is not working | [`Common Problems`](<troubleshooting/Common Problems.md>) |
| Build a mechanism or choose a Plant recipe | [`FTC Actuators & Plants`](<ftc-boundary/FTC Actuators & Plants.md>) |
| Build a macro or Auto routine | [`Tasks and Macros`](<design/Tasks & Macros Quickstart.md>) |
| Understand sources, edges, or signal shaping | [`Sources & Signals`](<core-concepts/Sources and Signals.md>) |
| Structure a larger robot | [`Architecture Roles, Framework Lanes, and Robot Controls`](<design/Framework Lanes & Robot Controls.md>) |
| Bring up or calibrate hardware | [`Testing & calibration`](<testing-calibration/README.md>) |
| Tune a supported velocity or position controller | [`Control Tuning Workflow`](<testing-calibration/Control Tuning Workflow.md>) |
| Add drive guidance or localization | [`Drive & vision`](<drive-vision/README.md>) |
| Build a Pedro autonomous | [`Pedro autonomous reference`](<examples/Pedro Autonomous Reference.md>) |
| Understand the production Phoenix robot | [`Phoenix production reference`](<../../robots/phoenix/README.md>) |
| Maintain or extend the framework | [`Maintainers`](<maintainers/README.md>) |

## Browse by audience

### Students building a robot

- [`Getting started`](<getting-started/README.md>) — the linear first-robot course
- [`Reference`](<reference/README.md>) — short lookups and exact current vocabulary
- [`Troubleshooting`](<troubleshooting/README.md>) — symptom-led recovery
- [`Examples`](<examples/README.md>) — clearly labeled starters, labs, and case studies
- [`Testing & calibration`](<testing-calibration/README.md>) — physical bring-up in a safe order

### Students adding a feature

- [`Core concepts`](<core-concepts/README.md>) — loop and source semantics
- [`Design`](<design/README.md>) — Tasks, capability families, supervisors, and output queues
- [`FTC boundary`](<ftc-boundary/README.md>) — actuator, sensor, UI, and handoff APIs
- [`Drive & vision`](<drive-vision/README.md>) — spatial queries, guidance, and localization
- [`Pedro Pathing integration`](<../integrations/pedro/README.md>) — the narrow vendor boundary

### Mentors and framework maintainers

- [`Framework Principles`](<../Framework Principles.md>) — architecture and API design authority
- [`Maintainer notes`](<maintainers/Maintainer Notes.md>) — tests and extension notes
- [`Phoenix Architecture`](<../../robots/phoenix/Phoenix Architecture.md>) — production robot ownership

## Example labels

Examples have different jobs. Check the label before copying one:

- **Copyable starter** — the ordinary managed robot structure to adapt.
- **Optional Pedro integration reference** — a managed integration example to study after the
  software-first Pedro lesson.
- **Concept lab** — a focused framework lesson that may expose advanced lifecycle ceremony.
- **Advanced case study** — a deeper design to study after the beginner course.
- **Production reference** — a real robot with season-specific hardware and strategy.

The repository guides, compiling examples, Javadocs, and tests are the current documentation
authority.
