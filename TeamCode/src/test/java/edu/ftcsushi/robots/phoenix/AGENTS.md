# Phoenix Application Test Instructions

This file applies to the Phoenix application test subtree. Before changing these tests, read the
repository-root instructions, the production application's
`TeamCode/src/main/java/edu/ftcsushi/robots/phoenix/AGENTS.md`, the Sushi Framework Principles, and
the Phoenix Architecture.

- Tests here may import Phoenix production code because both trees form one application bubble.
- Keep hardware-free application tests aligned with the ownership and lifecycle contracts in the
  production application instructions.
- Put reusable framework behavior and independent teaching scenarios in their framework or example
  test packages; do not turn this subtree into a second shared API surface.
- Keep the application-boundary regression strict. Narrow exceptions require an explicitly approved
  application-boundary change, not convenience for a new outside dependency.
