---
tags:
  - Reference
---

# Integrations and extension seams quick reference

## Entry points

| Need | API |
|---|---|
| turn one route execution into a truthful Task | [`RouteTasks`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/drive/route/RouteTasks.html>) |
| inspect endpoint, timeout, interruption, replacement, or failure | [`RouteStatus`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/drive/route/RouteStatus.html>) |
| own Pedro's stable follower heartbeat | [`PedroPathingRuntime`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/integrations/pedro/PedroPathingRuntime.html>) |
| adapt Pedro as route follower and final drive sink | [`PedroPathingDriveAdapter`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/integrations/pedro/PedroPathingDriveAdapter.html>) |
| expose Panels-backed tuning at the integration edge | [`FtcPanelsTuners`](<https://harishv-99.github.io/2025-PhoenixPedro/api/edu/ftcsushi/fw/integrations/panels/FtcPanelsTuners.html>) |

## Remember

An integration adapts one foreign lifecycle or value model into a narrow Sushi contract. It does
not become another robot scheduler or final writer. Keep vendor types at the integration edge and
retain status while the boundary still has enough evidence to classify it.

Read the [Pedro integration contract](<../../integrations/pedro/README.md>) and
[Framework Principles](<../../Framework Principles.md>) before adding another seam.
