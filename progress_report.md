# Progress Report - Valetudo Milestone 5: Maintenance / Consumables Card
Current report date: 2026-06-09.

## 1. What changed

This milestone adds display-first maintenance/consumables status for the Valetudo mock-backed path.

- **Runtime snapshots include maintenance**: The VM-managed Valetudo runtime now exposes normalized `maintenance.consumables` entries with stable IDs, operator labels, remaining percent/minutes, optional used/total minutes, status, and detail copy.
- **Valetudo source normalization added**: The HTTP mock source reads `ConsumableMonitoringCapability` status and properties, deriving remaining percentage from max values when the source reports minute-based lifetimes.
- **Fixed mock data added**: The fixed mock snapshot includes main brush, side brush, filter, sensor cleaning, and mop pad consumables.
- **Adapter state added**: `vacuum_adapter` now exposes backend-neutral `snapshot.maintenance.consumables`.
- **Capability descriptor updated**: `capabilities.consumables` is supported only when normalized consumable state exists, and becomes unavailable when the source/runtime is stale, unreachable, or offline. Raw Valetudo capability names remain diagnostics-only.
- **Maintenance card added**: Vacuum Control now shows a compact Maintenance card in the no-map/basic sidebar when normalized consumable support/data exists.

No consumable reset commands were added.

## 2. Which modes are affected or unchanged

- **Valetudo fixed mock runtime**: Affected. Consumable status appears in snapshots and the Vacuum Control Maintenance card.
- **Valetudo HTTP mock source**: Affected. The runtime reads Valetudo consumable status and properties, then exposes normalized maintenance state.
- **Valetudo MQTT mock path**: Existing basic control behavior is unchanged. Consumable MQTT production routing was not expanded.
- **Valetudo no-map/basic Vacuum Control UI**: Affected. The Maintenance card appears below Cleaning Settings when normalized support exists.
- **TurtleBot4/Nav2 simulation**: Unchanged. Simulation does not show the Maintenance card unless a future simulation adapter exposes normalized maintenance support.

## 3. Ownership boundaries

- Valetudo raw consumable types, subtypes, capability names, and HTTP endpoint details stay inside runtime/source diagnostics.
- The VM runtime owns Valetudo source reads, max-value interpretation, reachability/staleness handling, and conversion into normalized maintenance items.
- The Valetudo adapter owns backend-neutral descriptors and `snapshot.maintenance` mapping.
- Vacuum Control renders only `snapshot.maintenance` and `snapshot.capabilities.consumables`; it does not branch on backend IDs, raw Valetudo names, diagnostics, endpoint paths, MQTT topics, or source URLs.

## 4. Webview close/reopen behavior

No persistent UI-local authority was added. On close/reopen, the card hydrates from the latest adapter/runtime snapshot. Consumable values come from runtime state.

## 5. Real hardware compatibility

No real hardware support was added or validated. The shape is compatible with future Valetudo-compatible hardware because it uses Valetudo consumable status internally and exposes only normalized product fields externally. Real hardware validation remains pending.

## 6. Feature behavior changed

- Consumables are no longer deferred diagnostics-only for the mock/runtime/adapter/UI display path.
- Stale, unreachable, and offline source states make `capabilities.consumables` unavailable with readable reasons.
- If raw `ConsumableMonitoringCapability` is detected but no normalized consumable state exists, `consumables` remains detected-not-ready.
- Reset consumable commands remain out of scope.
- No Valetudo map rendering, go-to/navigation, mapping, Clean Area, rooms/zones/segments, diagnostics drawer, OpenClaw, MQTT production hardening, or real hardware support was added.

## 7. Files changed

- `/home/shane/firecracker-vm/tensorfleet-mgr/valetudo_runtime.go`
- `/home/shane/firecracker-vm/tensorfleet-mgr/handlers_test.go`
- `panels-standalone/src/vacuum-adapter/state.ts`
- `panels-standalone/src/vacuum-adapter/backends/valetudo/runtimeContract.ts`
- `panels-standalone/src/vacuum-adapter/backends/valetudo/types.ts`
- `panels-standalone/src/vacuum-adapter/backends/valetudo/capabilityMapper.ts`
- `panels-standalone/src/vacuum-adapter/backends/valetudo/stateMapper.ts`
- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.css`
- `scripts/vacuum-adapter-regression.ts`
- `review.md`
- `progress_report.md`

## 8. Tests / validation run

Passed on 2026-06-09:

```sh
bun run test:vacuum-adapter
go test ./...
bun run --cwd panels-standalone build
git diff --check
```

The panel build emitted only existing Vite/browser externalization, eval, and bundle-size warnings.

Manual live webview validation was not run.
Real hardware validation has not been run.
TurtleBot4/Nav2 live simulation validation has not been run.
