# Progress Report - Current Statistics Normalization
Current report date: 2026-06-12.

## 1. What changed

- Extended the VM-managed Valetudo integration runtime snapshot with `statistics.current`, including deterministic fixed mock values for current run duration and cleaned area.
- Added HTTP-source normalization for `CurrentStatisticsCapability` when the source reports parseable current statistics, including Valetudo-style `ValetudoDataPoint` arrays for `time` and `area`; missing or malformed source statistics are omitted without failing the snapshot.
- Extended the normalized `vacuum_adapter` contract with `snapshot.statistics.current` and a backend-neutral `statistics` capability with `attributes: ["current"]` and no commands.
- Mapped Valetudo runtime current statistics into the adapter snapshot only when the source is fresh and reachable; stale, unreachable, offline, or malformed statistics are absent and unsupported.
- Added a compact Current Statistics card to the Valetudo/no-map Vacuum Control sidebar, gated only by normalized `snapshot.statistics.current` plus `capabilities.statistics`.
- Updated `firecracker-vm/tensorfleet-mgr` runtime structs, fixed mock snapshot construction, HTTP source normalization, capability tier diagnostics, and handler tests.
- Synced the updated `tensorfleet-mgr` binary to the live VM at `172.16.0.10`, backed up the prior binary, restarted `tensorfleet-mgr.service`, and verified the live Valetudo runtime snapshot now includes `statistics.current`.
- No VS Code `vm-manager` proxy changes were required after inspection: the panel runtime client already uses the generic `/vms/self/tensorfleet/api/v1/valetudo` proxy path, and there is no typed Valetudo snapshot schema in `src/vm-manager.ts`.
- Updated adapter regression tests for fixed mock statistics, supported statistics capability, omitted statistics, malformed statistics, stale/unreachable source handling, and existing Valetudo controls/settings/maintenance coverage.

## 2. Product behavior

- Operators using the Valetudo/no-map Vacuum Control sidebar now see a Current Statistics card with current run duration and cleaned area when normalized current statistics are available.
- The card appears only when `capabilities.statistics.supported` is true and `snapshot.statistics.current` contains finite duration or area data.
- Missing, malformed, stale, unreachable, or offline statistics do not crash the runtime or UI and do not show a misleading card.
- The feature is read-only and backend-neutral: product UI does not depend on raw Valetudo capability names, HTTP routes, MQTT topics, source URLs, cache internals, or backend-specific response shapes.

## 3. Still deferred

- Total statistics.
- Statistics history/charts/export.
- Attachments and dock components.
- Advanced dock actions.
- Consumable reset commands.
- Map rendering and segment/room/zone/go-to behavior.
- Valetudo Clean Area behavior.
- Hardware validation.

## 4. Validation

```sh
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
go test ./...
git diff --check
curl -fsS -H 'Authorization: Bearer default-tensorfleet-token' http://172.16.0.10:9090/api/v1/valetudo/snapshot
```
