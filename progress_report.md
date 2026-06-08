# Progress Report - Valetudo Milestone 2A Command Hardening
Current report date: 2026-06-08.

## 1. What changed

This pass hardened Valetudo command result normalization and command availability without adding new product features.

- Fixed mock invalid-state command failures now return normalized `invalid_state` instead of the old mock-only `command_invalid_state`.
- Fixed mock now rejects unsafe repeat/state-conflicting `start_cleaning`, `pause`, `stop`, and `return_to_dock` requests with normalized invalid-state failures.
- MQTT command routing now returns `stale_source` when the runtime cache is stale, instead of allowing a command based on old source state.
- The Valetudo adapter now maps runtime-only aliases into shared `VacuumCommandErrorCode` values:
  `unsupported_command` and `capability_unavailable` -> `unsupported`,
  `command_invalid_state` -> `invalid_state`,
  `invalid_json` and `missing_command` -> `invalid_request`,
  `source_command_failed` and MQTT publish failure aliases -> `backend_error`,
  malformed backend response aliases -> `malformed_backend_response`.
- Capability descriptor `reasons[]` now keeps the stable reason code but stores readable messages such as "Robot state is stale." for operator-facing disabled reasons.
- The existing no-map/basic UI humanizer was extended for the normalized Valetudo command/source reasons. No advanced UI or new controls were added.

## 2. Which mode this affects

- Valetudo backend: Affected. Basic command failures and disabled reasons are now normalized across runtime, adapter, and UI boundary.
- Fixed mock Valetudo runtime: Affected. State-invalid basic commands now fail consistently.
- HTTP Valetudo mock source: Affected through adapter normalization and added source command failure coverage; command routing remains basic-control only.
- MQTT Valetudo mock source: Affected. Stale cached source state blocks command publishing with `stale_source`.
- Mapping: Unchanged. Valetudo map rendering remains unsupported.
- Navigation/go-to: Unchanged. Detected Valetudo go-to remains diagnostics-only/detected-not-ready.
- Clean Area: Unchanged. Coverage remains unsupported.
- Rooms / Zones: Unchanged. Segment/zone detection remains deferred and not usable controls.
- Fan/water/consumables: Unchanged. These remain unsupported or detected-not-ready only.
- TurtleBot4/Nav2 simulation: Unchanged except shared tests still pass.

## 3. Ownership check

- VM runtime owns source reachability, source freshness, fixed mock state transitions, HTTP/MQTT command routing, and command audit diagnostics.
- Backend adapter owns translation from runtime command result shapes into shared `VacuumCommandResult` / `VacuumCommandError` values.
- Shared `vacuum_adapter` continues to own the public command error union. Raw runtime aliases do not become public shared error codes.
- UI owns only presentation of command errors and disabled reasons. It does not use diagnostics to decide whether controls render or enable.
- Raw Valetudo capability names remain diagnostics-only. Product behavior still gates on normalized descriptors and state.

No new product feature or hardware path was added.

## 4. Webview close/reopen behavior

The webview still hydrates Valetudo state from the adapter snapshot, which is mapped from the VM runtime snapshot or a synthetic runtime-unavailable boundary.

- Command availability after close/reopen is runtime/snapshot-owned through normalized capability descriptors.
- Local command error banners remain local UI presentation and are not durable runtime state.
- Runtime `lastCommand` remains diagnostics/audit data only; it does not drive product control gating after reopen.
- Stale/unreachable/offline source state rehydrates from `snapshot.source`, `snapshot.health`, and `snapshot.capabilities`.

Manual live webview validation was not performed for this milestone.

## 5. Real hardware compatibility check

- Does this assume TurtleBot4/Nav2? No.
- Does this expose raw Valetudo capability names to product UI? No.
- Does this add real hardware support? No.
- Does this preserve a path to real Valetudo-compatible hardware? Yes. The runtime API still exposes health, snapshot, and command results; command errors are now more stable for future real-source failures.
- What remains deferred? Map rendering, go-to navigation, rooms/zones/segments, Clean Area, fan speed, water usage, consumables, OpenClaw, MQTT production behavior, and real hardware validation.

## 6. Feature behavior changed

- `start_cleaning`, `pause`, `stop`, and `return_to_dock` now have stricter invalid-state behavior in the fixed mock runtime.
- Runtime stale source, source unreachable, invalid request, unsupported command/capability, invalid robot state, and source command failure now map predictably through the adapter.
- Operator-facing disabled reasons continue to render readable copy rather than raw reason codes.
- Deferred capabilities remain unsupported or detected-not-ready and do not become usable controls.

## 7. Files changed

- `/home/shane/firecracker-vm/tensorfleet-mgr/valetudo_runtime.go`
  Normalized fixed mock invalid-state command failures and added state checks for basic commands.
- `/home/shane/firecracker-vm/tensorfleet-mgr/valetudo_mqtt.go`
  Blocks MQTT command publishing when cached source state is stale.
- `/home/shane/firecracker-vm/tensorfleet-mgr/handlers_test.go`
  Adds runtime coverage for malformed requests, invalid-state basic commands, source command failures, and stale MQTT command failures.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/commandMapper.ts`
  Collapses runtime-specific command aliases into shared command error codes and preserves readable messages.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/capabilityMapper.ts`
  Keeps descriptor reason codes normalized while using readable reason messages.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
  Extends the existing reason humanizer for Valetudo command/source codes.
- `/home/shane/vscode-tensorfleet/scripts/vacuum-adapter-regression.ts`
  Adds adapter/UI boundary regressions for command result aliases and readable disabled reasons.
- `/home/shane/vscode-tensorfleet/review.md`
  Refreshes the Milestone 1 inventory notes that were made stale by this hardening pass.
- `/home/shane/vscode-tensorfleet/progress_report.md`
  Replaces the inventory report with this Milestone 2A implementation report.

Note: `VacuumControlPanel.css` and broader no-map UI edits were already present in the dirty worktree and were not part of the command normalization patch.

## 8. Tests / validation run

```sh
bun run test:vacuum-adapter
go test ./...                         # in /home/shane/firecracker-vm/tensorfleet-mgr
bun run --cwd panels-standalone build
git diff --check                      # in /home/shane/vscode-tensorfleet
git diff --check                      # in /home/shane/firecracker-vm/tensorfleet-mgr
```

All commands passed. The panel build emitted existing Vite/browser externalization and bundle-size warnings only.

Manual live webview validation was not run.
Real hardware validation was not run.
TurtleBot4/Nav2 live simulation validation was not run.
