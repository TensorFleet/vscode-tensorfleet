# Progress Report - Milestone 7 Expanded Diagnostics
Current report date: 2026-06-05.

## 1. What changed

The VM-managed Valetudo integration runtime now exposes richer diagnostics for operators and future backend work without enabling new product UI controls.

Runtime snapshots still use the same `/health`, `/snapshot`, and `/command` API paths, but `/snapshot.diagnostics` now includes source freshness details, stale reasons, last successful source update, last source error, last command result, and capability tiers. Raw Valetudo capability names remain diagnostics only.

## 2. Which mode this affects

- Mapping: unchanged; Valetudo map rendering remains unsupported.
- Navigation: unchanged; go-to remains diagnostics-only.
- Clean Area: unchanged; Valetudo coverage remains unsupported.
- Rooms / Zones: unchanged; detected segment/zone capabilities remain diagnostics-only.
- Valetudo backend: runtime diagnostics are expanded.
- Shared adapter/runtime architecture: endpoint paths and product-facing adapter behavior remain stable.

## 3. Ownership check

VM runtime owns source diagnostics, stale reasons, transport diagnostics, capability tiers, and last command audit.

Valetudo backend adapter continues to map runtime snapshots into `vacuum_adapter`; it does not interpret raw Valetudo capability names as product behavior.

React/webview state remains presentation-only. The UI renders normalized adapter state and submits normalized commands such as `start_cleaning`, `pause`, `stop`, and `return_to_dock`.

This follows the rule: Product UI renders normalized adapter state and submits normalized commands. Backend adapter maps backend runtime state into `vacuum_adapter`. VM runtime owns backend connection, state cache, diagnostics, and command routing.

## 4. Webview close/reopen behavior

- idle Valetudo mock state: reopening polls `/snapshot` and hydrates from normalized runtime state plus diagnostics.
- unavailable VM runtime: the adapter still falls back to a safe offline snapshot.
- reachable mock runtime: reopening receives current source freshness and transport diagnostics.
- active mock cleaning state: reopening hydrates active cleaning state and preserves last command diagnostics when runtime memory is still alive.
- paused mock cleaning state: reopening hydrates paused state from runtime snapshot.
- terminal or stopped mock state: reopening hydrates the stopped/idle state, or stale/unavailable if the source is down.

The UI hydrates through `useVacuumAdapter`; it does not call Valetudo HTTP, MQTT, or VM runtime internals directly.

## 5. Real hardware compatibility check

- Does this assume the robot is TurtleBot4/Nav2? No.
- Does this expose TurtleBot4/Nav2 specifics to product UI? No.
- Does this expose Valetudo raw capability names to product UI? No; raw names live in runtime diagnostics and adapter test fixtures only.
- Can the same VM runtime API later connect to a real Valetudo robot? Yes; the diagnostics are source/transport metadata behind the same runtime API.
- What capability flags decide whether controls are shown/enabled? Normalized `vacuum_adapter` capabilities derived from runtime command availability.
- What operations are explicitly unsupported? Valetudo map rendering, go-to, Clean Area/coverage, rooms/zones, segment cleaning, fan/water setters, consumables UI, manual control, camera, and annotations.

## 6. Feature behavior changed

- Runtime `/snapshot.diagnostics.source` reports source kind, status, stale state, stale reason, last poll, last successful update, last error, and configured source URL.
- Runtime `/snapshot.diagnostics.lastCommand` reports the last structured runtime command result.
- Runtime `/snapshot.diagnostics.capabilityTiers` groups detected raw Valetudo capabilities into implemented, detected-not-ready, detected-not-implemented, and deferred tiers.
- HTTP and MQTT transport diagnostics now include last success and stale reason fields.
- Runtime version advanced to `0.7.0-layer6a-m7`.
- No new Valetudo controls were enabled in the product UI.

## 7. Files changed

- `/home/shane/firecracker-vm/tensorfleet-mgr/valetudo_runtime.go`: added runtime-owned diagnostics state, source diagnostics, last command audit, capability tiers, HTTP poll tracking, and Milestone 7 runtime version.
- `/home/shane/firecracker-vm/tensorfleet-mgr/valetudo_mqtt.go`: added MQTT source diagnostics, MQTT stale reasons, and expanded transport diagnostics.
- `/home/shane/firecracker-vm/tensorfleet-mgr/handlers_test.go`: added tests for capability tiers, last command diagnostics, HTTP source freshness/error preservation, and MQTT stale reasons.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/runtimeContract.ts`: mirrored optional diagnostics fields for the extension-side runtime contract.
- `/home/shane/vscode-tensorfleet/progress_report.md`: recorded Milestone 7 behavior, validation, risks, and contract status.

## 8. Tests / validation run

```sh
go test ./...
bun run test:vacuum-adapter
bun run typecheck
git diff --check
git -C /home/shane/firecracker-vm diff --check
git -C /home/shane/vm-manager diff --check
```

Passed:

```text
go test ./... in /home/shane/firecracker-vm/tensorfleet-mgr
bun run test:vacuum-adapter in /home/shane/vscode-tensorfleet
git diff --check in /home/shane/vscode-tensorfleet
git -C /home/shane/firecracker-vm diff --check
git -C /home/shane/vm-manager diff --check
```

`bun run typecheck` still fails on pre-existing unused variables in `/home/shane/vscode-tensorfleet/packages/tensorfleet-auth/src/oauth-core.ts`: `callbackBaseUrl` and `actualPort`.

Manual live VM endpoint validation was not rerun in this pass. The VM and mock source were reported running by the operator, and this milestone was validated with runtime tests around the VM runtime API handlers.

## 9. Remaining risks

- Diagnostics are runtime-memory based; last command audit is not durable across runtime restart.
- MQTT still has not been validated against a live broker in this pass.
- Real hardware is not connected.
- Raw payload sampling remains deferred; raw capability names are present, but full payload samples are not exposed.
- Live VS Code webview operator validation was not performed in this pass.
- Command availability is still broad when `BasicControlCapability` is present.

## 10. Next recommended step

Proceed to Milestone 8 documentation and Layer 6A summary after a quick live VM validation of `/health`, `/snapshot`, `/command`, and the Valetudo webview path.

## 11. Contract changes

- Runtime API changed? Endpoint paths did not change. Snapshot diagnostics gained `source`, `lastCommand`, `capabilityTiers`, transport `lastSuccessAt`, and transport `staleReason`.
- `vacuum_adapter` public contract changed? No.
- Capability descriptors changed? No public descriptor shape changed.
- UI behavior changed? No direct UI behavior changed; controls remain capability-gated through normalized adapter state.
- Backward compatibility impact? The extension-side diagnostics fields are optional, so Milestone 6 runtime responses still type-check. TurtleBot4/Nav2 remains the default product backend and regression path.
