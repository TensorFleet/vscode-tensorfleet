# Progress Report - Milestone 5 Connect Runtime To Actual Valetudo Mock Source
Current report date: 2026-05-29.

## 1. What changed

The VM-managed Valetudo integration runtime now reads live Valetudo mock HTTP data instead of only serving fixed internal data. By default it connects to `http://172.16.0.1:8081`, the host-side Firecracker TAP address, fetches robot identity, state attributes, and capabilities from the Valetudo mock API, and normalizes them into the existing `/health`, `/snapshot`, and `/command` runtime API.

Basic normalized commands now route through Valetudo `BasicControlCapability`: `start_cleaning` -> `start`, `pause` -> `pause`, `stop` -> `stop`, and `return_to_dock` -> `home`. The older fixed mock remains available through `VALETUDO_RUNTIME_SOURCE_MODE=fixed_mock` for regression coverage.

## 2. Which mode this affects

- Mapping: unchanged; Valetudo map rendering remains unsupported.
- Navigation: unchanged; Valetudo go-to remains detected but not product-ready.
- Clean Area: unchanged; Valetudo coverage remains unsupported.
- Rooms / Zones: unchanged; detected raw capabilities remain diagnostics only.
- Valetudo backend: runtime source data now comes from the actual Valetudo mock HTTP API.
- Shared adapter/runtime architecture: endpoint shapes remain stable.

## 3. Ownership check

VM runtime owns the Valetudo mock source connection, source health, state normalization, stale/unreachable behavior, capability diagnostics, and command routing.

Valetudo backend adapter continues to consume only the VM runtime API and map runtime snapshots into `vacuum_adapter`.

React/webview state owns presentation only. It renders normalized adapter state and submits normalized commands such as `start_cleaning`, `pause`, `stop`, and `return_to_dock`.

This follows the rule: Product UI renders normalized adapter state and submits normalized commands. Backend adapter maps backend runtime state into `vacuum_adapter`. VM runtime owns backend connection, state cache, and command routing.

## 4. Webview close/reopen behavior

- idle Valetudo mock state: reopening fetches `/snapshot` and hydrates identity, idle/docked state, battery, dock, and command availability from the mock source.
- unavailable VM runtime: reopening falls back through the existing adapter unavailable path with no map, pose, navigation, or capability-dependent controls.
- reachable mock runtime: polling fetches current runtime data from the Valetudo mock source.
- active mock cleaning state: reopening hydrates from Valetudo `StatusStateAttribute=cleaning`.
- paused mock cleaning state: reopening hydrates from Valetudo `StatusStateAttribute=paused`.
- terminal or stopped mock state: reopening hydrates from Valetudo `StatusStateAttribute=idle` or the current source state.

Hydration continues through `useVacuumAdapter`; the UI does not call Valetudo or VM runtime endpoints directly.

## 5. Real hardware compatibility check

- Does this assume the robot is TurtleBot4/Nav2? No.
- Does this expose TurtleBot4/Nav2 specifics to product UI? No.
- Does this expose Valetudo raw capability names to product UI? No; raw names remain runtime/adapter diagnostics.
- Can the same VM runtime API later connect to a real Valetudo robot? Yes; the source base URL is configurable with `VALETUDO_MOCK_SOURCE_URL` or `VALETUDO_SOURCE_URL`.
- What capability flags decide whether controls are shown/enabled? Normalized `vacuum_adapter` capability descriptors derived from runtime command availability and diagnostics.
- What operations are explicitly unsupported? Valetudo map rendering, pose product surface, navigation/go-to, Clean Area/coverage, mapping sessions, rooms/zones, segment cleaning, fan/water setters, consumables UI, manual control, camera, and annotations.

## 6. Feature behavior changed

- Valetudo mock robot state appears through runtime data read from the actual Valetudo mock HTTP source.
- Runtime `/health` reports `source.kind=valetudo_mock` and reachable/stale source status.
- Runtime `/snapshot` includes live mock robot identity, status, battery, dock, command availability, and raw capability diagnostics.
- Basic commands route from normalized runtime commands to Valetudo `BasicControlCapability`.
- Source failures return stale/unreachable runtime snapshots and structured unavailable command results.

## 7. Files changed

- `/home/shane/firecracker-vm/tensorfleet-mgr/valetudo_runtime.go`: added the Valetudo mock HTTP source client, source snapshot normalization, command routing, source mode configuration, and unreachable-source fallback snapshots.
- `/home/shane/firecracker-vm/tensorfleet-mgr/handlers_test.go`: kept fixed mock tests explicit and added HTTP mock-source tests for snapshot normalization, command routing, and unreachable source behavior.
- `/home/shane/firecracker-vm/tensorfleet-mgr/main.go`: updated endpoint log descriptions from fixed mock wording to runtime wording.
- `/home/shane/firecracker-vm/tensorfleet-mgr/README.md`: documented mock source mode, source URL configuration, and command routing through Valetudo `BasicControlCapability`.
- `/home/shane/vscode-tensorfleet/progress_report.md`: recorded Milestone 5 behavior, validation, risks, and contract status.

## 8. Tests / validation run

```sh
go test ./...
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
bun run typecheck
git diff --check
git -C /home/shane/firecracker-vm diff --check
```

`go test ./...` passed in `/home/shane/firecracker-vm/tensorfleet-mgr`.

`bun run test:vacuum-adapter` passed.

`bun run --cwd panels-standalone build` passed with existing Vite dependency/chunk-size warnings.

`bun run typecheck` failed on pre-existing unused variables in `/home/shane/vscode-tensorfleet/packages/tensorfleet-auth/src/oauth-core.ts` (`callbackBaseUrl`, `actualPort`). No Milestone 5 files were reported.

Manual live runtime validation:

```text
Confirmed Valetudo mock source responded on http://127.0.0.1:8081/api/v2/robot.
Started tensorfleet-mgr locally on PORT=19090.
Called /api/v1/valetudo/health and confirmed source.kind=valetudo_mock, status=reachable.
Called /api/v1/valetudo/snapshot and confirmed MockValetudoRobot identity, docked state, battery, dock, command availability, and raw capability diagnostics.
Triggered start_cleaning, pause, stop, and return_to_dock through /api/v1/valetudo/command and confirmed successful routing.
Called /snapshot after start_cleaning and return_to_dock and confirmed state changed from docked to cleaning to returning.
Stopped the local tensorfleet-mgr process after validation.
```

Unreachable source behavior was validated with automated `httptest` coverage rather than stopping the user-run Valetudo mock process.

## 9. Remaining risks

- Runtime source connection is HTTP only; MQTT remains deferred.
- Runtime source cache is request-time only and not yet a durable background cache.
- Real hardware is not connected.
- Valetudo map data is intentionally ignored by the product-facing adapter path.
- Command availability is still broad when `BasicControlCapability` is present; later milestones may add state-aware availability.
- Live VS Code webview operator validation was not performed in this pass.

## 10. Next recommended step

Implement Milestone 6 only if MQTT adds useful mock-source behavior; otherwise move to Milestone 7 diagnostics hardening by adding explicit runtime diagnostics for source URL, last successful poll, last error, and detected-but-unimplemented capabilities without exposing raw payloads to product UI.

## 11. Contract changes

- Runtime API changed? Endpoint paths and top-level shapes did not change. Default source kind changed from internal `fixed_mock` to HTTP-backed `valetudo_mock`; fixed mock remains opt-in.
- `vacuum_adapter` public contract changed? No.
- Capability descriptors changed? No public descriptor shape changed.
- UI behavior changed? No direct UI code changed; the Valetudo backend receives live mock-source state through the same runtime API.
- Backward compatibility impact? TurtleBot4/Nav2 remains the default product backend and regression path. Valetudo remains opt-in. Fixed mock runtime behavior remains available with `VALETUDO_RUNTIME_SOURCE_MODE=fixed_mock`.
