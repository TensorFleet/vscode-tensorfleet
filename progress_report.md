# Progress Report - Milestone 6 Optional MQTT Behind Runtime Boundary
Current report date: 2026-06-04.

## 1. What changed

The VM-managed Valetudo integration runtime now has optional MQTT support behind the existing runtime API. When `VALETUDO_MQTT_ENABLED=true`, the runtime subscribes to Valetudo/Homie-style topics, updates an internal state cache, and serves the cached state through the same `GET /health`, `GET /snapshot`, and `POST /command` endpoints already consumed by the Valetudo adapter.

HTTP remains the default mock-source path. Commands still use HTTP by default, with optional MQTT command publishing through `VALETUDO_MQTT_COMMANDS_ENABLED=true`.

## 2. Which mode this affects

- Mapping: unchanged; Valetudo map rendering remains unsupported.
- Navigation: unchanged; Valetudo go-to remains detected but not product-ready.
- Clean Area: unchanged; Valetudo coverage remains unsupported.
- Rooms / Zones: unchanged; detected raw capabilities remain diagnostics only.
- Valetudo backend: optional MQTT can hydrate runtime state cache.
- Shared adapter/runtime architecture: endpoint paths remain stable.

## 3. Ownership check

VM runtime owns MQTT broker/client integration, topic subscription, state cache, stale-state calculation, and optional MQTT command publishing.

Valetudo backend adapter continues to consume only the VM runtime API and map runtime snapshots into `vacuum_adapter`.

React/webview state owns presentation only. It renders normalized adapter state and submits normalized commands such as `start_cleaning`, `pause`, `stop`, and `return_to_dock`.

This follows the rule: Product UI renders normalized adapter state and submits normalized commands. Backend adapter maps backend runtime state into `vacuum_adapter`. VM runtime owns backend connection, state cache, and command routing.

## 4. Webview close/reopen behavior

- idle Valetudo mock state: reopening polls `/snapshot`; if MQTT is enabled and fresh, the runtime returns cached MQTT status/battery/dock state.
- unavailable VM runtime: reopening falls back through the existing adapter unavailable path with no map, pose, navigation, or capability-dependent controls.
- reachable mock runtime: polling fetches current runtime data regardless of whether the runtime got it from HTTP or MQTT.
- active mock cleaning state: reopening hydrates from normalized runtime state, including MQTT `StatusStateAttribute/status=cleaning` when enabled.
- paused mock cleaning state: reopening hydrates from normalized runtime state, including MQTT `StatusStateAttribute/status=paused` when enabled.
- terminal or stopped mock state: reopening hydrates from the latest runtime snapshot or shows stale/unavailable if MQTT data expired.

Hydration continues through `useVacuumAdapter`; the UI does not call Valetudo HTTP, Valetudo MQTT, or VM runtime internals directly.

## 5. Real hardware compatibility check

- Does this assume the robot is TurtleBot4/Nav2? No.
- Does this expose TurtleBot4/Nav2 specifics to product UI? No.
- Does this expose Valetudo raw capability names to product UI? No; raw names and MQTT topics remain runtime/adapter diagnostics.
- Can the same VM runtime API later connect to a real Valetudo robot? Yes; broker URL, topic prefix, identifier, and source URL are configurable.
- What capability flags decide whether controls are shown/enabled? Normalized `vacuum_adapter` capability descriptors derived from runtime command availability and diagnostics.
- What operations are explicitly unsupported? Valetudo map rendering, pose product surface, navigation/go-to, Clean Area/coverage, mapping sessions, rooms/zones, segment cleaning, fan/water setters, consumables UI, manual control, camera, and annotations.

## 6. Feature behavior changed

- Runtime can subscribe to `<prefix>/<identifier>/#` and cache Valetudo MQTT status, battery, dock, and capability signals.
- Runtime `/snapshot` can reflect MQTT-derived state without changing the adapter-facing shape.
- Runtime `/health` reports stale/unreachable MQTT source state when MQTT is enabled and no fresh messages arrive.
- Optional MQTT command routing publishes basic commands to `<prefix>/<identifier>/BasicControlCapability/operation/set`.
- vm-manager remains a plain HTTP proxy for `/vms/self/tensorfleet/...`; it does not expose MQTT.

## 7. Files changed

- `/home/shane/firecracker-vm/tensorfleet-mgr/valetudo_mqtt.go`: added optional MQTT client startup, topic parsing, cache snapshots, transport diagnostics, stale-state handling, and optional basic command publishing.
- `/home/shane/firecracker-vm/tensorfleet-mgr/valetudo_runtime.go`: wired MQTT health/snapshot/command paths behind env flags and added transport diagnostics to the runtime response.
- `/home/shane/firecracker-vm/tensorfleet-mgr/handlers_test.go`: added MQTT cache snapshot and disconnected MQTT command-routing tests.
- `/home/shane/firecracker-vm/tensorfleet-mgr/go.mod`, `/home/shane/firecracker-vm/tensorfleet-mgr/go.sum`: added Eclipse Paho MQTT client dependency.
- `/home/shane/firecracker-vm/tensorfleet-mgr/main.go`: starts MQTT when configured and logs the runtime subscription target.
- `/home/shane/firecracker-vm/tensorfleet-mgr/README.md`: documented MQTT env vars and runtime boundary behavior.
- `/home/shane/vm-manager/README.md`: documented that vm-manager proxies runtime HTTP endpoints and does not expose MQTT.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/runtimeContract.ts`: added optional runtime transport diagnostics type.
- `/home/shane/vscode-tensorfleet/scripts/vacuum-adapter-regression.ts`: added MQTT-shaped runtime snapshot regression coverage.
- `/home/shane/vscode-tensorfleet/progress_report.md`: recorded Milestone 6 behavior, validation, risks, and contract status.

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
go test ./... in /home/shane/vm-manager
bun run test:vacuum-adapter in /home/shane/vscode-tensorfleet
all three git diff --check runs
```

`bun run typecheck` still fails on pre-existing unused variables in `/home/shane/vscode-tensorfleet/packages/tensorfleet-auth/src/oauth-core.ts`: `callbackBaseUrl` and `actualPort`.

Manual broker validation was not run because no local MQTT broker CLI was installed in this environment. MQTT behavior was validated with runtime unit tests that feed Valetudo-style topics into the cache and verify the stable `/snapshot` shape.

## 9. Remaining risks

- MQTT has not been validated against a live broker in this pass.
- MQTT source configuration is env-based and mock-oriented, not production-hardened.
- Real hardware is not connected.
- Valetudo map data is intentionally ignored by the product-facing adapter path.
- Command availability is still broad when `BasicControlCapability` is present; later milestones may add state-aware availability.
- Live VS Code webview operator validation was not performed in this pass.

## 10. Next recommended step

Proceed to Milestone 7 diagnostics hardening: add clearer operator-facing runtime diagnostics for source URL, MQTT broker/topic root, last successful source update, last source error, and detected-but-unimplemented capabilities while keeping raw payloads and raw Valetudo names out of product UI logic.

## 11. Contract changes

- Runtime API changed? Endpoint paths did not change. Snapshot diagnostics gained optional `transports` metadata.
- `vacuum_adapter` public contract changed? No.
- Capability descriptors changed? No public descriptor shape changed.
- UI behavior changed? No direct UI behavior changed; the Valetudo backend still receives normalized state through the same runtime API.
- Backward compatibility impact? TurtleBot4/Nav2 remains the default product backend and regression path. Valetudo remains opt-in. HTTP mock-source mode remains default, fixed mock remains opt-in, and MQTT is disabled unless configured.
