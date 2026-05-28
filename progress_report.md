# Progress Report - Milestone 1 Valetudo Runtime API Contract + Fixed Mock Runtime
Current report date: 2026-05-29.

## 1. What changed

Added a VM-managed Valetudo integration runtime boundary inside `tensorfleet-mgr`. The runtime exposes health, snapshot, and command endpoints for a fixed mock Valetudo source. It returns one mock robot identity, distinct runtime/source health, connectivity, stale state, battery/dock state, normalized command availability, raw Valetudo capability names as diagnostics only, and structured command results.

## 2. Which mode this affects

- Mapping: unchanged.
- Navigation: unchanged.
- Clean Area: unchanged.
- Rooms / Zones: unchanged.
- Valetudo backend: fixed mock runtime contract is now available behind the VM-managed Tensorfleet runtime.
- Shared adapter/runtime architecture: adds the private runtime API that the future Valetudo backend adapter will call through `vm-manager`.

## 3. Ownership check

VM runtime owns fixed mock Valetudo state, runtime/source health, stale status, command availability, and command results.

Valetudo backend adapter is not changed in this milestone.

React/webview state and product UI are unchanged. The UI renders no new Valetudo state yet and submits no Valetudo commands yet.

This follows the rule: Product UI renders normalized adapter state and submits normalized commands; backend adapter maps backend runtime state into `vacuum_adapter`; VM runtime owns backend connection, state cache, and command routing.

## 4. Webview close/reopen behavior

The webview is unchanged in Milestone 1, so close/reopen behavior remains the existing TurtleBot4/Nav2 behavior.

For the new Valetudo runtime path once Milestone 2 connects it: idle fixed mock state should hydrate from `/snapshot`; unavailable VM runtime should become an adapter unavailable state; reachable mock runtime should hydrate identity, availability, battery, dock, and command availability; active, paused, terminal, or stopped mock state are not yet mutable in Milestone 1.

## 5. Real hardware compatibility check

- Does this assume the robot is TurtleBot4/Nav2? No.
- Does this expose TurtleBot4/Nav2 specifics to product UI? No.
- Does this expose Valetudo raw capability names to product UI? No UI path consumes them; they exist only in runtime diagnostics.
- Can the same VM runtime API later connect to a real Valetudo robot? Yes, the runtime/source split and source kind allow `real_robot` later.
- What capability flags decide whether controls are shown/enabled? Milestone 1 provides normalized `capabilities.commands.*.available`; UI gating is deferred to Milestone 2.
- What operations are explicitly unsupported? Any command outside `start_cleaning`, `pause`, `stop`, and `return_to_dock` returns structured `unsupported`.

## 6. Feature behavior changed

- VM guest runtime now exposes `/api/v1/valetudo/health`.
- VM guest runtime now exposes `/api/v1/valetudo/snapshot`.
- VM guest runtime now exposes `/api/v1/valetudo/command`.
- Fixed mock Valetudo robot appears in runtime snapshot data.
- Unsupported commands return structured unsupported results.
- Unreachable mock source can be simulated with `VALETUDO_FIXED_MOCK_SOURCE_REACHABLE=false`, causing commands to return structured unavailable results.

## 7. Files changed

- `/home/shane/firecracker-vm/tensorfleet-mgr/valetudo_runtime.go`: added private Valetudo runtime contract structs and fixed mock handlers.
- `/home/shane/firecracker-vm/tensorfleet-mgr/main.go`: registered Valetudo runtime routes under `/api/v1/valetudo/*`.
- `/home/shane/firecracker-vm/tensorfleet-mgr/handlers_test.go`: added runtime contract, unsupported command, and unavailable source tests.
- `/home/shane/firecracker-vm/tensorfleet-mgr/README.md`: documented the VM-managed Valetudo runtime endpoints and unreachable-source toggle.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/runtimeContract.ts`: added private TypeScript runtime contract types for the later Valetudo runtime client.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/index.ts`: exported the private runtime contract types from the Valetudo backend module.
- `/home/shane/vscode-tensorfleet/progress_report.md`: recorded Milestone 1 behavior and contract status.

## 8. Tests / validation run

```sh
go test ./...
go build ./...
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check
```

Manual runtime checks performed:

```text
Started Tensorfleet-Mgr locally on PORT=19090 with TENSORFLEET_AUTH_TOKEN=test-token.
Called GET /api/v1/valetudo/health, confirmed runtime status online and source reachable.
Called GET /api/v1/valetudo/snapshot, confirmed fixed mock Valetudo robot identity, battery, dock, diagnostics, and command availability.
Called POST /api/v1/valetudo/command with start_cleaning, confirmed structured success.
Called POST /api/v1/valetudo/command with clean_zone, confirmed structured unsupported.
Restarted with VALETUDO_FIXED_MOCK_SOURCE_REACHABLE=false.
Called GET /api/v1/valetudo/snapshot, confirmed identity and updatedAt remain available while source is unreachable and stale.
Called POST /api/v1/valetudo/command with start_cleaning, confirmed HTTP 503 structured unavailable.
```

## 9. Remaining risks

- Frontend is not connected yet.
- Runtime is fixed mock only.
- Commands do not mutate state yet.
- Real Valetudo HTTP and MQTT are not connected.
- Capability mapping into `vacuum_adapter` is not implemented in this milestone.
- No map, rooms, zones, go-to-location, fan, water, consumables, ROS2, or OpenClaw support is implemented.

## 10. Next recommended step

Implement the Valetudo runtime client and adapter skeleton, make `backend = valetudo` selectable, and map the fixed mock snapshot into a safe no-map `VacuumAdapterSnapshot`.

## 11. Contract changes

- Runtime API changed? Yes. Added private VM runtime endpoints under `/api/v1/valetudo/health`, `/api/v1/valetudo/snapshot`, and `/api/v1/valetudo/command`, plus matching private TypeScript runtime contract types.
- `vacuum_adapter` public contract changed? No.
- Capability descriptors changed? No public descriptors changed; runtime command availability was added privately.
- UI behavior changed? No.
- Backward compatibility impact? Existing TurtleBot4/Nav2 and Gazebo runtime endpoints are unchanged.
