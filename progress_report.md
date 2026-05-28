# Progress Report - Milestone 2 Runtime Client + Valetudo Adapter Selectable + No-Map UI Safety
Current report date: 2026-05-29.

## 1. What changed

The Valetudo backend can now be selected behind `useVacuumAdapter` without throwing. A private Valetudo runtime client fetches the VM-managed runtime snapshot, maps it into a safe `VacuumAdapterSnapshot`, and routes basic normalized commands to the runtime command API.

The product UI still consumes only `vacuum_adapter`. For a Valetudo fixed mock snapshot, the UI can show robot identity, connection state, mission state, battery, dock state, and basic cleaning controls. Map, pose, navigation, Clean Area, mapping, Rooms / Zones, teleop, camera, fan, water, segment, and zone controls remain unsupported unless their normalized capability descriptors become supported later.

The fixed runtime diagnostics now mirror the actual `MockValetudoRobot` capability list from `/home/shane/Valetudo` instead of a smaller hand-picked subset. These names remain diagnostics only.

Vacuum Control now includes a small adapter selector in the webview header. Changing it remounts the adapter content with the selected backend and persists the selection locally for the webview.

## 2. Which mode this affects

- Mapping: capability-gated; Valetudo shows an unavailable state instead of mounting mapping controls.
- Navigation: capability-gated; Valetudo go-to diagnostics do not enable product navigation controls.
- Clean Area: capability-gated; Valetudo does not expose coverage controls.
- Rooms / Zones: capability-gated; Valetudo diagnostics do not enable room or zone workflows.
- Valetudo backend: runtime client, snapshot mapper, adapter hook, backend selection, command routing, and unavailable recovery were added.
- Shared adapter/runtime architecture: VM runtime remains the owner of backend connection/state/commands; adapter maps runtime state into `vacuum_adapter`.

## 3. Ownership check

VM runtime owns runtime/source health, fixed mock state, stale/source status, and command routing.

Valetudo backend adapter owns runtime client usage, snapshot mapping, command mapping, unavailable/offline fallback, and conservative capability mapping.

React/webview state owns only presentation state such as active panel mode, local drafts, and button errors. The UI renders normalized adapter state and submits normalized commands like `start_cleaning`, `pause`, `stop`, and `return_to_dock`.

The adapter selector is React/webview-owned configuration state. It chooses which backend adapter is mounted, but product controls still render from normalized capabilities and state.

This follows the rule: Product UI renders normalized adapter state and submits normalized commands. Backend adapter maps backend runtime state into `vacuum_adapter`. VM runtime owns backend connection, state cache, and command routing.

## 4. Webview close/reopen behavior

- idle Valetudo mock state: reopening creates a new adapter hook, fetches `/snapshot`, and hydrates identity, online state, idle state, battery, dock, and basic command availability.
- unavailable VM runtime: reopening returns an offline/unavailable adapter snapshot with no map, no pose, no navigation, and no crash.
- reachable mock runtime: polling fetches the current runtime snapshot and recovers automatically after the runtime comes back.
- active mock cleaning state: the current fixed mock runtime does not mutate state yet; when mutable state lands, reopening will hydrate from `/snapshot`.
- paused mock cleaning state: same as active; supported by the adapter state mapper when the runtime snapshot reports paused.
- terminal or stopped mock state: fixed mock remains idle today; later terminal state should hydrate from runtime snapshot/recent mission data.

Hydration happens through `useVacuumAdapter`, not direct UI calls to Valetudo or runtime endpoints.

If the operator changes the header adapter selector, the webview remounts the adapter content and hydrates from the newly selected backend.

## 5. Real hardware compatibility check

- Does this assume the robot is TurtleBot4/Nav2? No.
- Does this expose TurtleBot4/Nav2 specifics to product UI? No.
- Does this expose Valetudo raw capability names to product UI? No. Raw capability names remain runtime diagnostics; public capability descriptors use normalized support.
- Can the same VM runtime API later connect to a real Valetudo robot? Yes.
- What capability flags decide whether controls are shown/enabled? `map`, `start_navigation`, `go_to_location`, `start_coverage`, `coverage_mission`, `mapping_session`, `auto_mapping`, `room_semantics`, `zone_semantics`, `room_cleaning`, `zone_cleaning`, `manual_control`, `battery`, `dock_state`, `start_cleaning`, `pause`, `stop`, and `return_to_dock`.
- What operations are explicitly unsupported? Valetudo map rendering, pose product surface, go-to/navigation, Clean Area/coverage, mapping sessions, rooms/zones, manual control, camera, fan/water setters, segment cleaning, zone cleaning, and map annotations.

## 6. Feature behavior changed

- `backend = valetudo` no longer throws in `useVacuumAdapter`.
- Valetudo fixed mock snapshots map into backend-neutral adapter snapshots.
- The extension can select the Valetudo adapter through `tensorfleet.vacuum.backend`.
- Basic Valetudo controls route through `adapter.sendCommand` to the VM runtime.
- Unsupported Valetudo diagnostics do not activate broken UI controls.
- MapCanvas is not mounted when the adapter says map is unsupported.
- Runtime outages become offline adapter state instead of UI crashes.
- The webview header can switch between the simulation backend and Valetudo runtime backend.

## 7. Files changed

- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/runtimeClient.ts`: added private runtime API client for health, snapshot, and command endpoints.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/useValetudoAdapter.ts`: added the selectable Valetudo adapter hook with polling, unavailable fallback, and command routing.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/stateMapper.ts`: mapped VM runtime snapshots into safe no-map adapter state.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/capabilityMapper.ts`: made Layer 6A capability mapping conservative and kept raw Valetudo names out of public capability flags.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/index.ts`: exported the runtime client and hook.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/useVacuumAdapter.ts`: made the Valetudo backend selectable and exposed backend normalization helpers.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`: added normalized capability gating, no-map fallback, basic cleaning controls, and a remounting backend selector.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/components/VacuumControl/VacuumControlPanel.css`: styled the no-map fallback and adapter selector.
- `/home/shane/vscode-tensorfleet/src/extension.ts`: injects the configured vacuum backend into the webview.
- `/home/shane/vscode-tensorfleet/package.json`: added `tensorfleet.vacuum.backend`.
- `/home/shane/vscode-tensorfleet/scripts/vacuum-adapter-regression.ts`: added Valetudo runtime snapshot regression coverage and tightened capability expectations.
- `/home/shane/vscode-tensorfleet/progress_report.md`: recorded Milestone 2 behavior and contracts.
- `/home/shane/firecracker-vm/tensorfleet-mgr/valetudo_runtime.go`: corrected fixed mock raw capability diagnostics to match local Valetudo `MockValetudoRobot`.
- `/home/shane/firecracker-vm/tensorfleet-mgr/handlers_test.go`: added regression checks for representative actual Valetudo mock capabilities.

## 8. Tests / validation run

```sh
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check
go test ./...
```

Additional checks:

```sh
bun run typecheck
bunx tsc -p panels-standalone/tsconfig.json --noEmit
```

Both typecheck commands are still blocked by pre-existing unrelated TypeScript errors outside this milestone, including unused variables in `packages/tensorfleet-auth`, missing `@jest/types`, existing `gzweb` type errors, missing `tensorfleet-ros` declarations, and existing `SensorView3D`/`RawMessages` errors.

Manual runtime checks performed:

```text
Started Tensorfleet-Mgr locally on PORT=19090 with TENSORFLEET_AUTH_TOKEN=test-token.
Called GET /api/v1/valetudo/health, confirmed runtime online and fixed mock source reachable.
Called GET /api/v1/valetudo/snapshot, confirmed fixed mock identity, battery, dock, diagnostics, and command availability.
Called POST /api/v1/valetudo/command with start_cleaning, confirmed structured success.
Called POST /api/v1/valetudo/command with go_to_location, confirmed structured unsupported.
Stopped the local runtime process.
Verified `/home/shane/Valetudo/backend/lib/robots/mock/MockValetudoRobot.js` and corrected fixed runtime diagnostics to include the actual mock capability names.
```

## 9. Remaining risks

- Runtime is still fixed mock only.
- Commands do not mutate mock state yet.
- Runtime client recovery is polling-based and not yet operator-tested inside a live VS Code webview.
- Real Valetudo HTTP and MQTT are not connected.
- No Valetudo map rendering exists.
- Rooms, zones, segment cleaning, go-to-location, fan, water, consumables, scheduling, and camera remain diagnostics-only or unsupported.
- Full repo typecheck remains blocked by existing unrelated errors.

## 10. Next recommended step

Implement Milestone 3: route `start_cleaning`, `pause`, `stop`, and `return_to_dock` into mutable fixed mock runtime state, then verify `/snapshot` reflects active, paused, stopped, and returning states after commands.

## 11. Contract changes

- Runtime API changed? No endpoint shape changed from Milestone 1.
- `vacuum_adapter` public contract changed? No.
- Capability descriptors changed? Behavior changed: Valetudo map, pose, go-to, fan, water, segment, and zone capabilities are explicitly unsupported in the public adapter until product workflows exist.
- UI behavior changed? Yes. The UI now gates map/navigation/mapping/Clean Area/Rooms/Zones/teleop on normalized capabilities, adds basic cleaning controls when available, and provides a small adapter selector that remounts the selected backend.
- Backward compatibility impact? TurtleBot4/Nav2 remains the default backend and regression path. Valetudo selection is opt-in through `tensorfleet.vacuum.backend`, `window.TENSORFLEET_VACUUM_BACKEND`, or `?vacuumBackend=valetudo`.
