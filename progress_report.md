# Progress Report - Milestone 4 Capability Mapper Hardening + Adapter Tests
Current report date: 2026-05-29.

## 1. What changed

The Valetudo backend capability mapper now treats raw Valetudo capability names as backend/runtime diagnostic input only. Implemented basic controls map to normalized public capabilities for `start_cleaning`, `pause`, `stop`, and `return_to_dock`. Detected but unimplemented Valetudo features such as fan speed, water usage, consumables, segment cleaning, zone cleaning, and go-to-location now map to explicit normalized unsupported descriptors.

The adapter boundary now includes known detected runtime diagnostics when building the Valetudo capability set, while preserving command availability as the source of product-ready basic controls. Battery state is represented as a normalized public `battery` capability when the runtime snapshot has battery state.

## 2. Which mode this affects

- Mapping: unchanged; Valetudo map rendering remains unsupported.
- Navigation: Valetudo go-to detection is now explicitly unsupported through normalized descriptors.
- Clean Area: unchanged; coverage remains unsupported.
- Rooms / Zones: zone and segment capability detection is now explicit but not product-ready.
- Valetudo backend: capability mapping is stricter and regression-tested.
- Shared adapter/runtime architecture: no public endpoint shape changed.

## 3. Ownership check

VM runtime owns raw Valetudo capability diagnostics, fixed mock source state, and command availability in `/snapshot`.

Valetudo backend adapter owns the mapping from runtime diagnostics and command availability into `vacuum_adapter` capability descriptors.

React/webview state owns presentation only. It renders normalized adapter state and submits normalized commands such as `start_cleaning`, `pause`, `stop`, and `return_to_dock`.

This follows the rule: Product UI renders normalized adapter state and submits normalized commands. Backend adapter maps backend runtime state into `vacuum_adapter`. VM runtime owns backend connection, state cache, and command routing.

## 4. Webview close/reopen behavior

- idle Valetudo mock state: reopening fetches `/snapshot` and hydrates normalized identity, idle state, battery, dock, and command availability.
- unavailable VM runtime: reopening falls back to an offline adapter snapshot with no map, pose, navigation, or capability-dependent controls.
- reachable mock runtime: polling fetches current runtime state and capability descriptors.
- active mock cleaning state: reopening hydrates as cleaning with basic controls only if command availability is true.
- paused mock cleaning state: reopening hydrates as paused with normalized pause/stop/return capability gating.
- terminal or stopped mock state: reopening hydrates as idle/stopped behavior with no active mission.

Hydration continues through `useVacuumAdapter`; the UI does not call Valetudo or VM runtime endpoints directly.

## 5. Real hardware compatibility check

- Does this assume the robot is TurtleBot4/Nav2? No.
- Does this expose TurtleBot4/Nav2 specifics to product UI? No.
- Does this expose Valetudo raw capability names to product UI? No; raw names remain in Valetudo backend/runtime diagnostics and tests.
- Can the same VM runtime API later connect to a real Valetudo robot? Yes.
- What capability flags decide whether controls are shown/enabled? Normalized `vacuum_adapter` descriptors such as `start_cleaning`, `pause`, `stop`, `return_to_dock`, `battery`, `fan_speed`, `water_usage`, `zone_cleaning`, and `go_to_location`.
- What operations are explicitly unsupported? Valetudo map rendering, pose product surface, navigation/go-to, Clean Area/coverage, mapping sessions, rooms/zones, segment cleaning, fan/water setters, consumables UI, manual control, camera, and annotations.

## 6. Feature behavior changed

- Basic Valetudo controls are supported only when `BasicControlCapability` is implemented through runtime command availability.
- Missing basic control support produces unsupported command mapping.
- Battery state maps to the normalized `battery` descriptor when present.
- Fan speed, water usage, consumables, segment cleaning, zone cleaning, and go-to-location are detected but remain unsupported.
- Regression coverage now checks that Vacuum Control UI code does not branch on raw Valetudo capability names.

## 7. Files changed

- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/capabilityMapper.ts`: hardened supported vs detected-but-unimplemented Valetudo capability mapping.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/stateMapper.ts`: carries known detected runtime diagnostics into adapter-private capability mapping.
- `/home/shane/vscode-tensorfleet/scripts/vacuum-adapter-regression.ts`: added capability, command, runtime snapshot, and UI-boundary regression checks.
- `/home/shane/firecracker-vm/tensorfleet-mgr/valetudo_runtime.go`: added `BatteryStateCapability` to fixed mock raw diagnostics.
- `/home/shane/firecracker-vm/tensorfleet-mgr/handlers_test.go`: added runtime diagnostics checks for implemented basic controls and detected unimplemented capabilities.
- `/home/shane/vscode-tensorfleet/progress_report.md`: recorded Milestone 4 behavior, validation, risks, and contract status.

## 8. Tests / validation run

```sh
bun run test:vacuum-adapter
go test ./...
bun run typecheck
```

`bun run test:vacuum-adapter` passed.

`go test ./...` passed in `/home/shane/firecracker-vm/tensorfleet-mgr`.

`bun run typecheck` failed on pre-existing unused variables in `/home/shane/vscode-tensorfleet/packages/tensorfleet-auth/src/oauth-core.ts` (`callbackBaseUrl`, `actualPort`). No Milestone 4 files were reported.

Manual live VM endpoint validation was not repeated in this pass; Milestone 4 was validated through mapper and runtime regression tests.

## 9. Remaining risks

- Runtime is still fixed mock only.
- Actual Valetudo mock source connection is not implemented.
- Real Valetudo HTTP and MQTT are not connected.
- Detected capability diagnostics are conservative and not yet driven by a live Valetudo source.
- Fan, water, consumables, zones, segments, and go-to remain unsupported.
- Live VS Code webview operator validation was not performed in this pass.

## 10. Next recommended step

Implement Milestone 5: connect the VM-managed Valetudo integration runtime to the actual Valetudo mock source while keeping the same `/health`, `/snapshot`, and `/command` runtime API.

## 11. Contract changes

- Runtime API changed? No endpoint shape changed. Fixed mock diagnostics now include `BatteryStateCapability`.
- `vacuum_adapter` public contract changed? No.
- Capability descriptors changed? No shape change; Valetudo mappings are now more conservative and explicit.
- UI behavior changed? Capability-gated controls continue to use normalized descriptors; detected unimplemented Valetudo features do not activate broken controls.
- Backward compatibility impact? TurtleBot4/Nav2 remains the default backend and regression path. Valetudo selection remains opt-in.
