# Progress Report - Milestone 3 Basic Command Routing + Mutable Mock State
Current report date: 2026-05-29.

## 1. What changed

The VM-managed Valetudo fixed mock runtime now owns a small mutable robot state machine. Normalized commands sent through the existing runtime API update the next snapshot: `start_cleaning` moves the mock robot to cleaning, `pause` moves active cleaning to paused, `stop` moves the robot to a stopped terminal state, and `return_to_dock` emits a deterministic returning snapshot before settling back to docked.

The Valetudo adapter path still sits behind `vacuum_adapter`. The UI submits normalized commands only; the adapter validates normalized capabilities and sends runtime commands; the VM runtime owns command routing and state mutation.

The runtime can also simulate the absence of `BasicControlCapability`. In that mode, basic command availability is false in `/snapshot`, raw diagnostics do not advertise `BasicControlCapability`, and the same normalized commands return structured unsupported results.

## 2. Which mode this affects

- Mapping: unchanged; Valetudo map rendering remains unsupported and no map is mounted for this backend.
- Navigation: unchanged; Valetudo go-to/navigation remains diagnostics-only and unsupported in product controls.
- Clean Area: unchanged; coverage/Clean Area remains unsupported.
- Rooms / Zones: unchanged; room, zone, and segment execution remain unsupported.
- Valetudo backend: basic command routing is now runtime-testable with mutable fixed mock state.
- Shared adapter/runtime architecture: command effects now round-trip through runtime snapshot hydration instead of being accepted without state change.

## 3. Ownership check

VM runtime owns fixed mock state, command routing, capability absence simulation, source reachability, and the state cache returned by `/snapshot`.

Valetudo backend adapter owns normalized capability validation, runtime client calls, command result mapping, and snapshot-to-`vacuum_adapter` mapping.

React/webview state owns presentation only. It renders normalized adapter state and submits normalized commands such as `start_cleaning`, `pause`, `stop`, and `return_to_dock`.

This follows the rule: Product UI renders normalized adapter state and submits normalized commands. Backend adapter maps backend runtime state into `vacuum_adapter`. VM runtime owns backend connection, state cache, and command routing.

## 4. Webview close/reopen behavior

- idle Valetudo mock state: reopening fetches `/snapshot` and hydrates the docked or idle normalized state, identity, battery, dock, and basic command availability.
- unavailable VM runtime: reopening falls back to the adapter's offline/unavailable snapshot with no map, no pose, and no navigation.
- reachable mock runtime: polling fetches the current runtime-owned state and recovers automatically after transient runtime errors.
- active mock cleaning state: reopening hydrates from `/snapshot` as cleaning with an active hardware-cleaning mission.
- paused mock cleaning state: reopening hydrates from `/snapshot` as paused with paused mission state.
- terminal or stopped mock state: reopening hydrates stopped as normalized idle/stopped terminal behavior with no active mission.

For return-to-dock, the first snapshot after the command reports `returning_to_dock`; the following snapshot settles to `docked`. Hydration still happens through `useVacuumAdapter`, not direct UI calls to Valetudo or runtime endpoints.

## 5. Real hardware compatibility check

- Does this assume the robot is TurtleBot4/Nav2? No.
- Does this expose TurtleBot4/Nav2 specifics to product UI? No.
- Does this expose Valetudo raw capability names to product UI? No. Raw names remain diagnostics only.
- Can the same VM runtime API later connect to a real Valetudo robot? Yes. The API shape remains `/health`, `/snapshot`, and `/command`.
- What capability flags decide whether controls are shown/enabled? `start_cleaning`, `pause`, `stop`, `return_to_dock`, plus existing normalized unsupported flags for map, pose, navigation, coverage, mapping, rooms, zones, manual control, fan, and water.
- What operations are explicitly unsupported? Valetudo map rendering, pose product surface, go-to/navigation, Clean Area/coverage, mapping sessions, rooms/zones, manual control, camera, fan/water setters, segment cleaning, zone cleaning, and map annotations.

## 6. Feature behavior changed

- `start_cleaning` now changes the fixed mock snapshot from docked/idle/stopped to cleaning.
- `pause` now changes an active cleaning snapshot to paused.
- `stop` now changes cleaning or paused state to stopped/idle behavior.
- `return_to_dock` now reports returning first, then docked on the next snapshot.
- Missing `BasicControlCapability` can be simulated and returns explicit unsupported command results.
- Adapter regression coverage now verifies Valetudo runtime states map into normalized mission states.

## 7. Files changed

- `/home/shane/firecracker-vm/tensorfleet-mgr/valetudo_runtime.go`: added runtime-owned mutable fixed mock state, deterministic basic command transitions, dock state updates, command diagnostics, and BasicControlCapability absence simulation.
- `/home/shane/firecracker-vm/tensorfleet-mgr/handlers_test.go`: added runtime tests for start, pause, stop, return-to-dock, source unavailable, unsupported command, and missing BasicControlCapability behavior.
- `/home/shane/vscode-tensorfleet/scripts/vacuum-adapter-regression.ts`: added adapter regression coverage for Valetudo cleaning, paused, stopped, and returning state mapping.
- `/home/shane/vscode-tensorfleet/progress_report.md`: recorded Milestone 3 behavior, validation, remaining risks, and contract status.

## 8. Tests / validation run

```sh
go test ./...
bun run test:vacuum-adapter
bun run build:panels
git diff --check
git -C /home/shane/firecracker-vm/tensorfleet-mgr diff --check
```

Manual runtime checks performed:

```text
Started tensorfleet-mgr locally on PORT=19090 with fixed mock source reachable.
Called GET /api/v1/valetudo/health, confirmed runtime online and fixed mock source reachable.
Called GET /api/v1/valetudo/snapshot, confirmed docked fixed mock identity, battery, dock, diagnostics, and command availability.
Called POST /api/v1/valetudo/command with start_cleaning, confirmed success and next snapshot state=cleaning.
Called POST /api/v1/valetudo/command with pause, confirmed success and next snapshot state=paused.
Called POST /api/v1/valetudo/command with stop, confirmed success and next snapshot state=stopped.
Called POST /api/v1/valetudo/command with return_to_dock, confirmed success, next snapshot state=returning_to_dock, and following snapshot state=docked.
Stopped the local runtime process.
```

`bun run build:panels` completed successfully with existing Vite warnings about browser-externalized Node modules, eval in `@protobufjs/inquire`, and large chunks.

## 9. Remaining risks

- Runtime is still fixed mock only.
- Mutable state is in-memory and resets when `tensorfleet-mgr` restarts.
- Command transitions are intentionally simple and not yet modeled from a real Valetudo source.
- Real Valetudo HTTP and MQTT are not connected.
- No Valetudo map rendering exists.
- Rooms, zones, segment cleaning, go-to-location, fan, water, consumables, scheduling, and camera remain diagnostics-only or unsupported.
- Live VS Code webview operator validation was not performed in this pass.

## 10. Next recommended step

Implement Milestone 4: harden the Valetudo capability mapper and adapter tests so detected-but-unimplemented capabilities stay diagnostics-only while public controls remain enabled only by normalized supported capability descriptors.

## 11. Contract changes

- Runtime API changed? No endpoint shape changed. Runtime command behavior changed from fixed success to stateful command routing.
- `vacuum_adapter` public contract changed? No.
- Capability descriptors changed? No public descriptor shape changed. Runtime command availability can now report `capability_unavailable` when BasicControlCapability is absent.
- UI behavior changed? No new UI surface changed in this pass; existing controls now see state changes after adapter refresh.
- Backward compatibility impact? TurtleBot4/Nav2 remains the default backend and regression path. Valetudo selection remains opt-in.
