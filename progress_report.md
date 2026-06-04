# Progress Report - Milestone 8 Documentation + Layer 6A Summary
Current report date: 2026-06-05.

## 1. What changed

Layer 6A documentation now describes the implemented Valetudo mock-through-VM path instead of leaving the feature as a planned or stubbed hardware milestone.

The docs distinguish the later real-hardware wording from the current mock milestone:

```text
Valetudo itself runs on the robot in the real hardware path.
The VM runs our Valetudo integration runtime.
Current Layer 6A uses fixed/mock Valetudo source first.
The adapter consumes the VM runtime API.
The UI consumes vacuum_adapter.
```

The documented implemented path is:

```text
Valetudo mock HTTP source or fixed mock runtime data
-> VM-managed Valetudo integration runtime
-> /api/v1/valetudo/health, /snapshot, /command
-> Valetudo backend adapter
-> vacuum_adapter
-> existing VS Code extension UI
```

## 2. Which mode this affects

- Mapping: documentation keeps Valetudo map rendering out of scope.
- Navigation: documentation keeps Valetudo go-to-location and ROS2 bridge support out of scope.
- Clean Area: documentation keeps Valetudo Clean Area execution out of scope.
- Rooms / Zones: documentation keeps Valetudo room/segment and zone execution out of scope.
- Valetudo backend: documentation now records the implemented mock/runtime/client/adapter path.
- Shared adapter/runtime architecture: documentation reinforces that product UI consumes `vacuum_adapter`, not backend endpoints.

## 3. Ownership check

React/webview state owns presentation, local mode selection, and UI hydration from `useVacuumAdapter`.

The VM runtime owns backend connection, mock/source state cache, runtime/source health, stale-state handling, command routing, and diagnostics.

The Valetudo backend adapter owns mapping VM runtime snapshots into `vacuum_adapter` snapshots and mapping normalized commands/results.

The UI only renders normalized adapter state: identity, availability, state, battery, dock/charging, capabilities, mission summary, fault/unavailable state, and unsupported map/pose/navigation surfaces.

The UI submits normalized commands: `start_cleaning`, `pause`, `stop`, and `return_to_dock`.

This follows the rule: Product UI renders normalized adapter state and submits normalized commands. Backend adapter maps backend runtime state into `vacuum_adapter`. VM runtime owns backend connection, state cache, and command routing.

## 4. Webview close/reopen behavior

- idle Valetudo mock state: reopening polls `/snapshot` through the adapter and hydrates identity, availability, idle state, battery, dock, and capabilities.
- unavailable VM runtime: the adapter maps fetch failure to a safe offline snapshot, so the panel remains mounted.
- reachable mock runtime: reopening receives the latest normalized runtime snapshot and runtime/source health.
- active mock cleaning state: reopening hydrates active cleaning from runtime state and renders a backend-neutral mission summary.
- paused mock cleaning state: reopening hydrates paused state from runtime state.
- terminal or stopped mock state: reopening hydrates stopped/idle state, or stale/unavailable state if the source is down.

The UI hydrates through `useVacuumAdapter`; it does not reconstruct Valetudo runtime authority from React state and does not call Valetudo source APIs directly.

## 5. Real hardware compatibility check

- Does this assume the robot is TurtleBot4/Nav2? No.
- Does this expose TurtleBot4/Nav2 specifics to product UI? No.
- Does this expose Valetudo raw capability names to product UI? No; raw names remain runtime diagnostics and adapter test fixture data.
- Can the same VM runtime API later connect to a real Valetudo robot? Yes; the runtime API boundary is stable enough for the source implementation to change behind it.
- What capability flags decide whether controls are shown/enabled? Public `vacuum_adapter` capability descriptors derived from runtime command availability and conservative Valetudo capability mapping.
- What operations are explicitly unsupported? Valetudo map rendering, robot movement visualization, go-to-location, Clean Area, rooms/zones, segment cleaning, fan/water setters, consumables UI, manual control, camera, ROS2 bridge for Valetudo, OpenClaw, and real hardware.

## 6. Feature behavior changed

- Architecture docs now say the VM runs our Valetudo integration runtime, not Valetudo itself.
- The Layer 6A summary records fixed mock data, HTTP mock source mode, optional MQTT diagnostics behind the runtime boundary, adapter mapping, and existing UI consumption.
- Extension docs now describe the Valetudo backend route, injected runtime configuration, supported commands, and unsupported/deferred behaviors.
- The Layer 6 plan now separates completed mock-through-VM behavior from later real-hardware work.

## 7. Files changed

- `/home/shane/vscode-tensorfleet/VACUUM_STACK_PLAN.md`: updated Layer 6 status and architecture text to describe completed Layer 6A mock-through-VM behavior and later real-hardware scope.
- `/home/shane/vscode-tensorfleet/extension.md`: added extension-side Valetudo backend path, runtime endpoints, configuration inputs, supported behavior, and unsupported/deferred behavior.
- `/home/shane/vscode-tensorfleet/6a.md`: converted current progress wording into a Layer 6A summary and recorded Milestone 8 documentation completion.
- `/home/shane/vscode-tensorfleet/progress_report.md`: replaced Milestone 7 report with this Milestone 8 documentation report.

## 8. Tests / validation run

```sh
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check
```

Passed:

```text
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check
```

`bun run --cwd panels-standalone build` emitted existing Vite warnings about browser-externalized `path`/`fs`, `eval` in `@protobufjs/inquire`, and large chunks, but the build completed successfully.

Live VM endpoint and webview validation were not rerun in this documentation-only pass. The latest recorded runtime validation remains the Milestone 7 state: `go test ./...` passed in `/home/shane/firecracker-vm/tensorfleet-mgr`, `bun run test:vacuum-adapter` passed, and VM-facing `/health`, `/snapshot`, and `/command` had previously been validated against the mock source.

## 9. Remaining risks

- Live VS Code webview operator validation still needs a final pass.
- Real Valetudo hardware has not been connected.
- MQTT is optional and not production-hardened.
- Runtime diagnostics and last-command audit are memory-backed.
- Command availability is still broad when `BasicControlCapability` is present.
- Valetudo map, room, zone, Clean Area, go-to-location, fan/water setters, and consumables remain unsupported or diagnostics-only.

## 10. Next recommended step

Run final live operator validation through vm-manager for `/health`, `/snapshot`, `/command`, and the Valetudo backend webview. After that, the next smallest architecture-aligned step is real-hardware reachability discovery behind the same VM runtime API, without adding ROS2/OpenClaw, map rendering, rooms/zones, or go-to-location.

## 11. Contract changes

- Runtime API changed? No endpoint or top-level contract change in this documentation pass.
- `vacuum_adapter` public contract changed? No.
- Capability descriptors changed? No.
- UI behavior changed? No implementation behavior changed; docs now clarify existing adapter-driven behavior.
- Backward compatibility impact? None. TurtleBot4/Nav2 remains the default simulation/regression backend and Valetudo remains opt-in.
