# Progress Report - Milestone C Command Availability / State-Aware Commands
Current report date: 2026-06-05.

## 1. What changed

Valetudo basic command capabilities now distinguish product support from current command availability. The adapter evaluates runtime health, source reachability/staleness, runtime-reported command readiness, and normalized robot state before marking `start_cleaning`, `pause`, `stop`, or `return_to_dock` available.

Commands are blocked before dispatch when the normalized capability says they are unsupported or unavailable. Blocked command results now preserve richer reasons such as `invalid_state`, `stale_source`, `source_unreachable`, `runtime_offline`, and `degraded_runtime`.

`return_to_dock` is only product-supported when BasicControl exists and the runtime reports return/home behavior. `resume` remains unsupported until a real runtime resume semantic exists.

## 2. Which mode this affects

- Mapping: no behavior change; Valetudo mapping remains unsupported.
- Navigation: no Valetudo go-to workflow was enabled.
- Clean Area: no Valetudo Clean Area execution was added.
- Rooms / Zones: no room, segment, or zone cleaning workflow was added.
- Valetudo backend: basic cleaning commands are now state-aware and health-aware.
- Shared adapter/runtime architecture: normalized capability availability is now the command gate used by both UI rendering and dispatch.

## 3. Ownership check

React/webview state owns rendering only: it shows or disables basic controls from normalized capability fields.

The VM runtime owns backend connection, source freshness, cached robot state, and command routing.

The Valetudo backend adapter owns the mapping from runtime/source/robot state into `vacuum_adapter` capability support and availability.

The UI only renders normalized adapter state: `supported`, `status`, `available`, and `availabilityReason`.

The UI submits normalized commands: `start_cleaning`, `pause`, `stop`, and `return_to_dock`.

This follows the rule: Product UI renders normalized adapter state and submits normalized commands. Backend adapter maps backend runtime state into `vacuum_adapter`. VM runtime owns backend connection, state cache, and command routing.

## 4. Webview close/reopen behavior

- idle Valetudo mock state: reopening hydrates from the runtime snapshot; start cleaning is available, pause/stop are unavailable with `invalid_state`, and return-to-dock depends on dock state.
- unavailable VM runtime: reopening maps to an offline adapter snapshot; basic commands are unavailable with `runtime_offline`.
- reachable mock runtime: reopening hydrates source health, dock state, mission state, and state-aware command availability from the runtime snapshot.
- active mock cleaning state: reopening hydrates cleaning state; pause, stop, and return-to-dock are available, while start cleaning is unavailable with `invalid_state`.
- paused mock cleaning state: reopening hydrates paused state; stop and return-to-dock are available, while start and pause are unavailable with `invalid_state`.
- terminal or stopped mock state: reopening hydrates idle/stopped state; start cleaning is available, stop/pause are unavailable with `invalid_state`, and return-to-dock is available only when not docked/charging/returning.

Hydration still flows through `useVacuumAdapter` and runtime snapshots. React does not reconstruct command authority after reopen.

## 5. Real hardware compatibility check

- Does this assume the robot is TurtleBot4/Nav2? No.
- Does this expose TurtleBot4/Nav2 specifics to product UI? No.
- Does this expose Valetudo raw capability names to product UI? No.
- Can the same VM runtime API later connect to a real Valetudo robot? Yes.
- What capability flags decide whether controls are shown/enabled? `supported`, `status`, `available`, `availabilityReason`, and structured `reasons`.
- What operations are explicitly unsupported? Valetudo map rendering, pose/navigation product surface, go-to, coverage, Clean Area, room cleaning, zone cleaning, segment cleaning, fan speed, water usage, consumables UI, manual control, mapping sessions, mission recovery commands without runtime support, and `resume`.

## 6. Feature behavior changed

- `start_cleaning` is available from idle/stopped states but blocked while cleaning, paused, or returning.
- `pause` is available only while actively cleaning.
- `stop` is available while cleaning, paused, or returning; it is intentionally not idempotent for idle/stopped/docked states.
- `return_to_dock` is available only when return/home behavior is reported and the robot is not already docked, charging, or returning.
- Stale, unreachable, offline, and degraded runtime/source states block basic commands with specific normalized reasons.
- Basic control buttons now disable from normalized availability, not just product support.

## 7. Files changed

- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/stateMapper.ts`: added state-aware Valetudo command availability rules and partial return-home support mapping.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/capabilityMapper.ts`: allowed the Valetudo mapper to mark individual product commands unsupported even when a broader backend capability exists.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/commandMapper.ts`: returned richer normalized command block codes from capability availability before dispatch.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/types.ts`: carried per-command unsupported reasons and richer mapping result error codes through the Valetudo boundary.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/useValetudoAdapter.ts`: mapped command send failures to `runtime_offline`.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`: disabled basic command buttons from normalized capability availability.
- `/home/shane/vscode-tensorfleet/scripts/vacuum-adapter-regression.ts`: added state-aware Valetudo command availability, health/source blocking, capability, dispatch-gate, UI-boundary, and regression coverage.
- `/home/shane/vscode-tensorfleet/progress_report.md`: updated this milestone report.

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

`bun run --cwd panels-standalone build` completed successfully with existing Vite warnings about browser-externalized `path`/`fs`, `eval` in `@protobufjs/inquire`, and large chunks.

Manual live webview validation was not run in this pass.
