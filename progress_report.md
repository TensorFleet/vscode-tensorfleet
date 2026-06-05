# Progress Report - Milestone D Activity vs Mission Split
Current report date: 2026-06-05.

## 1. What changed

The vacuum adapter contract now has a backend-neutral `activity` snapshot field for broad robot behavior. The chosen field name is `activity`, and its status enum is `VacuumRobotActivityStatus`: `unknown`, `unavailable`, `idle`, `cleaning`, `paused`, `returning`, `docked`, `charging`, `faulted`, `mapping`, `navigating`, and `covering`.

Valetudo basic robot states now map into `activity` without requiring every hardware state to become a product mission. A docked but not charging robot reports `activity.status = "docked"` while legacy `mission.state` remains `idle`; charging reports `charging`; cleaning reports `cleaning`; paused reports `paused`; returning reports `returning`; stopped/idle reports `idle`; source unreachable or runtime offline reports `unavailable`; error/fault states report `faulted`.

TurtleBot4/Nav2 keeps existing mission behavior and now adds activity from the same normalized runtime state: active navigation maps to `navigating`, coverage/room/zone cleaning workflows map to `covering` or `paused`, mapping maps to `mapping`, idle maps to `idle`, and disconnected runtime maps to `unavailable`.

Raw Valetudo state is retained only in diagnostics as `diagnostics.raw.valetudoState`.

## 2. Which mode this affects

- Mapping: active mapping now also reports `activity.status = "mapping"`.
- Navigation: active Nav2 navigation now also reports `activity.status = "navigating"`.
- Clean Area: coverage workflows now also report `activity.status = "covering"` or `paused`.
- Rooms / Zones: room and zone cleaning workflows now also report `activity.status = "covering"` or `paused`.
- Valetudo backend: basic robot activity is normalized separately from mission workflows.
- Shared adapter/runtime architecture: the snapshot can now distinguish broad activity from explicit product missions.

## 3. Ownership check

React/webview state owns rendering only. It may render `activity`, capabilities, legacy mission fields, and mission workflows, but it does not infer backend-specific behavior.

The VM runtime owns backend connection, source freshness, cached robot state, and command routing.

The Valetudo backend adapter owns mapping runtime/source/robot state into `activity`, legacy mission state, diagnostics, dock, battery, source, and normalized capabilities.

The UI only renders normalized adapter state such as `activity.status`, `capabilities.*.available`, `activeMission`, and `missions`.

The UI submits normalized commands such as `start_cleaning`, `pause`, `stop`, `return_to_dock`, `start_navigation`, and mission actions.

This follows the rule: Product UI renders normalized adapter state and submits normalized commands. Backend adapter maps backend runtime state into `vacuum_adapter`. VM runtime owns backend connection, state cache, and command routing.

## 4. Webview close/reopen behavior

- idle Valetudo mock state: reopening hydrates from the runtime snapshot with `activity.status = "idle"` or `docked` depending on dock state; legacy mission remains idle.
- unavailable VM runtime: reopening maps to offline/unavailable adapter state with `activity.status = "unavailable"` and commands unavailable with `runtime_offline`.
- reachable mock runtime: reopening hydrates source, dock, battery, capabilities, activity, and legacy mission state from the runtime snapshot.
- active mock cleaning state: reopening hydrates `activity.status = "cleaning"` and preserves the current compatibility `activeMission` bridge.
- paused mock cleaning state: reopening hydrates `activity.status = "paused"` and state-aware command availability.
- terminal or stopped mock state: reopening hydrates `activity.status = "idle"` with no active mission unless the runtime reports a still-active workflow.

Hydration still flows through the VM runtime snapshot and adapter mapper. React does not reconstruct activity or command authority after reopen.

## 5. Real hardware compatibility check

- Does this assume the robot is TurtleBot4/Nav2? No.
- Does this expose TurtleBot4/Nav2 specifics to product UI? No.
- Does this expose Valetudo raw capability names to product UI? No.
- Can the same VM runtime API later connect to a real Valetudo robot? Yes.
- What capability flags decide whether controls are shown/enabled? `supported`, `status`, `available`, `availabilityReason`, and structured `reasons`; `activity.availableActions` is derived from those normalized fields for Valetudo basic actions.
- What operations are explicitly unsupported? Valetudo map rendering, pose/navigation product surface, go-to, coverage, Clean Area, room cleaning, zone cleaning, segment cleaning, fan speed, water usage, consumables UI, manual control, mapping sessions, mission recovery commands without runtime support, and `resume`.

## 6. Feature behavior changed

- Adapter snapshots now include normalized `activity` alongside `mission`, `activeMission`, and `missions`.
- Valetudo docked idle state now reads as robot activity `docked` while remaining a non-mission idle state.
- Valetudo charging, cleaning, paused, returning, stopped/idle, unavailable, stale, and fault/error states have explicit product activity mapping.
- TurtleBot4/Nav2 navigation, mapping, and coverage workflows now expose broad activity without changing existing mission workflow fields.
- Raw Valetudo state is available only as diagnostics, not as a product behavior flag.
- Compatibility mission fields remain present; Valetudo basic cleaning still keeps the current bridge active mission where existing UI/tests require it.

## 7. Files changed

- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/state.ts`: added `VacuumRobotActivity`, activity statuses/actions, and optional `snapshot.activity`.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/stateMapper.ts`: mapped Valetudo runtime state/source health into normalized activity and moved raw backend state into diagnostics.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/types.ts`: carried normalized activity fields through the Valetudo boundary.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/stateMapper.ts`: derived activity from existing Nav2 connection, mapping, navigation, and mission state.
- `/home/shane/vscode-tensorfleet/scripts/vacuum-adapter-regression.ts`: added activity mapping, mission compatibility, stale/offline/unreachable, diagnostics privacy, and UI-boundary regression coverage.
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

Remaining mission/activity ambiguity: Valetudo basic cleaning still creates a compatibility `activeMission` bridge for current UI/tests. Future UI migration can prefer `activity` for hardware-style basic cleaning and reserve `activeMission` for product-owned workflows with target/progress/result.
