# Progress Report - Milestone E Optional Advanced Surfaces / No-Map Contract Hardening
Current report date: 2026-06-05.

## 1. What changed

Advanced robot surfaces are now treated as optional product surfaces instead of assumed simulation surfaces. A Valetudo robot can be online, reachable, dock-aware, battery-aware, and command-capable without exposing a map, pose, navigation route, mapping session, coverage planner, map annotations, room/zone semantics, camera, or manual teleop.

Valetudo no-map/no-pose state remains a normal snapshot. The adapter keeps compatibility fields such as `map`, `pose`, `navigation`, and `mapping`, but they now report unavailable or unsupported values with null data instead of fake map or pose data. Basic activity and basic cleaning controls still work from normalized capability and availability flags.

TurtleBot4/Nav2 map, pose, navigation, mapping, and coverage behavior remains intact. Regression coverage now proves Nav2 map/pose/navigation hydrate when runtime data exists while Valetudo advanced surfaces stay unsupported.

## 2. Which mode this affects

- Mapping: Valetudo mapping remains unsupported/unavailable; TurtleBot4/Nav2 mapping remains hydrated through existing mapping state.
- Navigation: Valetudo navigation/go-to remains unsupported and inactive; TurtleBot4/Nav2 navigation still hydrates active targets, pose, map, and progress.
- Clean Area: Valetudo coverage/Clean Area remains unsupported; TurtleBot4/Nav2 coverage remains supported.
- Rooms / Zones: Valetudo room/zone semantics and cleaning remain unsupported or detected-not-ready; TurtleBot4/Nav2 room/zone annotations and cleaning remain capability-gated.
- Valetudo backend: no-map/no-pose hardware is now explicitly covered as a valid adapter state.
- Shared adapter/runtime architecture: advanced surfaces are capability-gated compatibility fields, not mandatory product state.

## 3. Ownership check

React/webview state owns local drafts and rendering only. It renders normalized capability flags, map/pose availability, activity, dock, battery, command availability, mission snapshots, and unsupported cards/fallbacks.

The VM runtime owns backend connection, source freshness, cached robot state, and command routing.

The Valetudo backend adapter owns mapping runtime/source/robot state into `vacuum_adapter`, including unsupported advanced surface defaults.

The UI only renders normalized adapter state: `capabilities.map.supported`, `map.metadata.hasMap`, `pose.available`, `navigation.active`, `mapping.persistence`, `activity.status`, and command availability.

The UI submits normalized commands such as `start_cleaning`, `pause`, `stop`, `return_to_dock`, `start_navigation`, `start_coverage`, and mission controls when the normalized capabilities allow them.

This follows the rule: Product UI renders normalized adapter state and submits normalized commands. Backend adapter maps backend runtime state into `vacuum_adapter`. VM runtime owns backend connection, state cache, and command routing.

## 4. Webview close/reopen behavior

- idle Valetudo mock state: reopening hydrates identity, source, activity, dock, battery, basic controls, and explicit no-map/no-pose compatibility fields from the runtime snapshot.
- unavailable VM runtime: reopening maps to offline/unavailable adapter state with null map grid, `metadata.hasMap = false`, unavailable pose, inactive navigation, unsupported mapping, and disabled commands.
- reachable mock runtime: reopening hydrates reachable source state and basic control availability without requiring map or pose data.
- active mock cleaning state: reopening hydrates `activity.status = "cleaning"`, the compatibility hardware-cleaning mission bridge, and unsupported advanced surfaces.
- paused mock cleaning state: reopening hydrates `activity.status = "paused"` and state-aware basic command availability.
- terminal or stopped mock state: reopening hydrates idle activity with no active mission and no advanced surface fault.

Hydration continues to flow from VM/runtime snapshots through the adapter mapper. React does not reconstruct map, pose, navigation, or command authority after reopen.

## 5. Real hardware compatibility check

- Does this assume the robot is TurtleBot4/Nav2? No.
- Does this expose TurtleBot4/Nav2 specifics to product UI? No.
- Does this expose Valetudo raw capability names to product UI? No.
- Can the same VM runtime API later connect to a real Valetudo robot? Yes.
- What capability flags decide whether controls are shown/enabled? `capabilities.map.supported`, `start_navigation`, `go_to_location`, `mapping_session`, `auto_mapping`, `coverage_mission`, `start_coverage`, `map_annotations`, `room_semantics`, `zone_semantics`, `room_cleaning`, `zone_cleaning`, `manual_control`, plus `supported`, `status`, `available`, `availabilityReason`, and state-aware command availability.
- What operations are explicitly unsupported? Valetudo map rendering, product pose, navigation/go-to, navigation status, mapping sessions, auto mapping, coverage/Clean Area, map annotations, room semantics, zone semantics, room cleaning, product zone cleaning, manual control, resume, mission recovery, fan speed, water usage, and consumables UI.

## 6. Feature behavior changed

- Valetudo snapshots are valid without map or pose data.
- Valetudo map state reports unavailable with `grid = null`, `metadata.hasMap = false`, no receiving state, and no annotations.
- Valetudo pose state reports unavailable with no coordinates and no backend source marker.
- Valetudo navigation state reports inactive, no target, no backend goal state, no path, and an unsupported detail.
- Valetudo mapping state remains idle with unsupported persistence and no saved maps.
- No-map/no-pose Valetudo state is not treated as a fault.
- Basic Valetudo activity and basic commands remain available when the runtime reports they are safe.
- UI boundary tests verify product behavior does not branch on backend id, raw Valetudo capability names, Nav2 backend goal state, or saved-map file paths.

## 7. Files changed

- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/stateMapper.ts`: clarified unsupported Valetudo map, pose, and navigation compatibility fields.
- `/home/shane/vscode-tensorfleet/scripts/vacuum-adapter-regression.ts`: added regression coverage for Valetudo no-map/no-pose optional surfaces, TurtleBot4/Nav2 advanced surface compatibility, UI capability gating, and diagnostics-boundary checks.
- `/home/shane/vscode-tensorfleet/progress_report.md`: updated the milestone report.
- `/home/shane/knowledge/knowledge.md`: appended one transferable note about optional contract surfaces.

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

Remaining simulation-shaped fields: compatibility fields still include `map.topic`, `pose.source`, `navigation.backendGoalState`, and saved-map file paths such as `yamlPath`, `imagePath`, and `poseGraphPath`. This milestone keeps them for compatibility and verifies the product UI does not use them as behavior flags.
