# Progress Report - Milestone F Diagnostics Boundary Hardening
Current report date: 2026-06-05.

## 1. What changed

Backend-specific debug details now have explicit diagnostic surfaces on the vacuum adapter snapshot. Product-facing fields remain compatible, but Nav2 topics, pose sources, backend goal states, backend capability labels, and saved map file paths are mirrored into `snapshot.diagnostics` so debug visibility no longer depends on product behavior fields.

Valetudo diagnostics now separate detected/raw backend capability details from normalized public capabilities. No-map/no-pose Valetudo state remains valid, and unsupported map, pose, navigation, and mapping surfaces are described as diagnostics instead of product controls.

## 2. Which mode this affects

- Mapping: Saved map file path details are mirrored into diagnostics; product mapping state and saved map compatibility fields remain.
- Navigation: Nav2 backend goal state and action service identifiers are mirrored into diagnostics; normalized navigation state remains the product surface.
- Clean Area: No behavior change; controls remain capability-gated through normalized coverage capability and command availability.
- Rooms / Zones: No behavior change; controls remain gated through normalized room/zone capabilities and annotations.
- Valetudo backend: Raw capability names and detected diagnostic capabilities stay under diagnostics while detected-but-not-ready public capabilities remain unsupported.
- Shared adapter/runtime architecture: Diagnostics now has clearer `capabilities`, `map`, `pose`, `navigation`, and `mapping` buckets.

## 3. Ownership check

React/webview state owns local drafts and rendering only. It renders normalized adapter state such as capabilities, activity, health, source, dock, battery, mission snapshots, map/pose availability, and unsupported advanced surfaces.

The VM runtime owns backend connection, state cache, and command routing.

The Valetudo backend adapter owns mapping runtime/source/robot state into `vacuum_adapter`, including raw backend capability diagnostics and unsupported advanced surface diagnostics.

The UI only renders normalized adapter state. It must not branch on `diagnostics`, `map.topic`, `pose.source`, `navigation.backendGoalState`, saved map file paths, raw Valetudo capability names, or backend id outside backend selection/composition.

The UI submits normalized commands such as `start_cleaning`, `pause`, `stop`, `return_to_dock`, `start_navigation`, `start_coverage`, and mission controls when normalized capabilities and availability allow them.

This follows the rule: Product UI renders normalized adapter state and submits normalized commands. Backend adapter maps backend runtime state into `vacuum_adapter`. VM runtime owns backend connection, state cache, and command routing.

## 4. Webview close/reopen behavior

- idle Valetudo mock state: reopening hydrates identity, source, activity, dock, battery, basic controls, unsupported advanced surfaces, and Valetudo diagnostics from the runtime snapshot.
- unavailable VM runtime: reopening maps to offline/unavailable adapter state with disabled commands, no map, no pose, inactive navigation, unsupported mapping, and offline diagnostics.
- reachable mock runtime: reopening hydrates reachable source state and basic command availability without needing map or pose data.
- active mock cleaning state: reopening hydrates `activity.status = "cleaning"`, the hardware-cleaning mission bridge, command availability, and diagnostics.
- paused mock cleaning state: reopening hydrates `activity.status = "paused"` and state-aware command availability.
- terminal or stopped mock state: reopening hydrates idle activity with no active mission and no advanced surface fault.

Hydration still flows from VM/runtime snapshots through adapter mappers. React does not reconstruct backend diagnostics, mission authority, or advanced surface availability after reopen.

## 5. Real hardware compatibility check

- Does this assume the robot is TurtleBot4/Nav2? No.
- Does this expose TurtleBot4/Nav2 specifics to product UI? No; Nav2 details are mirrored into diagnostics while compatibility fields remain temporarily.
- Does this expose Valetudo raw capability names to product UI? No; raw names remain in diagnostics and regression tests scan the Vacuum Control UI for leaks.
- Can the same VM runtime API later connect to a real Valetudo robot? Yes.
- What capability flags decide whether controls are shown/enabled? `capabilities.map`, `pose`, `navigation_status`, `start_navigation`, `go_to_location`, `mapping_session`, `auto_mapping`, `coverage_mission`, `start_coverage`, `map_annotations`, `room_semantics`, `zone_semantics`, `room_cleaning`, `zone_cleaning`, `manual_control`, `start_cleaning`, `pause`, `stop`, `return_to_dock`, plus `supported`, `status`, `available`, and `availabilityReason`.
- What operations are explicitly unsupported? Valetudo map rendering, pose, navigation/go-to workflow, mapping sessions, auto mapping, coverage/Clean Area, map annotations, room semantics, room cleaning, manual control, resume, mission recovery, fan speed, water usage, consumables UI, and product zone cleaning until explicit geometry mapping exists.

## 6. Feature behavior changed

- Nav2 map topic diagnostics are available at `snapshot.diagnostics.map.topic`.
- Nav2 pose source diagnostics are available at `snapshot.diagnostics.pose.source`.
- Nav2 backend goal state diagnostics are available at `snapshot.diagnostics.navigation.backendGoalState`.
- Nav2 saved map paths are available at `snapshot.diagnostics.mapping.savedMapPaths`.
- Backend capability labels are mirrored into `snapshot.diagnostics.capabilities.backendCapabilities`.
- Valetudo raw capability names are available only through diagnostics surfaces.
- Existing public compatibility fields remain for now: `map.topic`, `pose.source`, `navigation.backendGoalState`, `CapabilitySupport.backendCapability`, and saved map path fields.
- Product UI boundary tests now cover map topic, pose source, backend goal state, saved map paths, raw Valetudo capability names, and backend id behavior branching.

## 7. Files changed

- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/state.ts`: added diagnostic buckets and deprecation notes for backend-shaped compatibility fields.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/capabilities.ts`: marked `backendCapability` as a compatibility mirror that should move to diagnostics for product behavior.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/stateMapper.ts`: mirrored Nav2 topic, pose source, goal state, action service, backend capability, and saved map path details into diagnostics.
- `/home/shane/vscode-tensorfleet/panels-standalone/src/vacuum-adapter/backends/valetudo/stateMapper.ts`: clarified Valetudo diagnostics for raw capabilities and unsupported map/pose/navigation/mapping surfaces.
- `/home/shane/vscode-tensorfleet/scripts/vacuum-adapter-regression.ts`: added diagnostics-boundary and compatibility regression checks for Nav2, Valetudo, and Vacuum Control UI behavior.
- `/home/shane/vscode-tensorfleet/progress_report.md`: updated the milestone report.

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

`git diff --check` passed.
