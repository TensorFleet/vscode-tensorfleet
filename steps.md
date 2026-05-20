# Vacuum / Nav2 Implementation Progress Report

## Purpose

This file is the progress report for the current implementation. It records what
has landed, what has been validated, what is still open, and which runtime
caveats matter when interpreting results.

It is not the architecture source of truth. Use `VACUUM_STACK_PLAN.md` for the
stack architecture and layer plan. Use `extension.md` for VS Code extension
files, panels, topics, endpoints, and extension follow-up work.

Current report date: May 18, 2026.

## Runtime-Owned Mission Architecture Progress

Phase 1 has started in `~/vscode-tensorfleet`.

Current contract progress:

- `vacuum_adapter` now defines a normalized mission model for mapping,
  navigation, coverage, return-to-dock, room cleaning, zone cleaning, and
  hardware-native cleaning runs.
- Adapter snapshots now include `snapshot.activeMission` and
  `snapshot.missions` alongside the legacy `snapshot.mission` field.
- Mission statuses are backend-neutral:
  `idle / preparing / running / paused / canceling / returning / charging /
  resuming / needs_assistance / completed / failed / canceled / unsupported`.
- Mission snapshots include id, type, backend source, requested command, phase,
  progress, available actions, terminal result, error, and target payload.
- TurtleBot4/Nav2 mapping and navigation state are wrapped into
  `activeMission` snapshots as a contract bridge.
- Valetudo stub state exposes the same mission snapshot shape.
- New intent command names are reserved for runtime-owned execution:
  `start_navigation`, `start_coverage`, `pause_mission`, `resume_mission`,
  `cancel_mission`, `retry_mission_step`, and `skip_mission_step`.

Current limitation:

- Navigation and Clean Area active execution are runtime-owned for the
  TurtleBot4/Nav2 simulation path.
- Clean Area now submits area-only `start_coverage` requests for active runs.
  The VM mission runtime owns route generation and per-cell coverage overlay
  snapshots for TurtleBot4/Nav2.

## Repositories And Commits

Runtime and extension work landed across three repositories:

- `~/vscode-tensorfleet`
- `~/firecracker-vm`
- `~/vm-manager`

Code-only commits in `~/vscode-tensorfleet`:

- `1472aa5` `Align TurtleBot4 panels with live Nav2 runtime`
- `3d9359f` `Add Vacuum Control panel and associated files`
- `6382b7f` `Extract shared Nav2 runtime layer`
- `eb84ee9` `Extract Vacuum Control map canvas and fix live occupancy rendering`
- `20047a3` `Add vacuum map layer overlays`
- `96e2e8d` `Fix vacuum sensor overlay TF projection`
- `b87855e` `Polish vacuum operator controls`
- `d664024` `Refactor VacuumControlPanel and related components`
- `cc48301` `Add TeleopCard and CameraOverlay to Vacuum Control panel`

Code-only commits in `~/firecracker-vm`:

- `d251a57` `Stabilize TurtleBot4 Nav2 runtime in Firecracker`

Code-only commits in `~/vm-manager`:

- `8fc9535` `Reserve more vCPUs for TurtleBot4 VMs`

## Current Status

The TurtleBot4/Nav2 simulation path has a working operator slice through
`Vacuum Control`.

Layer status:

```text
Layer 0 — Sensors                   validated
Layer 1 — Localization + Map        running
Layer 2 — Navigation                closed for TurtleBot4/Nav2 simulation
Layer 3 — Vacuum Adapter            closed for TurtleBot4/Nav2 simulation,
                                    Valetudo stub reserved for Layer 6
Layer 4 prerequisite: Mapping + Whole Map View
                                    implemented for TurtleBot4/Nav2 simulation
Layer 4 — Coverage                  Clean Area execution, route generation,
                                    and per-cell progress snapshots are
                                    runtime-owned for TurtleBot4/Nav2
Layer 5 — Room / Zone Semantics     Milestone 1 implemented:
                                    manual room/zone annotations
Layer 6 — Real Hardware (Valetudo)  planned
```

What is currently true:

- The panel transport supports hidden Nav2 action services.
- Topic assumptions match the live VM's global TurtleBot4/Nav2 topics.
- Nav2 output reaches the robot drive path.
- Duplicate odom/scan/TF bridges were removed from the VM runtime.
- TurtleBot4 VMs now get at least 4 vCPUs, avoiding the earlier 1-vCPU collapse
  mode.
- Short `NavigateToPose` runs succeed end-to-end.
- `Vacuum Control` is the validated operator surface for the current
  TurtleBot4/Nav2 simulation path.
- `Vacuum Control` consumes `vacuum_adapter` through `useVacuumAdapter` instead
  of reading `useNav2Runtime` directly.
- The adapter exposes normalized map, navigation, readiness, mission, command,
  capability, and mapping state.
- Clean Area submits `start_coverage` through the adapter; the VM mission
  runtime owns Nav2 waypoint sequencing, lifecycle actions, and progress.
- Clean Area route preview and active route overlays are visible above coverage
  cells.
- Rooms / Zones mode now supports manual rectangular room/zone drafts, naming,
  save/select/delete, and map overlays through `snapshot.map.annotations`.
- Terminal navigation snapshots no longer force the panel back into Navigate
  mode after the mission is completed, canceled, or failed.

## Implemented: Extension Side

The standalone panels now match the live TurtleBot4 runtime instead of the old
namespaced assumptions.

Implemented in `~/vscode-tensorfleet`:

- Nav2 validation uses the real action paths:
  - `/navigate_to_pose/_action/send_goal`
  - `/navigate_to_pose/_action/get_result`
  - `/navigate_to_pose/_action/cancel_goal`
  - `/navigate_to_pose/_action/status`
  - `/navigate_to_pose/_action/feedback`
- Foxglove service discovery is surfaced through the extension bridge.
- Foxglove service calls tolerate missing request schemas for common ROS 2
  action service shapes.
- Non-visual Nav2 runtime logic is extracted under:
  - `panels-standalone/src/components/Nav2/runtime/useNav2Runtime.ts`
  - `panels-standalone/src/components/Nav2/runtime/nav2RuntimeConstants.ts`
  - `panels-standalone/src/components/Nav2/runtime/nav2RuntimeTypes.ts`
  - `panels-standalone/src/components/Nav2/runtime/nav2RuntimeUtils.ts`
- `Nav2Panel` is now a verification harness for the shared runtime layer.
- TurtleBot4 defaults in Teleop, Raw Messages, and 3D view use global topics
  instead of `/turtlebot4/*`.
- Connection settings preserve the explicit target port for standalone panels.
- `ros2-bridge.ts` caches the latest message per topic and replays it to new
  subscribers.
- `ros2-bridge.ts` accumulates static TF transforms per unique edge and replays
  them as a synthetic `/tf_static` bundle to new subscribers.
- `mapOverlayUtils.ts` projects lidar and depth obstacle overlays into `map`
  frame with a local `TransformTree`.
- Overlay projection supports frame fallback candidates for robot pose and lidar
  frames.
- Laser scan discovery matches `sensor_msgs/msg/LaserScan` and
  `foxglove.LaserScan`, preferring `/scan`.
- Point projection supports plain arrays and typed arrays from Foxglove point
  cloud fields.

## Implemented: Vacuum Control

Current visible shell components:

- `LeftNavRail`
- `Header`
- connection / status pill
- `StatusStrip`
- `MapCanvas`
- bounded map zoom controls, Fit Map, Follow Robot, and zoom readout
- map legend
- floating `Layers` checklist for Map, Global costmap, Local costmap, Plan,
  Lidar, and Depth obstacles
- `CameraOverlay` floating PiP window inside the map canvas
- mode switcher for `Mapping`, `Navigate`, `Clean Area`, and `Rooms / Zones`
- `MappingCard`
- `CleanAreaCard`
- `RoomZonesCard`
- selected destination card
- navigation progress card
- action buttons for connection, start, stop, clear, retry, and rerun
- `TeleopCard`
- persistent settings access from header and left rail

Runtime seams:

- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
- `panels-standalone/src/components/VacuumControl/MapCanvas.tsx`
- `panels-standalone/src/components/VacuumControl/TeleopCard.tsx`
- `panels-standalone/src/components/VacuumControl/CameraOverlay.tsx`
- `panels-standalone/src/components/VacuumControl/mapOverlayUtils.ts`
- `panels-standalone/src/vacuum-adapter/useVacuumAdapter.ts`
- `panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/useTurtleBot4Nav2Adapter.ts`
- `panels-standalone/src/components/Nav2/runtime/useNav2Runtime.ts`
- `panels-standalone/src/ros2-bridge.ts`

Map and overlay behavior:

- `MapCanvas` renders adapter-normalized `snapshot.map.grid` for the product
  base map.
- Direct `/map` rendering remains available as a diagnostic/fallback path.
- Live occupancy rendering tolerates Foxglove plain arrays and typed arrays.
- `/global_costmap/costmap` and `/local_costmap/costmap` render as optional
  raster overlays.
- Plan visibility is operator-toggleable.
- Robot marker, destination marker labels, route overlay, staged preview line,
  clean-area selection, and coverage overlays share the same world-to-screen
  transform.
- Lidar and depth obstacles render on dedicated canvases below robot and target
  markers.
- Clean Area route preview renders completed, current, and remaining waypoint
  segments.

Right-column behavior:

- Start is gated on map/preflight readiness rather than requiring pose-ready to
  be true before dispatch.
- Active runs show route, progress, remaining distance, elapsed navigation time,
  and recovery count when feedback is available.
- ETA was removed because `estimated_time_remaining` is not reliable enough for
  the operator surface.
- Run-again reuses the last sent destination.
- Mode locking prevents mapping, point navigation, and clean-area runs from
  competing with each other.
- Active navigation missions auto-select Navigate mode, but terminal navigation
  snapshots are presentation context only and do not trap the operator in that
  mode.

## Implemented: VM Runtime

Implemented in `~/firecracker-vm`:

- Removed duplicate global Gazebo bridges for TurtleBot4 `/tf`, `/tf_static`,
  `/odom`, and `/scan`.
- Updated `twist_deadman.py` to accept both `Twist` and `TwistStamped` on
  `/cmd_vel_raw`.
- Added a lean Nav2 launch for the VM: `navigation_vm.launch.py`.
- Switched Nav2 bringup to the VM-specific launch.
- Replaced heavier controller settings with a lighter DWB configuration and
  lower controller frequency for VM use.
- Added a VM-owned frontier exploration node for TurtleBot4/Nav2 auto mapping.
- The mapping node owns long-running exploration, reads map/pose/TF evidence,
  sends Nav2 goals, publishes `/vacuum_mapping/status`, and exposes
  `/vacuum_mapping/*` command services.
- Mapping can enter `needs_assistance` for blocked or unreachable exploration,
  stale map, missing pose, repeated navigation failures, or unavailable Nav2.
- Accepting a reviewed map saves through `nav2_map_server map_saver_cli` under
  `/opt/tensorfleet/maps/current_map.*` by default.
- After a map is accepted, later `/map` growth from normal destination or vacuum
  runs is autosaved after the map settles, while SLAM continues publishing map
  updates.

Implemented in `~/vm-manager`:

- TurtleBot4 VMs get a minimum of 4 vCPUs at runtime conversion time.
- Other VM variants keep their existing CPU behavior.

## Implemented: Vacuum Adapter

Layer 3 is closed for the TurtleBot4/Nav2 simulation path.

What exists:

- Public adapter surface under `panels-standalone/src/vacuum-adapter/`.
- `VacuumAdapter` exposes `snapshot` and `sendCommand(command)`.
- Public types cover capabilities, commands, state, errors, map grid/metadata,
  mission status, navigation status, mapping status, readiness, and battery.
- `useVacuumAdapter` selects the backend. TurtleBot4/Nav2 is wired today.
- `useTurtleBot4Nav2Adapter` wraps `useNav2Runtime`.
- TurtleBot4/Nav2 command dispatch lives in pure
  `backends/turtlebot4-nav2/commandDispatcher.ts`.
- `start_navigation` submits a backend-neutral navigation intent.
- TurtleBot4/Nav2 navigation execution is owned by the VM mission runtime,
  which maps the intent to Nav2 `NavigateToPose`.
- `cancel_mission` cancels the active runtime-owned mission;
  `cancel_navigation` remains a compatibility fallback.
- Active/terminal navigation state hydrates from normalized mission snapshots
  through `/vacuum_mission/status` and `/vacuum_mission/get_snapshot`.
- The UI may dismiss a terminal navigation destination locally after
  completed/canceled/failed runs, but that does not clear runtime mission
  history.
- Vacuum-only commands fail explicitly as unsupported in TurtleBot4/Nav2:
  - `start_cleaning`
  - `pause`
  - `resume`
  - `return_to_dock`
  - `segment_cleaning`
  - `zone_cleaning`
  - `set_fan_speed`
  - `set_water_usage`
- `manual_control` is explicitly rejected by `sendCommand` because teleop is a
  streaming publisher through `TeleopCard`, not a one-shot command.
- Mission snapshots carry backend-neutral runtime statuses:
  `idle / preparing / running / paused / canceling / returning / charging /
  resuming / needs_assistance / completed / failed / canceled / unsupported`.
  The legacy coarse `snapshot.mission` field remains for current UI
  compatibility.
- `Vacuum Control` branches on capabilities and normalized state, not backend
  names.
- The Valetudo stub defines capability, state, command, and runtime-boundary
  mapper shapes for Layer 6.

Focused regression coverage:

- command dispatch for supported TurtleBot4/Nav2 commands
- explicit unsupported command failures
- normalized plan path mapping
- mission state mapping
- public contract import boundaries
- backend-name branching checks in `VacuumControlPanel.tsx`
- Valetudo capability/command mapper shape
- clean-area profile/planner/coverage contract checks

Primary verification command:

```sh
bun run test:vacuum-adapter
```

## Implemented: Mapping And Whole Map View

The Layer 4 prerequisite is implemented for TurtleBot4/Nav2 simulation.

Current behavior:

- First valid `/map` displays the full known occupancy-grid bounds by default.
- Fit Map returns to full map bounds, not robot pose, route geometry, selected
  target, costmaps, or known/free cells only.
- The viewport has explicit `fit`, `manual`, and `follow_robot` modes.
- Pan, zoom, Fit Map, Follow Robot, panel resize, base map, costmaps, plan path,
  lidar/depth overlays, robot marker, and target marker stay aligned.
- View labels distinguish Full known map, Manual view, Following robot, Waiting
  for map, and Map mostly unexplored.
- `MappingCard` supports Start auto mapping, Manual mapping, Pause, Resume auto
  mapping, Finish & review, Discard, Accept map, saved-map inventory, and load.
- Mapping/review disables navigation target staging by default.
- Active auto mapping disables competing teleop until paused.
- Manual mapping, paused mapping, and `needs_assistance` keep teleop available.
- Map metadata shows dimensions, resolution, known/free/occupied/unknown ratios,
  known area, last update age, pose availability, and readiness.
- Mapping status shows known/unknown ratio, frontier count, visited and failed
  goal counts, active goal, state reason, last error, update time, and
  persistence result.
- `snapshot.mapping.savedMaps`, `activeMapName`, `loadedMapPath`, and
  `loadError` expose saved-map inventory and load state.
- `load_map` is routed through the TurtleBot4/Nav2 VM mapping runtime.

VM mapping surfaces:

- `/vacuum_mapping/status`
- `/vacuum_mapping/start_auto`
- `/vacuum_mapping/start_manual`
- `/vacuum_mapping/pause`
- `/vacuum_mapping/resume`
- `/vacuum_mapping/finish`
- `/vacuum_mapping/discard`
- `/vacuum_mapping/accept`
- `/vacuum_mapping/save_map`
- `/vacuum_mapping/load_map`
- `/vacuum_mapping/list_maps`

## Implemented: Clean Area Runtime Mission

Clean Area is the current Layer 4 runtime-owned mission path. The UI owns
rectangle drafting and local preview before start; the VM mission runtime owns
active execution after `start_coverage`.

Current behavior:

- `Vacuum Control` has `Mapping`, `Navigate`, `Clean Area`, and
  `Rooms / Zones` modes.
- `MapCanvas` supports rectangular clean-area selection with draw, move, and
  resize interactions.
- Selection is validated against map bounds and occupancy data.
- `cleanAreaProfile.ts` owns swath width, overlap, navigation goal tolerance,
  boundary margin, minimum useful region size, completion threshold, and derived
  lane spacing / boundary extension.
- The default profile preserves the current 0.30 m TurtleBot4 simulation swath.
- `cleanAreaPlanner.ts` generates a profile-backed swath-overlap lawnmower
  waypoint preview.
- The planner chooses the longer axis for passes.
- Lane spacing stays at or below the configured swath.
- Sampled lanes are clipped to known free occupancy-grid cells instead of
  sweeping full-width rows through blocked or unknown cells.
- Boundary pass goals extend by tolerance plus boundary margin so close-enough
  Nav2 completion does not advance before the robot crosses the clean-area edge.
- `VacuumControlPanel.tsx` submits `start_coverage` through the adapter.
- The VM mission runtime executes coverage by privately sequencing Nav2 goals.
- `cleanAreaCoverage.ts` classifies selected map cells as cleanable, occupied,
  unknown, out-of-bounds, remaining, covered, or too small.
- Cleanable cells are decomposed into connected regions.
- Tiny disconnected regions are skipped rather than planned as pointless
  waypoints.
- Active clean-area runs hydrate from `snapshot.activeMission`.
- Runtime coverage progress uses map-frame robot pose history and configured
  swath width to mark cleanable cells covered.
- `MapCanvas` renders remaining cells, covered cells, excluded
  occupied/unknown cells, skipped cells, and current robot footprint.
- Clean Area route preview and runtime route overlays render above coverage
  cells so the planned path remains visible while reviewing or running an area.
- `CleanAreaCard` reports coverage percentage, cleaned area, remaining area,
  skipped area, route status, pass count, distance, and waypoint progress.
- Clean Area mode shows adapter-driven mission lifecycle state for dock,
  return-to-dock, battery, and charging capabilities.
- Clean-area state covers editing, confirmed, preparing, running, paused,
  canceling, completed, failed, and canceled.
- Operators can pause, cancel, retry waypoint, skip waypoint, and clear the
  area when inactive.
- `TeleopCard` is disabled during active clean-area waypoint runs.

Current limits:

- Runtime route generation is row-level occupancy-clipped; stronger
  obstacle-adjacent and component-level planning remains pending.
- Runtime coverage progress is first-pass footprint-history accounting, not
  production-complete coverage.
- Clipping is lane-level; connected-region accounting exists, but route
  generation is not yet component-level area planning.
- Stronger edge/corner coverage is pending.
- Stronger behavior near obstacles and unknown cells is pending.
- Dock / undock execution is pending.
- Battery-aware return/resume is pending.
- UI treats 95% covered cleanable cells as complete and can report route-done
  when uncovered cleanable cells remain.

## Implemented: Room / Zone Semantics Milestone 1

Rooms / Zones is the current Layer 5 prototype path. Milestone 1 adds manual
annotation state only; it does not start room/zone cleaning missions yet.

Current behavior:

- `Vacuum Control` has a `Rooms / Zones` mode.
- `MapCanvas` reuses the editable rectangle behavior from Clean Area for a
  room/zone draft.
- Operators can choose `Room` or `Zone`, enter a name, save the draft, select a
  saved annotation, and delete it.
- Saved annotations render as map overlays and side-panel list entries.
- Adapter snapshots expose saved annotations through `snapshot.map.annotations`.
- TurtleBot4/Nav2 persists prototype annotations in webview storage keyed to
  the active map identity where one is known.
- Valetudo currently reports map annotations and room/zone semantics as
  unsupported.

Current limits:

- Unsaved room/zone drafts are local webview state.
- Saved annotation persistence is not VM/runtime-owned yet.
- Room/zone geometry is rectangle-only.
- Room/zone cleanability preview and cleaning execution are not implemented
  until later Layer 5 milestones.

## Runtime Validation

Validated on:

- VM: `root@172.16.0.10`
- runtime date: April 22, 2026

Verified:

- VM runs with `4` vCPUs.
- Nav2 lifecycle services are active.
- `/navigate_to_pose` action is present.
- `/map`, `/scan`, `/odom`, `/pose`, `/tf`, `/tf_static`, `/plan`,
  `/cmd_vel_nav`, `/cmd_vel_raw`, and `/cmd_vel` are present.
- `map -> odom -> base_link` is available.

Smoke test results:

- `0.30 m` forward `NavigateToPose` succeeded.
- `1.00 m` forward `NavigateToPose` produced real motion and controller output,
  but the first long run timed out waiting for a terminal result.

Validated `Vacuum Control` operator flows:

- connect to live VM Foxglove endpoint
- render live occupancy map
- select navigation target from the rendered map
- send `NavigateToPose` through the live Nav2 action API
- observe live progress from feedback/status
- cancel active goal and observe canceled/stopped state
- observe terminal completed/failed/canceled state
- select a new target after cancel and send a second goal without reload
- exercise layer overlays, sensor overlays, teleop, and failure states through
  the VS Code webview

Closed Layer 2 / Layer 3 validation checklist:

- disconnected bridge shows offline header and state card
- connected bridge without `/map` shows waiting-for-map
- live `/map` and `/pose` activate readiness chips
- layers popover toggles map, costmaps, plan, lidar, and depth without placing
  a target
- clicking the map selects a target and updates the destination card
- sending a goal changes state, actions, and progress UI
- active run shows route, progress, remaining distance, elapsed navigation time,
  and recovery count when feedback is available
- lidar and depth overlays either project into map frame or report waiting /
  no-TF state
- cancel transitions to canceled state and disables right buttons while pending
- clear removes the target only when no active run exists
- success and failure terminal results render correctly
- settings buttons open the connection overlay
- stale canceled marker and stale plan geometry are cleared after fresh target
  selection

Suggested build verification:

```sh
bun run test:vacuum-adapter
bun run typecheck
bun run --cwd panels-standalone build
```

## Runtime Caveats

The main runtime caveat is dock-blocked starts.

Observed behavior:

- the robot can start physically blocked by dock geometry
- the dock is spawned adjacent to the TurtleBot4 at bringup time
- current Nav2 costmaps rely on `/scan`
- lidar does not reliably represent dock geometry as a blocking obstacle in the
  costmap
- a healthy Nav2 run can look stalled because the robot is pushing against the
  dock

Practical consequence:

- navigation validation must start from a clear, undocked pose
- clear-space validation is required before treating a failed or stalled run as
  a software failure
- dock / undock and battery-aware behavior remain Layer 4 work

## Open Work

Near-term validation:

- live-validate Clean Area MVP against the VM
- visually retest Clean Area preview/execution route overlays in the webview
- retest mode switching after canceled/completed/failed navigation missions
- validate pause, cancel, retry, skip, failure handling, and mode locking during
  Clean Area runs
- keep `RawMessagesPanel.tsx` and `SensorView3DPanel.tsx` useful as supporting
  debug surfaces

Production coverage follow-up:

- stronger edge/corner handling
- component-level route planning around obstacles and unknown cells
- resume/recovery semantics
- dock / undock execution
- battery-aware return/resume

Later layers:

- room / zone semantics
- Valetudo integration runtime in the VM
- real hardware validation
- consumables and scheduling UI
- OpenClaw workflow integration, if still useful above the adapter contract

## Completed Execution Order

The completed Layer 2 implementation order was:

1. Lock the current shell and document what already exists.
2. Complete visible components against live runtime behavior.
3. Validate the one-panel operator flow end-to-end.
4. Defer non-critical support surfaces until after the operator slice is
   runtime-solid.

The original shell stage is complete. It built a polished `Vacuum Control`
standalone panel first, then wired it to live runtime behavior. The shell is
map-first, user-oriented, and built around product concepts: state, target,
progress, and actions.

Deferred follow-up components from the original Layer 2 plan:

- `RunHistoryPanel`
- `GoalDetailsInspector`
- `RuntimeHealthDrawer`

`WarningCard` remains deferred and is not part of the closed Layer 2 operator
slice.

## Not Next

Do not make these the immediate milestone:

- room cleaning UI
- zone cleaning UI
- docking workflow UI
- simulated battery/charging behavior
- consumables UI
- scheduling UI
- OpenClaw workflow integration

Those belong after Layer 4/Layer 5 coverage and room/zone semantics, or in the
Layer 6 real-hardware path where noted.
