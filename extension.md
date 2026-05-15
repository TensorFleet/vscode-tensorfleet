# VS Code Extension Vacuum / Nav2 Reference

## Purpose

This file contains extension-specific knowledge for the TurtleBot4/Nav2 vacuum
work in `~/vscode-tensorfleet`.

It covers:

- extension repository structure
- standalone panel entrypoints
- runtime bridge endpoints
- panel topic and service maps
- `Vacuum Control` implementation boundaries
- supporting debug panel expectations
- extension follow-up work

It is not the implementation progress report. Use `steps.md` for progress and
validation status. It is not the architecture source of truth. Use
`VACUUM_STACK_PLAN.md` for the layer plan, product boundary, adapter contract,
and future architecture.

## Current Extension Goal

Use the existing VS Code extension panels against the current TurtleBot4/Nav2 VM
backend.

The extension should:

1. Connect to the running VM bridge endpoints.
2. Treat `Vacuum Control` as the normal operator surface for the closed Layer 2
   navigation slice and closed Layer 3 adapter slice.
3. Show live map, lidar, odom/TF, costmap, camera, navigation, and mapping state
   where the current panels support those message types.
4. Expose enough Nav2 action visibility to validate goal execution.
5. Treat Clean Area as the current Layer 4 runtime-owned mission path:
   rectangular draft selection and local preview before start, then
   adapter-backed `start_coverage` execution and snapshot hydration after
   start.
6. Record panel gaps as extension follow-up work, not as blockers for the
   already-closed Layer 2/Layer 3 simulation slice.

## Repository

Extension repo:

- `~/vscode-tensorfleet`

Important files:

- `src/extension.ts`
- `src/regions.ts`
- `src/templates/drone-view-list.html`
- `panels-standalone/src/vacuum_control.tsx`
- `panels-standalone/src/ros2-bridge.ts`
- `panels-standalone/src/foxglove-networking.ts`
- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
- `panels-standalone/src/components/VacuumControl/MapCanvas.tsx`
- `panels-standalone/src/components/VacuumControl/TeleopCard.tsx`
- `panels-standalone/src/components/VacuumControl/CameraOverlay.tsx`
- `panels-standalone/src/components/VacuumControl/mapOverlayUtils.ts`
- `panels-standalone/src/components/VacuumControl/cleanAreaProfile.ts`
- `panels-standalone/src/components/VacuumControl/cleanAreaPlanner.ts`
- `panels-standalone/src/components/VacuumControl/cleanAreaCoverage.ts`
- `panels-standalone/src/components/Nav2/runtime/useNav2Runtime.ts`
- `panels-standalone/src/components/Nav2/runtime/nav2RuntimeConstants.ts`
- `panels-standalone/src/components/Nav2/runtime/nav2RuntimeTypes.ts`
- `panels-standalone/src/components/Nav2/runtime/nav2RuntimeUtils.ts`
- `panels-standalone/src/components/SensorView3D/SensorView3DPanel.tsx`
- `panels-standalone/src/components/RawMessages/RawMessagesPanel.tsx`
- `panels-standalone/src/components/MissionControl/MissionControl.tsx`
- `panels-standalone/src/components/MissionControl/map/DroneMap.tsx`
- `panels-standalone/src/vacuum-adapter/`

Current notable extension state:

- `src/regions.ts` defines Foxglove and ROS bridge ports:
  - Foxglove: `8765`
  - rosbridge: `9091`
- `src/extension.ts` injects `window.TENSORFLEET_VM_CONFIG_ID` into standalone
  panels when a VM config is known.
- `panels-standalone/src/vacuum_control.tsx` mounts `VacuumControlPanel`.
- `panels-standalone/src/ros2-bridge.ts` currently uses Foxglove first.

## VM Bridge Endpoints

Current VM endpoints:

- Foxglove/Lichtblick panels: `ws://172.16.0.10:8765`
- rosbridge-specific panels: `ws://172.16.0.10:9091`

The current standalone panel path primarily uses Foxglove:

- `panels-standalone/src/ros2-bridge.ts`
- `panels-standalone/src/foxglove-networking.ts`

`ros2-bridge.ts` defines `ConnectionMode = "foxglove"`, so new panel work
should assume Foxglove first unless a panel explicitly needs rosbridge.

## Runtime Topic And Service Map

Current operator slice uses global topics and action paths. Older
`/turtlebot4/*` topic guidance is outdated for this slice.

Current topics and services used by the panel/runtime:

- `/map`
- `/scan`
- depth `sensor_msgs/msg/PointCloud2` topic discovered from advertised topics,
  preferring `/oakd/rgb/preview/depth/points`
- camera image topic discovered from advertised `sensor_msgs/msg/Image` and
  `sensor_msgs/msg/CompressedImage` topics, preferring
  `/oakd/rgb/preview/image_raw`
- `/odom`
- `/pose`
- `/tf`
- `/tf_static`
- `/plan`
- `/transformed_global_plan`
- `/cmd_vel_nav`
- `/cmd_vel_raw` publish path for `TeleopCard`
- `/cmd_vel`
- `/local_costmap/costmap`
- `/global_costmap/costmap`
- `/stop_status`
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
- `/navigate_to_pose/_action/send_goal`
- `/navigate_to_pose/_action/get_result`
- `/navigate_to_pose/_action/cancel_goal`
- `/navigate_to_pose/_action/status`
- `/navigate_to_pose/_action/feedback`

## Bridge Behavior

Current bridge behavior that extension work can rely on:

- Foxglove service discovery is available through the extension bridge.
- Foxglove service-call support tolerates missing request schemas for common
  ROS 2 action service shapes.
- `ros2-bridge.ts` caches the latest message per topic and replays it
  immediately to a new subscriber.
- `ros2-bridge.ts` accumulates static TF transforms per unique edge and replays
  them as a synthetic bundle when a new subscriber joins `/tf_static`.
- `CameraOverlay` receives frames as data URIs from the existing bridge image
  conversion path.
- New image subscriptions must pass both `{ topic, type }` so Foxglove channels
  wire correctly.

## Vacuum Control

`Vacuum Control` is the validated operator panel for the current
TurtleBot4/Nav2 simulation path.

Files:

- `panels-standalone/src/vacuum_control.tsx`
- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
- `panels-standalone/src/components/VacuumControl/MapCanvas.tsx`
- `panels-standalone/src/components/VacuumControl/TeleopCard.tsx`
- `panels-standalone/src/components/VacuumControl/CameraOverlay.tsx`
- `panels-standalone/src/components/VacuumControl/mapOverlayUtils.ts`
- `panels-standalone/src/components/VacuumControl/cleanAreaProfile.ts`
- `panels-standalone/src/components/VacuumControl/cleanAreaPlanner.ts`
- `panels-standalone/src/components/VacuumControl/cleanAreaCoverage.ts`
- `panels-standalone/src/components/Nav2/runtime/useNav2Runtime.ts`
- `panels-standalone/src/vacuum-adapter/useVacuumAdapter.ts`

Panel responsibilities:

- map-first goal selection
- operator-facing connection, readiness, state, progress, and actions
- `NavigateToPose` send/cancel through `vacuum_adapter`
- adapter-backed mapping controls
- saved-map inventory and loading
- Clean Area controls and visualization
- teleop and camera PiP inside the operator workflow

Current expectations:

- `Header` and `StatusStrip` remain inline in `VacuumControlPanel.tsx`.
- `VacuumControlPanel.tsx` owns the `Mapping`, `Navigate`, and `Clean Area`
  mode switcher.
- Mode switching prevents mapping, point navigation, and clean-area runs from
  conflicting.
- `VacuumControlPanel.tsx` consumes `useVacuumAdapter`, not raw
  `useNav2Runtime`, for product-facing behavior.
- Navigation state, readiness evidence, capability gating, plan path rendering,
  send goal, and cancel all flow through the adapter snapshot/commands.
- The panel branches on capability descriptors and normalized state, not backend
  names.
- Future changes should maintain live-runtime behavior rather than adding
  mock-only UI.

## MapCanvas

`MapCanvas` is the dedicated internal map rendering and interaction surface for
`Vacuum Control`.

Responsibilities:

- render adapter-normalized `snapshot.map.grid` as the product base map
- keep direct `/map` rendering available as diagnostic/fallback visualization
- render `/global_costmap/costmap` and `/local_costmap/costmap`
- render route overlays, staged preview lines, robot marker, destination marker,
  clean-area selection, clean-area path preview, and coverage overlays
- support pan, zoom, Fit Map, Follow Robot, map fit, manual view, and panel
  resize
- draw, move, and resize rectangular clean-area selections
- validate clean-area selections against live map bounds and occupancy data
- host `CameraOverlay` as an absolutely positioned floating window inside the
  map stage

Current behavior:

- First valid `/map` fits the full known occupancy-grid bounds.
- Fit Map uses map dimensions, resolution, panel size, and padding.
- Fit Map does not fit to robot pose, selected target, route geometry, costmaps,
  or known/free cells only.
- `fit`, `manual`, and `follow_robot` viewport modes are explicit.
- Manual pan/zoom disables follow mode.
- Base map, costmaps, plan path, lidar/depth overlays, robot marker, target
  marker, clean-area rectangle, route preview, and coverage cells share one
  world-to-screen transform.
- Unknown cells render as muted map space and the map boundary is outlined so a
  partial map is not confused with the panel background.
- The floating `Layers` control toggles Map, Global costmap, Local costmap,
  Plan, Lidar, and Depth obstacles.
- Pointer events inside camera overlay controls do not trigger map target
  placement.

## Overlay Utilities

`mapOverlayUtils.ts` owns extension-local sensor overlay projection for lidar
and depth obstacle points.

Current behavior:

- Builds a local `TransformTree` from `/tf` and `/tf_static`.
- Projects lidar and depth obstacle points into `map` frame.
- Uses robot pose frame fallback candidates:
  - `base_footprint`
  - `base_link`
  - namespaced variants
- Uses lidar frame fallback candidates:
  - `rplidar_link`
  - `base_scan`
  - `laser`
  - namespaced variants
- Laser scan topic discovery matches any advertised `sensor_msgs/msg/LaserScan`
  or `foxglove.LaserScan`, preferring `/scan`.
- Point cloud field decoding supports plain JS arrays and typed arrays such as
  `Float32Array`.

Overlay note:

- Lidar and depth obstacle overlays are extension-side visualization aids.
- They are projected into the map frame and rendered below robot/target markers.
- They are not new product-contract surfaces.

## TeleopCard

File:

- `panels-standalone/src/components/VacuumControl/TeleopCard.tsx`

Current behavior:

- Collapsible manual control card at the bottom of the sidebar.
- Directional D-pad.
- Publishes `geometry_msgs/msg/Twist` to `/cmd_vel_raw`.
- Publishes at 10 Hz while moving.
- Optional WASD / arrow-key mode is opt-in via a toggle so it does not conflict
  with map interactions.
- Shows live velocity readout while moving.
- Available during manual mapping, paused auto mapping, and `needs_assistance`.
- Disabled during active auto mapping and active clean-area waypoint runs.

Important boundary:

- Teleop is not routed through `adapter.sendCommand`.
- The adapter explicitly rejects `manual_control` as an invalid one-shot command
  because teleop is a streaming control channel.
- The VM `twist_deadman.py` accepts both `Twist` and `TwistStamped` on
  `/cmd_vel_raw`.

## CameraOverlay

File:

- `panels-standalone/src/components/VacuumControl/CameraOverlay.tsx`

Current behavior:

- Floating PiP camera window inside `MapCanvas`.
- Auto-discovers image topics through `ros2Bridge.getAvailableImageTopics()`.
- Prefers `/oakd/rgb/preview/image_raw`.
- Falls back to other advertised image topics.
- Supports `sensor_msgs/msg/Image` and `sensor_msgs/msg/CompressedImage`.
- Frames arrive as data URIs and render in an `<img>`.
- Draggable, minimizable, and hideable.
- Stops pointer events so camera interactions do not place map targets.

## Mapping UI

`MappingCard` is implemented in `VacuumControlPanel.tsx`.

Current behavior:

- Start auto mapping.
- Start manual mapping.
- Pause.
- Resume auto mapping.
- Finish & review.
- Discard session.
- Accept map.
- Saved-map inventory.
- Saved-map load.
- Improve current map.

Mapping status displays:

- known ratio
- unknown ratio
- frontier count
- visited goal count
- failed goal count
- active goal
- state reason
- last error
- update time
- persistence result
- active map name
- loaded map path
- saved map path
- saved maps
- load/save errors

Extension boundary:

- Autonomous exploration is VM-owned.
- React/webview code sends adapter commands and renders mapping state.
- The UI should not own the long-running exploration loop.

## Clean Area UI

Clean Area is the current Layer 4 runtime-owned mission path hosted inside
`Vacuum Control`.

Files:

- `VacuumControlPanel.tsx`
- `MapCanvas.tsx`
- `cleanAreaProfile.ts`
- `cleanAreaPlanner.ts`
- `cleanAreaCoverage.ts`

Current behavior:

- Rectangular selection through draw, move, and resize.
- Validation against map bounds and occupancy data.
- Coverage profile for swath width, overlap, navigation goal tolerance,
  boundary margin, minimum useful region size, completion threshold, lane
  spacing, and boundary extension.
- Lanes are generated as swath-overlap lawnmower passes.
- Sampled lanes are clipped to known free occupancy-grid cells.
- Boundary pass endpoints are extended so Nav2's close-enough goal completion
  does not leave clean-area edges uncovered.
- Execution submits one area-only runtime-owned mission through
  `adapter.sendCommand({ command: "start_coverage", ... })`.
- Coverage target is built from adapter-normalized map cells.
- Occupied, unknown, out-of-bounds, and too-small cells are excluded/skipped.
- Active route, coverage cells, and coverage progress come from the runtime
  mission snapshot.
- Confirmed runs freeze the coverage target so map updates do not erase
  covered cells.
- Map overlay renders remaining, covered, excluded, skipped, and footprint
  states.
- Sidebar reports percentage, cleaned area, remaining area, skipped area,
  simple route status, pass count, distance, and waypoint progress.
- States: editing, confirmed, preparing, running, paused, canceling, completed,
  failed, canceled.
- Controls: preview path, start, pause, resume, cancel, retry waypoint, skip
  waypoint, clear area.

Current MVP limits:

- Runtime route generation is row-level occupancy-clipped.
- Runtime coverage progress is first-pass footprint-history accounting.
- Route generation is not yet component-level area planning.
- Strong edge/corner, obstacle-adjacent, dock/undock, and battery-aware
  behavior remain future Layer 4 work.

## Vacuum Adapter Extension Boundary

Extension/product clients should talk in vacuum concepts and capability
descriptors, not raw TurtleBot4 topics, Nav2 internals, or Valetudo class
names.

Current adapter files:

- `panels-standalone/src/vacuum-adapter/adapter.ts`
- `panels-standalone/src/vacuum-adapter/capabilities.ts`
- `panels-standalone/src/vacuum-adapter/commands.ts`
- `panels-standalone/src/vacuum-adapter/errors.ts`
- `panels-standalone/src/vacuum-adapter/index.ts`
- `panels-standalone/src/vacuum-adapter/mapGrid.ts`
- `panels-standalone/src/vacuum-adapter/messageUtils.ts`
- `panels-standalone/src/vacuum-adapter/state.ts`
- `panels-standalone/src/vacuum-adapter/useVacuumAdapter.ts`
- `panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/`
- `panels-standalone/src/vacuum-adapter/backends/valetudo/`

Current target shape:

```text
VS Code extension / product UI
  -> vacuum_adapter contract
     -> TurtleBot4/Nav2 backend adapter
        -> VM TurtleBot4 simulation runtime

     -> Valetudo backend adapter
        -> VM-managed Valetudo integration runtime
           -> real Valetudo-compatible vacuum on local network
```

Layer 3 extension facts:

- Public `VacuumAdapter` exposes `snapshot` and `sendCommand`.
- `snapshot.activeMission` and `snapshot.missions` are the normalized
  runtime-owned mission surfaces for new work.
- `snapshot.mission` remains as the legacy coarse state used by the current UI.
- `useTurtleBot4Nav2Adapter` wraps `useNav2Runtime`.
- `VacuumControlPanel.tsx` consumes `useVacuumAdapter`.
- `start_navigation` and `cancel_mission` use `adapter.sendCommand`;
  `cancel_navigation` remains a fallback for older navigation paths.
- Navigation destination/progress/action state hydrates from the runtime-owned
  active mission snapshot. For TurtleBot4/Nav2 this currently uses
  `/vacuum_mission/status` plus `/vacuum_mission/get_snapshot`.
- Clear destination after a terminal navigation run is UI presentation state:
  it dismisses the completed/canceled/failed destination locally without
  clearing runtime mission history.
- Plan rendering consumes normalized `snapshot.navigation.planPath`.
- Mission state exposes `idle / navigating / cleaning / paused / returning /
  charging`.
- TurtleBot4/Nav2 reports unsupported vacuum-only operations explicitly.
- Valetudo backend stubs exist but do not implement hardware connectivity yet.

Capability model and Valetudo planning details are documented in
`VACUUM_STACK_PLAN.md`.

## Supporting Panel Mapping

### 3D Sensor View

File:

- `panels-standalone/src/components/SensorView3D/SensorView3DPanel.tsx`

Useful topics:

- `/scan`
- `/odom`
- `/tf`
- `/tf_static`
- `/map`
- `/local_costmap/costmap`
- `/global_costmap/costmap`

Expectation:

- Keep TF, odom, lidar, map, and costmap visibility aligned with the global
  topic map used by the Nav2 runtime seam.
- Use map coloring for `/map`.
- Use costmap coloring for local/global costmaps.
- Keep renderer config discovery-driven so the panel still works with other
  robots.
- Treat this as the best supporting debug surface for lidar, TF, odom, map, and
  costmaps without mixing raw debug UI into the main operator panel.

### Raw Messages Panel

File:

- `panels-standalone/src/components/RawMessages/RawMessagesPanel.tsx`

Useful topics:

- `/navigate_to_pose/_action/status`
- `/navigate_to_pose/_action/feedback`
- `/odom`
- `/scan`
- `/map`
- `/local_costmap/costmap`
- `/global_costmap/costmap`

Expectation:

- Add a pinned topic set aligned with the current global Nav2/SLAM runtime.
- Prefer advertised action status/feedback topics over hard-coded assumptions
  where possible.
- Use Raw Messages as the first detailed Nav2 visibility surface before adding
  richer diagnostics.
- The panel should work now for any advertised topic because it is generic.

### Map / Mission Control Panel

Files:

- `panels-standalone/src/components/MissionControl/MissionControl.tsx`
- `panels-standalone/src/components/MissionControl/map/DroneMap.tsx`

Current state:

- drone/GPS oriented
- uses `DroneStateModel`
- `DroneMap` is an OpenLayers world map with GPS-style vehicle state
- not a SLAM occupancy-grid map panel

Do not use this panel as the primary SLAM map validation surface without
rewriting it.

Follow-up options:

1. Rename/scope the current panel as drone-specific.
2. Add a new robot map panel that renders `nav_msgs/msg/OccupancyGrid` from
   `/map`, `/local_costmap/costmap`, and `/global_costmap/costmap`.

For vacuum debugging and regression checks, prefer `Vacuum Control`, 3D Sensor
View, and Raw Messages.

## Extension Follow-Up Work

These are extension maintenance and debugging follow-ups after the closed
Layer 2/Layer 3 operator slice. They are not the main Layer 4 coverage path.

1. Vacuum Control maintenance hardening

   Keep the validated single-panel operator flow stable:

   - map rendering
   - layer visibility controls
   - costmap overlays
   - projected lidar and depth obstacle overlays
   - target placement
   - route overlay
   - progress state
   - action state
   - mapping state
   - Clean Area state

2. 3D debug layout

   Add a one-click layout or panel state showing:

   - lidar scan
   - odom/follow frame
   - map occupancy grid
   - local/global costmaps

3. Nav2 action monitor

   Add a small read-only surface for advertised `NavigateToPose` status and
   feedback topics. It should show whether a goal is active, succeeded,
   canceled, or failed, plus latest feedback fields that are present.

4. Goal details / diagnostics surface

   Add a read-only surface for active goal status, latest feedback values, and
   terminal result summary.

5. Occupancy-grid map panel

   Add a robot/SLAM map panel rendering:

   - `/map`
   - `/local_costmap/costmap`
   - `/global_costmap/costmap`

   This is lower priority because `Vacuum Control` already renders base map and
   costmaps.

6. Robot status panel

   Add a compact status panel using discovered runtime status topics:

   - battery percentage/voltage/current
   - IMU presence and latest timestamp
   - hazard state if advertised
   - dock state if advertised
   - odom freshness
   - TF freshness
   - Nav2 action status

7. Topic health checklist

   Add a panel section that marks each expected Layer 2 topic as advertised,
   receiving messages, stale, or missing.

8. First-mile operator dashboard

   Add an `Open Navigation Dashboard` command that opens:

   - Vacuum Control
   - 3D Sensor View
   - Raw Messages Panel

   Leave the current drone mission-control panel out unless it has been
   rewritten for SLAM maps.

## Verification

Suggested verification after extension changes:

```sh
bun run test:vacuum-adapter
bun run typecheck
bun run --cwd panels-standalone build
```

Live extension validation should cover:

- Foxglove connection reaches `ws://172.16.0.10:8765`.
- `Vacuum Control` connects and shows live connection state.
- `/map` is received and rendered.
- typed-array occupancy payloads render correctly.
- layers toggle without placing a target.
- global/local costmaps align with the active viewport.
- lidar/depth overlays project into map frame or show waiting/no-TF state.
- live pose is received.
- target placement works from the map.
- goal send reaches `/navigate_to_pose/_action/send_goal`.
- feedback/status progress appears when available.
- cancel transitions correctly.
- stale canceled marker/plan state clears on fresh target selection.
- second goal after cancel works without reload.
- mapping controls reflect VM-owned mapping state.
- Clean Area preview and execution state render correctly.
- supporting debug panels can inspect `/scan`, `/odom`, `/tf`, `/tf_static`,
  `/map`, and costmaps.

## Not Extension-Immediate

Do not make these the next extension milestone:

- room cleaning UI
- zone cleaning UI
- docking workflow UI
- simulated battery/charging behavior
- consumables UI
- scheduling UI
- OpenClaw workflow integration

Those belong after Layer 4/Layer 5 coverage and room/zone semantics, or in the
Layer 6 real-hardware path where noted.
