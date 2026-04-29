# TurtleBot4 VS Code Extension Integration Notes

This file is the extension-side companion to:

- `steps.md`
- `VACUUM_STACK_PLAN.md`

It captures what the VS Code extension needs to know to use the current
TurtleBot4/Nav2 VM stack as an operator and debugging surface.

The important constraint is that this phase should stay extension-side. The VM
already exposes enough TurtleBot4, SLAM, and Nav2 surfaces for panel
validation. Do not block this work on a vacuum adapter, mission lifecycle,
docking behavior, or a normalized backend contract.

## Current Integration Goal

Use the existing VS Code extension panels in `~/vscode-tensorfleet` against the
current TurtleBot4/Nav2 VM backend.

The extension should:

1. Connect to the running VM bridge endpoints.
2. Treat the single-panel `Vacuum Control` operator workflow as the closed
   Layer 2 TurtleBot4/Nav2 slice.
3. Show live map, lidar, odom/TF, costmap, and navigation status data where the
   current panels support those message types.
4. Expose enough Nav2 action visibility to drive and validate goal execution.
5. Record panel gaps as extension follow-up tasks rather than adding a new
   robotics-stack layer.

## Layer 2 Freeze

As of April 29, 2026, the Layer 2 TurtleBot4/Nav2 operator slice is closed.

Frozen truth:

- `Vacuum Control` works against the live VM runtime through the current
  Foxglove bridge path.
- The operator panel validates the normal one-panel flow: connect, render map,
  select a map target, send a Nav2 goal, observe progress, cancel when needed,
  and observe terminal state.
- The panel uses the existing TurtleBot4/Nav2 VM surfaces directly. It does not
  require a vacuum adapter, mission lifecycle, docking workflow, or normalized
  backend contract.
- Remaining dock-blocked start behavior is a runtime caveat, not a Layer 2
  blocker.
- Clear-space validation is still required before interpreting a failed or
  stalled run as a software failure.

Explicitly out of Layer 2:

- `vacuum_adapter` messages, services, or backend contract
- vacuum mission lifecycle UI
- docking behavior or return-to-dock workflow
- room, zone, coverage, battery, and charging semantics

## Extension Repository

The extension repo is:

- `~/vscode-tensorfleet`

Important files for the closed Layer 2 slice:

- `src/extension.ts`
- `src/regions.ts`
- `src/templates/drone-view-list.html`
- `panels-standalone/src/ros2-bridge.ts`
- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
- `panels-standalone/src/components/VacuumControl/MapCanvas.tsx`
- `panels-standalone/src/components/VacuumControl/TeleopCard.tsx`
- `panels-standalone/src/components/VacuumControl/CameraOverlay.tsx`
- `panels-standalone/src/components/VacuumControl/mapOverlayUtils.ts`
- `panels-standalone/src/components/Nav2/runtime/useNav2Runtime.ts`
- `panels-standalone/src/components/Nav2/runtime/nav2RuntimeConstants.ts`
- `panels-standalone/src/components/SensorView3D/SensorView3DPanel.tsx`
- `panels-standalone/src/components/RawMessages/RawMessagesPanel.tsx`
- `panels-standalone/src/components/MissionControl/MissionControl.tsx`
- `panels-standalone/src/components/MissionControl/map/DroneMap.tsx`

Current notable extension state:

- `src/regions.ts` already defines Foxglove and ROS bridge ports:
  - Foxglove: `8765`
  - rosbridge: `9091`
- `src/extension.ts` injects `window.TENSORFLEET_VM_CONFIG_ID` into standalone
  panels when a VM config is known.
- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
  already exists as the closed Layer 2 operator shell.
- `panels-standalone/src/components/VacuumControl/MapCanvas.tsx`
  now holds the dedicated map rendering and interaction surface for that shell.
- `MapCanvas.tsx` now accepts both plain-array and typed-array occupancy-grid
  payloads from Foxglove so live `/map` data renders instead of falling back to
  placeholder content.
- `MapCanvas.tsx` now has a floating `Layers` control with checklist toggles
  for Map, Global costmap, Local costmap, Plan, Lidar, and Depth obstacles.
- `mapOverlayUtils.ts` owns extension-local sensor overlay projection for lidar
  and depth obstacle points using `/tf`, `/tf_static`, and a local
  `TransformTree`; it now uses frame ID fallback candidate lists for robot pose
  and lidar frames to handle naming variation across runtime configurations; laser
  scan topic discovery now matches any `sensor_msgs/msg/LaserScan` or
  `foxglove.LaserScan` topic, preferring `/scan`; point cloud field decoding
  supports both plain JS arrays and typed arrays from Foxglove payloads.
- `panels-standalone/src/components/Nav2/runtime/useNav2Runtime.ts` is the
  shared runtime seam used by both the debug-facing Nav2 panel and the operator
  panel.
- `panels-standalone/src/ros2-bridge.ts` now caches the latest message per
  topic and replays it to each new subscriber immediately on subscribe; static
  TF transforms are accumulated per unique edge and replayed as a synthetic
  bundle to new `/tf_static` subscribers so panels never miss static TF data
  on connect or reconnect.
- `SensorView3DPanel.tsx` is already useful as a supporting debug surface
  because it can render LaserScan, TF, odometry, occupancy grid, and costmap
  style data through the Lichtblick renderer.
- `MissionControl.tsx` and `DroneMap.tsx` are currently drone/GPS oriented and
  should not be treated as the primary SLAM map panel.

## VM Bridge Endpoints

For this closed Layer 2 slice, the extension uses the already running VM bridge
endpoints:

- Foxglove/Lichtblick panels: `ws://172.16.0.10:8765`
- rosbridge-specific panels: `ws://172.16.0.10:9091`

The current standalone panel path primarily uses Foxglove:

- `panels-standalone/src/ros2-bridge.ts`
- `panels-standalone/src/foxglove-networking.ts`

`ros2-bridge.ts` currently defines `ConnectionMode = "foxglove"`, so new panel
work should assume Foxglove first unless a panel explicitly needs rosbridge.

## Vacuum Control Current Truth

The closed Layer 2 slice is the dedicated `Vacuum Control` panel rather than a
broader TurtleBot4 preset effort across every existing panel.

Current runtime seam:

- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
- `panels-standalone/src/components/VacuumControl/MapCanvas.tsx`
- `panels-standalone/src/components/VacuumControl/TeleopCard.tsx`
- `panels-standalone/src/components/VacuumControl/CameraOverlay.tsx`
- `panels-standalone/src/components/Nav2/runtime/useNav2Runtime.ts`

Current runtime topic and service map for this slice:

- `/map`
- `/scan`
- depth point cloud topic discovered from advertised `sensor_msgs/msg/PointCloud2`
  topics, preferring `/oakd/rgb/preview/depth/points`
- camera image topic discovered from advertised `sensor_msgs/msg/Image` and
  `sensor_msgs/msg/CompressedImage` topics, preferring
  `/oakd/rgb/preview/image_raw` (`CameraOverlay`)
- `/odom`
- `/pose`
- `/tf`
- `/tf_static`
- `/plan`
- `/transformed_global_plan`
- `/cmd_vel_nav`
- `/cmd_vel_raw` (publish — `TeleopCard` manual control at 10 Hz)
- `/local_costmap/costmap`
- `/global_costmap/costmap`
- `/stop_status`
- `/navigate_to_pose/_action/send_goal`
- `/navigate_to_pose/_action/get_result`
- `/navigate_to_pose/_action/cancel_goal`
- `/navigate_to_pose/_action/status`
- `/navigate_to_pose/_action/feedback`

Important correction:

- older `/turtlebot4/*` topic guidance in earlier versions of this file reflects
  an outdated assumption for this slice
- the current operator panel and shared Nav2 runtime are built around the
  global topics and action paths listed above
- do not re-scope this closed operator slice around namespaced topic defaults
  unless the runtime changes again

## Supporting Panel Mapping

### Vacuum Control

Files:

- `~/vscode-tensorfleet/panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
- `~/vscode-tensorfleet/panels-standalone/src/components/VacuumControl/MapCanvas.tsx`
- `~/vscode-tensorfleet/panels-standalone/src/components/VacuumControl/TeleopCard.tsx`
- `~/vscode-tensorfleet/panels-standalone/src/components/VacuumControl/CameraOverlay.tsx`
- `~/vscode-tensorfleet/panels-standalone/src/components/VacuumControl/mapOverlayUtils.ts`
- `~/vscode-tensorfleet/panels-standalone/src/components/Nav2/runtime/useNav2Runtime.ts`

Purpose:

- the validated Layer 2 operator surface
- map-first goal selection
- operator-facing state, progress, and actions
- `NavigateToPose` send/cancel workflow using the shared runtime seam

Current expectation:

- this panel is the closed Layer 2 operator workflow
- `Header` and `StatusStrip` remain inline in `VacuumControlPanel.tsx`
- `MapCanvas` is the dedicated internal map seam for rendering and placement
- `MapCanvas` now renders the live occupancy grid correctly for Foxglove
  payloads that expose occupancy data as typed arrays
- `MapCanvas` now renders optional global and local costmap overlays on
  dedicated raster canvases
- `MapCanvas` now exposes a floating `Layers` button and checklist popover for
  Map, Global costmap, Local costmap, Plan, Lidar, and Depth obstacles
- `MapCanvas` hosts `CameraOverlay` as an absolutely-positioned floating window
  inside the map stage; pointer events on the overlay are stopped so the map's
  goal-placement handler is never triggered by camera window interaction
- Plan visibility is operator-toggleable from the layers popover
- lidar and depth obstacles are projected into `map` frame with the local
  `TransformTree` in `mapOverlayUtils.ts`
- sensor overlays render on dedicated canvases below robot and target markers
  so projected points do not hide primary navigation markers
- `TeleopCard.tsx` is the collapsible manual control card at the bottom of the
  sidebar; it publishes `geometry_msgs/msg/Twist` to `/cmd_vel_raw` at 10 Hz;
  keyboard WASD / arrow control is opt-in via a toggle so it does not conflict
  with map interactions; the card shows a live velocity readout while moving
- `CameraOverlay.tsx` auto-discovers image topics via `ros2Bridge.getAvailableImageTopics()`,
  requiring `{ topic, type }` in the `subscribe()` call to correctly wire the
  Foxglove channel; it prefers `/oakd/rgb/preview/image_raw` for TurtleBot4 and
  falls back to other discovered image topics; frames arrive as data URIs from
  the bridge's existing image conversion pipeline and are rendered in an `<img>`
- readiness model: start is gated on `mapReady` and `preflightReady` only;
  `poseReady` no longer blocks the start action
- `VacuumControlPanel.tsx` derives the current pose display via `useMemo` from
  `runtime.currentMapPose` through `getPoseCoordinates` rather than using
  `runtime.currentMapCoordinates` directly
- future changes should maintain this live-runtime behavior rather than adding
  mock-only UI

### 3D Sensor View

File:

- `~/vscode-tensorfleet/panels-standalone/src/components/SensorView3D/SensorView3DPanel.tsx`

Useful closed Layer 2 topics:

- `/scan`
- `/odom`
- `/tf`
- `/tf_static`
- `/map`
- `/local_costmap/costmap`
- `/global_costmap/costmap`

Needed extension-side work:

1. Keep TF, odom, lidar, map, and costmap visibility aligned with the global
   topic map used by the Nav2 runtime seam.
2. For occupancy grids, use map coloring for `/map` and costmap coloring for
   local/global costmaps.
3. Keep renderer config discovery-based so the panel still works with other
   robots.

This is the best supporting surface for validating lidar, TF, odom, map, and
costmaps without mixing raw debug UI into the main operator panel.

### Raw Messages Panel

File:

- `~/vscode-tensorfleet/panels-standalone/src/components/RawMessages/RawMessagesPanel.tsx`

Useful closed Layer 2 topics:

- `/navigate_to_pose/_action/status` if advertised
- `/navigate_to_pose/_action/feedback` if advertised
- `/odom`
- `/scan`
- `/map`
- `/local_costmap/costmap`
- `/global_costmap/costmap`

Needed extension-side work:

1. Add a pinned topic set aligned with the current global Nav2/SLAM runtime.
2. Prefer advertised action status/feedback topics over hard-coded assumptions
   where possible.
3. Use Raw Messages as the first detailed Nav2 visibility surface before adding
   richer diagnostics.

This should work now for any advertised topic because the raw panel is already
generic.

### Map / Mission Control Panel

Files:

- `~/vscode-tensorfleet/panels-standalone/src/components/MissionControl/MissionControl.tsx`
- `~/vscode-tensorfleet/panels-standalone/src/components/MissionControl/map/DroneMap.tsx`

Current state:

- this panel is drone/GPS oriented
- it uses a `DroneStateModel`
- `DroneMap` is an OpenLayers world map with GPS-style vehicle state
- it is not currently a SLAM occupancy-grid map panel

Do not use this panel as the primary SLAM map validation surface without
rewriting it.

Useful extension-side follow-up:

1. Either rename/scope the current panel as drone-specific, or
2. add a new robot map panel that renders `nav_msgs/msg/OccupancyGrid` from:
   - `/map`
   - `/local_costmap/costmap`
   - `/global_costmap/costmap`

For follow-up debugging and regression checks, prefer `Vacuum Control`, the 3D
sensor panel, and raw messages for map and costmap validation.

## Features We Can Add On The VS Code Extension Side After Layer 2

These features should work with the current VM stack and do not require a new
vacuum adapter. They are follow-up work after the closed Layer 2 operator
slice, not Layer 2 exit criteria.

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

   This is now maintenance and regression hardening for the closed Layer 2
   slice.

2. 3D debug layout

   Add a one-click layout or panel state that makes the 3D sensor view show:

   - lidar scan
   - odom/follow frame
   - map occupancy grid
   - local/global costmaps

   This can be implemented entirely as renderer defaults and topic visibility
   rules.

3. Nav2 action monitor

   Add a small status surface for the `NavigateToPose` action using advertised
   action topics:

   - status
   - feedback

   The first version can be read-only. It only needs to show whether a goal is
   active, succeeded, canceled, or failed, plus the latest feedback fields that
   are present.

4. Goal details / diagnostics surface

   Add a small read-only surface that exposes:

   - active goal status
   - latest feedback values
   - terminal result summary

   This complements `Vacuum Control` without forcing raw debug details into the
   operator cards.

5. Occupancy-grid map panel

   Add a new robot map panel that renders `nav_msgs/msg/OccupancyGrid` directly
   in canvas:

   - `/map` as the SLAM map
   - `/local_costmap/costmap` as local planner costmap
   - `/global_costmap/costmap` as global planner costmap

   This should be a robot/SLAM map panel, not an OpenLayers GPS map.

   This is now lower priority than before for the closed Layer 2 slice because
   `Vacuum Control` already renders the base occupancy map and local/global
   costmaps as operator overlays.

6. Robot status panel

   Add a compact robot status panel using discovered runtime status topics:

   - battery percentage/voltage/current
   - IMU presence and latest timestamp
   - hazard state if advertised
   - dock state if advertised
   - odom freshness
   - TF freshness
   - Nav2 action status

   This can be read-only and discovery-driven.

7. Topic health checklist

   Add a panel section that marks each expected Layer 2 topic as:

   - advertised
   - receiving messages
   - stale
   - missing

   This would make VM-side validation much faster because it turns the manual
   checklist from `steps.md` into an extension UI.

8. First-mile operator dashboard

   Add an `Open Navigation Dashboard` command that opens:

   - Vacuum Control
   - 3D Sensor View
   - Raw Messages Panel

   The command should leave the current drone mission-control panel out unless
   it has been rewritten for SLAM maps.

## Recommended Immediate Extension Patch

The current documentation freeze records that the immediate Layer 2 operator
patch is complete. Future patches in `~/vscode-tensorfleet` should stay small
and concrete:

1. [x] Keep `steps.md` aligned with the actual `Vacuum Control` component state.
2. [x] Keep `extension.md` aligned with the global Layer 2 topic map used by
   `useNav2Runtime.ts`.
3. [x] Validate `VacuumControlPanel.tsx` against the live VM as the
   runtime-testable operator surface.
4. [x] Add `TeleopCard` for manual robot control via `/cmd_vel_raw`.
5. [x] Add `CameraOverlay` for live camera feed from the OAK-D sensor.
6. [ ] Keep `RawMessagesPanel.tsx` and `SensorView3DPanel.tsx` useful as
   supporting debug surfaces for the same runtime topics.
7. [x] Build the extension panels and verify the panel bundle compiles.

Suggested verification after patching:

```sh
cd ~/vscode-tensorfleet
bun run typecheck
bun run --cwd panels-standalone build
```

The closed Layer 2 runtime validation covers:

- Foxglove connection reaches `ws://172.16.0.10:8765`.
- `Vacuum Control` connects and shows the live connection state.
- `Vacuum Control` receives `/map` and renders the occupancy map when present.
- live `/map` rendering still works when Foxglove delivers occupancy data as a
  typed array rather than a plain JS array.
- the layers popover toggles Map, Global costmap, Local costmap, Plan, Lidar,
  and Depth obstacles without placing a target.
- global and local costmaps align with the active map viewport when enabled.
- lidar and depth obstacle overlays either project into `map` frame or clearly
  show waiting / no-TF state.
- `Vacuum Control` receives live pose and can place a target from the map.
- `Vacuum Control` can send a goal through `/navigate_to_pose/_action/send_goal`.
- `Vacuum Control` shows live progress from advertised Nav2 feedback when
  available.
- `Vacuum Control` can cancel an active run and show canceled / terminal state.
- supporting debug panels can inspect `/scan`, `/odom`, `/tf`, `/tf_static`,
  `/map`, and costmaps as needed.

## Not In Scope For This Extension Step

Do not add these yet:

- `vacuum_adapter_msgs`
- `vacuum_adapter_core`
- vacuum mission lifecycle UI
- room cleaning UI
- zone cleaning UI
- docking workflow UI
- simulated battery/charging behavior
- normalized vacuum backend API
- OpenClaw workflow integration

Those belong after the extension can already operate against the TurtleBot4/Nav2
navigation slice.
