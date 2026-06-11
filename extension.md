# VS Code Extension Vacuum / Nav2 Implementation Reference

This file keeps extension-specific implementation details for the TensorFleet
VS Code extension in `~/vscode-tensorfleet`. It is limited to facts that are
useful when changing the extension code.

## Repository Entrypoints

Extension host:

- `src/extension.ts`
- `src/regions.ts`
- `src/templates/drone-view-list.html`

Standalone panel entrypoints:

- `panels-standalone/src/vacuum_control.tsx`
- `panels-standalone/src/nav2.tsx`
- `panels-standalone/src/raw_messages.tsx`
- `panels-standalone/src/SensorView3D.tsx`
- `panels-standalone/src/image.tsx`
- `panels-standalone/src/teleops.tsx`

Shared ROS bridge package:

- `panels-standalone/packages/tensorfleet-ros/src/ros2-bridge.ts`
- `panels-standalone/packages/tensorfleet-ros/src/foxglove-networking.ts`
- `panels-standalone/packages/tensorfleet-ros/src/index.ts`

Vacuum operator UI:

- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
- `panels-standalone/src/components/VacuumControl/MapCanvas.tsx`
- `panels-standalone/src/components/VacuumControl/TeleopCard.tsx`
- `panels-standalone/src/components/VacuumControl/CameraOverlay.tsx`
- `panels-standalone/src/components/VacuumControl/mapOverlayUtils.ts`
- `panels-standalone/src/components/VacuumControl/cleanAreaProfile.ts`
- `panels-standalone/src/components/VacuumControl/cleanAreaPlanner.ts`
- `panels-standalone/src/components/VacuumControl/cleanAreaCoverage.ts`
- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.css`

Nav2 runtime and adapter:

- `panels-standalone/src/components/Nav2/runtime/useNav2Runtime.ts`
- `panels-standalone/src/components/Nav2/runtime/nav2RuntimeConstants.ts`
- `panels-standalone/src/components/Nav2/runtime/nav2RuntimeTypes.ts`
- `panels-standalone/src/components/Nav2/runtime/nav2RuntimeUtils.ts`
- `panels-standalone/src/vacuum-adapter/`

Debug/support panels:

- `panels-standalone/src/components/Nav2/Nav2Panel.tsx`
- `panels-standalone/src/components/RawMessages/RawMessagesPanel.tsx`
- `panels-standalone/src/components/SensorView3D/SensorView3DPanel.tsx`
- `panels-standalone/src/components/MissionControl/MissionControl.tsx`
- `panels-standalone/src/components/MissionControl/map/DroneMap.tsx`

## Extension Registration

`package.json` contributes the command `tensorfleet.openVacuumControlPanel`.
`src/extension.ts` registers the corresponding `DRONE_VIEWS` item:

- id: `tensorfleet-vacuum-control-panel`
- title: `Vacuum Control`
- command: `tensorfleet.openVacuumControlPanel`
- `panelKind`: `standard`
- `htmlTemplate`: `vacuum-control-standalone`

`panels-standalone/src/vacuum_control.tsx` mounts `VacuumControlPanel` inside
`ConnectionSettingsProvider`.

`src/extension.ts` injects `window.TENSORFLEET_VM_CONFIG_ID` into standalone
panels when a VM config is known. Runtime/panel code reads this value directly
from `window` where needed.

## VM Bridge Endpoints

`src/regions.ts` defines the current ROS bridge ports:

- Foxglove: `8765`
- rosbridge: `9091`

Default VM endpoints for the current TurtleBot4/Nav2 simulation:

- Foxglove/Lichtblick panels: `ws://172.16.0.10:8765`
- rosbridge-specific panels: `ws://172.16.0.10:9091`

The shared `tensorfleet-ros` bridge uses Foxglove only:

- `ConnectionMode = "foxglove"`
- `ros2Bridge.connect()` ignores rosbridge modes and connects through Foxglove.
- New panel code should use `tensorfleet-ros` unless a panel explicitly owns a
  separate rosbridge path.

## Bridge Behavior

Bridge behavior available through `tensorfleet-ros`:

- Topic, service, and image-topic discovery.
- Topic subscription by string or `{ topic, type }`; image subscriptions should
  pass `{ topic, type }` so Foxglove channels wire correctly.
- Publish through `ros2Bridge.publish(topic, messageType, message)`.
- Service calls through `ros2Bridge.callService(...)`.
- Latest-message cache with immediate replay to new subscribers.
- `/tf_static` accumulation by unique transform edge and replay as a synthetic
  bundle to new subscribers.
- TF graph helpers exposed by `getTfGraphSnapshot()` and `getKnownTfFrames()`.
- Image conversion to data URIs for raw and compressed image messages.

`foxglove-networking.ts` tolerates missing request schemas for common ROS 2
service/action shapes and overrides schemas where needed for service calls.

## Runtime Topics And Services

The TurtleBot4/Nav2 operator path uses global topics and action paths.
`/turtlebot4/*` topic names are not used by the current operator slice.

Core topics:

- `/map`
- `/scan`
- `/odom`
- `/pose`
- `/tf`
- `/tf_static`
- `/plan`
- `/transformed_global_plan`
- `/cmd_vel_nav`
- `/cmd_vel_raw`
- `/cmd_vel`
- `/local_costmap/costmap`
- `/global_costmap/costmap`
- `/stop_status`

Mapping topics/services:

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

Mission topics/services:

- `/vacuum_mission/status`
- `/vacuum_mission/start_navigation`
- `/vacuum_mission/start_coverage`
- `/vacuum_mission/cancel`
- `/vacuum_mission/pause`
- `/vacuum_mission/resume`
- `/vacuum_mission/retry_step`
- `/vacuum_mission/skip_step`
- `/vacuum_mission/get_snapshot`
- `/vacuum_mission_runtime/set_parameters`

Nav2 action paths:

- `/navigate_to_pose/_action/send_goal`
- `/navigate_to_pose/_action/get_result`
- `/navigate_to_pose/_action/cancel_goal`
- `/navigate_to_pose/_action/status`
- `/navigate_to_pose/_action/feedback`

Image/depth topic discovery:

- Depth uses advertised `sensor_msgs/msg/PointCloud2`, preferring
  `/oakd/rgb/preview/depth/points`.
- Camera uses advertised `sensor_msgs/msg/Image` and
  `sensor_msgs/msg/CompressedImage`, preferring `/oakd/rgb/preview/image_raw`.

## Vacuum Adapter Boundary

Extension/product clients should depend on vacuum concepts and capability
descriptors rather than raw TurtleBot4 topics, Nav2 internals, or Valetudo class
names.

Adapter files:

- `panels-standalone/src/vacuum-adapter/adapter.ts`
- `panels-standalone/src/vacuum-adapter/capabilities.ts`
- `panels-standalone/src/vacuum-adapter/commands.ts`
- `panels-standalone/src/vacuum-adapter/errors.ts`
- `panels-standalone/src/vacuum-adapter/index.ts`
- `panels-standalone/src/vacuum-adapter/mapGrid.ts`
- `panels-standalone/src/vacuum-adapter/messageUtils.ts`
- `panels-standalone/src/vacuum-adapter/primaryState.ts`
- `panels-standalone/src/vacuum-adapter/state.ts`
- `panels-standalone/src/vacuum-adapter/useVacuumAdapter.ts`
- `panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/`
- `panels-standalone/src/vacuum-adapter/backends/valetudo/`

Runtime shape:

```text
VS Code extension / product UI
  -> vacuum_adapter contract
     -> TurtleBot4/Nav2 backend adapter
        -> VM TurtleBot4 simulation runtime
     -> Valetudo backend adapter
        -> VM-managed Valetudo integration runtime
```

Adapter facts:

- Public `VacuumAdapter` exposes `snapshot` and `sendCommand`.
- `VacuumControlPanel.tsx` consumes `useVacuumAdapter`, not raw
  `useNav2Runtime`, for product-facing behavior.
- `useTurtleBot4Nav2Adapter` wraps `useNav2Runtime`.
- Navigation destination, progress, action state, and runtime mission state
  hydrate from `/vacuum_mission/status` and `/vacuum_mission/get_snapshot`.
- `snapshot.activeMission` and `snapshot.missions` are the normalized
  runtime-owned mission surfaces.
- `snapshot.mission` remains as a legacy coarse state used by existing UI.
- `mission.result.details` may carry backend-neutral structured details for
  coverage-style runs.
- `snapshot.navigation.planPath` is the normalized source for plan rendering.
- TurtleBot4/Nav2 reports unsupported vacuum-only operations explicitly.
- Valetudo backend adapter code polls the VM-managed Valetudo runtime API, maps
  snapshots into `vacuum_adapter`, and routes normalized commands.

Normalized command names are defined in
`panels-standalone/src/vacuum-adapter/commands.ts`. Current command names:

- `start_navigation`
- `go_to_location`
- `cancel_navigation`
- `manual_control`
- `start_mapping`
- `pause_mapping`
- `resume_mapping`
- `finish_mapping`
- `discard_mapping`
- `accept_map`
- `load_map`
- `save_map_annotation`
- `delete_map_annotation`
- `start_coverage`
- `start_room_cleaning`
- `start_zone_cleaning`
- `pause_mission`
- `resume_mission`
- `cancel_mission`
- `retry_mission_step`
- `skip_mission_step`
- `start_cleaning`
- `pause`
- `resume`
- `stop`
- `return_to_dock`
- `segment_cleaning`
- `zone_cleaning`
- `set_fan_speed`
- `set_water_usage`

Capability names are defined in
`panels-standalone/src/vacuum-adapter/capabilities.ts`. UI controls should gate
behavior through these capability descriptors and runtime action availability.

## Vacuum Control Panel

`Vacuum Control` is the primary operator panel for TurtleBot4/Nav2 vacuum
operation inside the extension.

Panel responsibilities:

- Map-first goal selection.
- Operator-facing connection, readiness, state, progress, and actions.
- `NavigateToPose` send/cancel through `vacuum_adapter`.
- Adapter-backed mapping controls.
- Saved-map inventory and loading.
- Clean Area controls and visualization.
- Rooms / Zones controls and manual annotation visualization.
- Selected room/zone cleaning target preview.
- Selected room/zone cleaning execution, recovery controls, and recent
  summaries.
- Teleop and camera PiP inside the operator workflow.

Implementation boundaries:

- `Header` and `StatusStrip` are inline in `VacuumControlPanel.tsx`.
- `VacuumControlPanel.tsx` owns the `Mapping`, `Navigate`, `Clean Area`, and
  `Rooms / Zones` mode switcher.
- Mode switching prevents mapping, point navigation, and clean-area runs from
  conflicting.
- Active navigation missions may auto-select Navigate mode.
- Terminal navigation snapshots are presentation context only and must not
  prevent switching back to Mapping or Clean Area.
- Clear destination after a terminal navigation run is local UI presentation
  state; it does not clear runtime mission history.
- Navigation state, readiness evidence, capability gating, plan path rendering,
  send goal, and cancel flow through adapter snapshot/commands.
- The panel branches on capability descriptors and normalized state, not backend
  implementation names.
- Live runtime behavior should remain the default behavior for panel changes.

## MapCanvas

`MapCanvas` is the internal map rendering and interaction surface for
`Vacuum Control`.

Responsibilities:

- Render adapter-normalized `snapshot.map.grid` as the product base map.
- Keep direct `/map` rendering available as diagnostic/fallback visualization.
- Render `/global_costmap/costmap` and `/local_costmap/costmap`.
- Render route overlays, staged preview lines, robot marker, destination marker,
  clean-area selection, clean-area path preview, room/zone annotation overlays,
  and coverage overlays.
- Support pan, zoom, Fit Map, Follow Robot, map fit, manual view, and panel
  resize.
- Draw, move, and resize rectangular clean-area and room/zone selections.
- Validate clean-area selections against live map bounds and occupancy data.
- Host `CameraOverlay` as an absolutely positioned floating window inside the
  map stage.

Viewport behavior:

- First valid `/map` fits the full known occupancy-grid bounds.
- Fit Map uses map dimensions, resolution, panel size, and padding.
- Fit Map does not fit to robot pose, selected target, route geometry, costmaps,
  or known/free cells only.
- `fit`, `manual`, and `follow_robot` viewport modes are explicit.
- Manual pan/zoom disables follow mode.
- Base map, costmaps, plan path, lidar/depth overlays, robot marker, target
  marker, clean-area rectangle, route preview, and coverage cells share one
  world-to-screen transform.
- Clean Area route preview and active route overlays render above coverage
  cells.
- Unknown cells render as muted map space and the map boundary is outlined.
- The floating `Layers` control toggles Map, Global costmap, Local costmap,
  Plan, Lidar, and Depth obstacles.
- Pointer events inside camera overlay controls do not trigger map target
  placement.

## Sensor Overlays

`mapOverlayUtils.ts` owns extension-local sensor overlay projection for lidar
and depth obstacle points.

Behavior:

- Builds a local `TransformTree` from `/tf` and `/tf_static`.
- Projects lidar and depth obstacle points into `map` frame.
- Uses robot pose frame fallback candidates: `base_footprint`, `base_link`, and
  namespaced variants.
- Uses lidar frame fallback candidates: `rplidar_link`, `base_scan`, `laser`,
  and namespaced variants.
- Laser scan topic discovery matches advertised `sensor_msgs/msg/LaserScan` or
  `foxglove.LaserScan`, preferring `/scan`.
- Depth point cloud discovery prefers `/oakd/rgb/preview/depth/points`.
- Point cloud field decoding supports plain JS arrays and typed arrays such as
  `Float32Array`.

Lidar and depth obstacle overlays are visualization aids only. They are
projected into the map frame and rendered below robot/target markers.

## TeleopCard

`TeleopCard` provides manual control inside `Vacuum Control`.

Behavior:

- Collapsible manual control card at the bottom of the sidebar.
- Directional D-pad.
- Publishes `geometry_msgs/msg/Twist` to `/cmd_vel_raw`.
- Publishes at 10 Hz while moving.
- Optional WASD / arrow-key mode is opt-in via a toggle.
- Shows live velocity readout while moving.
- Available during manual mapping, paused auto mapping, and `needs_assistance`.
- Disabled during active auto mapping and active clean-area waypoint runs.

Boundary:

- Teleop is a streaming control channel and is not routed through
  `adapter.sendCommand`.
- The adapter rejects `manual_control` as an invalid one-shot command.
- The VM `twist_deadman.py` accepts both `Twist` and `TwistStamped` on
  `/cmd_vel_raw`.

## CameraOverlay

`CameraOverlay` provides a floating PiP camera window inside `MapCanvas`.

Behavior:

- Auto-discovers image topics through `ros2Bridge.getAvailableImageTopics()`.
- Prefers `/oakd/rgb/preview/image_raw`.
- Falls back to other advertised image topics.
- Supports `sensor_msgs/msg/Image` and `sensor_msgs/msg/CompressedImage`.
- Frames arrive as data URIs and render in an `<img>`.
- Draggable, minimizable, and hideable.
- Stops pointer events so camera interactions do not place map targets.

## Mapping UI

Mapping controls are implemented inside `VacuumControlPanel.tsx`.

Supported controls:

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

Mapping status fields:

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

Boundary:

- Autonomous exploration is VM-owned.
- React/webview code sends adapter commands and renders mapping state.
- The UI does not own the long-running exploration loop.

## Clean Area UI

Clean Area is a runtime-owned mission path hosted inside `Vacuum Control`.

Files:

- `VacuumControlPanel.tsx`
- `MapCanvas.tsx`
- `cleanAreaProfile.ts`
- `cleanAreaPlanner.ts`
- `cleanAreaCoverage.ts`

Behavior:

- Rectangular selection through draw, move, and resize.
- Validation against map bounds and occupancy data.
- Coverage profile for swath width, overlap, navigation goal tolerance,
  boundary margin, minimum useful region size, completion threshold, lane
  spacing, and boundary extension.
- Lanes generated as swath-overlap lawnmower passes.
- Sampled lanes clipped to known free occupancy-grid cells.
- Boundary pass endpoints extended so Nav2's close-enough goal completion does
  not leave clean-area edges uncovered.
- Preview and active route overlays drawn above coverage cells.
- Execution submits one area-only runtime-owned mission through
  `adapter.sendCommand({ command: "start_coverage", ... })`.
- Coverage target built from adapter-normalized map cells.
- Occupied, unknown, out-of-bounds, and too-small cells excluded/skipped.
- Active route, coverage cells, and coverage progress hydrated from runtime
  mission snapshot.
- Confirmed runs freeze the coverage target so map updates do not erase covered
  cells.
- Map overlay renders remaining, covered, excluded, skipped, and footprint
  states.
- Sidebar reports percentage, cleaned area, remaining area, skipped area, simple
  route status, pass count, distance, and waypoint progress.
- States: editing, confirmed, preparing, running, paused, canceling, completed,
  failed, canceled.
- Controls: preview path, start, pause, resume, cancel, retry waypoint, skip
  waypoint, clear area.

Implementation limits:

- Runtime route generation is row-level occupancy-clipped.
- Runtime coverage progress is first-pass footprint-history accounting.
- Route generation is not component-level area planning.

## Rooms / Zones UI

Rooms / Zones is implemented inside `Vacuum Control`.

Files:

- `VacuumControlPanel.tsx`
- `MapCanvas.tsx`
- `panels-standalone/src/vacuum-adapter/state.ts`
- `panels-standalone/src/vacuum-adapter/commands.ts`
- `panels-standalone/src/vacuum-adapter/capabilities.ts`
- `panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/useTurtleBot4Nav2Adapter.ts`

Behavior:

- Operators can draw a rectangular room/zone draft using the shared editable
  rectangle behavior from Clean Area.
- Operators can set the draft kind to `Room` or `Zone`.
- Operators can name, save, select, and delete saved annotations.
- Saved annotations render as map overlays and side-panel list entries.
- Saved annotations hydrate from `snapshot.map.annotations`.
- UI owns only unsaved draft rectangle/name/kind state.
- Selecting a saved annotation previews it as a cleaning target.
- Preview reuses Clean Area coverage-target evaluation, per-cell cleanability
  overlay, and lawnmower route generation.
- Side panel reports whether the selected target is cleanable, partially
  cleanable, or invalid.
- Selecting a saved room/zone can start `start_room_cleaning` or
  `start_zone_cleaning` through the adapter.
- Active room/zone cleaning hydrates from `snapshot.activeMission`.
- Pause, resume, cancel, retry step, and skip step dispatch through normalized
  mission commands only when runtime reports those actions as available.
- Terminal room/zone summaries hydrate from `snapshot.missions.recent` and
  render in Recent Missions.
- Runtime room/zone result details preserve annotation target metadata and
  coverage outcome details when the backend provides them.
- TurtleBot4/Nav2 stores prototype annotations in webview storage keyed to the
  active map identity where available.
- Controls are capability-gated through `map_annotations`, `room_semantics`,
  `zone_semantics`, `room_cleaning`, `zone_cleaning`, and mission lifecycle
  capabilities.

Implementation limits:

- Recent mission persistence is prototype webview storage when the VM runtime
  does not provide durable mission history.
- Unsaved drafts are local React/webview state and are not durable.
- Annotation durability is not VM/runtime-owned.
- Geometry is rectangle-only.

## Valetudo Backend Path

The product UI consumes `useVacuumAdapter`; it does not call Valetudo HTTP,
Valetudo MQTT, VM runtime endpoints, or raw Valetudo source endpoints directly.

Runtime path:

```text
Vacuum Control
  -> useVacuumAdapter({ backend: "valetudo" })
  -> Valetudo backend adapter
  -> Valetudo runtime client
  -> vm-manager /vms/self/tensorfleet/... proxy by default
  -> VM-managed Valetudo integration runtime
```

Runtime endpoints consumed by the adapter:

```text
GET  /api/v1/valetudo/health
GET  /api/v1/valetudo/snapshot
POST /api/v1/valetudo/command
```

Default extension route through vm-manager:

```text
{TENSORFLEET_VM_MANAGER_URL}/vms/self/tensorfleet/api/v1/valetudo/health
{TENSORFLEET_VM_MANAGER_URL}/vms/self/tensorfleet/api/v1/valetudo/snapshot
{TENSORFLEET_VM_MANAGER_URL}/vms/self/tensorfleet/api/v1/valetudo/command
```

Local development may use `TENSORFLEET_VALETUDO_RUNTIME_URL` for direct runtime
access. Runtime client configuration comes from injected window values:

```text
window.TENSORFLEET_VM_MANAGER_URL
window.TENSORFLEET_VALETUDO_RUNTIME_URL
window.TENSORFLEET_VALETUDO_RUNTIME_ROUTE_MODE
window.TENSORFLEET_VACUUM_BACKEND
window.TENSORFLEET_JWT
```

VM runtime source configuration:

```text
VALETUDO_RUNTIME_SOURCE_MODE=fixed_mock | valetudo_mock_http | valetudo_mock_mqtt | valetudo_http
VALETUDO_MOCK_SOURCE_URL=http://172.16.0.1:8081
VALETUDO_SOURCE_URL=http://<valetudo-hostname-or-ip>
VALETUDO_SOURCE_TIMEOUT_MS=2000
VALETUDO_SOURCE_STALE_TIMEOUT_MS=15000
VALETUDO_MQTT_ENABLED=false
VALETUDO_MQTT_BROKER_URL=tcp://172.16.0.1:1883
```

Valetudo adapter behavior:

- `snapshot.connection`, `snapshot.robot`, `snapshot.battery`,
  `snapshot.dock`, `snapshot.map`, `snapshot.capabilities`, and
  `snapshot.diagnostics` hydrate from the Valetudo runtime snapshot.
- Supported commands route through the Valetudo runtime command endpoint:
  `start_cleaning`, `pause`, `resume`, `stop`, `return_to_dock`,
  `set_fan_speed`, and `set_water_usage`.
- Fan speed and water usage controls should use normalized option values from
  the snapshot/capabilities.
- Unsupported map-navigation, coverage, room, and zone commands should remain
  capability-gated in the UI.

## Debug Panels

`Nav2Panel.tsx` is the lower-level Nav2 operator/debug surface. It uses
`useNav2Runtime` directly and is useful for checking NavigateToPose action
services, action feedback/status topics, topic health, and lifecycle evidence.

`RawMessagesPanel.tsx` is the generic ROS topic inspection surface. It uses
`tensorfleet-ros` topic discovery and subscription helpers.

`SensorView3DPanel.tsx` is the 3D sensor/debug visualization surface. It uses
`tensorfleet-ros` subscriptions and Lichtblick/Foxglove visualization pieces.

`MissionControl` and `DroneMap` are drone-oriented surfaces, not the vacuum
operator path:

- `MissionControl` uses drone state and mission controller code.
- `DroneMap` is an OpenLayers world map with GPS-style vehicle state.
