# TurtleBot4 VS Code Extension Integration Notes

This file is the extension-side companion to:

- `steps.md`
- `VACUUM_STACK_PLAN.md`

It captures what the VS Code extension needs to know about the validated
TurtleBot4/Nav2 operator slice and the next adapter-backed product direction.

The old Layer 2 constraint was extension-side only: validate the VM runtime
through `Vacuum Control` without blocking on a vacuum adapter, mission
lifecycle, docking behavior, or normalized backend contract. That constraint is
closed, and Layer 3 is now wrapped for the TurtleBot4/Nav2 simulation path. The
next major milestone is Layer 4 coverage above the `vacuum_adapter` contract.

The full seven-layer plan this file aligns with:

```text
Layer 0 — Sensors                   validated
Layer 1 — Localization + Map        running
Layer 2 — Navigation                closed for TurtleBot4/Nav2 simulation
Layer 3 — Vacuum Adapter            closed for TurtleBot4/Nav2 simulation,
                                    Valetudo stub reserved for Layer 6
Layer 4 — Coverage                  planned, YOU ARE HERE
Layer 5 — Room / Zone Semantics     planned
Layer 6 — Real Hardware (Valetudo)  planned
```

## Current Integration Goal

Use the existing VS Code extension panels in `~/vscode-tensorfleet` against the
current TurtleBot4/Nav2 VM backend.

The extension should:

1. Connect to the running VM bridge endpoints.
2. Treat the single-panel `Vacuum Control` operator workflow as the closed
   Layer 2/Layer 3 TurtleBot4/Nav2 simulation slice.
3. Show live map, lidar, odom/TF, costmap, and navigation status data where the
   current panels support those message types.
4. Expose enough Nav2 action visibility to drive and validate goal execution.
5. Record panel gaps as extension follow-up tasks, not as blockers for the
   closed Layer 3 adapter slice.

## Layer 2 / Layer 3 Closure

As of April 30, 2026, the Layer 2 TurtleBot4/Nav2 operator slice and Layer 3
adapter slice are closed for the simulation path. The operator flow is
validated end-to-end and `Vacuum Control` consumes the `vacuum_adapter`
contract through `useVacuumAdapter` instead of `useNav2Runtime`.

Frozen truth:

- `Vacuum Control` works against the live VM runtime through the current
  Foxglove bridge path.
- The operator panel validates the normal one-panel flow: connect, render map,
  select a map target, send a Nav2 goal, observe progress, cancel when needed,
  and observe terminal state.
- The panel uses the `vacuum_adapter` contract for product-facing navigation
  state, capabilities, normalized plan path, goal send, and cancel behavior.
- VS Code webview validation covers target replacement, stale canceled marker
  reset, stale plan clearing, failure states, overlays, teleop, and sending a
  second goal after cancel without reload.
- Remaining dock-blocked start behavior is a runtime caveat, not a Layer 2
  blocker.
- Clear-space validation is still required before interpreting a failed or
  stalled run as a software failure.

Explicitly out of the closed Layer 2/Layer 3 simulation slice:

- coverage cleaning flows and lawnmower paths (Layer 4)
- docking / return-to-dock workflow and battery-aware execution (Layer 4)
- room, zone, and segmentation UI (Layer 5)
- real vacuum hardware and Valetudo integration runtime (Layer 6)

## Layer 3 Result

Layer 3 is closed for the TurtleBot4/Nav2 simulation path.

Layer 3 should define the product-facing capability, state, and command
boundary above robot backends. Extension/product clients should talk in vacuum
concepts and capability descriptors, not raw TurtleBot4 topics, Nav2 internals,
or Valetudo-specific class names.
The public `vacuum_adapter` contract should own its coordinate, state, and
command payload types; backend/runtime-specific types should be imported only by
backend adapters.

Current target architecture:

```text
VS Code extension / product UI          (Layers 4 and 5 live here)
  -> vacuum_adapter contract            (Layer 3)
     -> TurtleBot4/Nav2 backend adapter (Layer 3, over Layer 2 runtime)
        -> VM TurtleBot4 simulation runtime

     -> Valetudo backend adapter        (Layer 6)
        -> VM-managed Valetudo integration runtime
           -> real Valetudo-compatible vacuum on local network
```

Layer 3 scope for the extension:

1. Consume the `vacuum_adapter` capability / state / command contract.
2. Migrate `Vacuum Control` from raw ROS topics onto the contract so the panel
   branches on capabilities and normalized state instead of backend specifics.
3. Surface the mission state machine (`idle / navigating / cleaning / paused /
   returning / charging`) from the adapter.
4. Leave a Valetudo adapter interface stub for the Layer 6 integration to
   populate later.

Current Layer 3 status (April 30, 2026):

- The `vacuum_adapter` contract now owns a public `VacuumAdapter` surface with
  `snapshot` + `sendCommand`, and a TurtleBot4/Nav2 backend hook
  (`useTurtleBot4Nav2Adapter`) that wraps `useNav2Runtime` behind the contract.
- `VacuumControlPanel.tsx` consumes the adapter through `useVacuumAdapter`
  instead of `useNav2Runtime` directly; all navigation state, readiness
  evidence, capability gating, and plan path rendering in the panel now flow
  through the adapter snapshot.
- `go_to_location` and `cancel_navigation` are dispatched as
  `adapter.sendCommand(...)` instead of raw `runtime.sendGoal` / `cancelGoal`
  calls.
- TurtleBot4/Nav2 command dispatch is extracted into a pure
  `commandDispatcher.ts` helper used by `useTurtleBot4Nav2Adapter`, so
  supported and unsupported command behavior can be tested outside React.
- The adapter emits a `mission` field with the explicit mission state machine
  (`idle / navigating / cleaning / paused / returning / charging`). The
  TurtleBot4/Nav2 backend reports `idle` or `navigating` today and keeps the
  remaining states as "not supported" so the UI can branch on capability
  descriptors rather than backend identity.
- Plan path rendering in `MapCanvas` now consumes a normalized
  `VacuumPathPoint[]` surface (`snapshot.navigation.planPath`) instead of the
  raw `nav_msgs/msg/Path` message shape, keeping the visualization surface
  decoupled from backend-specific pose shapes.
- `manual_control` (teleop) remains wired through the existing `TeleopCard`
  publisher to `/cmd_vel_raw` and is explicitly rejected by
  `adapter.sendCommand` with an `invalid_request` result, because routing it
  through `sendCommand` would require a streaming teleop channel that is not
  part of this adapter slice.
- A focused adapter regression harness is available through
  `bun run test:vacuum-adapter`. It checks capability coverage, TurtleBot4/Nav2
  supported and unsupported commands, normalized plan-path mapping, mission
  state mapping, public contract import boundaries, and backend-name branching
  in `VacuumControlPanel.tsx`.
- The Valetudo backend stub now includes capability, state, command, and
  runtime-boundary mapper shapes for Layer 6. It does not implement hardware
  connectivity yet.
- Live VS Code webview validation confirms the adapter-backed `Vacuum Control`
  flow against the VM: connect, render map, select target, send goal, cancel,
  see terminal state, handle failure paths, use overlays, use teleop, and send
  a second goal after cancel without reloading.
- Fresh target selection after cancel now resets the marker to staged/selected
  state and clears stale canceled plan geometry.

Out of Layer 3 (later layers):

- coverage path planning UI and "clean this area" flows (Layer 4)
- dock / undock and battery-aware execution beyond what the contract models
  (Layer 4)
- room / zone naming and segmentation UI (Layer 5)
- Valetudo VM service plan, MQTT vs HTTP choice, and real-hardware validation
  (Layer 6)

## Later Layers At A Glance

These layers are out of scope for now but the extension plan should respect
them so Layer 3 work does not box them out.

- Layer 4 (Coverage): coverage path planner (lawnmower), dock / undock, and
  battery-aware execution above the adapter contract. A future "Clean Area"
  interaction in the extension belongs here, not in Layer 3.
- Layer 5 (Room / Zone Semantics): named zones and a "clean room 3" flow that
  translates into Layer 4 coverage goals. Any room / zone UI in the extension
  belongs here.
- Layer 6 (Real Hardware / Valetudo): the Valetudo backend adapter and the
  VM-managed Valetudo integration runtime. When this lands, the extension and
  UI should run unchanged against the same `vacuum_adapter` contract.

Not the next milestone:

- more TurtleBot4 UI polish
- more debug panels
- OpenClaw
- room cleaning UI
- docking UI
- zone cleaning UI
- consumables UI
- scheduling UI

## Capability Decisions

The public contract should use backend-neutral product capability names:

- `start_cleaning`
- `pause`
- `resume`
- `stop`
- `return_to_dock`
- `go_to_location`
- `cancel_navigation`
- `manual_control`
- `navigation_status`
- `segment_cleaning`
- `zone_cleaning`
- `fan_speed`
- `water_usage`
- `consumables`
- `events`
- `dock_state`

Do not expose Valetudo implementation class names as public flags, such as:

- `BasicControlCapability`
- `MapSegmentationCapability`
- `ZoneCleaningCapability`
- `FanSpeedControlCapability`
- `WaterUsageControlCapability`

Valetudo concepts should map privately inside the backend adapter:

- Valetudo `BasicControlCapability` -> `start_cleaning` / `pause` / `stop` /
  `return_to_dock`
- Valetudo `GoToLocationCapability` -> `go_to_location`
- Valetudo `MapSegmentationCapability` -> `segment_cleaning`
- Valetudo `ZoneCleaningCapability` -> `zone_cleaning`
- Valetudo `FanSpeedControlCapability` -> `fan_speed`
- Valetudo `WaterUsageControlCapability` -> `water_usage`
- Nav2 `NavigateToPose` -> `go_to_location`

Capabilities should be descriptors, not booleans only:

```ts
type CapabilitySupport = {
  supported: boolean;
  source?: "turtlebot4_nav2" | "valetudo" | "mock";
  backendCapability?: string;
  commands?: string[];
  attributes?: string[];
  notes?: string;
};
```

Product/UI logic should ask whether capabilities are supported. It should not
branch on backend names such as TurtleBot4, Valetudo, Roborock, or any specific
robot model.

Layer 3 acceptance criterion:

When Layer 6 (real hardware) ships and Valetudo is integrated, existing vacuum
UI surfaces should continue to work through the `vacuum_adapter` contract
without backend-specific UI rewrites. UI code may branch on adapter
capabilities and normalized state, but it must not branch on whether the
backend is TurtleBot4/Nav2, Valetudo, or a vendor/model. Backend-specific
behavior belongs in backend adapters.

Command contracts should be explicit about payloads. For example,
`go_to_location` carries a backend-neutral target pose, and setter commands such
as `set_fan_speed` and `set_water_usage` carry a selected value rather than
being modeled as payload-free simple commands.

## VM-Managed Valetudo Path (Layer 6)

Valetudo-compatible vacuums are the first real-hardware backend target and
belong to Layer 6. Valetudo should influence the Layer 3 capability model, but
it should not define the public contract. Valetudo-specific naming remains
inside the Valetudo backend adapter.

Reference docs:

- [Valetudo capabilities overview](https://valetudo.cloud/pages/usage/capabilities-overview.html)
- [Valetudo MQTT implementation details](https://valetudo.cloud/pages/development/mqtt/)
- [Valetudo project](https://github.com/Hypfer/Valetudo)

The VM should eventually host the Valetudo integration runtime so users do not
need to install local integration tooling. Depending on the backend choice, the
VM may run:

- MQTT broker, if needed
- Valetudo client/integration service
- Valetudo backend adapter process
- discovery/config service
- runtime health checks

Boundary decision:

```text
VM owns backend runtime/integration services.
vacuum_adapter owns product-facing contract/capabilities/state.
```

In the VM:

- TurtleBot4/Nav2 runtime
- Foxglove/ROS bridge
- Valetudo integration runtime
- MQTT broker/client if needed
- hardware discovery/config

Outside VM / shared product layer:

- `vacuum_adapter` contract
- normalized state model
- capability descriptors
- command semantics
- UI-facing assumptions

Docs should say "Valetudo integration runtime in the VM," not "Valetudo in the
VM." Valetudo normally runs on the robot vacuum itself; the VM hosts the
integration layer around that robot.

First Valetudo hardware validation should stay basic:

```text
Valetudo robot reachable
-> VM receives status/capabilities
-> adapter normalizes state
-> extension displays capability/state summary
-> one basic command works
```

Good first commands:

- start
- pause
- stop
- return_to_dock

## Extension Repository

The extension repo is:

- `~/vscode-tensorfleet`

Important files for the closed Layer 2/Layer 3 simulation slice:

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
  exists as the closed Layer 2/Layer 3 operator shell.
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

For this closed Layer 2/Layer 3 simulation slice, the extension uses the
already running VM bridge
endpoints:

- Foxglove/Lichtblick panels: `ws://172.16.0.10:8765`
- rosbridge-specific panels: `ws://172.16.0.10:9091`

The current standalone panel path primarily uses Foxglove:

- `panels-standalone/src/ros2-bridge.ts`
- `panels-standalone/src/foxglove-networking.ts`

`ros2-bridge.ts` currently defines `ConnectionMode = "foxglove"`, so new panel
work should assume Foxglove first unless a panel explicitly needs rosbridge.

## Vacuum Control Current Truth

The closed Layer 2/Layer 3 simulation slice is the dedicated `Vacuum Control`
panel rather than a broader TurtleBot4 preset effort across every existing
panel.

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

- this panel is the closed Layer 2/Layer 3 operator workflow
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

Useful closed Layer 2/Layer 3 topics:

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

Useful closed Layer 2/Layer 3 topics:

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

## Extension Follow-Up After Layer 3

These features should work with the current VM stack. They are maintenance and
debugging follow-ups after the closed Layer 2/Layer 3 operator slice, not the
main Layer 4 coverage path.

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

   This is now maintenance and regression hardening for the closed Layer 2/Layer
   3 slice.

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

   This is now lower priority than before for the closed Layer 2/Layer 3 slice
   because `Vacuum Control` already renders the base occupancy map and
   local/global costmaps as operator overlays.

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

## Recommended Immediate Work

The Layer 3 contract hardening pass is now wrapped. `Vacuum Control` consumes
the `vacuum_adapter` contract through `useVacuumAdapter`, the TurtleBot4/Nav2
backend hook normalizes Nav2 runtime into the contract, command dispatch is
covered by a focused regression harness, and the Valetudo backend has
non-hardware mapper/interface stubs for Layer 6. Live VM validation through the
VS Code webview is complete. The remaining near-term work is Layer 4 coverage
planning and keeping supporting debug panels aligned, not a new contract
rewrite.

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
8. [x] Define the first `vacuum_adapter` capability/state/command contract.
9. [x] Add a TurtleBot4/Nav2 adapter mapping the validated runtime into the
   contract.
10. [x] Migrate `Vacuum Control` to consume the `vacuum_adapter` contract
    instead of raw ROS topics (Layer 3).
11. [x] Add an explicit mission state machine covering `idle / navigating /
    cleaning / paused / returning / charging` to the adapter (Layer 3).
12. [x] Add a focused `vacuum_adapter` regression harness for command,
    capability, state, and UI boundary checks.
13. [x] Expand the Valetudo adapter interface stub with capability, state,
    command, and runtime-boundary mapper shapes.
14. [x] Validate the hardened adapter contract against the live VM.
15. [ ] Keep the VM integration service plan for Valetudo in Layer 6.

Suggested verification after patching:

```sh
cd ~/vscode-tensorfleet
bun run test:vacuum-adapter
bun run typecheck
bun run --cwd panels-standalone build
```

The closed Layer 2/Layer 3 runtime validation covers:

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
- `Vacuum Control` clears stale canceled marker/plan state when a fresh target
  is selected after cancel.
- `Vacuum Control` can send a second goal after cancel without reloading.
- failure paths, layer overlays, sensor overlays, and teleop work in the VS
  Code webview.
- supporting debug panels can inspect `/scan`, `/odom`, `/tf`, `/tf_static`,
  `/map`, and costmaps as needed.

## Not Next

Do not make these the next milestone:

- room cleaning UI
- zone cleaning UI
- docking workflow UI
- simulated battery/charging behavior
- consumables UI
- scheduling UI
- OpenClaw workflow integration

Those belong after Layer 4/Layer 5 coverage and room/zone semantics, or in the
Layer 6 real-hardware path where noted.
