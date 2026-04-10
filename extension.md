# TurtleBot4 VS Code Extension Integration Notes

This file is the extension-side companion to:

- `steps.md`
- `docs/VACUUM_STACK_PLAN.md`

It captures what the VS Code extension needs to know to use the current
TurtleBot4/Nav2 VM stack as an operator and debugging surface.

The important constraint is that this phase should stay extension-side. The VM
already exposes enough TurtleBot4, SLAM, and Nav2 surfaces for panel validation.
Do not block this work on a vacuum adapter, mission lifecycle, docking behavior,
or normalized backend contract.

## Current Integration Goal

Use the existing VS Code extension panels in `~/vscode-tensorfleet` against the
current TurtleBot4/Nav2 VM backend.

The extension should:

1. Connect to the running VM bridge endpoints.
2. Prefer TurtleBot4 topic defaults when the active VM config is `turtlebot4`.
3. Show live map, lidar, odom/TF, RGB/depth camera, point cloud, costmap, and
   robot status data where current panels support those message types.
4. Expose enough Nav2 action visibility to debug goal execution.
5. Record panel gaps as extension follow-up tasks rather than adding a new
   robotics-stack layer.

## Extension Repository

The extension repo is:

- `~/vscode-tensorfleet`

Important files inspected for this integration:

- `src/vm-manager.ts`
- `src/extension.ts`
- `src/regions.ts`
- `src/templates/drone-view-list.html`
- `panels-standalone/src/ros2-bridge.ts`
- `panels-standalone/src/components/ImagePanel.tsx`
- `panels-standalone/src/components/Teleop/TeleopPanel.tsx`
- `panels-standalone/src/components/SensorView3D/SensorView3DPanel.tsx`
- `panels-standalone/src/components/RawMessages/RawMessagesPanel.tsx`
- `panels-standalone/src/components/MissionControl/MissionControl.tsx`
- `panels-standalone/src/components/MissionControl/map/DroneMap.tsx`

Current notable extension state:

- `src/vm-manager.ts` already contains a `turtlebot4` VM config.
- `src/regions.ts` already defines Foxglove and ROS bridge ports:
  - Foxglove: `8765`
  - rosbridge: `9091`
- `src/extension.ts` injects `window.TENSORFLEET_VM_CONFIG_ID` into standalone
  panels when a VM config is known.
- `panels-standalone/src/ros2-bridge.ts` already groups image topic suggestions
  by VM config, but it does not yet include a TurtleBot4 image/default topic
  group.
- `TeleopPanel.tsx` currently defaults to `/cmd_vel`; TurtleBot4 should default
  to `/turtlebot4/cmd_vel`.
- `SensorView3DPanel.tsx` is already close to useful for TurtleBot4 because it
  can render LaserScan, PointCloud2, TF, odometry, occupancy grid, and costmap
  style data through the Lichtblick renderer.
- `MissionControl.tsx` and `DroneMap.tsx` are currently drone/GPS oriented and
  should not be treated as the primary SLAM map panel yet.

## VM Bridge Endpoints

For this phase, the extension should use the already running VM bridge
endpoints:

- Foxglove/Lichtblick panels: `ws://172.16.0.10:8765`
- rosbridge-specific panels: `ws://172.16.0.10:9091`

The current standalone panel path primarily uses Foxglove:

- `panels-standalone/src/ros2-bridge.ts`
- `panels-standalone/src/foxglove-networking.ts`

`ros2-bridge.ts` currently defines `ConnectionMode = "foxglove"`, so new panel
work should assume Foxglove first unless a panel explicitly needs rosbridge.

## TurtleBot4 Topic Map

Use these topics as the initial extension-side mapping.

Navigation action:

- `/turtlebot4/navigate_to_pose`

SLAM and map:

- `/turtlebot4/map`
- `/turtlebot4/map_metadata`

Lidar:

- `/turtlebot4/scan`

Odometry and TF:

- `/turtlebot4/odom`
- `/turtlebot4/tf`
- `/turtlebot4/tf_static`

RGB camera:

- `/turtlebot4/oakd/rgb/preview/image_raw`

Camera info:

- `/turtlebot4/oakd/rgb/preview/camera_info`

Depth camera:

- `/turtlebot4/oakd/rgb/preview/depth`

Point cloud:

- `/turtlebot4/oakd/rgb/preview/depth/points`

Velocity and debug:

- `/turtlebot4/cmd_vel`
- `/turtlebot4/cmd_vel_nav`
- `/turtlebot4/cmd_vel_smoothed`

Costmaps:

- `/turtlebot4/local_costmap/costmap`
- `/turtlebot4/global_costmap/costmap`

Basic robot status topics that are useful if present on the bridge:

- battery state topic under the TurtleBot4 namespace
- hazard detection topic under the TurtleBot4 namespace
- dock status topic under the TurtleBot4 namespace
- IMU topic under the TurtleBot4 namespace

The exact status topic names should be discovered from Foxglove topic
advertisements before hard-coding UI labels.

## Panel Mapping

### Image Panel

File:

- `~/vscode-tensorfleet/panels-standalone/src/components/ImagePanel.tsx`

Primary TurtleBot4 default:

- `/turtlebot4/oakd/rgb/preview/image_raw`

Additional camera-related suggestions:

- `/turtlebot4/oakd/rgb/preview/depth`
- `/turtlebot4/oakd/rgb/preview/camera_info` for metadata, not as an image
  stream selector

Needed extension-side work:

1. [x] Add a `turtlebot4` entry in `TOPICS_BY_VM_CONFIG` in
   `panels-standalone/src/ros2-bridge.ts`.
2. [x] Ensure `ImagePanel` auto-selects the TurtleBot4 RGB image when the VM config
   is `turtlebot4`.
3. [x] Keep discovered live topics in the dropdown so unexpected camera publishers
   remain debuggable.

This should work with the current VM as long as Foxglove advertises the image
topic and the existing image decoder path accepts the message encoding.

### Teleop Panel

File:

- `~/vscode-tensorfleet/panels-standalone/src/components/Teleop/TeleopPanel.tsx`

Primary TurtleBot4 default:

- `/turtlebot4/cmd_vel`

Needed extension-side work:

1. Make the default topic depend on `window.TENSORFLEET_VM_CONFIG_ID`.
2. Use `/turtlebot4/cmd_vel` when the config is `turtlebot4`.
3. Preserve the current `/cmd_vel` default for simple robot and other generic
   configs.
4. Keep the topic dropdown discovery-based so `/turtlebot4/cmd_vel_nav` and
   `/turtlebot4/cmd_vel_smoothed` can be inspected but not accidentally used as
   the main manual teleop command topic.

This should work immediately because the VM already exposes a TurtleBot4
velocity control surface.

### 3D Sensor View

File:

- `~/vscode-tensorfleet/panels-standalone/src/components/SensorView3D/SensorView3DPanel.tsx`

Primary TurtleBot4 defaults:

- `/turtlebot4/scan`
- `/turtlebot4/odom`
- `/turtlebot4/tf`
- `/turtlebot4/tf_static`
- `/turtlebot4/oakd/rgb/preview/depth/points`
- `/turtlebot4/local_costmap/costmap`
- `/turtlebot4/global_costmap/costmap`
- `/turtlebot4/map`

Needed extension-side work:

1. Update TF detection to include namespaced TF topics:
   - `/turtlebot4/tf`
   - `/turtlebot4/tf_static`
2. Update odometry detection to include:
   - `/turtlebot4/odom`
3. Update auto-visible topic rules to include:
   - `/turtlebot4/scan`
   - `/turtlebot4/oakd/rgb/preview/depth/points`
   - `/turtlebot4/map`
   - `/turtlebot4/local_costmap/costmap`
   - `/turtlebot4/global_costmap/costmap`
4. For occupancy grids, use map coloring for `/turtlebot4/map` and costmap
   coloring for local/global costmaps.
5. Keep renderer config discovery-based so the panel still works with other
   robots.

This is the best current place to validate lidar, point cloud, TF, odom, map,
and costmaps without building a new SLAM-specific panel first.

### Raw Messages Panel

File:

- `~/vscode-tensorfleet/panels-standalone/src/components/RawMessages/RawMessagesPanel.tsx`

Useful TurtleBot4 defaults:

- `/turtlebot4/navigate_to_pose/_action/status` if advertised
- `/turtlebot4/navigate_to_pose/_action/feedback` if advertised
- `/turtlebot4/navigate_to_pose/_action/result` if advertised
- `/turtlebot4/odom`
- `/turtlebot4/scan`
- `/turtlebot4/map`
- `/turtlebot4/local_costmap/costmap`
- `/turtlebot4/global_costmap/costmap`
- status topics discovered under `/turtlebot4/*battery*`, `/turtlebot4/*dock*`,
  `/turtlebot4/*hazard*`, and `/turtlebot4/*imu*`

Needed extension-side work:

1. Add a TurtleBot4 quick-filter or default pinned topic set.
2. Prefer advertised action status/feedback/result topics over hard-coded
   assumptions where possible.
3. Use Raw Messages as the first Nav2 visibility surface before adding a richer
   Nav2 goal panel.

This should work now for any advertised topic because the raw panel is already
generic.

### Map / Mission Control Panel

Files:

- `~/vscode-tensorfleet/panels-standalone/src/components/MissionControl/MissionControl.tsx`
- `~/vscode-tensorfleet/panels-standalone/src/components/MissionControl/map/DroneMap.tsx`

Current state:

- This panel is drone/GPS oriented.
- It uses a `DroneStateModel`.
- `DroneMap` is an OpenLayers world map with GPS-style vehicle state.
- It is not currently a SLAM occupancy-grid map panel.

Do not use this panel as the primary TurtleBot4 SLAM map validation surface
without rewriting it.

Useful extension-side follow-up:

1. Either rename/scope the current panel as drone-specific, or
2. add a new robot map panel that renders `nav_msgs/msg/OccupancyGrid` from:
   - `/turtlebot4/map`
   - `/turtlebot4/local_costmap/costmap`
   - `/turtlebot4/global_costmap/costmap`

For the current step, prefer the 3D sensor panel and raw messages panel for map
and costmap validation.

## Features We Can Add On The VS Code Extension Side Right Now

These features should work with the current VM stack and do not require a new
vacuum adapter.

1. TurtleBot4 panel preset

   Add a preset that selects:

   - image topic: `/turtlebot4/oakd/rgb/preview/image_raw`
   - teleop topic: `/turtlebot4/cmd_vel`
   - lidar topic: `/turtlebot4/scan`
   - odom topic: `/turtlebot4/odom`
   - TF topics: `/turtlebot4/tf`, `/turtlebot4/tf_static`
   - map topic: `/turtlebot4/map`
   - costmap topics: `/turtlebot4/local_costmap/costmap`,
     `/turtlebot4/global_costmap/costmap`

   This is the highest-value first extension change.

2. TurtleBot4 3D debug layout

   Add a one-click layout or panel state that makes the 3D sensor view show:

   - lidar scan
   - depth point cloud
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
   - result

   The first version can be read-only. It only needs to show whether a goal is
   active, succeeded, canceled, or failed, plus the latest feedback fields that
   are present.

4. Nav2 goal sender

   Add a simple goal publisher/client for:

   - `/turtlebot4/navigate_to_pose`

   The first version can accept numeric x/y/yaw in the map frame. Do not add
   room cleaning, zone cleaning, or mission semantics yet.

   If the Foxglove bridge action-client path is not available in the current
   extension networking layer, defer this to a small transport follow-up and
   keep read-only status in Raw Messages.

5. Occupancy-grid map panel

   Add a new robot map panel that renders `nav_msgs/msg/OccupancyGrid` directly
   in canvas:

   - `/turtlebot4/map` as the SLAM map
   - `/turtlebot4/local_costmap/costmap` as local planner costmap
   - `/turtlebot4/global_costmap/costmap` as global planner costmap

   This should be a robot/SLAM map panel, not an OpenLayers GPS map.

6. TurtleBot4 robot status panel

   Add a compact robot status panel using discovered TurtleBot4 status topics:

   - battery percentage/voltage/current
   - IMU presence and latest timestamp
   - hazard state if advertised
   - dock state if advertised
   - odom freshness
   - TF freshness
   - Nav2 action status

   This can be read-only and discovery-driven.

7. Topic health checklist

   Add a panel section that marks each expected TurtleBot4 topic as:

   - advertised
   - receiving messages
   - stale
   - missing

   This would make VM-side validation much faster because it turns the manual
   checklist from `steps.md` into an extension UI.

8. First-mile operator dashboard

   Add an `Open TurtleBot4 Dashboard` command that opens:

   - Image Panel
   - Teleop Panel
   - 3D Sensor View
   - Raw Messages Panel

   The command should apply the TurtleBot4 topic preset and leave the current
   drone mission-control panel out unless it has been rewritten for SLAM maps.

## Recommended Immediate Extension Patch

The next patch in `~/vscode-tensorfleet` should be small and concrete:

1. [x] Add `turtlebot4` to `TOPICS_BY_VM_CONFIG` in
   `panels-standalone/src/ros2-bridge.ts`.
2. [ ] Make `TeleopPanel.tsx` choose `/turtlebot4/cmd_vel` for the `turtlebot4`
   VM config.
3. [ ] Update `SensorView3DPanel.tsx` helper functions so namespaced TurtleBot4 TF,
   odom, scan, point cloud, map, and costmap topics are auto-listed and
   auto-visible.
4. [ ] Add a small TurtleBot4 pinned-topic set to `RawMessagesPanel.tsx`.
5. [x] Build the extension panels and verify the panel bundle compiles.

Suggested verification after patching:

```sh
cd ~/vscode-tensorfleet
bun run build:panels
bun run typecheck
```

Then run the extension against the VM and validate:

- Foxglove connection reaches `ws://172.16.0.10:8765`.
- Image Panel receives `/turtlebot4/oakd/rgb/preview/image_raw`.
- Teleop publishes to `/turtlebot4/cmd_vel`.
- 3D Sensor View shows `/turtlebot4/scan`.
- 3D Sensor View receives `/turtlebot4/tf`, `/turtlebot4/tf_static`, and
  `/turtlebot4/odom`.
- 3D Sensor View can list or display `/turtlebot4/map` and costmaps.
- Raw Messages can inspect Nav2 action status/feedback/result topics if they
  are advertised.

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
