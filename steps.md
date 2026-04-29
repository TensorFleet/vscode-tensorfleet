# Nav2 Panel Runtime Status

## Scope

This file records the current state of the TurtleBot4 + Nav2 validation work as
of April 29, 2026.

It is a runtime handoff, not a design doc.

## Repos Changed

Runtime and extension work landed in three repos:

- `~/vscode-tensorfleet`
- `~/firecracker-vm`
- `~/vm-manager`

Code-only commits:

- `~/vscode-tensorfleet`
  - `1472aa5` `Align TurtleBot4 panels with live Nav2 runtime`
  - `3d9359f` `Add Vacuum Control panel and associated files`
  - `6382b7f` `Extract shared Nav2 runtime layer`
  - `eb84ee9` `Extract Vacuum Control map canvas and fix live occupancy rendering`
  - `20047a3` `Add vacuum map layer overlays`
  - `96e2e8d` `Fix vacuum sensor overlay TF projection`
  - `b87855e` `Polish vacuum operator controls`
  - `d664024` `Refactor VacuumControlPanel and related components`
  - `cc48301` `Add TeleopCard and CameraOverlay to Vacuum Control panel`
- `~/firecracker-vm`
  - `d251a57` `Stabilize TurtleBot4 Nav2 runtime in Firecracker`
- `~/vm-manager`
  - `8fc9535` `Reserve more vCPUs for TurtleBot4 VMs`

## What Changed

### Extension-side

The standalone panels now match the live TurtleBot4 runtime instead of the old
namespaced assumptions.

Implemented in `~/vscode-tensorfleet`:

- Nav2 validation panel now uses the real action path:
  - `/navigate_to_pose/_action/send_goal`
  - `/navigate_to_pose/_action/get_result`
  - `/navigate_to_pose/_action/cancel_goal`
  - `/navigate_to_pose/_action/status`
  - `/navigate_to_pose/_action/feedback`
- Foxglove service discovery is surfaced through the extension bridge.
- Foxglove service-call support now tolerates missing request schemas for common
  ROS 2 action service shapes.
- the non-visual Nav2 runtime logic is now extracted into a shared module under:
  - `panels-standalone/src/components/Nav2/runtime/`
  - `useNav2Runtime.ts`
  - `nav2RuntimeConstants.ts`
  - `nav2RuntimeTypes.ts`
  - `nav2RuntimeUtils.ts`
- `Nav2Panel` now acts as the verification harness for that shared runtime
  layer instead of owning all action/preflight/runtime logic directly
- TurtleBot4 defaults in Teleop, Raw Messages, and 3D view now use global
  topics instead of `/turtlebot4/*`.
- Connection settings now preserve the explicit target port for standalone
  panels.

### VM runtime-side

Implemented in `~/firecracker-vm`:

- removed duplicate global Gazebo bridges for TurtleBot4 `/tf`, `/tf_static`,
  `/odom`, and `/scan`
- updated `twist_deadman.py` to accept both `Twist` and `TwistStamped` on
  `/cmd_vel_raw`
- added a lean Nav2 launch for the VM:
  - `navigation_vm.launch.py`
- switched Nav2 bringup to use the VM-specific launch
- replaced the heavier controller settings with a lighter DWB configuration and
  reduced controller frequency for VM use

### VM sizing

Implemented in `~/vm-manager`:

- TurtleBot4 VMs now get a minimum of 4 vCPUs at runtime conversion time
- other VM variants keep their existing CPU behavior

## Runtime Validation

Validated on:

- VM: `root@172.16.0.10`
- runtime date: April 22, 2026

Verified:

- VM is running with `4` vCPUs
- Nav2 lifecycle services are active
- `/navigate_to_pose` action is present
- `/map`, `/scan`, `/odom`, `/pose`, `/tf`, `/tf_static`, `/plan`,
  `/cmd_vel_nav`, `/cmd_vel_raw`, and `/cmd_vel` are present
- `map -> odom -> base_link` is available

Smoke test results:

- `0.30 m` forward `NavigateToPose` succeeded
- `1.00 m` forward `NavigateToPose` produced real motion and controller output,
  but the first long run timed out waiting for a terminal result

Important runtime observation:

- the robot was starting physically blocked by the dock
- the dock is spawned adjacent to the TurtleBot4 at bringup time
- the current Nav2 costmaps rely on `/scan`
- the lidar does not reliably represent the dock geometry as a blocking obstacle
  in the costmap

Practical consequence:

- Nav2 was driving, but it was starting in a bad physical situation
- this looked like “accepted but not really moving” until runtime inspection
  showed the robot was pushing against the dock

## Current Truth

The main earlier blockers are resolved, and the Layer 2 TurtleBot4/Nav2
operator slice is now closed.

What is now true:

- panel transport supports hidden action services
- panel topic assumptions match the live VM
- Nav2 output reaches the robot drive path
- duplicate odom/scan/TF bridges are removed
- TurtleBot4 VMs now get enough CPU to avoid the earlier 1-vCPU collapse mode
- short `NavigateToPose` runs succeed end-to-end
- the extension now has a reusable Nav2 runtime seam shared by the debug Nav2
  panel and the `Vacuum Control` operator panel
- `Vacuum Control` works against the live VM runtime as the current Layer 2
  operator surface

Validated `Vacuum Control` operator flows:

- connect to the live VM Foxglove endpoint
- render the live occupancy map in the operator map canvas
- select a navigation target from the rendered map
- send a `NavigateToPose` goal through the live Nav2 action API
- observe live progress from Nav2 feedback/status
- cancel an active goal and observe the stopped/canceled state
- observe terminal navigation state after a run completes, fails, or is canceled

This closes the Layer 2 operator slice. The current panels remain useful for
debugging and runtime validation, but the normal Layer 2 path is now the
single-panel `Vacuum Control` workflow.

The dock issue remains important, but it should now be treated as a runtime
caveat:

- navigation validation must still start from a clear, undocked pose
- starting on or against dock geometry can still make a healthy run look broken
- clear-space validation is still required before treating a failed or stalled
  navigation run as a software failure

Explicitly out of Layer 2:

- `vacuum_adapter` contract work
- mission lifecycle semantics
- docking workflow behavior
- battery or charging simulation
- room, zone, or coverage-cleaning workflows

## Layer 2 Status

Layer 2 is closed for the TurtleBot4/Nav2 operator slice.

What is proven:

- Nav2 brings up
- goals are accepted through the real action API
- feedback and status are visible in the extension
- controller output reaches the robot
- robot motion exists in simulation
- short end-to-end navigation success is observed
- the shared Nav2 runtime layer exists and `Nav2Panel` already runs on it
- `Vacuum Control` connects to the live VM runtime
- `Vacuum Control` renders the live map
- map target selection drives the Nav2 goal-send path
- progress, cancel, and terminal states are visible in the operator panel

What remains after Layer 2 is follow-up work, not Layer 2 exit work.

What remains a runtime caveat:

- blocked starts near the dock can still invalidate a run
- clear-space validation is still required until runtime setup is improved

## Recommended Next Step

The next solid step is to move beyond the closed Layer 2 operator slice without
expanding its scope retroactively.

Recommended follow-up work should be tracked separately and can include:

- supporting debug surface polish for Raw Messages and 3D Sensor View
- clear-space / undocked-start validation improvements
- later adapter, mission, and docking layers after the Nav2 operator flow stays
  stable

## Completed Execution Order

This was the completed order for the Layer 2 implementation slice:

1. lock the current shell and document what already exists
2. complete each visible component against live runtime behavior
3. validate the full one-panel operator flow end-to-end
4. defer non-critical follow-up surfaces until after the operator slice is
   runtime-solid

## Step 1: Operator Shell UI

This shell stage is complete. It built the panel first as a polished UI shell
before full topic wiring.

The shell is:

- a new standalone panel: `Vacuum Control`
- map-first and user-oriented rather than debug-oriented
- visually aligned with the future vacuum product shell
- built around product concepts:
  - state
  - target
  - progress
  - actions

The first shell included:

- a left navigation rail
- a header / top bar
- a quick readiness strip
- a hero map canvas region
- a right-side state / action column

Visible shell components should be:

- `Header`
- `StatusStrip`
- `MapCanvas`
- `CurrentStateCard`
- `SelectedDestinationCard`
- `ProgressCard`
- `ActionsCard`
- `LeftNavRail`

The product-facing structure and visual language are now locked for the closed
Layer 2 operator slice.

`WarningCard` remains deferred. It is not part of the closed Layer 2 operator
slice.

## Current UI Truth

The `Vacuum Control` shell now exists in the repo and is the validated Layer 2
operator surface rather than a future mockup target.

Current visible shell components:

- `LeftNavRail`
- `Header`
- connection / status pill
- `StatusStrip`
- `MapCanvas`
- `MapControls` with bounded zoom controls, fit-to-map, and zoom readout
- `MapLegend`
- floating `Layers` control with checklist popover
- `CameraOverlay` floating PiP window inside the map canvas; auto-discovers
  image topics; draggable, minimizable, and hideable
- `CurrentStateCard` with operator state, readiness evidence grid, and
  pending-state icons
- `SelectedDestinationCard` with distance, facing, bearing, and map coordinates
- `ProgressCard` with progress percentage, indeterminate animation, remaining
  distance, elapsed navigation time, and recovery count
- `ActionsCard` with state-aware start / stop / clear / connection actions;
  run-again reuses the last sent destination
- `TeleopCard` collapsible manual control card; directional D-pad; publishes
  to `/cmd_vel_raw`; optional WASD / arrow key toggle
- persistent settings button in header (always accessible)
- settings button at bottom of left nav rail

Current runtime seam:

- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
- `panels-standalone/src/components/VacuumControl/MapCanvas.tsx`
- `panels-standalone/src/components/VacuumControl/TeleopCard.tsx`
- `panels-standalone/src/components/VacuumControl/CameraOverlay.tsx`
- `panels-standalone/src/components/VacuumControl/mapOverlayUtils.ts`
- `panels-standalone/src/components/Nav2/runtime/useNav2Runtime.ts`
- `panels-standalone/src/ros2-bridge.ts`

Current map-canvas truth:

- `MapCanvas.tsx` now owns `/map`, `/global_costmap/costmap`, and
  `/local_costmap/costmap` subscriptions; occupancy rendering; map controls;
  pointer target placement; and route/marker overlays
- live occupancy rendering now tolerates Foxglove array and typed-array
  payloads so valid `/map` traffic does not fall back to the placeholder canvas
- the map now has a Google Maps-style floating `Layers` button with checklist
  toggles for Map, Global costmap, Local costmap, Plan, Lidar, and Depth
  obstacles
- Plan is operator-toggleable instead of always visible
- Lidar and depth obstacle overlays are projected into the `map` frame with a
  local `TransformTree` built from `/tf` and `/tf_static`
- Lidar and depth obstacles render on dedicated canvases beneath robot and
  target markers so sensor points do not obscure the primary navigation state
- map controls are isolated from pointer-based target placement and show bounded
  zoom state
- robot marker, destination marker labels, route overlay, and staged preview
  line have been restyled for contrast against the occupancy map
- route and staged preview visibility follow the Plan layer toggle

Current right-column card truth:

- `CurrentStateCard` is driven by connection, map, localization, preflight, and
  goal state
- `CurrentStateCard` does not duplicate the top status-strip badge; it uses
  state title, detail, icon, and readiness evidence instead
- `SelectedDestinationCard` renders the active displayed target, including a
  sent target during an active run
- `SelectedDestinationCard` shows empty state, distance from robot, target
  facing, bearing from robot, and map coordinates
- `ActionsCard` opens connection settings while disconnected, starts / retries /
  reruns when ready, stops active runs, and only clears destinations when no run
  is active
- `ProgressCard` no longer shows ETA because `estimated_time_remaining` is not
  reliable enough for the operator surface; it shows remaining distance, elapsed
  navigation time, and recovery count from Nav2 feedback
- readiness model: start is gated only on `mapReady` and `preflightReady`;
  `poseReady` no longer blocks the start action because map availability alone
  is sufficient to attempt goal dispatch

Current bridge and overlay truth:

- `ros2-bridge.ts` now caches the latest message per topic and replays it
  immediately to any new subscriber so panels receive current state on mount
  rather than waiting for the next publish cycle
- `ros2-bridge.ts` accumulates static TF transforms per unique edge and replays
  them as a synthetic bundle when a new subscriber joins `/tf_static`
- `mapOverlayUtils.ts` uses frame ID fallback candidate lists for robot pose
  (`base_footprint`, `base_link`, and namespaced variants) and lidar frame
  (`rplidar_link`, `base_scan`, `laser`, and namespaced variants) so overlay
  projection tolerates frame naming variation across runtime configurations
- `mapOverlayUtils.ts` laser scan topic discovery now matches any advertised
  `sensor_msgs/msg/LaserScan` or `foxglove.LaserScan` topic, preferring `/scan`
- point projection in `mapOverlayUtils.ts` supports both plain JS arrays and
  typed arrays (e.g. `Float32Array`) from Foxglove-encoded point cloud fields

Closed Layer 2 topics and services already used by the panel/runtime:

- `/map`
- `/scan`
- depth `sensor_msgs/msg/PointCloud2` topic discovered from advertised topics,
  preferring `/oakd/rgb/preview/depth/points`
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
- `/cmd_vel_raw` (publish — `TeleopCard` manual control)
- `/local_costmap/costmap`
- `/global_costmap/costmap`
- `/stop_status`
- `/navigate_to_pose/_action/send_goal`
- `/navigate_to_pose/_action/get_result`
- `/navigate_to_pose/_action/cancel_goal`
- `/navigate_to_pose/_action/status`
- `/navigate_to_pose/_action/feedback`

## Layer 2 Closure Checklist

The following runtime checks define the closed Layer 2 operator slice:

- disconnected bridge shows offline header and state card
- connected bridge without `/map` shows waiting-for-map
- live `/map` and `/pose` activate readiness chips
- layers popover toggles map, costmaps, plan, lidar, and depth obstacle
  visibility without placing a target
- clicking the map selects a target and updates the destination card
- sending a goal changes the state, actions, and progress UI
- active run shows route, progress, remaining distance, elapsed navigation time,
  and recovery count when feedback is available
- lidar and depth obstacle overlays either project into the map frame or report
  waiting / no-TF state
- cancel transitions to canceled state and disables the right buttons while
  pending
- clear removes the target only when no active run exists
- success and failure terminal results render correctly
- settings button opens the connection overlay from both the header and the rail

Suggested build verification:

```sh
bun run typecheck
bun run --cwd panels-standalone build
```

### Suggested Components After Step 2

These are useful follow-up components, but not part of the closed Layer 2
operator slice:

- `RunHistoryPanel`
- `GoalDetailsInspector`
- `RuntimeHealthDrawer`
