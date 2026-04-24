# Nav2 Panel Runtime Status

## Scope

This file records the current state of the TurtleBot4 + Nav2 validation work as
of April 22, 2026.

It is a runtime handoff, not a design doc.

## Repos Changed

Runtime and extension work landed in three repos:

- `~/vscode-tensorfleet`
- `~/firecracker-vm`
- `~/vm-manager`

Code-only commits:

- `~/vscode-tensorfleet`
  - `1472aa5` `Align TurtleBot4 panels with live Nav2 runtime`
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

The main earlier blockers are resolved.

What is now true:

- panel transport supports hidden action services
- panel topic assumptions match the live VM
- Nav2 output reaches the robot drive path
- duplicate odom/scan/TF bridges are removed
- TurtleBot4 VMs now get enough CPU to avoid the earlier 1-vCPU collapse mode
- short `NavigateToPose` runs succeed end-to-end
- the extension now has a reusable Nav2 runtime seam for the future operator UI

The main missing Layer 2 deliverable is no longer raw Nav2 transport validation.
It is a usable operator surface in the extension.

Current panels are now sufficient for debugging and runtime validation, but they
are not yet the intended near-final Layer 2 operator experience.

The dock issue remains important, but it should now be treated as a runtime
caveat:

- navigation validation must still start from a clear, undocked pose
- starting on or against dock geometry can still make a healthy run look broken

## Layer 2 Status

Layer 2 is close, but the exit condition should stay strict.

What is proven:

- Nav2 brings up
- goals are accepted through the real action API
- feedback and status are visible in the extension
- controller output reaches the robot
- robot motion exists in simulation
- short end-to-end navigation success is observed
- the shared Nav2 runtime layer exists and `Nav2Panel` already runs on it

What is still missing for Layer 2:

- completion of the dedicated user-facing operator panel in the extension
- runtime-hardening of the occupancy-map-based navigation UI
- explicit runtime validation of visual goal placement from the map
- operator-facing progress and terminal result presentation that is verified
  against the live runtime
- a one-panel workflow that does not require opening debug tools in the normal
  path

What remains a runtime caveat:

- blocked starts near the dock can still invalidate a run
- clear-space validation is still required until runtime setup is improved

## Recommended Next Step

The next solid step is to finish the first real Layer 2 operator UI in the
extension: the dedicated `Vacuum Control` panel that already exists in the repo
as the first version of the future vacuum operator shell.

The focus is no longer creating the shell from scratch. The focus is completing
each visible component as a runtime-testable Layer 2 slice.

## Execution Order

Build this in ordered steps inside the same Layer 2 implementation slice:

1. lock the current shell and document what already exists
2. complete each visible component against live runtime behavior
3. validate the full one-panel operator flow end-to-end
4. defer non-critical follow-up surfaces until after Step 2 is runtime-solid

## Step 1: Operator Shell UI

Build the panel first as a polished UI shell, even before topic wiring is
complete.

The shell should be:

- a new standalone panel: `Vacuum Control`
- map-first and user-oriented rather than debug-oriented
- visually aligned with the future vacuum product shell
- built around product concepts:
  - state
  - target
  - progress
  - actions

The first shell should include:

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

At this step, the panel can use local mock state and placeholder map content if
needed. The goal is to lock the product-facing structure and visual language
before runtime wiring.

`WarningCard` is deferred. It is not part of the current Layer 2 exit
criterion.

## Current UI Truth

The `Vacuum Control` shell now exists in the repo and should be treated as the
current Step 2 baseline rather than as a future mockup target.

Current visible shell components:

- `LeftNavRail`
- `Header`
- connection / status pill
- `StatusStrip`
- `MapCanvas`
- `MapControls`
- `MapLegend`
- floating `Layers` control with checklist popover
- `CurrentStateCard`
- `SelectedDestinationCard`
- `ProgressCard`
- `ActionsCard`
- settings entrypoint
- shell-only `History` nav item

Current runtime seam:

- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`
- `panels-standalone/src/components/VacuumControl/MapCanvas.tsx`
- `panels-standalone/src/components/Nav2/runtime/useNav2Runtime.ts`

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

Current Layer 2 topics and services already used by the panel/runtime:

- `/map`
- `/scan`
- depth `sensor_msgs/msg/PointCloud2` topic discovered from advertised topics,
  preferring `/oakd/rgb/preview/depth/points`
- `/odom`
- `/pose`
- `/tf`
- `/tf_static`
- `/plan`
- `/transformed_global_plan`
- `/cmd_vel_nav`
- `/local_costmap/costmap`
- `/global_costmap/costmap`
- `/stop_status`
- `/navigate_to_pose/_action/send_goal`
- `/navigate_to_pose/_action/get_result`
- `/navigate_to_pose/_action/cancel_goal`
- `/navigate_to_pose/_action/status`
- `/navigate_to_pose/_action/feedback`

## Step 2: Operator Interaction Model

This step is no longer “build the mockup.” The shell already exists.

Step 2 should now be treated as ordered component-completion work where each
item is runtime-testable on its own.

The important rule here:

- keep operator-facing copy product-oriented
- do not let raw ROS terminology leak into the visible operator copy
- treat each item as done only when it works against real runtime conditions,
  not just local mock state

### 2.1 `Header`

Implement:

- show live connection state in the header pill
- keep the product title and section label stable
- keep the settings entrypoint available from the header

Runtime done when:

- disconnected runtime shows offline header state
- reconnecting runtime shows connecting state
- connected runtime shows connected state
- header settings button opens the connection overlay

### 2.2 `StatusStrip`

Implement:

- map, localization, readiness, and target-selection chips
- chip state derived from live runtime state rather than hard-coded shell values

Runtime done when:

- no live `/map` keeps map readiness inactive
- live `/map` activates the map chip
- live `/pose` or equivalent active localization activates the localization chip
- readiness becomes active only when the panel is actually preflight-ready
- selecting a target activates the target chip

### 2.3 `MapCanvas`

Implement:

- render the real occupancy grid from `/map`
- render global and local costmaps as optional overlays
- expose a floating layers checklist for Map, Global costmap, Local costmap,
  Plan, Lidar, and Depth obstacles
- let the operator toggle the active plan overlay
- project lidar and depth obstacle points into map coordinates before drawing
- use placeholder content only when a live map is not available
- keep map-first interaction as the center of the panel

Runtime done when:

- live `/map` renders occupancy content rather than the placeholder map
- live `/global_costmap/costmap` and `/local_costmap/costmap` can be toggled
  without disrupting target placement
- the Plan layer can be hidden and restored while preserving current route
  state
- lidar and depth obstacle layers show `No TF` or `Waiting` instead of drawing
  incorrect points when projection inputs are unavailable
- lack of `/map` falls back to placeholder content instead of a broken canvas
- the map remains usable as the main target-selection surface

### 2.4 `MapControls`

Implement:

- zoom in
- zoom out
- reset / center zoom
- controls that do not interfere with pointer-based target selection

Runtime done when:

- zoom controls visually change the map viewport
- reset returns the map to the default zoom
- using the controls does not unintentionally place or rotate a target

### 2.5 `Robot Marker + Route Overlay`

Implement:

- robot marker from the live localized pose
- route overlay from the active plan
- target marker aligned with the selected or sent target
- staged preview line before send
- keep sensor overlays visually below robot and target markers

Runtime done when:

- live pose places the robot marker correctly on the map
- selecting a target shows a destination marker in the expected location
- active route data renders a visible route overlay
- route visibility follows the Plan layer toggle
- lidar/depth overlay points do not cover the robot or destination markers
- route and markers remain aligned while the map view is zoomed

### 2.6 `CurrentStateCard`

Implement:

- operator-facing state mapping for:
  - disconnected
  - waiting for map
  - waiting for localization
  - ready
  - active
  - completed
  - failed
  - canceled

Runtime done when:

- disconnected bridge shows the offline card state
- connected runtime without `/map` shows waiting-for-map
- live map with incomplete localization shows a positioning / waiting state
- ready runtime without an active goal shows ready
- active goal shows in-progress copy
- terminal success, failure, and canceled outcomes map to the correct card

### 2.7 `SelectedDestinationCard`

Implement:

- empty state when no target is selected
- selected state with distance and heading derived from current pose and target

Runtime done when:

- no target shows the empty card state
- clicking the map updates the card to a selected state
- displayed distance and heading update to match the chosen target

### 2.8 `ActionsCard`

Implement:

- `Send`
- `Cancel`
- `Clear destination`
- correct enable, disable, and loading behavior

Runtime done when:

- `Send` is disabled until a target exists and preflight is ready
- sending a goal shows the sending state and starts the run
- active runs swap the primary action to `Cancel`
- cancel shows a pending state while cancellation is in progress
- `Clear destination` works only when no active run exists

### 2.9 `ProgressCard`

Implement:

- show progress only once a run has started or reached a terminal state
- show progress label, percent, and distance remaining
- show ETA when feedback exposes it

Runtime done when:

- idle state hides the progress card
- sending or active run shows progress
- live feedback updates distance remaining and ETA when available
- success, failure, and canceled runs keep a correct terminal progress state

### 2.10 `LeftNavRail`

Implement:

- `Navigation` stays the active item for this slice
- `Settings` remains functional
- `History` stays explicitly placeholder-only until a real runtime-backed
  history surface exists

Runtime done when:

- the current screen is clearly `Navigation`
- the settings entrypoint works from the rail
- `History` is visibly non-functional or clearly marked as coming soon

### Step 2 Validation Checklist

Treat Step 2 as complete only when these runtime checks pass:

- disconnected bridge shows offline header and state card
- connected bridge without `/map` shows waiting-for-map
- live `/map` and `/pose` activate readiness chips
- layers popover toggles map, costmaps, plan, lidar, and depth obstacle
  visibility without placing a target
- clicking the map selects a target and updates the destination card
- sending a goal changes the state, actions, and progress UI
- active run shows route, progress, distance remaining, and ETA when feedback is
  available
- lidar and depth obstacle overlays either project into the map frame or report
  waiting / no-TF state
- cancel transitions to canceled state and disables the right buttons while
  pending
- clear removes the target only when no active run exists
- success and failure terminal results render correctly
- settings button opens the connection overlay
- history remains visibly non-functional or explicitly “coming soon”

Suggested build verification:

```sh
bun run typecheck
bun run --cwd panels-standalone build
```

### Suggested Components After Step 2

These are useful follow-up components, but not part of the current Step 2 exit
criteria:

- `RunHistoryPanel`
- `GoalDetailsInspector`
- `RuntimeHealthDrawer`

## Step 3: Runtime Wiring

Once the shell and interactions are correct, wire the panel to the extracted
shared Nav2 runtime layer.

This runtime step should consume the existing shared seam:

- `useNav2Runtime.ts`
