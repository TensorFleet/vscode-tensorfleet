# Nav2 Panel Runtime Status

## Scope

This file records the current state of the TurtleBot4 + Nav2 validation work as
of May 6, 2026.

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
- `vacuum_adapter` now exposes backend-neutral mapping capabilities, commands,
  normalized map grid/metadata, and `snapshot.mapping`.
- `Vacuum Control` now starts/pauses/resumes/finishes/discards/accepts mapping
  through adapter commands instead of owning the autonomous loop.
- `MapCanvas` can render the product base map from `snapshot.map.grid` while
  keeping the direct `/map` path available as a diagnostic/fallback render
  source.
- Teleop remains available during manual mapping, paused auto mapping, and
  assistance states; active auto mapping blocks competing teleop until paused.
- Valetudo remains a stub and explicitly reports auto mapping unsupported.

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
- added a VM-owned frontier exploration node for TurtleBot4/Nav2 auto mapping
- the node owns long-running exploration behavior, reads map/pose/TF evidence,
  sends Nav2 goals, publishes `/vacuum_mapping/status`, and exposes
  `/vacuum_mapping/*` command services
- mapping can enter `needs_assistance` for blocked/unreachable exploration,
  stale map, missing pose, repeated navigation failures, or unavailable Nav2
- accepting a reviewed map saves it through `nav2_map_server map_saver_cli`
  under `/opt/tensorfleet/maps/current_map.*` by default
- after a map is accepted, later `/map` growth from normal destination or
  vacuum runs is autosaved after the map settles, as long as SLAM continues to
  publish map updates

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

The main earlier blockers are resolved. The Layer 2 TurtleBot4/Nav2 operator
slice is closed for the simulation path, and Layer 3 adapter hardening is
wrapped.

Where we are in the layer plan:

```text
Layer 0 — Sensors                   validated
Layer 1 — Localization + Map        running
Layer 2 — Navigation                closed for TurtleBot4/Nav2 simulation
Layer 3 — Vacuum Adapter            closed for TurtleBot4/Nav2 simulation,
                                    Valetudo stub reserved for Layer 6
Layer 4 prerequisite: Mapping + Whole Map View
                                    implemented with VM-owned auto mapping
                                    and saved-map load/list plumbing
Layer 4 — Coverage                  Clean Area MVP implemented as
                                    waypoint-based validation; true coverage,
                                    dock, and battery behavior pending
Layer 5 — Room / Zone Semantics     planned
Layer 6 — Real Hardware (Valetudo)  planned
```

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
- a `vacuum_adapter` contract lives under
  `panels-standalone/src/vacuum-adapter/` with public capability, command,
  state, error, and adapter-interface modules plus a `useVacuumAdapter` hook
  that selects a backend (TurtleBot4/Nav2 today)
- the TurtleBot4/Nav2 backend (`useTurtleBot4Nav2Adapter`) wraps
  `useNav2Runtime` and exposes the adapter's `snapshot` + `sendCommand`
  interface, including the explicit mission state machine
  (`idle / navigating / cleaning / paused / returning / charging`)
- TurtleBot4/Nav2 command dispatch now lives in a pure
  `commandDispatcher.ts` helper, so supported and unsupported command behavior
  can be regression-tested without mounting React hooks
- `Vacuum Control` now consumes the adapter through `useVacuumAdapter` instead
  of reading `useNav2Runtime` directly, so navigation state, capabilities,
  readiness evidence, plan path rendering, and send/cancel dispatch all flow
  through the Layer 3 contract
- a Valetudo backend stub now defines the normalized capability mapper, state
  mapper shape, command mapper shape, and runtime boundary types reserved for
  the later Layer 6 integration
- `bun run test:vacuum-adapter` runs a focused contract regression harness for
  the adapter boundary
- the hardened adapter contract has been validated in the VS Code webview
  against the live VM through `Vacuum Control`
- fresh target selection after cancel resets the map marker back to staged
  state and no longer keeps stale canceled plan geometry
- `Vacuum Control` now has the Layer 4 prerequisite map foundation: full known
  `/map` fit by default, explicit fit/manual/follow viewport modes, map
  metadata/readiness labels, adapter-backed mapping state, auto/manual mapping
  commands, review, discard, and map acceptance
- auto mapping is runtime-owned in `~/firecracker-vm`, not owned by React hooks
  or the VS Code webview
- accepting a reviewed TurtleBot4/Nav2 map now attempts VM-side persistence;
  unsupported or failed persistence is reported in `snapshot.mapping`

Validated `Vacuum Control` operator flows:

- connect to the live VM Foxglove endpoint
- render the live occupancy map in the operator map canvas
- select a navigation target from the rendered map
- send a `NavigateToPose` goal through the live Nav2 action API
- observe live progress from Nav2 feedback/status
- cancel an active goal and observe the stopped/canceled state
- observe terminal navigation state after a run completes, fails, or is canceled
- select a new target after cancel and send a second goal without reloading
- exercise layer overlays, sensor overlays, teleop, and failure states through
  the VS Code webview

The Layer 2 operator slice and Layer 3 adapter slice are validated end-to-end
through the single-panel `Vacuum Control` workflow. The panel now also contains
the first Layer 4 Clean Area MVP, implemented above the adapter as a rectangular
lawnmower waypoint workflow. The supporting debug panels remain useful for
runtime validation, but they are no longer the normal path.

The dock issue remains important, but it should now be treated as a runtime
caveat:

- navigation validation must still start from a clear, undocked pose
- starting on or against dock geometry can still make a healthy run look broken
- clear-space validation is still required before treating a failed or stalled
  navigation run as a software failure

Explicitly out of the closed Layer 2/Layer 3 simulation slice:

- Clean Area MVP behavior, true coverage planning, dock / undock workflow, and
  battery-aware execution (Layer 4)
- room / zone naming and segmentation (Layer 5)
- real vacuum hardware and Valetudo integration runtime (Layer 6)

## Layer 2 Status

Layer 2 is closed for the TurtleBot4/Nav2 simulation operator slice.

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

Layer 2 closure is now folded into the closed Layer 3 adapter path:

- `Vacuum Control` consumes the adapter contract rather than raw Nav2 action
  state for product navigation behavior
- live VM goal send, cancel, terminal state, overlay, teleop, and failure
  paths have been validated in the VS Code webview

What remains a runtime caveat:

- blocked starts near the dock can still invalidate a run
- clear-space validation is still required until runtime setup is improved

## Recommended Next Step

The Mapping + Whole Map View prerequisite is implemented, including saved-map
inventory/load state. The next validation focus is the current Clean Area MVP
and the remaining true-coverage gaps.

The mapping milestone makes `Vacuum Control` feel like a robot-vacuum product:

- first valid `/map` opens in a full known map view
- Fit Map returns to the full `/map` occupancy-grid bounds, not the robot,
  selected target, route, or known/free cells only
- the map viewport has explicit `fit`, `manual`, and `follow_robot` modes
- pan, zoom, Fit Map, Follow Robot, and resize behavior keep all overlays aligned
- the UI clearly labels Full known map, Manual view, Following robot, Waiting
  for map, and mostly unexplored states
- mapping can be started in auto or manual mode, paused/resumed, finished for
  review, accepted, or discarded through `vacuum_adapter`
- saved maps can be listed through `snapshot.mapping.savedMaps` and loaded
  through adapter `load_map`
- the autonomous exploration loop lives in the VM runtime and continues across
  panel reloads, UI closure, and websocket reconnects
- manual teleop is available for manual mapping, paused mapping, and
  `needs_assistance` recovery
- true coverage accounting, room segmentation, zone editing, dock UI,
  scheduling, consumables, or Valetudo hardware behavior remains outside this
  milestone; row-level obstacle/unknown clipping has started in Clean Area
- map readiness metadata is visible: dimensions, resolution,
  free/occupied/unknown/known ratios, approximate known area, last update age,
  and pose availability

Acceptance criteria for this prerequisite:

- First valid `/map` displays the full known occupancy grid by default.
- Fit Map always returns to the full known `/map` bounds.
- User zoom and pan do not break map, plan, costmap, lidar, depth, robot, or
  target overlays.
- Follow Robot can be enabled explicitly and manual pan/zoom disables it.
- Auto Mapping starts from `Vacuum Control`, blocks competing teleop while
  active, and shows map growth metadata without fake whole-home progress.
- Manual Mapping starts from `Vacuum Control`, keeps Teleop available, disables
  target staging by default, and shows map growth metadata.
- Finish Mapping switches to review state and fits the full known map.
- Accept Map marks the reviewed map accepted/current and reports whether
  persistence succeeded or remains session-level.
- Discard Mapping Session removes only the UI session state unless a real
  backend reset exists.
- After an accepted map is saved, normal point navigation or vacuum runs update
  the persisted map when SLAM publishes new `/map` data.
- Normal point navigation still works outside Mapping Mode.

Runtime validation checklist after VM rebuild:

- `/vacuum_mapping/status` is published.
- `/vacuum_mapping/start_auto`, `/pause`, `/resume`, `/finish`, `/discard`,
  `/accept`, and `/save_map` are present.
- start auto mapping, close/reload the UI, reconnect, and confirm exploration
  continues from VM state.
- pause, teleop, resume, finish review, and accept.
- accepted maps appear under `/opt/tensorfleet/maps/current_map.*`.
- repeated failures enter `needs_assistance`.

Current Clean Area MVP:

- `Vacuum Control` has `Mapping`, `Navigate`, and `Clean Area` modes.
- `MapCanvas` supports drawing, moving, and resizing a rectangular clean-area
  selection.
- the selection is validated against map bounds and occupancy data.
- `VacuumControlPanel.tsx` generates a simple lawnmower waypoint preview from
  the selected rectangle.
- route generation clips sampled rows to known free cells in the normalized
  occupancy grid instead of sending full-width rows through blocked or unknown
  cells.
- the run dispatches each waypoint through `adapter.sendCommand({ command:
  "go_to_location", ... })`, keeping execution above the adapter boundary.
- the UI exposes preparing, running, paused, canceling, completed, failed, and
  canceled states with pause, cancel, retry waypoint, skip waypoint, and clear
  controls.
- mode switching prevents mapping, point navigation, and clean-area runs from
  conflicting with each other.

Treat this as waypoint-based coverage validation, not proof that every cell was
cleaned. A later true coverage pass must account for cleaning swath width and
overlap, edge/corner coverage, full obstacle/unknown-space decomposition inside
the selected area, and progress from actual robot footprint history instead of
waypoint count alone.

Layer 3 now has a working and live-validated TurtleBot4/Nav2 adapter, a mission
state machine, `Vacuum Control` consumption through `useVacuumAdapter`, focused
command/contract regression coverage, and a Valetudo non-hardware interface
stub reserved for Layer 6. Layer 4 (coverage) and later layers must stay above
this adapter boundary.

More TurtleBot4 UI polish and supporting debug panel cleanup can happen, but
they are no longer the main path.

Layer 3 should define the stable product-facing capability, state, and command
boundary above robot backends. It should not expose raw TurtleBot4 topics, Nav2
internals, or Valetudo-specific class names to product/UI clients.
Public `vacuum_adapter` contract files should own their state and command shape
types. Backend/runtime types such as Nav2 goal status or panel-specific pose
helpers belong inside backend mappers only.
Valetudo references for this work (informing the capability shape for the
later Layer 6 integration):

- [Capabilities overview](https://valetudo.cloud/pages/usage/capabilities-overview.html)
- [MQTT implementation details](https://valetudo.cloud/pages/development/mqtt/)
- [Valetudo project](https://github.com/Hypfer/Valetudo)

Completed Layer 3 hardening in the current pass:

1. Added a contract regression harness for `vacuum_adapter`.
   - Supported TurtleBot4/Nav2 commands are covered:
     - `go_to_location`
     - `cancel_navigation`
   - Explicit unsupported TurtleBot4/Nav2 command results are covered:
     - `start_cleaning`
     - `pause`
     - `resume`
     - `return_to_dock`
     - `segment_cleaning`
     - `zone_cleaning`
     - `set_fan_speed`
     - `set_water_usage`
   - The harness checks that `Vacuum Control` branches on capabilities and
     normalized state, not backend name.
2. Audited the public contract files through the harness.
   - Public adapter types do not import Nav2/runtime/panel-specific types.
   - Nav2 goal status, raw ROS messages, and helper shapes stay inside backend
     mappers.
   - Setter command payloads remain explicit.
3. Expanded the Valetudo backend interface stub.
   - No real hardware integration is implemented yet.
   - The stub defines the normalized capability map, state mapper shape,
     command mapper shape, and connection/runtime boundary.
   - Valetudo class names remain private to the backend mapper layer.

Verification command set:

   ```sh
   cd ~/vscode-tensorfleet
   bun run test:vacuum-adapter
   bun run typecheck
   bun run --cwd panels-standalone build
   ```

Layer 3 exit validation completed:

1. Automated verification passes.
   - `bun run test:vacuum-adapter`
   - `bun run typecheck`
   - `bun run --cwd panels-standalone build`
2. Live VS Code webview validation passes.
   - Connect to Foxglove.
   - Open `Vacuum Control`.
   - Render `/map`.
   - Select a target.
   - Send a goal.
   - Cancel a goal.
   - Confirm terminal state.
   - Select a fresh target after cancel.
   - Confirm stale canceled marker and stale plan geometry are cleared.
   - Confirm failure paths and unsupported command behavior are explicit and
     predictable.

Layer 4 has started with the Clean Area MVP. Further coverage logic must keep
consuming normalized adapter state and commands instead of raw backend topics.

Out of Layer 3 (defer to later layers):

- Clean Area MVP, true coverage planning, and "clean this area" behavior
  (Layer 4)
- dock / undock and battery-aware execution beyond what the contract models
  (Layer 4)
- room / zone naming and segmentation UI (Layer 5)
- VM service plan, MQTT vs HTTP choice, and real-hardware validation for
  Valetudo (Layer 6)

Supporting follow-up work should be tracked separately and can include:

- supporting debug surface polish for Raw Messages and 3D Sensor View
- clear-space / undocked-start validation improvements
- runtime regression checks for the closed Layer 2/Layer 3 operator flow

Do not make these the next milestone:

- more TurtleBot4 operator-panel feature work
- OpenClaw integration
- room cleaning UI
- docking UI
- zone cleaning UI
- consumables UI
- scheduling UI

## Layer Roadmap

The stack is built in seven layers. Each layer is stable before the next one
starts.

```text
Layer 0 — Sensors                   validated
Layer 1 — Localization + Map        running
Layer 2 — Navigation                closed for TurtleBot4/Nav2 simulation
Layer 3 — Vacuum Adapter            closed for TurtleBot4/Nav2 simulation,
                                    Valetudo stub reserved for Layer 6
Layer 4 prerequisite: Mapping + Whole Map View
                                    implemented for TurtleBot4/Nav2
                                    simulation
Layer 4 — Coverage                  Clean Area MVP implemented as
                                    waypoint-based validation; true coverage,
                                    dock, and battery behavior pending
Layer 5 — Room / Zone Semantics     planned
Layer 6 — Real Hardware (Valetudo)  planned
```

### Layer 0: Sensors

Status: validated.

Meaning:

- TurtleBot4 simulation boots in the VM and publishes sensor data
- camera, lidar, and depth point clouds are visible in the extension
- `CameraOverlay` in `Vacuum Control` proves the camera path end-to-end

### Layer 1: Localization + Map

Status: running.

Meaning:

- SLAM Toolbox runs in the VM and builds the map as the robot moves
- `/map` is published and consumed by `Vacuum Control`
- the TF tree `map -> odom -> base_link` is continuously available
- `/odom`, `/tf`, and `/tf_static` are stable enough for overlay projection

Layer 1 is the foundation Layer 2 navigation and every higher layer assumes.

### Layer 2: Navigation

Status: closed for TurtleBot4/Nav2 simulation.

Meaning:

```text
Nav2 is running with planner + controller + costmaps.
NavigateToPose works end-to-end.
The extension can send a goal, see progress, and see the terminal result.
```

Validated flow through `Vacuum Control`:

```text
connect -> render map -> select target -> send goal -> observe progress/result
-> cancel/clear/retry as needed
```

Backend:

- TurtleBot4 simulation in the VM
- Nav2 action runtime
- Foxglove bridge used by the extension

Remaining runtime caveat:

- dock-blocked starts still require clear-space validation

### Layer 3: Vacuum Adapter

Status: closed for TurtleBot4/Nav2 simulation; Valetudo backend interface stub
reserved for Layer 6.

Purpose:

- normalize TurtleBot4/Nav2 runtime into a `vacuum_adapter` contract with
  `VacuumState` and `VacuumCommands` shapes
- make the extension talk to the contract, not raw ROS topics
- introduce a mission state machine covering `idle / navigating / cleaning /
  paused / returning / charging`
- support TurtleBot4/Nav2 as the first backend and reserve the Valetudo
  backend for Layer 6

What now exists in the repo:

- a public `VacuumAdapter` surface with `snapshot` + `sendCommand(command)` and
  explicit `VacuumMissionStatus`, `VacuumNavigationStatus`, `VacuumReadiness`,
  and capability descriptor types
- a TurtleBot4/Nav2 backend hook (`useTurtleBot4Nav2Adapter`) that wraps
  `useNav2Runtime`, reports mission state as `idle`, `navigating`, or
  `mapping` for the currently implemented TurtleBot4/Nav2 flows, maps
  `go_to_location` to the Nav2 `NavigateToPose` action, and rejects vacuum-only
  commands (`start_cleaning`, `pause`, `resume`, `return_to_dock`, etc.) with
  `unsupported`
- a normalized `VacuumPathPoint[]` plan path on the adapter snapshot so the
  map canvas consumes a backend-neutral path shape instead of the raw
  `nav_msgs/msg/Path` message
- normalized `snapshot.map.grid`, `snapshot.map.metadata`, and
  `snapshot.mapping` surfaces for adapter-backed mapping workflows
- saved-map inventory and load state through `snapshot.mapping.savedMaps`,
  `activeMapName`, `loadedMapPath`, `loadError`, and the `load_map` command
- a `useVacuumAdapter` entry point that picks the backend (only TurtleBot4/Nav2
  wired today), plus Valetudo capability, state, command, and runtime-boundary
  mapper stubs reserved for Layer 6

Current contract boundary rule:

- `vacuum_adapter` owns public capability, state, and command types
- backend adapters map backend/runtime-specific types into those public types
- public command payloads must be explicit; setter commands should not be
  represented as payload-free simple commands
- diagnostic backend state may be carried as strings, but product logic should
  branch on normalized adapter state and capabilities

The TurtleBot4/Nav2 backend can initially support:

- `map`
- `pose`
- `go_to_location`
- `cancel_navigation`
- `manual_control`
- `navigation_status`
- `mapping_session`
- `auto_mapping`

It should explicitly report unsupported vacuum features such as:

- `start_cleaning`
- `segment_cleaning`
- `zone_cleaning`
- `fan_speed`
- `water_usage`
- `consumables`
- real dock behavior

### Layer 4 Prerequisite: Mapping + Whole Map View

Status: implemented for TurtleBot4/Nav2 simulation.

Purpose:

- build, view, review, accept, discard, list, and load maps before true
  coverage cleaning
- keep autonomous exploration in the VM runtime, not in React/webview state
- expose mapping commands and status through `vacuum_adapter`
- persist accepted maps when the backend supports saving

Current TurtleBot4/Nav2 behavior:

- VM runtime publishes `/vacuum_mapping/status` and owns the frontier
  exploration loop.
- Adapter commands map to VM services for start auto/manual, pause, resume,
  finish, discard, accept, save, and load.
- `finish_mapping` stops exploration and enters review.
- `accept_map` marks the reviewed map current and attempts to save
  `/opt/tensorfleet/maps/current_map.*`.
- After a map is accepted, later `/map` updates from normal destination or
  vacuum runs are autosaved after a quiet interval while SLAM remains active.
- `needs_assistance` is first-class for blocked motion, unreachable frontiers,
  stale map, missing pose, repeated failures, or unavailable Nav2.
- saved-map inventory is carried in `snapshot.mapping.savedMaps`, and loading a
  saved map is dispatched through `load_map`.

### Layer 4: Coverage

Status: Clean Area MVP implemented as waypoint-based validation; true coverage,
dock / undock, and battery-aware behavior pending.

Purpose:

- add a coverage path planner that produces a lawnmower pattern over a bounded
  region; the current MVP does this as an in-panel waypoint sequence
- teach the adapter about dock / undock behavior and battery awareness
- make "clean this area" work end-to-end in simulation through the adapter

Current Clean Area MVP:

- `MapCanvas` supports rectangular clean-area selection with draw, move, and
  resize interactions.
- `VacuumControlPanel.tsx` generates a lawnmower waypoint preview and executes
  it by sending adapter `go_to_location` commands.
- `cleanAreaPlanner.ts` owns route generation and clips sampled rows to known
  free occupancy-grid cells.
- Clean-area UI state covers editing, confirmed, preparing, running, paused,
  canceling, completed, failed, and canceled.
- Operators can pause, cancel, retry a failed waypoint, skip a waypoint, and
  clear the area when a run is inactive.
- Mapping, navigation, and clean-area modes are mutually locked while a
  conflicting workflow is active.

Constraints:

- the planner must consume the adapter's normalized map and pose, not raw ROS
  topics
- coverage should succeed or fail through the adapter's navigation and mission
  state surfaces
- Layer 4 has started because Layer 3 is real, and coverage logic should still
  never touch backend-specific topics directly
- the current Clean Area MVP is not true coverage accounting; footprint-history
  progress, swath/overlap, edge/corner handling, and complete area
  decomposition around obstacles or unknown cells remain future work

### Layer 5: Room / Zone Semantics

Status: planned.

Purpose:

- divide the map into named zones / rooms
- translate "clean room 3" into one or more coverage goals for Layer 4
- expose room / zone state through the adapter's normalized state
- complete the full vacuum workflow end-to-end in simulation

Constraints:

- room / zone naming is product-facing and must not live in backend topics
- segmentation logic may consume backend input (for example, Valetudo zones)
  but the public shape must be backend-neutral
- Layer 5 should be demonstrable in simulation before Layer 6 begins

### Layer 6: Real Hardware (Valetudo)

Status: planned.

Purpose:

- ship a Valetudo backend that implements the same `vacuum_adapter` contract
  the TurtleBot4/Nav2 backend uses
- swap the TurtleBot4 simulation for a real Valetudo-compatible vacuum with no
  changes above the adapter
- run the Valetudo integration runtime in the VM so users do not need to
  install local integration tooling
- keep `vacuum_adapter` as the product contract outside the VM runtime details

Important wording:

- say "Valetudo integration runtime in the VM"
- do not imply that the VM replaces the Valetudo instance on the robot

Target path:

```text
real vacuum running Valetudo
  -> VM-managed Valetudo integration client / MQTT broker / adapter service
  -> vacuum_adapter
  -> extension / product UI
```

First Layer 6 validation path should be basic:

```text
Valetudo robot reachable
-> VM receives status / capabilities
-> adapter normalizes state
-> extension displays capability / state summary
-> one basic command works
```

Good first commands for Layer 6:

- start
- pause
- stop
- return_to_dock

Only after that basic reachability slice works should Layer 6 exercise the
Layer 4 coverage and Layer 5 room / zone flows against real hardware.

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

The product-facing structure and visual language are now locked for the
closed Layer 2 operator slice.

`WarningCard` remains deferred. It is not part of the closed Layer 2
operator slice.

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
- mode switcher for `Mapping`, `Navigate`, and `Clean Area`
- `MappingCard` with auto/manual mapping, review/accept/discard, saved-map
  inventory, and load controls
- `CleanAreaCard` with rectangle selection, lawnmower waypoint preview,
  progress, pause, cancel, retry waypoint, skip waypoint, and clear controls
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
- `panels-standalone/src/vacuum-adapter/useVacuumAdapter.ts`
- `panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/useTurtleBot4Nav2Adapter.ts`
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
- clean-area selection overlays use the same world-to-screen transform as the
  base map, robot marker, navigation target, and route
- clean-area route preview renders completed/current/remaining waypoint
  segments for the active MVP run state

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
- mode locking prevents mapping, point navigation, and clean-area runs from
  competing with each other
- Clean Area MVP progress is waypoint-based; it is not yet true cell-coverage
  progress from robot footprint history

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

## Layer 2 Closure Checklist

The following runtime checks define the Layer 2 operator slice and are the
basis for regression testing the closed Layer 2/Layer 3 operator path:

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

### Suggested Components After Layer 2

These are useful follow-up components, but not part of the closed Layer 2
operator slice. They sit above the Layer 3 adapter contract once it exists:

- `RunHistoryPanel`
- `GoalDetailsInspector`
- `RuntimeHealthDrawer`
