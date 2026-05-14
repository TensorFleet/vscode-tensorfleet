# Vacuum Stack Plan

## Purpose

This document is the planning source of truth for the vacuum stack.

It covers both:

- the long-term product and architecture direction; and
- the current TurtleBot4-based simulation and VM integration reality.

## Goal

Build the software stack for a future robot vacuum before buying hardware.

The near-term goal is:

- a simulated robot appears in our system;
- it is controllable through ROS 2;
- it can navigate in simulation;
- the VS Code extension and related panels are useful operator surfaces;
- we avoid coupling product behavior to TurtleBot4-specific APIs.

The longer-term goal is:

- a vacuum-facing contract exists above the robot backend;
- docking, pause/resume, room cleaning, battery-aware execution, and mission
  state fit naturally into that contract;
- the same higher-level product logic can later move from TurtleBot4
  simulation to a real vacuum backend with minimal churn.

TurtleBot4 is not the product. It is the first development backend.

## Product Boundary

TurtleBot4 is the first development backend.

The product-facing boundary is a repo-owned vacuum contract.

Backend differences must surface through:

- capabilities;
- normalized state; and
- explicit unsupported operations.

No client above the adapter may depend on TurtleBot4-specific topics, helper
services, or node APIs. TurtleBot4-specific helpers such as
`TurtleBot4Navigator` are useful backend implementation details because they
add docking and undocking behavior on top of Nav2, but they must stay private
to the TurtleBot4 backend and must not define the public contract. See the
current TurtleBot4 docs for navigation and `TurtleBot4Navigator` behavior:
[Navigation](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html),
[TurtleBot 4 Navigator](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/turtlebot4_navigator.html).

## Planning Decisions

These are the current working decisions unless we find a concrete reason to
change them:

- ROS 2 is the foundation.
- TurtleBot4 is the first backend, not the public product contract.
- The public boundary should be a repo-owned vacuum-facing interface, not raw
  TurtleBot4 topics.
- Layers are built in order: sensors first, then localization + map, then
  navigation, then the vacuum adapter contract, then coverage, then room/zone
  semantics, and finally real hardware.
- Current position: Layer 4 prerequisite (Mapping + Whole Map View) is
  implemented for TurtleBot4/Nav2 simulation with VM-owned auto mapping and
  saved-map load/list plumbing.
- Clean Area MVP exists above the adapter: the UI selects a bounded region,
  previews occupancy-clipped swath-overlap lawnmower waypoints, executes each
  waypoint through `go_to_location`, and reports footprint-history coverage
  progress over normalized map cells.
- Coverage now uses a product-level coverage profile for swath width, overlap,
  navigation goal tolerance, boundary margin, minimum useful region size, and
  completion threshold. The default profile preserves the TurtleBot4 simulation
  behavior.
- Boundary pass goals are extended from the configured navigation tolerance
  plus boundary margin, and the current UI treats 95% covered cleanable cells
  as the completion threshold.
- Connected cleanable-region decomposition has started for coverage accounting
  and tiny-region skipping. Component-level route planning, stronger
  edge/corner behavior, dock / undock execution, and battery-aware
  return/resume are still Layer 4 follow-ups.
- Coverage (Layer 4) and room/zone semantics (Layer 5) must sit above the
  adapter contract, not below it, so they stay backend-neutral.
- Real hardware (Layer 6) is the last layer; it targets Valetudo-compatible
  vacuums and reuses the same adapter contract and UI that were validated in
  simulation.
- The first real-hardware path should target Valetudo-compatible vacuums.
- Valetudo should influence the capability model from the start, but it should
  not define the public contract.
- Valetudo-specific capability names/classes should stay inside the Valetudo
  backend adapter.
- Capabilities should be descriptors with support status, source, backend
  mapping, commands, attributes, and notes, not simple booleans only.
- Product/UI clients must branch on capability flags, not backend names.
- Public `vacuum_adapter` contract files should own backend-neutral state and
  command types; backend/runtime-specific imports belong inside backend mappers.
- Setter commands should have explicit payload shapes rather than being modeled
  as payload-free simple commands.
- Long-running robot behavior belongs in the VM runtime or backend process, not
  React/webview hooks. UI and adapter code issue commands and render state.
- Auto mapping is exposed through backend-neutral capabilities and commands:
  `mapping_session`, `auto_mapping`, `start_mapping`, `pause_mapping`,
  `resume_mapping`, `finish_mapping`, `discard_mapping`, and `accept_map`.
- `finish_mapping` stops exploration and enters review. `accept_map` marks the
  reviewed map current and reports whether persistence succeeded or remains
  session-level.
- The VM should eventually run the Valetudo integration runtime so users do not
  need to install local integration tooling.
- The VM may host MQTT/client/discovery/adapter services, but it should not own
  the public `vacuum_adapter` contract.
- Docs should say "Valetudo integration runtime in the VM," not imply that
  Valetudo itself necessarily runs in the VM.
- First Valetudo hardware validation should prove reachability, status,
  capabilities, normalized state, and one basic command before room/zone
  workflows.
- The contract should reuse standard ROS 2 and Nav2 interfaces where they
  already fit.
- Simulation should be realistic enough to validate workflow, not just expose
  `cmd_vel`.
- Simulation should become the regression harness for later hardware work.
- The system should be local-first.

## Current Layer Structure

The stack is built bottom-up in seven layers. Each layer is stable before the
next one starts, and every higher layer depends on the lower ones being
trustworthy.

```text
Layer 0 — Sensors                   (validated)
Layer 1 — Localization + Map        (running)
Layer 2 — Navigation                (closed for TurtleBot4/Nav2 simulation)
Layer 3 — Vacuum Adapter            (closed for TurtleBot4/Nav2 simulation,
                                     Valetudo stub reserved for Layer 6)
Layer 4 prerequisite: Mapping + Whole Map View
                                     (implemented for TurtleBot4/Nav2
                                      simulation)
Layer 4 — Coverage                  (Clean Area MVP implemented with
                                     occupancy-clipped waypoints and
                                     footprint-history progress; production
                                     coverage, dock, and battery behavior
                                     pending)
Layer 5 — Room / Zone Semantics     (planned)
Layer 6 — Real Hardware (Valetudo)  (planned)
```

### Layer 0: Sensors

Status: validated.

Meaning:

- the simulated TurtleBot4 boots reliably in the VM;
- camera, lidar, and depth point cloud streams are visible in the extension;
- the robot exists and publishes sensor data.

### Layer 1: Localization + Map

Status: running.

Meaning:

- SLAM Toolbox runs in the VM and builds a map as the robot moves;
- the robot knows where it is in that map;
- the TF tree `map -> odom -> base_link` is continuously available;
- `/map`, `/odom`, `/tf`, and `/tf_static` are observable from the extension.

Layer 1 is the foundation every higher layer assumes. Coverage and room/zone
semantics especially depend on a stable map frame and consistent pose.

### Layer 2: Navigation

Status: closed for TurtleBot4/Nav2 simulation.

Meaning:

```text
Nav2 is running with planner + controller + costmaps.
NavigateToPose works end-to-end.
The extension can send a goal, see progress, and see the terminal result.
```

Validated operator flow through `Vacuum Control`:

```text
connect -> render map -> select target -> send goal -> observe progress/result
-> cancel/clear/retry as needed
```

Main surface:

- `Vacuum Control`

Backend:

- TurtleBot4 simulation in VM
- Nav2 runtime
- Foxglove bridge

Remaining runtime caveat:

- dock-blocked starts still require clear-space validation;

### Layer 3: Vacuum Adapter

Status: closed for TurtleBot4/Nav2 simulation; Valetudo backend interface stub
reserved for Layer 6.

Purpose:

- normalize TurtleBot4/Nav2 runtime behavior into a `vacuum_adapter` contract
  with `VacuumState` and `VacuumCommands` shapes;
- make the extension talk to the contract, not raw ROS topics;
- introduce an explicit mission state machine covering
  `idle / navigating / cleaning / paused / returning / charging`;
- support a TurtleBot4/Nav2 backend adapter first and a Valetudo backend
  adapter second;
- make clients branch on capability descriptors, not backend names;
- keep the public contract types backend-neutral while backend adapters
  normalize Nav2, Valetudo, or other runtime details.

Layer 3 is the first layer that product clients depend on. Layers 4, 5, and 6
all assume this contract already exists.

Current truth (May 14, 2026):

- the public `VacuumAdapter` surface and a `useVacuumAdapter` hook live under
  `panels-standalone/src/vacuum-adapter/`;
- a TurtleBot4/Nav2 backend (`useTurtleBot4Nav2Adapter`) wraps
  `useNav2Runtime` and emits the normalized snapshot, mission state, and
  command dispatcher;
- TurtleBot4/Nav2 command dispatch is extracted into a pure helper so
  supported commands and explicit unsupported failures can be tested without
  mounting React hooks;
- `Vacuum Control` reads snapshot fields and dispatches `go_to_location` and
  `cancel_navigation` through `adapter.sendCommand(...)` instead of calling
  Nav2 runtime methods directly;
- vacuum-only commands (`start_cleaning`, `pause`, `resume`, `return_to_dock`,
  `segment_cleaning`, `zone_cleaning`, `set_fan_speed`, `set_water_usage`)
  are advertised as unsupported by the TurtleBot4/Nav2 adapter and fail
  explicitly through the contract;
- the mission state machine reports `idle` or `navigating` on TurtleBot4/Nav2
  and keeps the remaining states available for later backends;
- `bun run test:vacuum-adapter` covers capability coverage, TurtleBot4/Nav2
  command dispatch, unsupported command failures, normalized plan path mapping,
  mission state mapping, public contract import boundaries, and backend-name
  branching in `VacuumControlPanel.tsx`;
- the Valetudo backend stub now defines capability, state, command, and
  runtime-boundary mapper shapes for the Layer 6 integration to populate.
- live VS Code webview validation confirms connection, map rendering, target
  selection, goal send, goal cancel, terminal states, failure paths, overlays,
  teleop, and a second goal after cancel without reloading.
- fresh target selection after cancel resets marker state and clears stale
  canceled plan geometry.

### Layer 4 Prerequisite: Mapping + Whole Map View

Status: implemented for the TurtleBot4/Nav2 simulation path.

Purpose:

- make the first valid `/map` display as the full known occupancy-grid bounds;
- provide an explicit map viewport model with `fit`, `manual`, and
  `follow_robot` modes;
- keep the base map, costmaps, plan path, lidar/depth overlays, robot marker,
  and target marker aligned through one world-to-screen transform;
- give the operator auto and manual mapping workflows before coverage cleaning;
- show whether the current map is missing, active, mostly unknown, in review,
  or accepted for later navigation/coverage.
- keep autonomous exploration in the VM runtime so it survives panel reloads,
  UI closure, websocket reconnects, and extension hiccups.

Current implementation:

- `vacuum_adapter` exposes `snapshot.map.grid`, `snapshot.map.metadata`, and
  `snapshot.mapping` without ROS/Nav2/TurtleBot4/Valetudo imports in the public
  contract.
- `MapCanvas` renders the product base map from the adapter-normalized grid
  when available and keeps live `/map` as a fallback/diagnostic path.
- `MapCanvas` still renders `/global_costmap/costmap`,
  `/local_costmap/costmap`, lidar, depth obstacles, route overlays, markers,
  and camera overlay.
- Fit Map uses the full occupancy-grid dimensions and panel size with padding;
  it does not fit to robot pose, route geometry, target position, costmaps, or
  known/free cells only.
- First valid `/map` enters Full known map view. If map dimensions or panel
  size change while still in fit mode, the canvas refits. Manual pan/zoom keeps
  the user's viewport. Follow Robot recenters around the robot pose until the
  user pans, zooms, or fits.
- Map controls expose Zoom In, Zoom Out, Fit Map, Follow Robot, zoom readout,
  and a view-state label such as Full known map, Manual view, Following robot,
  Waiting for map, or Map mostly unexplored.
- `snapshot.mapping.state` reports `idle`, `manual_mapping`, `auto_mapping`,
  `paused`, `needs_assistance`, `review`, `accepted`, `discarded`, or `error`.
- Mapping status reports known/unknown ratios, frontier count, visited and
  failed goal counts, active goal, state reason, timestamps, and persistence
  outcome instead of a fake whole-home progress percentage.
- `MappingCard` shows Start auto mapping, Manual mapping, Pause, Resume auto
  mapping, Finish & review, Discard, and Accept map flows with map metadata.
- Map metadata includes dimensions, resolution, known/free/occupied/unknown
  ratios, approximate known area, last update age, pose availability, and a
  simple readiness label.
- During mapping/review states, map clicks do not stage navigation targets by
  default.
- Teleop is available during manual mapping, paused auto mapping, and
  `needs_assistance`; active auto mapping blocks competing teleop until paused.
- Accept map asks the backend to mark the reviewed map current. The
  TurtleBot4/Nav2 VM runtime attempts to persist it through
  `nav2_map_server map_saver_cli` under `/opt/tensorfleet/maps/current_map.*`
  by default and reports save path/error state through `snapshot.mapping`.
- After an accepted map is saved, later `/map` growth from normal destination
  or vacuum runs is autosaved after a quiet interval, as long as SLAM remains
  active.
- `snapshot.mapping.savedMaps`, `activeMapName`, `loadedMapPath`, and
  `loadError` expose saved-map inventory and load state.
- The TurtleBot4/Nav2 adapter maps `load_map` through the VM mapping runtime;
  `MappingCard` can load a saved map back into the current mapping/navigation
  context.

Boundary rule:

- UI rendering may keep direct diagnostic subscriptions for overlays and
  fallback visualization.
- Product workflows should consume adapter-level map/session state.
- Future coverage planning must consume normalized map and pose data above the
  adapter boundary, not become coupled to raw ROS topics.

Still out of scope for this prerequisite:

- production-complete coverage behavior beyond the current Clean Area MVP;
- configurable production coverage semantics;
- room segmentation;
- zone/no-go editors;
- dock UI;
- battery-aware return/resume;
- Valetudo hardware integration;
- scheduling and consumables.

### Layer 4: Coverage

Status: Clean Area MVP implemented with adapter-backed waypoint execution,
occupancy-grid lane clipping, footprint-history coverage progress, a
profile-backed coverage configuration, connected-region accounting, and a 95%
covered-cell completion threshold. Component-level area route planning, stronger
edge/corner behavior, dock / undock execution, and battery-aware execution are
still pending.

Purpose:

- add a coverage path planner that produces a lawnmower pattern over a region;
- teach the adapter about dock / undock behavior and battery awareness;
- make "clean this area" work end-to-end in simulation through the adapter.

Current Clean Area MVP:

- `Vacuum Control` has `Mapping`, `Navigate`, and `Clean Area` modes.
- `MapCanvas` supports drawing, moving, and resizing a rectangular clean-area
  selection on the normalized map viewport.
- The clean-area selection is validated against map bounds and occupancy data
  before a run can start.
- `cleanAreaProfile.ts` owns the current coverage profile: swath width,
  overlap, navigation goal tolerance, boundary margin, minimum useful region
  size, completion threshold, and derived lane spacing / boundary extension.
- `cleanAreaPlanner.ts` builds a profile-backed swath-overlap lawnmower
  waypoint preview from the selected rectangle, chooses the longer axis for
  passes, and keeps lane spacing at or below the configured swath.
- Boundary pass endpoints are extended by the profile-derived boundary
  extension so close-enough goal tolerance does not advance to the next
  waypoint before the robot physically crosses the clean-area edge.
- Clean-area route generation now uses the normalized occupancy grid to clip
  each sampled lane to known free cells instead of blindly sweeping through
  occupied or unknown cells.
- The run executes each waypoint through `adapter.sendCommand({ command:
  "go_to_location", ... })`, so it stays above the adapter boundary.
- `cleanAreaCoverage.ts` computes cleanable target cells from the selected
  rectangle, excluding occupied, unknown, and out-of-bounds cells from the
  coverage denominator.
- `cleanAreaCoverage.ts` decomposes cleanable cells into connected regions and
  marks tiny disconnected regions as skipped, so the coverage target is no
  longer just a flat rectangle of free cells.
- Clean-area progress is now based on covered square meters: robot pose history
  in map frame is treated as the configured cleaning swath, and cleanable cells
  touched by that swath become covered.
- Confirmed clean-area runs freeze the coverage target and render coverage
  cells from world-space bounds so live map/grid updates do not make already
  covered cells disappear during a run.
- `MapCanvas` renders a coverage overlay for remaining target cells, covered
  cells, excluded occupied/unknown cells, and the current robot footprint.
- `CleanAreaCard` shows an end-user cleaning summary: coverage percentage,
  cleaned area, remaining area, skipped area, and simple route status.
- Clean Area mode includes a capability-driven mission lifecycle card for dock,
  return-to-dock, battery, and charging state. Unsupported operations remain
  explicit through adapter capabilities.
- The UI exposes preparing, running, paused, canceling, completed, failed, and
  canceled states; it also supports pause, cancel, retry waypoint, skip
  waypoint, and clear.
- Mapping, navigation, and clean-area mode switches lock each other while a
  conflicting workflow is active.
- Teleop remains available outside active clean-area runs and is disabled while
  the waypoint run is active.

MVP limits:

- this proves adapter-backed waypoint execution plus first-pass physical
  coverage accounting over selected free cells;
- clipping is lane-level waypoint clipping; connected-region accounting exists,
  but route generation is not yet component-level area planning;
- stronger edge/corner coverage, obstacle-adjacent behavior, dock / undock
  execution, and battery-aware return/resume remain follow-up work.

Layer 4 should be implemented above the `vacuum_adapter` contract so the same
coverage logic works for any backend that advertises the required capabilities.

### Layer 5: Room / Zone Semantics

Status: planned.

Purpose:

- divide the map into named zones / rooms;
- translate "clean room 3" into one or more coverage goals;
- make the full vacuum workflow work end-to-end in simulation;
- keep zone naming and segmentation in product-facing state, not in backend
  topics.

Layer 5 depends on Layer 4 (coverage) and Layer 3 (adapter contract). It is
where simulation reaches product-complete behavior.

### Layer 6: Real Hardware (Valetudo)

Status: planned.

Purpose:

- ship a Valetudo backend that implements the same `vacuum_adapter` contract
  the TurtleBot4/Nav2 backend uses;
- swap the TurtleBot4 simulation for a real Valetudo-compatible vacuum with no
  changes above the adapter;
- keep the same extension, same UI, and same workflows — only the backend
  changes.

Layer 6 also owns the VM-managed real-vacuum runtime:

- make Valetudo-backed hardware usable without local user setup;
- host the Valetudo integration client, MQTT / client service, discovery /
  config, backend adapter process, and health checks in the VM as needed;
- keep `vacuum_adapter` as the product contract outside the VM runtime
  details.

## Success Criteria

### First usable vertical slice (Layers 0–2)

- one simulated robot appears in our system;
- the robot boots reliably in the VM;
- pose, map, and camera are visible;
- a ROS client can send a navigation goal and observe progress and result;
- the system uses standard ROS 2 / Nav2 interfaces where they fit;
- the path toward a later vacuum-specific abstraction stays clean.

### Adapter slice (Layer 3)

- the extension operates the robot through `vacuum_adapter` rather than raw
  ROS topics;
- a TurtleBot4/Nav2 backend normalizes into `VacuumState` and `VacuumCommands`;
- unsupported vacuum capabilities are advertised as unsupported rather than
  hidden;
- the mission state machine carries `idle / navigating / cleaning / paused /
  returning / charging` explicitly.

### Simulation-complete slice (Layers 4–5)

- Clean Area MVP can execute a selected rectangular lawnmower waypoint sequence
  through the adapter;
- row-level clean-area waypoint generation clips sampled rows to known free
  cells in the occupancy grid;
- production coverage cleaning accounts for robot footprint history,
  configurable swath width, overlap, edge/corner handling, and full
  obstacle/unknown-space decomposition;
- dock / undock and battery-aware behavior are visible through the adapter
  state;
- the map is divided into named zones and "clean room 3" translates into
  coverage goals;
- the full vacuum workflow is demonstrated in simulation without leaking
  backend-specific details to the UI.

### Full platform success (Layer 6)

- the same client-facing vacuum contract works against TurtleBot4 simulation
  and a real vacuum backend;
- a Valetudo backend implements the same contract and the extension / UI run
  unchanged against real hardware;
- backend differences are expressed through capability flags and explicit
  unsupported operations, not product forks;
- docking, mission lifecycle, battery state, and room / zone workflows fit the
  same contract;
- simulation acts as the regression harness for real hardware work.

## Non-Goals For The First Milestone

- perfect simulation of brushes, dust bin, water tank, or consumables;
- support for every vacuum vendor;
- exact reproduction of a commercial vacuum UI;
- hardware procurement decisions;
- forcing OpenClaw to be the foundation before the robot stack is stable.

## Why TurtleBot4

TurtleBot4 is a good stand-in because it already gives us most of the surfaces
needed to validate the robotics side of the platform:

- differential-drive motion;
- lidar, IMU, odometry, and camera streams;
- hazard and contact-related signals;
- docking-related topics;
- strong ROS 2 compatibility;
- a simulation path we already control inside this repo and VM environment.

TurtleBot4 is still not a vacuum. It does not natively model:

- cleaning coverage;
- room and zone semantics;
- suction or water modes;
- consumables or maintenance state;
- vendor-specific charging, mapping, and docking behavior.

Those concepts should be introduced in repo-owned layers above the robot
backend, not leaked directly into UI or product logic.

## Product And Operator Positioning

The system should first work as a deterministic, local ROS 2 vacuum platform.

Current positioning:

- the VS Code extension should be treated as an important part of the platform,
  not just a debugging extra;
- the VS Code extension and related panels should be the main first-mile UI
  for bringup and validation;
- TurtleBot4 should be treated as the development harness;
- OpenClaw may become useful later, but should sit above a stable
  vacuum-facing contract rather than directly on TurtleBot4 internals.

Current extension truth for the closed Layer 2/Layer 3 simulation slice:

- Layers 0 and 1 (sensors, SLAM map + TF) are validated through the extension
  as the foundation every higher layer assumes
- the Layer 2 TurtleBot4/Nav2 operator slice is closed as of April 30, 2026,
  and the Layer 3 adapter contract is closed for the simulation path
- `Vacuum Control` is the validated operator surface for this slice
- the panel is map-first and consumes the `vacuum_adapter` contract for
  product-facing navigation state and commands
- the dedicated `MapCanvas` surface now renders live occupancy-grid data from
  `/map` and tolerates Foxglove typed-array payload shapes
- `MapCanvas` now exposes a floating layers checklist for the base map, global
  costmap, local costmap, active plan, lidar, and depth obstacles
- lidar and depth obstacle overlays are extension-side visualization aids: they
  are projected into the map frame from TF and rendered below robot / target
  markers, not introduced as new product-contract surfaces
- `TeleopCard` provides compact manual robot control inside the panel sidebar;
  it publishes `geometry_msgs/msg/Twist` to `/cmd_vel_raw` — a deliberate
  choice because the VM's `twist_deadman.py` accepts both `Twist` and
  `TwistStamped` on that topic and it keeps teleop off the Nav2 command path
- `CameraOverlay` provides a floating PiP camera window inside the map canvas;
  it discovers image topics from the bridge and prefers the OAK-D RGB preview
  stream (`/oakd/rgb/preview/image_raw`); camera access validates one of the
  first usable vertical slice success criteria (pose, map, and camera visible)
- `MappingCard` exposes saved-map inventory and load behavior through
  adapter-level `snapshot.mapping.savedMaps` and `load_map`
- `Clean Area` mode is the current Layer 4 MVP: it selects a bounded rectangle,
  previews lawnmower waypoints, and drives them through adapter
  `go_to_location` commands
- live VS Code webview validation covers connect, map render, target select,
  send goal, progress visibility, cancel, terminal state, failure paths,
  overlays, teleop, and second-goal-after-cancel behavior

Remaining caveats for this slice:

- dock-blocked starts can still make a healthy Nav2 stack look stalled or
  failed
- clear-space validation is still required before treating navigation failure
  as a software defect
- Clean Area MVP has first-pass cell coverage progress, but it is still not
  production-complete coverage behavior

Practical implication:

- the TurtleBot4/Nav2 operator flow is proven through the current extension
  panel and supporting runtime surfaces;
- Layer 3 adapter work is closed for the TurtleBot4/Nav2 simulation path;
- production coverage semantics, room / zone semantics (Layer 5), and real
  hardware (Layer 6) all sit above the adapter contract;
- docking, battery, charging, scheduling, consumables, and OpenClaw integration
  all belong to later Layer 4+ work above the adapter contract.

## Role Of The VS Code Extension

The existing VS Code extension is part of the platform strategy.

During the first milestones, it should be the main operator and developer
surface because it already gives us fast feedback on robot and simulation
state.

The extension should be the main place where we validate:

- whether the robot is present and healthy;
- whether lidar, camera, 3D, and simulation views are actually useful;
- whether navigation behavior is understandable enough to debug;
- whether real-time state is visible enough to support development;
- whether the vacuum-facing contract exposes the right product information.

The important existing surfaces are:

- lidar panels;
- camera panels;
- 3D panels;
- Gazebo web or simulation views;
- any robot/entity discovery and state surfaces already present in the
  extension.

Current truth:

- the existing TurtleBot4-facing panels are primarily debugging and validation
  surfaces;
- they are useful for proving Layer 0 (sensors) and Layer 1 (SLAM, TF, odom)
  runtime health, plus Nav2 traffic and motion for Layer 2;
- `Vacuum Control` now provides the validated Layer 2 navigation operator
  workflow.

Near-term implication:

- near-term extension work should maintain and regress-test the closed
  Layer 2/Layer 3 TurtleBot4/Nav2 operator slice;
- Layer 4 should build coverage behavior above the existing adapter contract
  instead of raw ROS topics;
- room / zone UI (Layer 5), real-hardware Valetudo work (Layer 6), consumables
  UI, scheduling, and OpenClaw work should remain above the adapter boundary.

As the stack matures, the extension should also be a natural place for a small
set of vacuum-specific controls and status surfaces, such as:

- start mission;
- pause, resume, and stop;
- return to dock;
- room or zone selection;
- mission state;
- battery and charging status;
- fault and recovery state.

## Architecture Direction

The current target architecture is:

```text
VS Code extension / product UI
  -> vacuum_adapter contract
     -> TurtleBot4/Nav2 backend adapter
        -> VM TurtleBot4 simulation runtime

     -> Valetudo backend adapter
        -> VM-managed Valetudo integration runtime
           -> real Valetudo-compatible vacuum on local network
```

The important boundary is `vacuum_adapter`.

That should become the stable surface that higher-level clients use.
TurtleBot4 and later vacuum-vendor specifics should remain implementation
details behind it.
Contract modules should not import panel/runtime-specific backend types. Those
imports are allowed in backend adapters, which normalize the backend shape into
the public capability, state, and command contract.

VM boundary:

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

## Capability Model

The vacuum contract must be capability-driven from the start.

Valetudo explicitly models robots as different subsets and supersets of
capabilities, and not every robot supports the same commands, status surfaces,
or map workflows. See
[Capabilities overview](https://valetudo.cloud/pages/usage/capabilities-overview.html)
and [MQTT implementation details](https://valetudo.cloud/pages/development/mqtt/).
The upstream project is
[Hypfer/Valetudo](https://github.com/Hypfer/Valetudo).

Valetudo should influence the capability model, but it must not define the
public contract. Public capability names should be backend-neutral product
terms, not Valetudo class names.

Use public names such as:

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

Do not expose public flags such as:

- `BasicControlCapability`
- `MapSegmentationCapability`
- `ZoneCleaningCapability`
- `FanSpeedControlCapability`
- `WaterUsageControlCapability`

Those are backend-mapper details.

Private mapping examples:

- Valetudo `BasicControlCapability` -> `start_cleaning` / `pause` / `stop` /
  `return_to_dock`
- Valetudo `GoToLocationCapability` -> `go_to_location`
- Valetudo `MapSegmentationCapability` -> `segment_cleaning`
- Valetudo `ZoneCleaningCapability` -> `zone_cleaning`
- Valetudo `FanSpeedControlCapability` -> `fan_speed`
- Valetudo `WaterUsageControlCapability` -> `water_usage`
- Nav2 `NavigateToPose` -> `go_to_location`

Capability flags should be descriptors, not simple booleans:

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

Reason: real backends may support the same high-level feature with different
constraints, coordinate systems, command shapes, or partial behavior.

Commands should use backend-neutral payloads. `go_to_location` should carry a
target pose owned by the adapter contract, while setter commands such as
`set_fan_speed` and `set_water_usage` should carry the selected value explicitly.
Backend adapters can map those values to backend-specific presets or command
formats.

### Core capability tiers

**Tier 1: Required on every backend**

- robot identity
- availability / connectivity
- pose
- map access
- battery state
- mission status
- navigation-to-pose or equivalent move command

**Tier 2: Common vacuum controls**

- return to dock
- pause mission
- resume mission
- stop mission
- dock state
- charging state

**Tier 3: Advanced vacuum workflows**

- room / segment cleaning
- zone cleaning
- cleaning mode selection
- consumable state
- maintenance state

**Tier 4: Vendor-specific extensions**

- any backend-specific feature not portable enough for the shared contract

### Capability design rules

- every backend must advertise capability flags explicitly;
- clients must branch on capability flags, not backend names;
- clients must not ask whether the backend is TurtleBot4, Valetudo, Roborock,
  or a specific robot model before deciding which controls to show;
- capability presence must be validated against actual behavior;
- reconnect or backend reconfiguration may require capability refresh;
- vendor-specific features may exist, but they must not silently redefine the
  shared contract.

## Vacuum Contract

The public contract should describe a vacuum robot, not a TurtleBot4 and not a
specific vacuum vendor.

The contract must be:

- capability-based;
- stable across backends;
- built on standard ROS 2 / Nav2 interfaces where they already fit;
- extended only where vacuum-specific semantics are required.

### Core normalized surfaces

Every backend should provide the following normalized surfaces:

- robot identity
- connectivity / availability
- pose
- odometry
- map
- battery state
- mission state
- fault state
- capability flags

Where standard ROS 2 / Nav2 types already fit, they should be reused directly.
Examples include:

- `nav2_msgs/action/NavigateToPose`
- `sensor_msgs/msg/BatteryState`
- `nav_msgs/msg/OccupancyGrid`
- `nav_msgs/msg/Odometry`
- `geometry_msgs/msg/PoseStamped`
- `sensor_msgs/msg/Image` when a backend meaningfully exposes a camera feed

### Vacuum-specific normalized surfaces

Custom messages or services should exist only for concepts that are not
cleanly expressed through standard ROS 2 or Nav2 types. Likely examples
include:

- dock state
- mission lifecycle state
- pause / resume / stop mission controls
- room / segment cleaning requests
- zone cleaning requests
- backend capability advertisement

### Unsupported operations

If a backend does not support a capability:

- it must advertise that capability as unavailable;
- the unsupported command must fail explicitly and predictably;
- clients must not infer support from backend type;
- higher-level workflows must branch on capability flags, not backend name.

TurtleBot4/Nav2 should explicitly report unsupported vacuum capabilities rather
than hiding them or pretending to support them.

Initial TurtleBot4/Nav2 supported capabilities can include:

- `map`
- `pose`
- `go_to_location`
- `cancel_navigation`
- `manual_control`
- `navigation_status`

Initial TurtleBot4/Nav2 unsupported capabilities should include:

- `start_cleaning`
- `segment_cleaning`
- `zone_cleaning`
- `fan_speed`
- `water_usage`
- `consumables`
- real dock behavior

## Layer 3 Milestone

Layer 3 is closed for the TurtleBot4/Nav2 simulation path.

Acceptance criterion:

When Layer 6 (real hardware) ships and Valetudo is integrated, existing vacuum
UI surfaces should continue to work through the `vacuum_adapter` contract
without backend-specific UI rewrites. UI code may branch on adapter
capabilities and normalized state, but it must not branch on whether the
backend is TurtleBot4/Nav2, Valetudo, or a vendor/model. Backend-specific
behavior belongs in backend adapters.

Completed scope:

1. Define the `vacuum_adapter` capability / state / command contract.
2. Add a TurtleBot4/Nav2 adapter from the already-working simulation runtime.
3. Migrate `Vacuum Control` to consume the contract instead of raw ROS topics.
4. Add an explicit mission state machine covering `idle / navigating /
   cleaning / paused / returning / charging`.
5. Add a contract regression harness for the adapter boundary.
6. Add a Valetudo adapter interface stub that will be populated in Layer 6.

Completed exit validation:

1. The documented verification command set passes.
2. The hardened contract is revalidated against the live VM through the VS Code
   webview `Vacuum Control` panel.
3. Target selection, goal send, cancel, terminal state, failure path, overlays,
   teleop, and second-goal-after-cancel behavior are validated.
4. Dock interaction remains a runtime caveat and moves to Layer 4 runtime setup
   work where dock / undock and battery-aware execution belong.

Out of Layer 3:

- production-complete coverage behavior beyond the current Clean Area MVP
  (Layer 4)
- dock / undock and battery-aware execution beyond what the contract models
  (Layer 4)
- room / zone naming and segmentation UI (Layer 5)
- VM service plan for the Valetudo integration runtime (Layer 6)
- MQTT vs HTTP/client API decisions for Valetudo (Layer 6)
- first real hardware validation flow (Layer 6)

Do not start with:

- room cleaning UI
- zone editor
- multi-room workflows
- map editing
- scheduling
- consumables UI
- OpenClaw integration

## Layer 4 Milestone: Coverage

Layer 4 adds the first cleaning behavior above the adapter contract.

Scope:

- the current Clean Area MVP produces a simple lawnmower waypoint sequence over
  a bounded rectangle;
- production coverage planning must harden the MVP's coverage accounting
  beyond the current waypoint runner;
- dock / undock awareness in the adapter state and commands;
- battery-aware execution, so the robot can return to dock when needed and
  resume from where it stopped;
- a "clean this area" flow that works end-to-end in simulation through the
  adapter.

Constraints:

- the planner must consume the adapter's normalized map and pose, not raw ROS
  topics;
- coverage should succeed or fail through the adapter's navigation and mission
  state surfaces;
- UI for selecting an area belongs above the contract, not in backend-specific
  code;
- the current Clean Area MVP executes a lawnmower-shaped sequence of
  `go_to_location` waypoints through `vacuum_adapter`, clips sampled lanes to
  known free cells, tracks footprint-history progress, and uses a
  product-level coverage profile. Production coverage still needs stronger edge
  and corner handling, component-level route planning around interior obstacles
  or unknown space, and resume/recovery semantics.

## Layer 5 Milestone: Room / Zone Semantics

Layer 5 turns coverage into product-complete vacuum behavior.

Scope:

- divide the map into named zones / rooms;
- translate "clean room 3" into one or more coverage goals for Layer 4;
- expose room / zone state through the adapter's normalized state;
- complete the full vacuum workflow end-to-end in simulation.

Constraints:

- room / zone naming is product-facing and must not live in backend topics;
- segmentation logic may use backend input where Valetudo provides it, but the
  public shape must be backend-neutral;
- Layer 5 should be demonstrable in simulation before Layer 6 begins.

## Layer 6 Milestone: Real Hardware (Valetudo)

Layer 6 swaps the simulated TurtleBot4 backend for a real Valetudo-compatible
vacuum. No changes above the adapter contract should be required.

Valetudo-compatible vacuums are the first real-hardware target.

The VM should eventually run the Valetudo integration runtime so users do not
need to install or operate extra integration tooling locally. Depending on the
implementation, the VM may run:

- MQTT broker, if needed
- Valetudo client / integration service
- Valetudo backend adapter process
- discovery / config service
- runtime health checks

Important wording:

- say "Valetudo integration runtime in the VM"
- do not imply that the VM replaces the Valetudo instance on the robot

Valetudo normally runs on or with the robot vacuum firmware. The VM-managed
layer should sit around that robot-side Valetudo instance:

```text
real vacuum running Valetudo
  -> VM-managed Valetudo integration client / MQTT broker / adapter service
  -> vacuum_adapter
  -> extension / product UI
```

The first real-hardware validation path should be basic:

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

Only after that basic reachability slice works should Layer 6 exercise Layer 4
coverage and Layer 5 room / zone flows against real hardware.
