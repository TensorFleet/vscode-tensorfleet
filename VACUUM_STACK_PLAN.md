# Vacuum Stack Plan

## Purpose

This document is the architecture and planning source of truth for the vacuum
stack.

It holds the pieces together:

- product goal
- backend boundary
- layer plan
- capability model
- adapter contract
- current layer status
- future milestones

Use `steps.md` for current implementation progress and validation. Use
`extension.md` for VS Code extension files, panels, topics, endpoints, and
extension follow-up work.

## Goal

Build the software stack for a future robot vacuum before buying hardware.

Near-term goal:

- a simulated robot appears in our system
- it is controllable through ROS 2
- it can navigate in simulation
- the VS Code extension and panels are useful operator surfaces
- product behavior does not couple to TurtleBot4-specific APIs

Longer-term goal:

- a vacuum-facing contract exists above robot backends
- docking, pause/resume, room cleaning, battery-aware execution, and mission
  state fit naturally into that contract
- higher-level product logic can move from TurtleBot4 simulation to a real
  vacuum backend with minimal churn
- simulation becomes the regression harness for later hardware work
- the system remains local-first

TurtleBot4 is not the product. It is the first development backend.

## Product Boundary

The product-facing boundary is a repo-owned `vacuum_adapter` contract.

Backend differences must surface through:

- capabilities
- normalized state
- explicit unsupported operations

No client above the adapter may depend on TurtleBot4-specific topics, helper
services, node APIs, Nav2 internals, or Valetudo class names.

TurtleBot4-specific helpers such as `TurtleBot4Navigator` may be useful backend
implementation details because they add docking and undocking behavior on top
of Nav2. They must stay private to the TurtleBot4 backend and must not define
the public contract.

Reference docs for TurtleBot4 behavior:

- [Navigation](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html)
- [TurtleBot 4 Navigator](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/turtlebot4_navigator.html)

## Architecture Direction

Current target architecture:

```text
VS Code extension / product UI
  -> vacuum_adapter contract
     -> TurtleBot4/Nav2 backend adapter
        -> VM TurtleBot4 simulation runtime

	     -> Valetudo backend adapter
	        -> VM-managed Valetudo integration runtime
	           -> future Valetudo-compatible vacuum validation path
```

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
- runtime health checks

Outside VM / shared product layer:

- `vacuum_adapter` contract
- normalized state model
- capability descriptors
- command semantics
- UI-facing assumptions
- coverage and room/zone logic above the adapter

## Planning Decisions

Current working decisions:

- ROS 2 is the foundation.
- TurtleBot4 is the first backend, not the public product contract.
- The public boundary is repo-owned and vacuum-facing, not raw TurtleBot4
  topics.
- Layers are built bottom-up: sensors, localization/map, navigation, adapter,
  coverage, room/zone semantics, real hardware.
- Higher layers must depend on adapter capabilities and normalized state.
- Product/UI clients must branch on capability flags, not backend names.
- Public contract files own backend-neutral state and command types.
- Backend/runtime-specific imports belong inside backend mappers.
- Setter commands use explicit payload shapes.
- Long-running robot behavior belongs in the VM runtime or backend process, not
  React/webview hooks.
- Auto mapping is VM-owned and exposed through backend-neutral capabilities and
  commands.
- Coverage and room/zone semantics sit above the adapter, not below it.
- Real hardware is the last layer and targets Valetudo-compatible vacuums first.
- Valetudo influences capability design, but does not define the public
  contract.
- Valetudo-specific capability names/classes stay private to the Valetudo
  backend adapter.
- The VM may host MQTT/client/discovery/adapter services, but it does not own
  the public `vacuum_adapter` contract.
- Say "Valetudo integration runtime in the VM," not "Valetudo in the VM."
- First Valetudo hardware validation proves reachability, status, capabilities,
  normalized state, and one basic command before room/zone workflows.
- Standard ROS 2 and Nav2 interfaces should be reused where they fit.
- Simulation should be realistic enough to validate workflow, not just expose
  `cmd_vel`.

## Runtime-Owned Mission Contract

All long-running robot behavior is modeled as a mission. The UI may create
drafts, submit intent commands, and render snapshots, but it must not own active
mission execution.

Mission types include:

- `mapping`
- `navigation`
- `coverage`
- `return_to_dock`
- `room_cleaning`
- `zone_cleaning`
- `hardware_cleaning`

Mission statuses are backend-neutral:

- `idle`
- `preparing`
- `running`
- `paused`
- `canceling`
- `returning`
- `charging`
- `resuming`
- `needs_assistance`
- `completed`
- `failed`
- `canceled`
- `unsupported`

The adapter exposes mission state through:

```text
snapshot.activeMission
snapshot.missions
snapshot.mission
```

`snapshot.mission` is the legacy coarse state used by the current UI.
`snapshot.activeMission` and `snapshot.missions` are the normalized product
contract for new runtime-owned work.

Each active mission snapshot carries:

- mission id
- mission type
- backend source
- started / updated timestamps
- requested command
- current phase
- normalized status
- normalized progress
- available actions
- terminal result
- structured result details when the runtime provides them
- recoverable error / needs-assistance state
- target payload

Hydration rule:

```text
When a webview opens, it must render from adapter/runtime snapshots.
It must not reconstruct active mission authority from React state.
```

Ownership rule:

```text
Before Start: UI may own draft state and local preview.
After Start: runtime/backend owns confirmed mission state.
```

Current implementation note:

- Mapping already follows this model most closely because autonomous
  exploration is VM-owned.
- Navigation is runtime-owned for TurtleBot4/Nav2 simulation: the UI submits a
  `start_navigation` intent, the VM mission runtime owns the Nav2 goal, and the
  adapter hydrates destination/progress/action state from
  `/vacuum_mission/status` and `/vacuum_mission/get_snapshot`.
- Terminal navigation destinations can be dismissed in the UI after
  completed/canceled/failed runs; that is presentation state only and does not
  clear runtime mission history.
- Clean Area active execution is now runtime-owned for TurtleBot4/Nav2:
  the UI submits `start_coverage`, the VM mission runtime owns Nav2 waypoint
  sequencing and progress, and the adapter hydrates coverage mission state from
  `/vacuum_mission/status` and `/vacuum_mission/get_snapshot`.
- Coverage route generation and per-cell overlay/progress details are now
  runtime snapshot data for TurtleBot4/Nav2. The UI keeps only draft selection
  and local preview before start.
- Room/zone cleaning uses the same runtime-owned mission path: the UI submits
  `start_room_cleaning` or `start_zone_cleaning`, the TurtleBot4/Nav2 backend
  maps the selected annotation to a coverage target privately, and the UI
  hydrates active and terminal summaries from `snapshot.activeMission` and
  `snapshot.missions.recent`.
- Room/zone recovery controls are runtime/action gated: pause, resume, cancel,
  retry step, and skip step are shown or dispatched only when
  `activeMission.availableActions` and adapter capabilities allow them.
- Coverage-style terminal results may carry structured details for cleaned
  area, remaining area, skipped area, skipped reasons, route completion, and
  coverage-threshold status. Product UI may use these details to label a run
  as cleaned or partially cleaned without exposing Nav2 waypoint internals.

Current position:

- Layer 0 is validated.
- Layer 1 is running.
- Layer 2 is closed for TurtleBot4/Nav2 simulation.
- Layer 3 is closed for TurtleBot4/Nav2 simulation.
- The Layer 4 prerequisite, Mapping + Whole Map View, is implemented.
- Layer 4 Clean Area MVP exists above the adapter.
- Layer 5 Room / Zone Semantics prototype is implemented.
- Layer 6A is implemented for the Valetudo mock/runtime/adapter/UI path.
- Layer 6 Valetudo hardware validation remains planned.

## Current Layer Structure

```text
Layer 0 — Sensors                   validated
Layer 1 — Localization + Map        running
Layer 2 — Navigation                closed for TurtleBot4/Nav2 simulation
Layer 3 — Vacuum Adapter            closed for TurtleBot4/Nav2 simulation;
                                    Valetudo path implemented for Layer 6A
Layer 4 prerequisite: Mapping + Whole Map View
                                    implemented for TurtleBot4/Nav2 simulation
Layer 4 — Coverage                  Clean Area execution, route generation,
                                    and per-cell progress snapshots are
                                    runtime-owned for TurtleBot4/Nav2
Layer 5 — Room / Zone Semantics     prototype implemented:
                                    manual annotations, target preview,
                                    room/zone cleaning, recovery controls,
                                    hydration, result details, and recent
                                    summaries
Layer 6A — Valetudo Mock Through VM implemented for mock/runtime/adapter/UI
Layer 6 — Real Hardware (Valetudo)  deferred until Layer 6A remains stable
```

Each layer should be stable before the next layer depends on it.

## Layer 0: Sensors

Status: validated.

Meaning:

- simulated TurtleBot4 boots reliably in the VM
- camera, lidar, and depth point cloud streams are visible in the extension
- the robot exists and publishes sensor data

Layer 0 proves the robot exists as an observable runtime object.

## Layer 1: Localization + Map

Status: running.

Meaning:

- SLAM Toolbox runs in the VM and builds a map as the robot moves
- the robot knows where it is in that map
- TF tree `map -> odom -> base_link` is continuously available
- `/map`, `/odom`, `/tf`, and `/tf_static` are observable from the extension

Layer 1 is the foundation for navigation, coverage, room/zone semantics, and
later hardware regression.

## Layer 2: Navigation

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

Backend:

- TurtleBot4 simulation in VM
- Nav2 runtime
- Foxglove bridge

Current caveat:

- dock-blocked starts still require clear-space validation

Layer 2 is closed as a simulation operator slice. Further docking/undocking
behavior belongs to Layer 4 mission execution, not Layer 2 navigation closure.

## Layer 3: Vacuum Adapter

Status: closed for TurtleBot4/Nav2 simulation; Valetudo mock/runtime/adapter/UI
path implemented for Layer 6A.

Purpose:

- normalize TurtleBot4/Nav2 runtime behavior into a `vacuum_adapter` contract
- make the extension talk to vacuum concepts, not raw ROS topics
- introduce explicit capabilities, normalized state, and command results
- expose the normalized runtime-owned mission model:
  `idle / preparing / running / paused / canceling / returning / charging /
  resuming / needs_assistance / completed / failed / canceled / unsupported`
- support TurtleBot4/Nav2 first and Valetudo later

Current implementation:

- public `VacuumAdapter` surface and `useVacuumAdapter` hook live under
  `panels-standalone/src/vacuum-adapter/`
- TurtleBot4/Nav2 backend `useTurtleBot4Nav2Adapter` wraps `useNav2Runtime`
- adapter snapshot exposes availability, identity, pose, map, navigation,
  mapping, mission, battery, readiness, and capabilities
- `Vacuum Control` consumes the adapter instead of raw Nav2 runtime
- `start_navigation` and `cancel_mission` dispatch through
  `adapter.sendCommand`; `cancel_navigation` remains a compatibility fallback
- TurtleBot4/Nav2 navigation missions hydrate through the runtime-owned
  `/vacuum_mission/status` topic and `/vacuum_mission/get_snapshot` service
- command dispatch is extracted into a pure helper for regression tests
- vacuum-only commands unsupported by TurtleBot4/Nav2 fail explicitly
- Valetudo backend maps VM-managed runtime snapshots and commands into
  normalized adapter state for the mock-backed Layer 6A path
- `bun run test:vacuum-adapter` covers the adapter boundary

Layer 3 acceptance criterion:

When Layer 6 ships, existing vacuum UI surfaces should continue to work through
`vacuum_adapter` without backend-specific UI rewrites. UI code may branch on
capabilities and normalized state, but not on whether the backend is
TurtleBot4/Nav2, Valetudo, or a vendor/model.

Out of Layer 3:

- production-complete coverage behavior beyond the current Clean Area MVP
- dock / undock and battery-aware execution beyond what the contract models
- room / zone naming and segmentation UI
- Valetudo VM service plan and MQTT vs HTTP/client API decisions
- first hardware validation

## Layer 4 Prerequisite: Mapping + Whole Map View

Status: implemented for TurtleBot4/Nav2 simulation.

Purpose:

- make the first valid `/map` display as full known occupancy-grid bounds
- provide explicit map viewport modes: `fit`, `manual`, `follow_robot`
- keep base map, costmaps, plan path, lidar/depth overlays, robot marker, and
  target marker aligned through one world-to-screen transform
- give the operator auto and manual mapping workflows before coverage cleaning
- show whether the current map is missing, active, mostly unknown, in review, or
  accepted for later navigation/coverage
- keep autonomous exploration in the VM runtime so it survives panel reloads,
  UI closure, websocket reconnects, and extension hiccups

Current implementation:

- `vacuum_adapter` exposes `snapshot.map.grid`, `snapshot.map.metadata`, and
  `snapshot.mapping`
- `MapCanvas` renders product base map from adapter-normalized grid and keeps
  live `/map` as fallback/diagnostic visualization
- Fit Map uses occupancy-grid dimensions and panel size with padding
- map controls expose Zoom In, Zoom Out, Fit Map, Follow Robot, zoom readout,
  and view-state label
- mapping state reports `idle`, `manual_mapping`, `auto_mapping`, `paused`,
  `needs_assistance`, `review`, `accepted`, `discarded`, or `error`
- mapping status reports known/unknown ratio, frontier count, visited/failed
  goals, active goal, reason, timestamps, and persistence outcome
- `MappingCard` supports auto/manual mapping, pause/resume, finish/review,
  discard, accept, saved-map inventory, and load
- active auto mapping blocks competing teleop until paused
- manual mapping, paused mapping, and `needs_assistance` keep teleop available
- Accept map attempts VM-side persistence under
  `/opt/tensorfleet/maps/current_map.*`
- after accepted map save, later `/map` growth is autosaved after a quiet
  interval while SLAM remains active
- saved-map inventory and load state flow through adapter mapping fields

Boundary rule:

- UI rendering may keep direct diagnostic subscriptions for overlays.
- Product workflows should consume adapter-level map/session state.
- Coverage planning must consume normalized map and pose above the adapter.

## Layer 4: Coverage

Status: Clean Area execution is runtime-owned for TurtleBot4/Nav2. The UI owns
draft selection and preview before start; the VM mission runtime owns active
coverage execution, lifecycle actions, progress, and terminal snapshots.

Purpose:

- add coverage path planning over a bounded region
- teach the adapter and UI how cleaning state differs from simple navigation
- add dock / undock and battery-aware execution
- make "clean this area" work end-to-end in simulation through the adapter

Current Clean Area MVP:

- `Vacuum Control` has `Mapping`, `Navigate`, and `Clean Area` modes
- `MapCanvas` supports drawing, moving, and resizing a rectangular clean-area
  selection
- clean-area selection is validated against map bounds and occupancy data
- `cleanAreaProfile.ts` owns swath width, overlap, navigation goal tolerance,
  boundary margin, minimum useful region size, completion threshold, derived
  lane spacing, and boundary extension
- default profile preserves the 0.30 m simulation swath
- `cleanAreaPlanner.ts` builds a swath-overlap lawnmower waypoint preview
- the planner chooses the longer axis for passes
- sampled lanes are clipped to known free occupancy-grid cells
- boundary pass endpoints are extended to compensate for Nav2 goal tolerance
- the run starts with `adapter.sendCommand({ command: "start_coverage", ... })`
- the VM mission runtime privately sequences Nav2 goals for TurtleBot4/Nav2
- `cleanAreaCoverage.ts` classifies cells as cleanable, occupied, unknown,
  out-of-bounds, too small, remaining, or covered
- cleanable cells are decomposed into connected regions
- tiny disconnected regions are skipped
- active progress hydrates from `snapshot.activeMission`
- runtime progress is covered square meters from robot footprint history in map
  frame
- `MapCanvas` renders remaining, covered, excluded, skipped, and footprint
  overlays
- `CleanAreaCard` reports percentage, cleaned area, remaining area, skipped
  area, waypoint progress, pass count, and route status
- Clean Area mode includes a capability-driven mission lifecycle card for dock,
  return-to-dock, battery, and charging state
- states include editing, confirmed, preparing, running, paused, canceling,
  completed, failed, and canceled
- operators can pause, resume, cancel, retry waypoint, skip waypoint, and clear

Production Layer 4 follow-up:

- stronger edge/corner coverage
- obstacle-adjacent behavior
- component-level route planning around interior obstacles and unknown space
- dock / undock execution
- battery-aware return/resume
- resume/recovery semantics
- stronger failure semantics for route-finished with uncovered cells

Layer 4 must stay above `vacuum_adapter`.

## Layer 5: Room / Zone Semantics

Status: prototype implemented for the TurtleBot4/Nav2 simulation path.

Purpose:

- divide the map into named zones / rooms
- translate "clean room 3" into one or more Layer 4 coverage goals
- expose room / zone state through normalized product-facing state
- make the full vacuum workflow work end-to-end in simulation

Constraints:

- room / zone naming is product-facing and must not live in backend topics
- segmentation logic may consume backend input where useful, including Valetudo
  zones later, but the public shape must be backend-neutral
- Layer 5 should be demonstrable in simulation before Layer 6 begins

Current prototype:

- `Vacuum Control` has a `Rooms / Zones` mode.
- Operators can draw a rectangular room/zone draft on the map using the shared
  editable rectangle behavior from Clean Area.
- Operators can name the draft, choose `Room` or `Zone`, save it, select it
  later, and delete it.
- Saved annotations are exposed as `snapshot.map.annotations`.
- Selecting a saved room/zone converts its product-level annotation into the
  same coverage target preview used by Clean Area.
- The selected target is highlighted on the map with route and per-cell
  cleanability preview.
- The side panel reports whether the selected target is cleanable, partially
  cleanable, or invalid.
- Selecting a saved room/zone can start a product-facing
  `start_room_cleaning` or `start_zone_cleaning` intent.
- The TurtleBot4/Nav2 adapter translates that intent into the existing
  runtime-owned coverage mission request while preserving the room/zone label
  and mission intent in the request payload and optimistic mission snapshot.
- Active room/zone cleaning hydrates from `snapshot.activeMission`; the UI
  renders lifecycle/progress state and routes pause/resume/cancel/retry/skip
  through mission actions.
- Terminal room/zone results hydrate through `snapshot.missions.recent` and
  render as recent mission summaries after webview reopen.
- Runtime room/zone snapshots preserve product-level annotation metadata
  including id, kind, name, and map id.
- Runtime coverage-style results preserve cleaned/remaining/skipped area,
  skipped-reason counts, route completion, and threshold status when available.
- Recent mission labels can distinguish cleaned from partially cleaned based on
  normalized result details.
- TurtleBot4/Nav2 persists annotations in webview storage keyed to the active
  map identity when available.
- Valetudo annotation/room/zone semantics remain explicitly unsupported until
  the Layer 6 backend maps them.

Still pending after the prototype:

- VM/runtime-owned annotation durability
- VM/runtime-owned durable mission history

## Layer 6A: Valetudo Mock Through VM

Status: implemented for the mock/runtime/adapter/UI path.

Purpose:

- prove that Valetudo-shaped state, capabilities, and basic commands can travel
  through the same `vacuum_adapter` boundary as the TurtleBot4/Nav2 simulation
  backend
- keep TurtleBot4/Nav2 as the simulation and regression backend
- run our Valetudo integration runtime in the VM while the mock source runs
  locally or in a local development environment
- keep the same extension and product UI consuming only normalized adapter
  state and commands

Important wording:

- say "Valetudo integration runtime in the VM"
- say "Valetudo runs on the robot" for the later real hardware path
- do not imply the VM replaces the Valetudo instance on the robot

Implemented Layer 6A path:

```text
Valetudo mock HTTP source or fixed mock runtime data
  -> VM-managed Valetudo integration runtime
  -> /api/v1/valetudo/health
  -> /api/v1/valetudo/snapshot
  -> /api/v1/valetudo/command
  -> Valetudo backend adapter
  -> vacuum_adapter
  -> extension / product UI
```

Supported Layer 6A behavior:

- robot identity
- availability/connectivity
- runtime/source health
- stale, unreachable, and offline source handling
- activity/state
- battery
- dock/charging
- conservative capability mapping
- command result normalization for unsupported, unavailable, invalid-state,
  failed, and successful outcomes
- `start_cleaning`
- `pause`
- `stop`
- `return_to_dock`
- fan speed current value and options
- water usage current value and options
- `set_fan_speed`
- `set_water_usage`
- maintenance/consumables display:
  - main brush
  - side brush
  - filter
  - sensor cleaning
  - mop pad when present in mock data
- no-map Vacuum Control UI cards:
  - Basic Cleaning
  - Cleaning Settings
  - Maintenance
  - compact unavailable workflows

Deferred or unsupported in Layer 6A:

- Valetudo-compatible robot hardware validation
- Valetudo map rendering
- Valetudo robot movement visualization
- go-to/navigation
- mapping
- Clean Area execution
- zone cleaning
- room editor and map annotation editing for Valetudo targets
- consumable reset commands
- diagnostics drawer
- OpenClaw
- MQTT production hardening
- production hardware support

The runtime source modes are explicit:

- `fixed_mock`: VM-owned fixed mock state.
- `valetudo_mock_http`: HTTP mock source; may use `VALETUDO_MOCK_SOURCE_URL`
  and keeps the local mock default.
- `valetudo_mock_mqtt`: MQTT cache/source diagnostics for mock-shaped data.
- `valetudo_http`: future Valetudo HTTP source config path; requires
  `VALETUDO_SOURCE_URL` and has no hardcoded robot address.

Valetudo runs on the robot. The VM hosts only the TensorFleet Valetudo
integration runtime. Missing HTTP source config, unreachable source, stale
source, malformed source responses, missing capability data, and reachable
sources without `BasicControlCapability` map to stable unavailable/degraded
runtime snapshots and passive diagnostics.

Runtime diagnostics expose a readiness summary for hardware prep:

- runtime online
- source reachable
- source stale
- supported normalized capability inputs
- detected Valetudo capabilities that are not product-ready
- basic command availability
- segment target availability

The adapter and UI do not know whether runtime state came from HTTP, MQTT, or
fixed mock data except through normalized `availability`, `health`, `source`,
`capabilities`, `battery`, `dock`, `activity`, `cleaningSettings`,
and `maintenance`.

Later real-hardware target path:

```text
real vacuum running Valetudo
  -> VM-managed Valetudo integration runtime
  -> Valetudo backend adapter
  -> vacuum_adapter
  -> extension / product UI
```

First real-hardware validation path after Layer 6A:

```text
Valetudo robot reachable
-> VM receives status / capabilities
-> adapter normalizes state
-> extension displays capability / state summary
-> one basic command works
```

First real-hardware validation checklist:

- Network assumptions:
  - robot is running Valetudo on the local network
  - VM can reach the robot IP or hostname
  - no cloud dependency is required
- Runtime startup:
  - set `VALETUDO_RUNTIME_SOURCE_MODE=valetudo_http`
  - set `VALETUDO_SOURCE_URL` to the robot Valetudo HTTP base URL
  - start the Valetudo integration runtime in the VM
  - confirm `GET /api/v1/valetudo/health`
- Adapter/UI validation:
  - extension connects through the VM runtime route
  - robot identity appears
  - runtime/source health appears
  - battery, dock, and activity appear if exposed
  - normalized capabilities appear
  - unsupported operations remain explicit
- First safe command:
  - prefer `pause`, `stop`, or `return_to_dock` depending on robot state
  - use `start_cleaning` only if the robot is in a safe open area
- Segment validation:
  - validate segment targets only after a later implementation maps them into
    normalized product state
  - do not assume room labels exist
  - treat segment IDs as opaque
- Rollback and safety:
  - know how to stop the robot
  - keep physical access to the robot
  - avoid testing near stairs, cables, fragile objects, pets, or people

Only after that basic reachability slice works should Layer 6 exercise Layer 4
coverage and Layer 5 room/zone flows against real hardware.

## Capability Model

The vacuum contract must be capability-driven from the start.

Valetudo models robots as different subsets and supersets of capabilities. Not
every robot supports the same commands, status surfaces, or map workflows.

Reference docs:

- [Valetudo capabilities overview](https://valetudo.cloud/pages/usage/capabilities-overview.html)
- [Valetudo MQTT implementation details](https://valetudo.cloud/pages/development/mqtt/)
- [Valetudo project](https://github.com/Hypfer/Valetudo)

Valetudo should influence the capability model, but it must not define the
public contract.

Public backend-neutral capability names include:

- `start_cleaning`
- `start_navigation`
- `start_coverage`
- `map_annotations`
- `room_semantics`
- `zone_semantics`
- `room_cleaning`
- `zone_cleaning`
- `pause`
- `resume`
- `stop`
- `return_to_dock`
- `go_to_location`
- `cancel_mission`
- `cancel_navigation`
- `pause_mission`
- `resume_mission`
- `retry_mission_step`
- `skip_mission_step`
- `coverage_mission`
- `mission_state`
- `manual_control`
- `navigation_status`
- `mapping_session`
- `auto_mapping`
- `segment_cleaning`
- `fan_speed`
- `water_usage`
- `consumables`
- `events`
- `dock_state`
- `battery`

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
- VM `/vacuum_mapping/*` services and `/vacuum_mapping/status` ->
  `mapping_session` / `auto_mapping`

Capability flags should be descriptors:

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

### Capability Tiers

Tier 1: required on every backend

- robot identity
- availability / connectivity
- pose
- map access
- battery state
- mission status
- navigation-to-pose or equivalent move command

Tier 2: common vacuum controls

- return to dock
- pause mission
- resume mission
- stop mission
- dock state
- charging state

Tier 3: advanced vacuum workflows

- room / segment cleaning
- zone cleaning
- cleaning mode selection
- consumable state
- maintenance state

Tier 4: vendor-specific extensions

- backend-specific features not portable enough for the shared contract

### Capability Design Rules

- every backend advertises capability flags explicitly
- clients branch on capability flags, not backend names
- clients must not ask whether the backend is TurtleBot4, Valetudo, Roborock,
  or a specific model before deciding which controls to show
- capability presence must be validated against actual behavior
- reconnect or backend reconfiguration may require capability refresh
- vendor-specific features may exist but must not silently redefine the shared
  contract

## Vacuum Contract

The public contract describes a vacuum robot, not a TurtleBot4 and not a
specific vacuum vendor.

The contract must be:

- capability-based
- stable across backends
- built on standard ROS 2 / Nav2 interfaces where they fit
- extended only where vacuum-specific semantics are required

### Core Normalized Surfaces

Every backend should provide normalized surfaces for:

- robot identity
- connectivity / availability
- pose
- odometry
- map
- battery state
- mission state
- fault state
- capability flags
- readiness evidence
- navigation state and progress
- mapping state when mapping is supported

Where standard ROS 2 / Nav2 types already fit, they should be reused or mapped:

- `nav2_msgs/action/NavigateToPose`
- `sensor_msgs/msg/BatteryState`
- `nav_msgs/msg/OccupancyGrid`
- `nav_msgs/msg/Odometry`
- `geometry_msgs/msg/PoseStamped`
- `sensor_msgs/msg/Image` when a backend meaningfully exposes a camera feed

### Vacuum-Specific Normalized Surfaces

Custom messages or services should exist only for concepts not cleanly expressed
through standard ROS 2 or Nav2 types:

- dock state
- mission lifecycle state
- pause / resume / stop mission controls
- room / segment cleaning requests
- zone cleaning requests
- backend capability advertisement
- cleaning mode / fan / water presets
- consumable and maintenance state

### Commands

Current public command names include:

- `go_to_location`
- `cancel_navigation`
- `manual_control`
- `start_navigation`
- `start_coverage`
- `pause_mission`
- `resume_mission`
- `cancel_mission`
- `retry_mission_step`
- `skip_mission_step`
- `start_mapping`
- `pause_mapping`
- `resume_mapping`
- `finish_mapping`
- `discard_mapping`
- `accept_map`
- `load_map`
- `save_map_annotation`
- `delete_map_annotation`
- `start_cleaning`
- `start_room_cleaning`
- `start_zone_cleaning`
- `pause`
- `resume`
- `stop`
- `return_to_dock`
- `segment_cleaning`
- `zone_cleaning`
- `set_fan_speed`
- `set_water_usage`

Payload rules:

- `go_to_location` carries a backend-neutral target pose
- `start_navigation` carries a backend-neutral navigation intent and is the
  preferred runtime-owned navigation command
- `start_coverage` carries a backend-neutral area target; backend route
  generation and progress snapshots remain runtime-owned after start
- `save_map_annotation` and `delete_map_annotation` carry product-level map
  annotation state, not backend room/segment identifiers
- `segment_cleaning` remains unsupported until backend-provided segment targets
  are normalized in a later implementation
- `start_room_cleaning` carries saved map annotations for annotation-backed
  workflows
- `start_zone_cleaning` carries saved map annotations; backend route generation
  and mission progress remain runtime-owned after start
- mapping commands carry mode/name where needed
- setter commands carry selected values explicitly
- unsupported commands fail predictably
- streaming teleop should not be forced into a one-shot command model

### Unsupported Operations

If a backend does not support a capability:

- advertise that capability as unavailable
- return explicit unsupported command results
- do not hide the operation silently
- do not infer support from backend type
- keep higher-level workflows branching on capability flags

Initial TurtleBot4/Nav2 supported capabilities can include:

- map
- pose
- `go_to_location`
- `start_navigation`
- `start_coverage`
- `cancel_navigation`
- `cancel_mission`
- `pause_mission`
- `resume_mission`
- `retry_mission_step`
- `skip_mission_step`
- `manual_control`
- `navigation_status`
- `mapping_session`
- `auto_mapping`
- `map_annotations`
- `room_semantics`
- `zone_semantics`
- `room_cleaning`
- `zone_cleaning`

Initial TurtleBot4/Nav2 unsupported capabilities include:

- `start_cleaning`
- `segment_cleaning`
- `fan_speed`
- `water_usage`
- `consumables`
- real dock behavior

## Product And Operator Positioning

The system should first work as a deterministic, local ROS 2 vacuum platform.

Current positioning:

- the VS Code extension is an important part of the platform, not just a
  debugging extra
- the extension and related panels are the main first-mile UI for bringup and
  validation
- TurtleBot4 is the development harness
- OpenClaw may become useful later, but should sit above a stable
  vacuum-facing contract rather than directly on TurtleBot4 internals

The current operator flow is proven through `Vacuum Control`. Supporting panels
remain useful for runtime validation, but they are not the primary product
surface.

As the stack matures, the extension should naturally expose vacuum-specific
controls and status:

- start mission
- pause / resume / stop
- return to dock
- room or zone selection
- mission state
- battery and charging status
- fault and recovery state

Extension-specific implementation details are documented in `extension.md`.

## Success Criteria

### First Usable Vertical Slice: Layers 0-2

- one simulated robot appears in our system
- the robot boots reliably in the VM
- pose, map, and camera are visible
- a ROS client can send a navigation goal and observe progress/result
- standard ROS 2 / Nav2 interfaces are used where they fit
- the path toward a vacuum-specific abstraction stays clean

### Adapter Slice: Layer 3

- extension operates the robot through `vacuum_adapter`
- TurtleBot4/Nav2 backend normalizes into vacuum state and commands
- unsupported vacuum capabilities are advertised as unsupported
- normalized mission snapshots explicitly carry runtime-owned statuses:
  `idle / preparing / running / paused / canceling / returning / charging /
  resuming / needs_assistance / completed / failed / canceled / unsupported`
- public contract files do not import backend-specific runtime or panel types
- adapter boundary has regression coverage

### Simulation-Complete Slice: Layers 4-5

- Clean Area starts selected rectangular coverage missions through the adapter
  and the VM runtime owns active execution
- row-level waypoint generation clips sampled rows to known free cells
- production coverage accounts for footprint history, configurable swath,
  overlap, edge/corner handling, and full obstacle/unknown decomposition
- dock / undock and battery-aware behavior are visible through adapter state
- map is divided into named zones
- "clean room 3" translates into coverage goals
- full vacuum workflow is demonstrated in simulation without leaking backend
  details to UI

### Full Platform: Layer 6

- same vacuum contract works against TurtleBot4 simulation and real hardware
- Valetudo backend implements the same contract
- extension/UI run unchanged against real hardware
- backend differences are capability flags and explicit unsupported operations,
  not product forks
- docking, mission lifecycle, battery state, and room/zone workflows fit the
  same contract
- simulation acts as the regression harness for hardware work

## Milestones

### Layer 3 Milestone: Closed

Completed scope:

1. Define `vacuum_adapter` capability/state/command contract.
2. Add TurtleBot4/Nav2 adapter from the working simulation runtime.
3. Migrate `Vacuum Control` to consume the contract.
4. Add mission state machine.
5. Add adapter regression harness.
6. Add the Valetudo adapter boundary, then extend it through the mock-backed
   Layer 6A runtime path.

Completed exit validation:

1. Verification command set passes.
2. Hardened contract is validated against the live VM through `Vacuum Control`.
3. Target selection, goal send, cancel, terminal state, failure path, overlays,
   teleop, and second-goal-after-cancel behavior are validated.
4. Dock interaction remains a runtime caveat and moves to Layer 4 runtime setup.

### Layer 4 Milestone: In Progress

Scope:

- maintain the current runtime-owned Clean Area simulation flow
- harden runtime coverage route generation beyond row-level clipping
- continue improving runtime mission summaries for uncovered/skipped cells
- dock / undock awareness in adapter state/commands
- battery-aware execution and resume

Constraints:

- planner consumes normalized map and pose
- coverage succeeds/fails through normalized runtime mission state
- UI selection belongs above the contract
- backend-specific code must keep execution details private behind the adapter

### Layer 5 Prototype: Implemented

Scope:

- named rooms/zones
- manual room/zone editor
- room/zone target preview
- translate room/zone requests into Layer 4 coverage goals
- hydrate active and terminal room/zone missions from adapter snapshots
- expose room/zone pause, resume, cancel, retry step, and skip step through
  normalized runtime mission actions
- preserve room/zone annotation identity and coverage-style result details in
  normalized mission snapshots
- keep public shape backend-neutral even if backend input helps segmentation

### Layer 6 Milestone: Planned

Scope:

- VM-managed Valetudo integration runtime
- Valetudo backend adapter
- real hardware reachability
- status/capability normalization
- first basic command
- later: run Layer 4 and Layer 5 flows against hardware

## Non-Goals For First Milestones

- perfect simulation of brushes, dust bin, water tank, or consumables
- support for every vacuum vendor
- exact reproduction of a commercial vacuum UI
- hardware procurement decisions
- forcing OpenClaw to be the foundation before the robot stack is stable
- production-grade room cleaning UI before coverage is production-ready
- production-grade zone editor before coverage and segmentation boundaries are
  clear
- scheduling and consumables before the core mission lifecycle is stable

## Why TurtleBot4

TurtleBot4 is a good stand-in because it provides most surfaces needed to
validate the robotics side:

- differential-drive motion
- lidar, IMU, odometry, and camera streams
- hazard and contact-related signals
- docking-related topics
- strong ROS 2 compatibility
- a simulation path we control inside this repo and VM environment

TurtleBot4 is still not a vacuum. It does not natively model:

- cleaning coverage
- room and zone semantics
- suction or water modes
- consumables or maintenance state
- vendor-specific charging, mapping, and docking behavior

Those concepts belong in repo-owned layers above the robot backend.

## Documentation Ownership

Keep the docs separated this way:

- `VACUUM_STACK_PLAN.md`: architecture, product boundary, layer plan,
  capability model, contract, milestones, non-goals.
- `steps.md`: current implementation progress, completed work, validation,
  runtime caveats, open work.
- `extension.md`: VS Code extension reference, file map, endpoints, topics,
  panels, bridge behavior, and extension follow-up work.

When updating docs:

- do not duplicate long topic/file lists into the architecture doc unless they
  define a contract boundary
- do not put future architecture decisions into `steps.md`
- do not put broad product-layer planning into `extension.md`
- preserve implementation evidence in `steps.md`
- preserve extension operational facts in `extension.md`
- preserve cross-layer design decisions here
