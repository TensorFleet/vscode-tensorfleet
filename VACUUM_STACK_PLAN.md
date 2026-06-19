# Vacuum Stack Architecture Reference

This document keeps durable architecture and contract details for the vacuum
stack in `~/vscode-tensorfleet`. It is limited to implementation facts and
cross-layer rules that later changes must preserve.

Extension file maps, panel entrypoints, ROS topics, and bridge endpoints live
in `extension.md`.

## Product Boundary

The product-facing boundary is the repo-owned `vacuum_adapter` contract.

Backend differences surface through:

- capability descriptors
- normalized state
- explicit unsupported/unavailable command results
- diagnostics that do not drive product behavior

No client above the adapter may depend on TurtleBot4-specific topics, helper
services, node APIs, Nav2 internals, raw Valetudo capability names, or vendor
model names.

TurtleBot4-specific helpers, including `TurtleBot4Navigator`, are backend
implementation details. They can be used inside the TurtleBot4 backend but must
not define the public product contract.

Valetudo-specific capability names/classes may appear in backend diagnostics and
adapter fixtures. Product UI behavior must branch only on normalized adapter
state and normalized capability descriptors.

## Runtime Shape

Current architecture:

```text
VS Code extension / product UI
  -> vacuum_adapter contract
     -> TurtleBot4/Nav2 backend adapter
        -> VM TurtleBot4 simulation runtime
     -> Valetudo backend adapter
        -> VM-managed Valetudo integration runtime
```

Boundary ownership:

```text
VM owns backend runtime/integration services.
vacuum_adapter owns product-facing contract, capabilities, commands, and state.
```

VM-owned runtime services include:

- TurtleBot4/Nav2 runtime
- Foxglove/ROS bridge
- Valetudo integration runtime
- MQTT broker/client when configured
- hardware discovery/config
- runtime health checks

Product/shared layer owns:

- `vacuum_adapter` contract
- normalized state model
- capability descriptors
- command semantics
- UI-facing assumptions
- coverage and room/zone logic above the adapter

## Core Architecture Rules

- ROS 2 is the robotics foundation.
- TurtleBot4 is a development backend, not the public product contract.
- Higher layers depend on adapter capabilities and normalized state.
- Product/UI clients branch on capability descriptors and command availability,
  not backend names.
- Public contract files own backend-neutral state and command types.
- Backend/runtime-specific imports belong inside backend mappers and dispatchers.
- Setter commands use explicit payload shapes.
- Long-running robot behavior belongs in the VM runtime or backend process, not
  React/webview hooks.
- Auto mapping is VM-owned and exposed through backend-neutral capabilities and
  commands.
- Coverage and room/zone semantics sit above the adapter.
- Standard ROS 2 and Nav2 interfaces are reused where they fit.
- Simulation remains a regression backend behind the same public contract as
  real integrations.
- Say "Valetudo integration runtime in the VM," not "Valetudo in the VM."
- Valetudo runs on the robot for the real-hardware path; the VM hosts only the
  TensorFleet Valetudo integration runtime.

## Runtime-Owned Mission Contract

All long-running robot behavior is modeled as a mission. The UI may create
drafts, submit intent commands, and render snapshots, but it does not own active
mission execution.

Mission types:

- `mapping`
- `navigation`
- `coverage`
- `return_to_dock`
- `room_cleaning`
- `zone_cleaning`
- `hardware_cleaning`

Mission statuses:

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

`snapshot.mission` is the legacy coarse state used by existing UI.
`snapshot.activeMission` and `snapshot.missions` are the normalized
runtime-owned mission surfaces.

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
When a webview opens, it renders from adapter/runtime snapshots.
It does not reconstruct active mission authority from React state.
```

Ownership rule:

```text
Before Start: UI may own draft state and local preview.
After Start: runtime/backend owns confirmed mission state.
```

Current mission implementation facts:

- Mapping follows the runtime-owned model because autonomous exploration is
  VM-owned.
- TurtleBot4/Nav2 navigation is runtime-owned: UI submits `start_navigation`,
  the VM mission runtime owns the Nav2 goal, and the adapter hydrates
  destination/progress/action state from `/vacuum_mission/status` and
  `/vacuum_mission/get_snapshot`.
- Terminal navigation destinations can be dismissed in UI as presentation state
  without clearing runtime mission history.
- TurtleBot4/Nav2 Clean Area execution is runtime-owned: UI submits
  `start_coverage`, the VM mission runtime owns Nav2 waypoint sequencing and
  progress, and the adapter hydrates coverage mission state from
  `/vacuum_mission/status` and `/vacuum_mission/get_snapshot`.
- Coverage route generation and per-cell overlay/progress details hydrate from
  runtime snapshot data after start.
- Room/zone cleaning uses the same mission path: UI submits
  `start_room_cleaning` or `start_zone_cleaning`, the TurtleBot4/Nav2 backend
  maps the annotation to a coverage target privately, and UI hydrates active and
  terminal summaries from `snapshot.activeMission` and
  `snapshot.missions.recent`.
- MCP does not currently expose `vacuum_start_room_cleaning` or
  `vacuum_start_zone_cleaning`; those product starts remain deferred in the MCP
  surface.
- Room/zone recovery controls are runtime/action gated through
  `activeMission.availableActions` and adapter capabilities.
- Coverage-style terminal results may carry details for cleaned area, remaining
  area, skipped area, skipped reasons, route completion, and
  coverage-threshold status.

## Vacuum Adapter Contract

Adapter files:

- `panels-standalone/src/vacuum-adapter/adapter.ts`
- `panels-standalone/src/vacuum-adapter/state.ts`
- `panels-standalone/src/vacuum-adapter/capabilities.ts`
- `panels-standalone/src/vacuum-adapter/commands.ts`
- `panels-standalone/src/vacuum-adapter/errors.ts`
- `panels-standalone/src/vacuum-adapter/useVacuumAdapter.ts`
- `panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/`
- `panels-standalone/src/vacuum-adapter/backends/valetudo/`

Public adapter shape:

```ts
type VacuumAdapter = {
  snapshot: VacuumAdapterSnapshot;
  sendCommand: (command: VacuumCommand) => Promise<VacuumCommandResult>;
};
```

`VacuumAdapterSnapshot` currently includes:

- `identity`
- `availability`
- `capabilities`
- `health`
- `source`
- `dock`
- `cleaningSettings`
- `maintenance`
- `statistics`
- `attachments`
- `diagnostics`
- `map`
- `pose`
- `navigation`
- `activity`
- `mission`
- `activeMission`
- `missions`
- `mapping`
- `readiness`
- `fault`
- `battery`

Core normalized surfaces:

- robot identity
- connectivity / availability
- pose
- odometry-derived/navigation state
- map
- battery state
- mission state
- fault state
- capability descriptors
- readiness evidence
- navigation state and progress
- mapping state when mapping is supported
- dock state when supported
- cleaning settings when supported
- maintenance/consumable state when supported
- current cleaning statistics when supported
- robot-side attachment/material state when supported
- dock component/material state when supported

Standard ROS 2 / Nav2 types are reused or mapped where they fit:

- `nav2_msgs/action/NavigateToPose`
- `sensor_msgs/msg/BatteryState`
- `nav_msgs/msg/OccupancyGrid`
- `nav_msgs/msg/Odometry`
- `geometry_msgs/msg/PoseStamped`
- `sensor_msgs/msg/Image` when a backend meaningfully exposes a camera feed

Vacuum-specific normalized surfaces cover concepts not cleanly expressed
through standard ROS 2/Nav2 types:

- dock state
- mission lifecycle state
- pause / resume / stop mission controls
- room / segment cleaning requests
- zone cleaning requests
- backend capability advertisement
- cleaning mode / fan / water presets
- consumable and maintenance state

## Capability Model

The vacuum contract is capability-driven. Every backend advertises capability
descriptors explicitly. UI controls and workflows branch on these descriptors
and current action availability.

Capability names are defined in
`panels-standalone/src/vacuum-adapter/capabilities.ts`:

- `mission_state`
- `start_navigation`
- `go_to_location`
- `cancel_navigation`
- `manual_control`
- `map`
- `mapping_session`
- `auto_mapping`
- `coverage_mission`
- `map_annotations`
- `room_semantics`
- `zone_semantics`
- `room_cleaning`
- `zone_cleaning`
- `pose`
- `navigation_status`
- `start_coverage`
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
- `dock_state`
- `segment_cleaning`
- `fan_speed`
- `water_usage`
- `battery`
- `consumables`
- `statistics`
- `attachments`
- `dock_components`
- `events`
- `fault_state`

Capability descriptor shape:

```ts
type CapabilitySupport = {
  supported: boolean;
  source?: "turtlebot4_nav2" | "valetudo" | "mock";
  backendCapability?: string; // deprecated compatibility mirror only
  commands?: string[];
  attributes?: string[];
  notes?: string;
  status?: "supported" | "unsupported" | "unavailable" | "detected_not_ready";
  available?: boolean;
  availabilityReason?: string;
  reasons?: Array<{ code: string; message: string }>;
};
```

Capability rules:

- Clients branch on capability descriptors, not backend names.
- Capability support is separate from current command availability.
- Reconnect or backend reconfiguration may require capability refresh.
- Vendor-specific features may exist in diagnostics but must not silently
  redefine the shared contract.
- Unsupported capabilities should be advertised explicitly.
- Unsupported commands should return structured unsupported results.

Private mapping examples:

- Valetudo `BasicControlCapability` -> `start_cleaning` / `pause` / `stop` /
  `return_to_dock`
- Valetudo `GoToLocationCapability` -> `go_to_location`
- Valetudo `MapSegmentationCapability` -> `segment_cleaning`
- Valetudo `ZoneCleaningCapability` -> `zone_cleaning`
- Valetudo `FanSpeedControlCapability` -> `fan_speed`
- Valetudo `WaterUsageControlCapability` -> `water_usage`
- Nav2 `NavigateToPose` -> `go_to_location` and `start_navigation`
- VM `/vacuum_mapping/*` services and `/vacuum_mapping/status` ->
  `mapping_session` / `auto_mapping`

Do not expose public flags such as:

- `BasicControlCapability`
- `MapSegmentationCapability`
- `ZoneCleaningCapability`
- `FanSpeedControlCapability`
- `WaterUsageControlCapability`

## Commands

Public command names are defined in
`panels-standalone/src/vacuum-adapter/commands.ts`:

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

Payload rules:

- `go_to_location` carries a backend-neutral target pose.
- `start_navigation` carries a backend-neutral navigation intent and is the
  preferred runtime-owned navigation command.
- `start_coverage` carries a backend-neutral area target; backend route
  generation and progress snapshots remain runtime-owned after start.
- `save_map_annotation` and `delete_map_annotation` carry product-level map
  annotation state, not backend room/segment identifiers.
- `start_room_cleaning` and `start_zone_cleaning` carry saved map annotations;
  backend route generation and mission progress remain runtime-owned after
  start.
- Mapping commands carry mode/name where needed.
- Setter commands carry selected values explicitly.
- Unsupported commands fail predictably.
- Streaming teleop is not forced into the one-shot `sendCommand` model.

Unsupported operation rules:

- Advertise unsupported capabilities as unsupported/unavailable.
- Return explicit unsupported command results.
- Do not hide unsupported operations silently when a caller dispatches them.
- Do not infer support from backend type.
- Keep higher-level workflows branching on capability descriptors.

## TurtleBot4/Nav2 Backend

The TurtleBot4/Nav2 backend normalizes the simulation runtime into
`vacuum_adapter`.

Implementation facts:

- `useTurtleBot4Nav2Adapter` wraps `useNav2Runtime`.
- Adapter snapshot exposes availability, identity, pose, map, navigation,
  mapping, mission, battery, readiness, fault, and capabilities.
- `Vacuum Control` consumes the adapter instead of raw Nav2 runtime.
- `start_navigation`, `start_coverage`, and `cancel_mission` dispatch through
  `adapter.sendCommand`; `cancel_navigation` remains as a compatibility
  fallback.
- MCP exposes `vacuum_start_navigation` as the first simulation-only
  movement-start write tool; it requires a target, runs readiness gates, and
  dispatches only through the product-level vacuum command route.
- MCP exposes `vacuum_start_clean_area` as the second simulation-only
  movement-start write tool; it accepts only simple rectangles, runs Clean Area
  readiness gates, and dispatches only the normalized `start_coverage` command
  through the product-level vacuum command route.
- TurtleBot4/Nav2 navigation missions hydrate through
  `/vacuum_mission/status` and `/vacuum_mission/get_snapshot`.
- Command dispatch is extracted into a pure helper for regression tests.
- Vacuum-only commands unsupported by TurtleBot4/Nav2 fail explicitly.

Supported TurtleBot4/Nav2 capability surfaces include:

- `map`
- `pose`
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

Unsupported TurtleBot4/Nav2 vacuum surfaces include:

- `start_cleaning`
- `segment_cleaning`
- `fan_speed`
- `water_usage`
- `consumables`
- real dock behavior

## Mapping And Map State

Mapping and map state are exposed through the adapter:

- `snapshot.map.grid`
- `snapshot.map.metadata`
- `snapshot.map.annotations`
- `snapshot.mapping`

Mapping state values:

- `idle`
- `manual_mapping`
- `auto_mapping`
- `paused`
- `needs_assistance`
- `review`
- `accepted`
- `discarded`
- `error`

Mapping status fields include:

- mode
- state reason
- known/unknown ratio
- frontier count
- visited/failed goal counts
- active goal
- last error
- update timestamp
- persistence mode
- accepted session flag
- saved/loaded map paths
- last saved time
- save/load errors
- active map name
- saved map summaries

Boundary rules:

- UI rendering may keep direct diagnostic subscriptions for overlays.
- Product workflows consume adapter-level map/session state.
- Coverage route generation consumes normalized map and pose above the adapter.
- Autonomous exploration stays in the VM runtime so it survives panel reloads,
  UI closure, websocket reconnects, and extension restarts.

## Coverage

Coverage is a runtime-owned mission after start. The UI owns draft selection and
local preview before start.

Current Clean Area behavior:

- `Vacuum Control` has `Mapping`, `Navigate`, and `Clean Area` modes.
- `MapCanvas` supports drawing, moving, and resizing rectangular clean-area
  selections.
- Clean-area selections are validated against map bounds and occupancy data.
- `cleanAreaProfile.ts` owns swath width, overlap, navigation goal tolerance,
  boundary margin, minimum useful region size, completion threshold, derived
  lane spacing, and boundary extension.
- Default profile preserves the 0.30 m simulation swath.
- `cleanAreaPlanner.ts` builds a swath-overlap lawnmower waypoint preview.
- The planner chooses the longer axis for passes.
- Sampled lanes are clipped to known free occupancy-grid cells.
- Boundary pass endpoints are extended to compensate for Nav2 goal tolerance.
- Runs start through
  `adapter.sendCommand({ command: "start_coverage", ... })`.
- The VM mission runtime privately sequences Nav2 goals for TurtleBot4/Nav2.
- `cleanAreaCoverage.ts` classifies cells as cleanable, occupied, unknown,
  out-of-bounds, too small, remaining, or covered.
- Cleanable cells are decomposed into connected regions.
- Tiny disconnected regions are skipped.
- Active progress hydrates from `snapshot.activeMission`.
- Runtime progress is covered square meters from robot footprint history in map
  frame.
- `MapCanvas` renders remaining, covered, excluded, skipped, and footprint
  overlays.
- `CleanAreaCard` reports percentage, cleaned area, remaining area, skipped
  area, waypoint progress, pass count, and route status.
- States include editing, confirmed, preparing, running, paused, canceling,
  completed, failed, and canceled.
- Operators can pause, resume, cancel, retry waypoint, skip waypoint, and clear.

Implementation limits:

- Runtime route generation is row-level occupancy-clipped.
- Runtime coverage progress is first-pass footprint-history accounting.
- Route generation is not component-level area decomposition.

## Room / Zone Semantics

Room and zone semantics are product-facing. Backend room/segment concepts must
be normalized before product UI consumes them.

Current behavior:

- `Vacuum Control` has a `Rooms / Zones` mode.
- Operators can draw a rectangular room/zone draft on the map using the shared
  editable rectangle behavior from Clean Area.
- Operators can name the draft, choose `Room` or `Zone`, save it, select it,
  and delete it.
- Saved annotations are exposed as `snapshot.map.annotations`.
- Selecting a saved room/zone converts its product-level annotation into the
  same coverage target preview used by Clean Area.
- Selected targets are highlighted on the map with route and per-cell
  cleanability preview.
- The side panel reports whether the selected target is cleanable, partially
  cleanable, or invalid.
- The product runtime path can start `start_room_cleaning` or
  `start_zone_cleaning` from a saved room/zone selection, but MCP room/zone
  start tools are not exposed yet.
- The TurtleBot4/Nav2 adapter translates that intent into a runtime-owned
  coverage mission request while preserving room/zone label and mission intent
  in request payload and optimistic mission snapshot.
- Active room/zone cleaning hydrates from `snapshot.activeMission`.
- Pause, resume, cancel, retry, and skip route through normalized mission
  actions.
- Terminal room/zone results hydrate through `snapshot.missions.recent`.
- Runtime room/zone snapshots preserve product-level annotation metadata:
  id, kind, name, and map id.
- Runtime coverage-style results preserve cleaned/remaining/skipped area,
  skipped-reason counts, route completion, and threshold status when available.
- Recent mission labels can distinguish cleaned from partially cleaned based on
  normalized result details.
- TurtleBot4/Nav2 persists annotations in webview storage keyed to the active
  map identity when available.

Implementation limits:

- Annotation durability is not VM/runtime-owned.
- Durable mission history is not VM/runtime-owned.
- Geometry is rectangle-only.
- Valetudo annotation/room/zone semantics are unsupported until a backend maps
  them into normalized product state.

## Valetudo Backend

The Valetudo backend keeps the extension UI backend-neutral. The product UI
consumes `useVacuumAdapter`; it does not call Valetudo HTTP, Valetudo MQTT, VM
runtime endpoints, or raw Valetudo source endpoints directly.

Runtime path:

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

Runtime endpoints consumed by the adapter:

```text
GET  /api/v1/valetudo/health
GET  /api/v1/valetudo/snapshot
POST /api/v1/valetudo/command
```

Runtime source modes:

- `fixed_mock`: VM-owned fixed mock state.
- `valetudo_mock_http`: HTTP mock source using `VALETUDO_MOCK_SOURCE_URL`.
- `valetudo_mock_mqtt`: MQTT cache/source diagnostics for mock-shaped data.
- `valetudo_http`: Valetudo HTTP source config path requiring
  `VALETUDO_SOURCE_URL`; there is no hardcoded robot address.

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

Adapter runtime configuration comes from injected window values:

```text
window.TENSORFLEET_VM_MANAGER_URL
window.TENSORFLEET_VALETUDO_RUNTIME_URL
window.TENSORFLEET_VALETUDO_RUNTIME_ROUTE_MODE
window.TENSORFLEET_VACUUM_BACKEND
window.TENSORFLEET_JWT
```

Valetudo runtime behavior:

- Missing HTTP source config, unreachable source, stale source, malformed source
  responses, missing capability data, and reachable sources without
  `BasicControlCapability` map to stable unavailable/degraded snapshots and
  passive diagnostics.
- Runtime diagnostics expose runtime online state, source reachability, source
  staleness, supported normalized capability inputs, detected-but-not-ready
  Valetudo capabilities, basic command availability, and segment target
  availability.
- The adapter and UI do not know whether runtime state came from HTTP, MQTT, or
  fixed mock data except through normalized `availability`, `health`, `source`,
  `capabilities`, `battery`, `dock`, `activity`, `cleaningSettings`, and
  `maintenance`.

Valetudo adapter behavior:

- VM-managed runtime snapshots and commands map into normalized adapter state.
- Supported commands route through the Valetudo runtime command endpoint:
  `start_cleaning`, `pause`, `resume`, `stop`, `return_to_dock`,
  `set_fan_speed`, and `set_water_usage`.
- Command results normalize unsupported, unavailable, invalid-state, failed,
  and successful outcomes.
- Fan speed and water usage controls use normalized current values and options.
- Maintenance/consumables display uses normalized `snapshot.maintenance`.
- Map-navigation, coverage, room, and zone workflows remain capability-gated
  when unsupported.

Valetudo-backed normalized surfaces include:

- robot identity
- availability/connectivity
- runtime/source health
- stale, unreachable, and offline source handling
- activity/state
- battery
- dock/charging
- capability descriptors
- cleaning settings
- maintenance/consumables
- current cleaning statistics
- robot-side attachments/materials
- dock-side components/materials
- layered map metadata, static preview layers/entities, and target inventory
  when fresh normalized map data exists
- command result normalization

Unsupported Valetudo product surfaces in the current adapter path include:

- full interactive Valetudo map support (`capabilities.map.supported` remains
  false)
- robot movement visualization
- go-to/navigation
- mapping
- Clean Area execution
- zone cleaning
- room editor and map annotation editing for Valetudo targets
- consumable reset commands

Implemented Valetudo/no-map UI facts:

- The basic robot profile sidebar is grouped as Operate, Readiness, Configure,
  Maintain, and Context.
- Operate renders Robot Overview, Basic Cleaning Controls, Current Statistics
  when supported, a fallback static Map Preview when the main preview is not
  visible, and Map Targets when normalized target rows exist.
- Readiness renders Battery and Dock plus Attachments and Dock Components when
  their normalized capabilities and rows exist.
- Configure renders fan speed and water usage controls from normalized option
  values.
- Maintain renders read-only consumables; reset commands are not implemented.
- A static Valetudo layered preview renders in the main workspace when
  `snapshot.map.layeredPreview` is renderable and the normal occupancy-grid
  `MapCanvas` path is unavailable. The sidebar `MapPreviewCard` is only a
  fallback to avoid mounting the same SVG preview twice.
- Map Targets are read-only. Hover/focus and click selection highlight matching
  normalized preview geometry when available and show selected target details.
- Target presence, preview data, segments, zones, or metadata do not enable
  cleaning, go-to, Clean Area, or full map commands.
- Missing, stale, unreachable, malformed, or all-invalid map data fails closed:
  no preview or targets are exposed as current truth.

## Operator Surface Positioning

The VS Code extension is the first-mile UI for bringup and validation.
`Vacuum Control` is the primary product/operator surface. Supporting panels are
useful for runtime validation, but they are not the primary vacuum product
surface.

Operator workflows should expose vacuum-specific controls and status through
the adapter:

- start mission
- pause / resume / stop
- return to dock
- room or zone selection
- mission state
- battery and charging status
- fault and recovery state

OpenClaw or other higher-level workflow tools should sit above the
vacuum-facing contract rather than directly on TurtleBot4, Nav2, or Valetudo
internals.

## Backend Reference Links

Primary upstream references:

- [TurtleBot4 Navigation](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html)
- [TurtleBot4 Navigator](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/turtlebot4_navigator.html)
- [Valetudo capabilities overview](https://valetudo.cloud/pages/usage/capabilities-overview.html)
- [Valetudo MQTT implementation details](https://valetudo.cloud/pages/development/mqtt/)
- [Valetudo project](https://github.com/Hypfer/Valetudo)
