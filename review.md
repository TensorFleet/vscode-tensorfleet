# Post-Layer-6A Vacuum Control Data Inventory Review

Current review date: 2026-06-11.

## 1. Current decision

This is a post-Layer-6A UI data inventory and review. Layer 6A is complete for
the Valetudo mock/runtime/adapter/UI path; this pass does not change product
code and does not add UI components.

The purpose is to collect real data first, then decide which Vacuum Control
sidebar cards to build next. The review is based on current code, current tests,
current docs, and visible local screenshots. It must not claim support from
Valetudo screenshots or raw backend data unless the value is normalized through
`vacuum_adapter`.

Decision for the next implementation pass:

- Safe-now UI work should use existing normalized adapter fields only:
  identity, availability, health/source, activity, dock, battery, basic command
  capability availability, fan speed, water usage, consumable display, no-map
  state, faults, and unsupported-workflow explanations.
- Adapter work is required before product UI exposes go-to, segments, zones,
  rooms, statistics, scheduling, advanced dock features, voice, Wi-Fi, firmware,
  logs, model settings, raw map entities, or consumable reset actions.
- Runtime/mock work is required when the value does not appear in the runtime
  snapshot at all, even if Valetudo can expose it upstream.
- Raw Valetudo capability names, HTTP routes, MQTT topics, source URLs, and
  runtime transport diagnostics stay diagnostics-only.

## 2. Valetudo UI functional reference from attached screenshots

No separate attached Valetudo screenshot image files were visible in this
workspace during this pass. Local June 10 screenshots inspected from
`~/Pictures/Screenshots` showed TensorFleet Vacuum Control no-map behavior, not
Valetudo UI. Therefore, this section uses only functional categories explicitly
called out in `goal.md` as reference-only categories. It does not describe a
layout to copy, does not require visual parity, and does not license copying
Valetudo styling, spacing, grouping, icons, or UX.

Reference-only functional categories:

- Basic cleaning controls: start, pause, resume, stop, return-to-dock/home.
- Robot status: activity/state, fault/error state, dock state, charging state.
- Battery: level and charging state.
- Cleaning settings: fan speed and water usage presets.
- Consumables/maintenance: main brush, side brush, filter, sensor cleaning, mop
  pad, detergent, dustbin or other consumables when a source reports them.
- Attachments/materials: dustbin, water tank, mop pad, detergent, and similar
  equipment indicators when normalized data exists.
- Map/area concepts: map availability, rooms, zones, segments, map metadata,
  go-to targets, and cleaning areas.
- Scheduling and settings: schedules, do-not-disturb, voice/speaker, Wi-Fi,
  firmware/updater, logs, model-specific settings.
- Advanced dock features: auto-empty, mop wash, mop dry, water refill.
- Statistics: current and total cleaning statistics.

## 3. Actual mock/runtime data inventory

Runtime endpoints and source modes:

| endpoint / mode | real fields found | product status |
|---|---|---|
| `GET /api/v1/valetudo/health` | `runtime.{id,version,status}`, `source.{kind,status,stale,lastSeenAt}`, `updatedAt` | Runtime health source; normalized by adapter into `snapshot.health` and `snapshot.source`. |
| `GET /api/v1/valetudo/snapshot` | Full runtime snapshot: runtime, backend, robot, source, connectivity, state, optional battery, optional dock, optional cleaning settings, optional maintenance, command availability, capability diagnostics, diagnostics, raw diagnostics, updated timestamp | Primary adapter input. |
| `POST /api/v1/valetudo/command` | `{ok,status,command,message,reason?,code?,updatedAt,diagnostics?}` | Adapter maps to normalized command result and refreshes snapshot. |
| `fixed_mock` | Fixed runtime-owned state and capabilities without external source calls | Mock-backed product-ready source for no-map UI. |
| `valetudo_mock_http` | HTTP source at `VALETUDO_MOCK_SOURCE_URL`, `VALETUDO_SOURCE_URL`, or default `http://172.16.0.1:8081`; source kind normalizes as `valetudo_mock` | Mock HTTP path; product-ready only after runtime/adapter normalization. |
| `valetudo_http` | Real Valetudo HTTP source from required `VALETUDO_SOURCE_URL`; missing URL fails closed | First hardware validation path, not broad hardware support. |
| `valetudo_mock_mqtt` | MQTT cache from `$name`, `$nodes`, `StatusStateAttribute`, `BatteryStateAttribute`, and `DockStatusStateAttribute` topics | Mock MQTT snapshot path; transport remains diagnostics/internal. |
| rejected `valetudo_mqtt` mode | Falls back to mock HTTP and does not enable production MQTT through `VALETUDO_MQTT_ENABLED` | Prevents production MQTT claims. |

Runtime snapshot fields:

- `runtime`: `id`, `version`, `status`.
- `backend`: currently `"valetudo"`.
- `robot`: `id`, `name`.
- `source`: `kind`, `status`, `stale`, `lastSeenAt`.
- `connectivity`: `reachable`, `online`.
- `state`: `value`, `label`, `started`, `paused`.
- `battery`: optional `level`, `charging`.
- `dock`: optional `state`, `docked`.
- `cleaningSettings.fanSpeed`: optional `current`, `options`.
- `cleaningSettings.waterUsage`: optional `current`, `options`.
- `maintenance.consumables[]`: `id`, `label`, optional remaining percent,
  remaining minutes, used minutes, total minutes, `status`, `detail`.
- `capabilities.commands`: command availability records keyed by runtime
  command name.
- `capabilities.diagnostics[]`: raw capability detection and implementation
  notes.
- `diagnostics`: mode, raw capability names, source diagnostics, readiness,
  last command, capability tiers, transports, notes.
- `rawDiagnostics`: source/runtime detail such as base URL, mock source kind,
  MQTT root, broker URL, last topic, and similar internal data.
- `updatedAt`: runtime snapshot timestamp.

Fixed mock data:

- Robot identity: `valetudo-fixed-mock-001`, `Valetudo Fixed Mock`.
- Default state: `docked`, label `Docked`, not started, not paused, docked,
  not charging.
- Scenarios: default/docked, cleaning, paused, returning, charging,
  source stale, source unreachable, fault/maintenance warning.
- Battery: level `82` in fixed mock source code, with charging derived from
  scenario/state.
- Dock: `docked`, `returning`, or `available` from mock state.
- Fan speed: current `medium`; options `off`, `min`, `low`, `medium`, `high`,
  `turbo`, `max`.
- Water usage: current `medium`; options `off`, `min`, `low`, `medium`, `high`,
  `max`.
- Consumables: main brush, side brush, filter, sensor cleaning, mop pad, with
  remaining minutes, total minutes, computed percent, status, and detail.
- Raw capability names include implemented product-ready data plus many
  diagnostic-only Valetudo capability names.

HTTP source data:

- Runtime fetches `/api/v2/robot` for manufacturer, model name, model details,
  implementation, stable robot ID, and display name.
- Runtime fetches `/api/v2/robot/state/attributes` and reads
  `StatusStateAttribute`, `BatteryStateAttribute`, and
  `PresetSelectionStateAttribute`.
- Runtime fetches `/api/v2/robot/capabilities` for raw capability names.
- If `FanSpeedControlCapability` is present, runtime fetches
  `/api/v2/robot/capabilities/FanSpeedControlCapability/presets`.
- If `WaterUsageControlCapability` is present, runtime fetches
  `/api/v2/robot/capabilities/WaterUsageControlCapability/presets`.
- If `ConsumableMonitoringCapability` is present, runtime fetches
  `/api/v2/robot/capabilities/ConsumableMonitoringCapability` and optional
  `/properties`.
- Runtime does not fetch map payloads, segment geometry, zones, schedules,
  logs, Wi-Fi data, firmware data, voice settings, or dock advanced state into
  the adapter-facing snapshot.

Command availability and transitions:

- Supported runtime command names: `start_cleaning`, `pause`, `resume`, `stop`,
  `return_to_dock`, `set_fan_speed`, `set_water_usage`.
- Basic commands require `BasicControlCapability`.
- `set_fan_speed` requires `FanSpeedControlCapability`.
- `set_water_usage` requires `WaterUsageControlCapability`.
- Fixed mock `start_cleaning` changes state to `cleaning`.
- Fixed mock `pause` changes `cleaning` to `paused`.
- Fixed mock `resume` changes `paused` to `cleaning`.
- Fixed mock `stop` changes `cleaning`, `paused`, or `returning` to `idle`.
- Fixed mock `return_to_dock` changes to `returning`, then settles to `docked`
  on the next read.
- Invalid state returns `status:"failed"` and `code:"invalid_state"`.
- Missing capability returns `status:"unsupported"` and
  `code:"capability_unavailable"`.
- Source unreachable returns `status:"unavailable"` and
  `code:"source_unreachable"`.
- Stale MQTT command state returns `status:"unavailable"` and
  `code:"stale_source"`.
- Unknown commands return `status:"unsupported"` and
  `code:"unsupported_command"`.
- Invalid preset requests return `invalid_request` with `missing_value` or
  `invalid_value`.

Raw runtime capability inventory:

| raw Valetudo capability | runtime status | product status |
|---|---|---|
| `BasicControlCapability` | Implemented for start, pause, resume/start mapping, stop, home | Product-facing via normalized commands. |
| `BatteryStateCapability` | Implemented when battery exists | Product-facing via normalized battery state. |
| `FanSpeedControlCapability` | Implemented current/options/setter | Product-facing via normalized cleaning settings. |
| `WaterUsageControlCapability` | Implemented current/options/setter | Product-facing via normalized cleaning settings. |
| `ConsumableMonitoringCapability` | Runtime normalizes display status | Product-facing display only when consumable entries exist; reset actions absent. |
| `CurrentStatisticsCapability`, `TotalStatisticsCapability` | Detected in fixed mock raw names | Diagnostics-only. |
| `GoToLocationCapability` | Detected raw capability | Product workflow not implemented. |
| `MapSegmentationCapability` | Detected raw capability | Product segment target geometry and commands not implemented. |
| `ZoneCleaningCapability` | Detected raw capability | Product zone geometry and commands not implemented. |
| `MappingPassCapability`, `PersistentMapControlCapability`, `MapResetCapability`, `PendingMapChangeHandlingCapability` | Detected raw names | Diagnostics/deferred. |
| `AutoEmptyDockAutoEmptyIntervalControlCapability`, `AutoEmptyDockManualTriggerCapability` | Detected raw names | Diagnostics/deferred. |
| `MopDockCleanManualTriggerCapability`, `MopDockDryManualTriggerCapability` | Detected raw names | Diagnostics/deferred. |
| `CarpetModeControlCapability`, `CarpetSensorModeControlCapability`, `CollisionAvoidantNavigationControlCapability`, `ObstacleAvoidanceControlCapability`, `OperationModeControlCapability`, `PetObstacleAvoidanceControlCapability` | Detected raw names | Diagnostics/deferred. |
| `DoNotDisturbCapability`, `KeyLockCapability`, `LocateCapability`, `ManualControlCapability`, `SpeakerTestCapability`, `SpeakerVolumeControlCapability`, `VoicePackManagementCapability`, `WifiConfigurationCapability`, `WifiScanCapability` | Detected raw names | Diagnostics/deferred. |

## 4. Normalized adapter inventory

Shared `vacuum_adapter` snapshot fields currently available to product UI:

- `identity`: `{id,label,source,model?}`.
- `availability`: `{status,connected,detail?}`.
- `capabilities`: descriptor record for all shared capability names.
- `health`: runtime status, timestamp, detail.
- `source`: source kind/status/stale/last-seen/reason.
- `dock`: support flag, normalized dock state, charging, detail.
- `cleaningSettings`: `fanSpeed` and/or `waterUsage`, each with current value,
  normalized labeled options, readiness, status, detail.
- `maintenance`: consumable array with normalized display data.
- `diagnostics`: backend/runtime/source/capability/map/pose/navigation/mapping
  diagnostics, warnings, raw detail.
- `map`: readiness, receiving flag, detail, grid, metadata, annotations.
- `pose`: readiness, available flag, coordinates, detail.
- `navigation`: state, target, plan path, progress, active/sending/canceling.
- `activity`: normalized robot activity status, label, timestamp, source,
  reason, available actions, details.
- `mission`: legacy coarse state.
- `activeMission` and `missions`: runtime-owned mission snapshots.
- `mapping`: mapping state, saved maps, persistence, progress, errors.
- `readiness`: ready flag and blocking reasons.
- `fault`: readiness, fault messages, detail.
- `battery`: readiness, percentage, charging, detail.

Valetudo adapter values currently normalized:

- Identity from runtime robot ID/name.
- Availability from runtime connectivity.
- Runtime health from runtime status.
- Source kind/status/stale/last-seen/reason.
- Activity status from runtime state/source: unavailable, faulted, paused,
  returning, cleaning, charging, docked, idle.
- Activity available actions from normalized supported and available basic
  command capabilities.
- Battery percentage and charging when runtime reports battery.
- Dock state normalized to unknown, docked, undocked, returning, charging, or
  error.
- Faults from robot fault/error state, stale source, source unreachable,
  degraded runtime, and runtime offline last error.
- Basic command descriptors: `start_cleaning`, `pause`, `resume`, `stop`,
  `return_to_dock`, plus mission aliases `pause_mission`, `resume_mission`,
  `cancel_mission`.
- Fan speed and water usage descriptors, options, and setter commands.
- Consumables descriptor only when normalized maintenance entries exist.
- Active mission for non-idle hardware cleaning / return-to-dock state, with
  coarse status and actions.

Valetudo adapter values explicitly unavailable or diagnostics-only:

- `map`: always unavailable for Valetudo; `grid:null`,
  `metadata.hasMap:false`, no annotations.
- `pose`: unavailable; no product coordinates.
- `navigation`: idle/unsupported; go-to remains diagnostics-only.
- `mapping`: idle, persistence unsupported.
- `coverage_mission` and `start_coverage`: unsupported.
- `map_annotations`, `room_semantics`, `zone_semantics`, `room_cleaning`:
  unsupported.
- `segment_cleaning` and `zone_cleaning`: detected-not-ready when raw
  capability exists; still not product-ready.
- `manual_control`: unsupported for Valetudo.
- `retry_mission_step` and `skip_mission_step`: unsupported.
- Statistics, scheduling, DND, voice, Wi-Fi, firmware, logs, advanced dock,
  model settings, and consumable resets: no normalized public fields or
  commands yet.

## 5. Existing UI consumption inventory

Current Vacuum Control consumption points:

- `VacuumControlPanel` consumes `useVacuumAdapter`, not the raw Valetudo client.
- Adapter selector chooses `turtlebot4_nav2` or `valetudo`; product behavior
  then branches on normalized capabilities.
- Header and status strip use normalized identity, availability, source,
  health, activity, dock, battery, pose, map, readiness, and selected workflow
  state.
- `NoMapCanvasPlaceholder` renders when `capabilities.map.supported` is false.
- `BasicControlsCard` renders supported basic command descriptors and dispatches
  normalized `start_cleaning`, `pause`, `resume`, `stop`, and `return_to_dock`.
- `CleaningSettingsCard` renders `snapshot.cleaningSettings.fanSpeed` and
  `waterUsage` only when corresponding normalized capabilities are supported;
  it dispatches normalized `set_fan_speed` and `set_water_usage`.
- `MaintenanceCard` renders only normalized `snapshot.maintenance.consumables`
  when `capabilities.consumables.supported`.
- `BasicRobotStatusCard` uses normalized availability, health, source,
  activity, dock, battery, and fault.
- `MissionLifecycleCard` uses coarse mission, battery, dock state, and
  return-to-dock capability in map-supported workflow sections.
- `RecentMissionsCard` uses `snapshot.missions.recent`; Valetudo currently
  supplies no recent mission history.
- `UnsupportedFeatureCard` explains unavailable mapping/navigation/Clean
  Area/room-zone workflow capability notes.
- `MapCanvas` is mounted only when map is supported; it consumes adapter map and
  pose but also directly subscribes to ROS `/map`, costmaps, overlays, and
  camera through simulation-specific paths.
- `TeleopCard` directly publishes ROS `/cmd_vel_raw` and is gated by
  `manual_control`; Valetudo marks that unsupported.
- `CameraOverlay` is a ROS image topic surface inside `MapCanvas`, not a
  Valetudo product surface.

Current no-map behavior:

- Valetudo basic profile is detected when map, navigation, clean-area,
  rooms/zones, mapping, and manual-control capabilities are absent.
- For that profile, the main canvas shows a no-map placeholder instead of
  mounting `MapCanvas`.
- Sidebar shows Basic Cleaning, Cleaning Settings, Maintenance, and Robot
  status when normalized data supports them.
- Simulation-heavy components remain separate and hidden unless capabilities
  support them.

## 6. Capability-to-UI candidate matrix

| candidate card | purpose | current data availability | source | gate | safe next? | adapter work? | runtime/mock work? | notes and risks |
|---|---|---|---|---|---|---|---|---|
| Robot summary | Name, primary state, key detail | Available | Normalized adapter | `identity`, `activity` | Yes | No | No | Already partly exists; can be expanded without backend names. |
| Connection/source summary | Online, source status, stale reason | Available | Normalized adapter | `availability`, `source` | Yes | No | No | Keep transport URLs/topics out of product card. |
| Runtime health summary | Runtime online/degraded/offline | Available | Normalized adapter | `health` | Yes, if product-level | No | No | Avoid exposing runtime ID/version outside diagnostics. |
| Basic cleaning controls | Start/pause/resume/stop/home | Available | Normalized adapter | command capabilities | Yes | No | No | Already present; next work may polish state explanations. |
| Battery | Level and charging | Available when source reports battery | Normalized adapter | `battery` capability/state | Yes | No | No | Good candidate for a dedicated compact card. |
| Dock/charging | Docked/undocked/returning/charging | Available | Normalized adapter | `dock_state` | Yes | No | No | Dock state is inferred from runtime dock/battery. |
| Activity/state | Idle/cleaning/paused/returning/faulted | Available | Normalized adapter | `activity` | Yes | No | No | Product-safe. |
| Fault/error | Stale, unreachable, degraded, robot fault | Available | Normalized adapter | `fault_state` | Yes | No | No | Keep raw event payloads out. |
| Fan speed | Current preset and options | Available when runtime reports setting | Normalized adapter | `fan_speed` | Yes | No | No | Already present. |
| Water usage | Current preset and options | Available when runtime reports setting | Normalized adapter | `water_usage` | Yes | No | No | Already present. |
| Maintenance/consumables | Display remaining life | Available when runtime reports entries | Normalized adapter | `consumables` | Yes, display-only | No | No | No reset commands yet. |
| Main brush | Consumable line item | Available in fixed mock and HTTP source when present | Normalized adapter | `consumables` | Yes | No | No | Not guaranteed on every source. |
| Side brush | Consumable line item | Available in fixed mock and HTTP source when present | Normalized adapter | `consumables` | Yes | No | No | Handles side-left/right labels. |
| Filter | Consumable line item | Available in fixed mock and HTTP source when present | Normalized adapter | `consumables` | Yes | No | No | Display-only. |
| Sensor cleaning | Consumable line item | Available in fixed mock and HTTP source when present | Normalized adapter | `consumables` | Yes | No | No | Display-only. |
| Mop pad | Consumable line item | Available in fixed mock and HTTP source when present | Normalized adapter | `consumables` | Yes | No | No | Display-only. |
| Detergent | Label mapper exists | Only if source reports such consumable | Normalized adapter when present | `consumables` | Conditional | No | Maybe source fixture if desired | Do not show placeholder support. |
| Dustbin | Label mapper exists | Only if source reports such consumable | Normalized adapter when present | `consumables` | Conditional | No | Maybe source fixture if desired | Do not imply bin sensor state. |
| Water tank | Not currently a normalized field | Missing | Missing | none | No | Yes | Yes | Need attachment/material contract. |
| Mop state | Not currently normalized | Missing | Missing | none | No | Yes | Yes | Mop pad consumable is not mop mode/state. |
| Attachments | Not normalized | Missing | Missing | none | No | Yes | Yes | Needs generic attachment model. |
| Cleaning mode | Not normalized | Raw `OperationModeControlCapability` only | Raw Valetudo only | none | No | Yes | Yes | Deferred. |
| Operation mode | Raw capability detected | Raw Valetudo only | Raw Valetudo only | none | No | Yes | Yes | Deferred. |
| Current statistics | Raw capability detected | Diagnostics-only | Raw Valetudo only | none | No | Yes | Yes | Need runtime fields and adapter type. |
| Total statistics | Raw capability detected | Diagnostics-only | Raw Valetudo only | none | No | Yes | Yes | Need runtime fields and adapter type. |
| Unavailable workflows | Explain hidden advanced workflows | Available capability notes | Normalized adapter | unsupported descriptors | Yes | No | No | Good for narrow expectation-setting. |
| Map availability | Unsupported with detail | Normalized adapter | `map` | Yes | No | No | Only "not available", not map rendering. |
| Map metadata | `hasMap:false` only | Normalized adapter | `map` | No meaningful card | Yes | Yes | Product map needs grid/metadata. |
| Map rendering | No product map grid | Missing normalized data | Raw/mock only at best | `map.supported` | No | Yes | Yes | Do not build from raw Valetudo internals. |
| Rooms | Not normalized | Missing | Missing/raw segments only | `room_semantics` | No | Yes | Yes | Need target model. |
| Zones | Not normalized | Missing | Raw capability only | `zone_semantics` | No | Yes | Yes | Need geometry contract. |
| Segments | Raw capability only | Diagnostics-only | Raw Valetudo only | `segment_cleaning` detected-not-ready | No | Yes | Yes | Need segment targets and names. |
| Segment targets | Readiness says count 0 | Missing | Missing | none | No | Yes | Yes | No mock target list exists. |
| Go-to/location target | Raw capability detected | Unsupported | Raw Valetudo only | `go_to_location` detected-not-ready | No | Yes | Yes | Need pose/map/target contract and command routing. |
| Clean Area | Unsupported | Missing | Missing | `start_coverage` | No | Yes | Yes | Current implementation depends on map grid. |
| Scheduling | Not normalized | Missing | Missing/raw upstream possible | none | No | Yes | Yes | Deferred. |
| Do-not-disturb | Raw capability detected | Diagnostics-only | Raw Valetudo only | none | No | Yes | Yes | Deferred. |
| Voice/speaker | Raw capabilities detected | Diagnostics-only | Raw Valetudo only | none | No | Yes | Yes | Deferred. |
| Wi-Fi | Raw capabilities detected | Diagnostics-only | Raw Valetudo only | none | No | Yes | Yes | Deferred. |
| Firmware/updater | Not found in normalized data | Missing | Missing | none | No | Yes | Yes | Deferred. |
| Logs | Not found in normalized data | Missing | Missing | none | No | Yes | Yes | Deferred. |
| Advanced dock | Raw auto-empty/mop dock capabilities detected | Diagnostics-only | Raw Valetudo only | none | No | Yes | Yes | Needs model-specific safe command semantics. |
| Auto-empty | Raw auto-empty capabilities detected | Diagnostics-only | Raw Valetudo only | none | No | Yes | Yes | Deferred. |
| Mop wash/dry | Raw mop dock capabilities detected | Diagnostics-only | Raw Valetudo only | none | No | Yes | Yes | Deferred. |
| Water refill | Not currently found | Missing | Missing | none | No | Yes | Yes | Deferred. |
| Model-specific settings | Several raw config capabilities detected | Diagnostics-only | Raw Valetudo only | none | No | Yes | Yes | Avoid product UI until normalized. |

## 7. Long list of possible sidebar cards

Safe now with existing normalized mock-backed data:

- Robot Summary: identity label, primary normalized status, brief detail.
- Connection Summary: connection status and source reachable/stale state.
- Runtime/Source Health Summary: product-level runtime and source status.
- Basic Cleaning Controls: start, pause, resume, stop, return-to-dock.
- Battery Card: percentage and charging state.
- Dock Card: docked/undocked/returning/charging/error state.
- Activity Card: idle, cleaning, paused, returning, docked, charging, faulted.
- Cleaning Settings Card: fan speed and water usage preset controls.
- Maintenance Card: normalized consumable display.
- Fault Card: stale source, source unreachable, degraded runtime, robot error.
- Map Unavailable Card: explains no normalized map is available.
- Unavailable Workflows Card: explains navigation, Clean Area, rooms/zones, and
  segment workflows are deferred.

Possible now only as conditional rows inside safe cards:

- Main brush, side brush, filter, sensor cleaning, mop pad, detergent, dustbin:
  safe only when actual `maintenance.consumables[]` entries exist.
- Runtime degraded/offline details: safe if phrased as product health, not raw
  runtime IDs or transport endpoints.
- Source stale/unreachable reason: safe if using normalized `source.reason` and
  capability disabled reasons.

Needs adapter changes:

- Attachment status, dustbin installed/full, water tank installed, mop attached,
  mop active state, detergent tank state.
- Current statistics and total statistics.
- Consumable reset actions.
- Map metadata beyond "not available".
- Rooms, zones, segments, segment targets, go-to targets.
- Scheduling, do-not-disturb, locate, key lock, voice/speaker, Wi-Fi,
  firmware/updater, logs.
- Advanced dock cards: auto-empty, mop wash, mop dry, water refill.
- Model-specific settings and operation/carpet/obstacle/pet/collision settings.

Needs runtime/mock changes:

- Fetch and normalize map data, robot pose, segment geometry, named rooms,
  zones, go-to execution state, cleaning statistics, schedule data, DND state,
  voice/speaker state, Wi-Fi scan/config state, firmware/update/log state,
  advanced dock state, attachments, consumable reset endpoints, and
  model-specific settings.
- Add mock fixtures and tests for every new normalized field before product UI
  depends on it.

Should stay deferred:

- Anything that depends only on raw Valetudo capability names.
- Anything that would expose HTTP routes, MQTT topics, source URLs, broker URLs,
  or raw capability class names as product requirements.
- Any map/segment/zone/go-to implementation built directly from raw Valetudo
  internals without updating the adapter/runtime contract.
- Activity/history/recent-command cards unless explicitly approved later.
- Main-panel diagnostics panels.

## 8. Map feasibility

Current Valetudo product map feasibility: not ready.

Evidence:

- The Valetudo adapter returns `capabilities.map.supported=false`.
- The Valetudo adapter returns `snapshot.map.grid=null`.
- The Valetudo adapter returns `snapshot.map.metadata.hasMap=false`.
- The Valetudo adapter returns `snapshot.map.annotations=[]`.
- The Valetudo adapter diagnostics say map data is diagnostics-only until
  product map rendering is implemented.
- The runtime HTTP snapshot path fetches robot info, state attributes,
  capabilities, fan/water presets, and consumables; it does not fetch Valetudo
  map payloads into the adapter-facing snapshot.
- The runtime readiness summary keeps `segmentTargetCount=0`.
- Existing `MapCanvas` is simulation-heavy and subscribes to ROS `/map` and
  costmap topics; it is hidden for Valetudo no-map states.

Conclusion:

- Map UI cannot honestly be built for Valetudo without hardware or at least a
  runtime/mock/adapter map contract update.
- A "map unavailable" explanation can be improved now.
- Product map rendering from raw Valetudo map internals should not be
  implemented until the VM runtime exposes normalized map data and the adapter
  declares `capabilities.map.supported=true`.
- Valetudo map screenshots can be used only as a functional reference for
  categories such as map, rooms, zones, segments, and target selection. They are
  not a layout or UX source.

## 9. Exclusions for next UI pass

- No diagnostics panel in the main Vacuum Control panel.
- No activity/history/recent-command card unless the user later approves.
- No copied Valetudo UI/UX, layout, spacing, icons, grouping, or visual design.
- No raw Valetudo capability names in product controls.
- No HTTP/MQTT/source URL/broker/topic details in product controls.
- No hardware claims.
- No production MQTT claims.
- No map rendering, go-to, segment, room, zone, Clean Area, scheduling,
  advanced dock, voice, Wi-Fi, firmware, logs, or model settings without a
  normalized runtime/adapter contract.
- No consumable reset actions until runtime command semantics and adapter
  commands exist.

## 10. Recommended next implementation options

Ranked safe-now options using existing normalized data:

1. Split the current Robot status card into clearer Robot Summary, Battery, Dock,
   and Source/Health cards while keeping the same normalized inputs.
2. Improve Basic Cleaning disabled-state explanations using capability
   `availabilityReason` and structured reasons already present.
3. Keep Cleaning Settings as a dedicated card and refine fan/water option
   presentation without changing command behavior.
4. Keep Maintenance display-only, possibly grouping consumables by status while
   avoiding reset actions.
5. Improve the no-map/unavailable-workflows explanation so operators understand
   what is intentionally unsupported.

Needs adapter changes before UI:

- Statistics cards.
- Attachment/material cards.
- Consumable reset actions.
- Segment, room, zone, and go-to target cards.
- Schedule, DND, speaker, Wi-Fi, firmware, logs, advanced dock, and
  model-settings cards.

Needs runtime/mock changes before UI:

- Any value not present in `ValetudoRuntimeSnapshot`, including map data,
  segment targets, zones, statistics, schedules, attachment states, advanced
  dock states, logs, updater state, Wi-Fi scan/config, and voice settings.

Should stay deferred:

- Map rendering, segment cleaning, zone cleaning, room cleaning, Clean Area,
  go-to/navigation, advanced dock actions, production MQTT, and copied Valetudo
  parity work.

## 11. Validation

This is a docs/research pass only. Product code was not changed.

Required validation:

```sh
git diff --check
```

Full build/test suites are not required by this pass because no code changed.

## 12. Progress report update

`progress_report.md` was updated after this review. The report records:

- What changed.
- Which docs were updated.
- What data was inspected.
- Key findings.
- Validation performed.
- Remaining gaps.
- Next recommended step.

Future implementation runs must update `progress_report.md` every time.
