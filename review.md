# Vacuum Adapter Hardware Readiness Review

Scope: Milestone A only. This is an inventory and classification of the current `vacuum_adapter` contract and backend mappings. No adapter implementation changes are proposed here.

## Summary

The adapter is mostly moving in the right direction: the public contract is normalized, Valetudo raw capability names are backend-private, TurtleBot4/Nav2 details are mostly hidden in the simulation backend, and the UI appears to branch on normalized capability fields instead of backend names.

The main readiness gaps are contract granularity rather than architecture:

- Capabilities are static `supported: boolean` descriptors, not state-aware command availability.
- Command errors collapse too many cases into `unsupported`, `not_ready`, `backend_error`, and `invalid_request`.
- `VacuumAdapterSnapshot` requires advanced surfaces such as `map`, `pose`, `navigation`, and `mapping` even when the backend has none.
- Mission modeling is useful for runtime-owned simulation workflows, but Valetudo basic cleaning is currently forced into an `activeMission` shape without enough distinction between raw activity and product mission.
- Dock, maintenance, consumables, fan/water presets, rooms/segments, zones, stale-source state, runtime health, and diagnostics are under-modeled in the public snapshot.

## Adapter Surface Inventory

| current field/command/capability | current owner | backend-neutral? | TurtleBot4/Nav2-shaped? | Valetudo-ready? | keep/change/defer/remove | notes |
|---|---|---:|---:|---:|---|---|
| `VacuumAdapter` | public adapter | yes | no | yes | keep | Good minimal boundary: snapshot plus command dispatcher. |
| `VacuumBackendSource` | public capabilities | mostly | partly | partly | change later | Useful for diagnostics/identity, but product UI should not branch on it. |
| `VacuumAdapterBackendId` / backend selection | composition hook | no | no | yes | keep at composition boundary | Backend choice is dependency injection; acceptable outside product behavior. |
| `identity.id`, `identity.label`, `identity.model` | public snapshot | yes | no | yes | keep | Product-level robot identity. |
| `identity.source` | public snapshot | diagnostic | no | yes | keep diagnostic-only | Should not drive UI behavior. |
| `availability.status`, `connected`, `detail` | public snapshot | yes | no | yes | keep | Needs source/runtime split later. |
| `readiness.ready`, `blockingReasons` | public snapshot | yes | partly | partly | change | Currently broad adapter readiness; needs command-specific availability. |
| `capabilities` record | public snapshot | yes | no | yes | keep/change | Correct capability-first direction; add state availability and detected/product-ready status. |
| `CapabilitySupport.supported` | public capabilities | yes | no | partly | change | Boolean cannot express unsupported vs unavailable vs detected-but-not-product-ready. |
| `CapabilitySupport.backendCapability` | public capabilities | diagnostic | yes | yes | change | Useful in dev, risky in product contract; move under diagnostics or mark diagnostic-only. |
| `CapabilitySupport.commands`, `attributes`, `notes` | public capabilities | yes | no | yes | keep/change | Good, but command availability should reference safe/current state. |
| `map.grid`, `metadata`, `annotations` | public snapshot | yes | partly | no | keep optional/capability-gated | Product concepts are valid, but mandatory object shape is simulation-shaped. |
| `map.topic` | public snapshot | no | yes | no | move diagnostic-only | ROS topic leak in public map state. |
| `pose.source` | public snapshot | diagnostic | yes | partly | move diagnostic-only | Pose is product-level; source string can leak runtime/helper detail. |
| `navigation.backendGoalState` | public snapshot | diagnostic | yes | no | move diagnostic-only | Raw backend/action phase should not be product behavior. |
| `navigation.progress.navigationTime`, `estimatedTimeRemaining` as `unknown` | public snapshot | partly | yes | no | change | Needs normalized duration/seconds/null shape. |
| `mission` legacy state | public snapshot | yes | partly | partly | remove later | Duplicates `activeMission`/`missions`; keep as compatibility until UI moves off it. |
| `activeMission` | public snapshot | yes | no | partly | keep/change | Good runtime-owned abstraction, but should not be required for every hardware activity. |
| `missions.active`, `missions.recent` | public snapshot | yes | no | partly | keep | Useful product history; Valetudo has no recent missions yet. |
| `mapping` | public snapshot | partly | yes | no | make optional/gated | Strongly shaped by frontier exploration and map persistence runtime. |
| `fault` | public snapshot | yes | no | partly | keep/change | Needs fault severity, recoverability, assistance, and source freshness. |
| `battery` | public snapshot | yes | no | yes | keep/change | Missing charging source/dock relationship and health detail. |
| dock state | capability only | yes | no | under-modeled | add snapshot field | `dock_state` exists as capability but no `dock` snapshot exists. |
| source health/staleness | Valetudo runtime private, fault string public | yes | no | under-modeled | add snapshot field | Stale source should not be only a string fault. |
| runtime health | Valetudo runtime private | yes | no | under-modeled | add snapshot diagnostics/health | Needed to distinguish runtime offline from source unreachable. |
| consumables | capability only | yes | no | under-modeled | defer public state | Keep unsupported/detected until consumable schema exists. |
| fan/water settings | capability only | yes | no | under-modeled | defer public state | Need presets/current value before enabling UI. |
| diagnostics | backend/runtime private mostly | yes | no | yes | add explicit diagnostic boundary | Raw payloads are not product state but should be inspectable. |

## Capability Gap Table

| current field/command/capability | current owner | backend-neutral? | TurtleBot4/Nav2-shaped? | Valetudo-ready? | keep/change/defer/remove | notes |
|---|---|---:|---:|---:|---|---|
| `mission_state` | public capability | yes | no | yes | keep | Supported by both mappers as normalized state. |
| `start_navigation` | public capability | yes | partly | no | keep/change | Product-level point mission; Valetudo should stay unsupported until go-to workflow is product-ready. |
| `go_to_location` | public capability | yes | partly | deferred | keep/change | Currently direct Nav2 action path; for Valetudo detected-but-unsupported is correct. |
| `cancel_navigation` | public capability | yes | partly | deferred | keep | Direct navigation cancel; should be unavailable when no active navigation. |
| `manual_control` | public capability | yes | partly | no | change | Capability says command exists, but `sendCommand` returns invalid_request; model as streaming control surface, not command. |
| `map` | public capability | yes | partly | deferred | keep | Valetudo map rendering unsupported is correct for now. |
| `mapping_session`, `auto_mapping` | public capability | yes | yes | no | keep/defer | Product concepts are valid but simulation/runtime-specific implementation. |
| `coverage_mission`, `start_coverage` | public capability | yes | partly | deferred | keep/change | Generic area cleaning can fit Valetudo later via zones/segments, but not yet. |
| `map_annotations` | public capability | yes | partly | deferred | keep/change | Product-owned annotations are neutral; current persistence is VM-shaped. |
| `room_semantics`, `zone_semantics` | public capability | yes | partly | deferred | keep/change | Need distinction between product annotations and vendor segments/zones. |
| `room_cleaning`, `zone_cleaning` | public capability | yes | partly | deferred | keep/change | Good product capabilities; Valetudo must wait for segment/zone mapping. |
| `pose` | public capability | yes | partly | deferred | keep | Unsupported Valetudo pose is acceptable. |
| `navigation_status` | public capability | yes | partly | deferred | keep/change | Should not require Nav2-like action progress. |
| `pause_mission`, `resume_mission`, `cancel_mission` | public capability | yes | no | partly | keep/change | Needs state-aware availability and mapping to basic pause/stop when activity is active. |
| `retry_mission_step`, `skip_mission_step` | public capability | yes | yes | no | defer | Runtime-owned multi-step workflows only. |
| `start_cleaning`, `pause`, `resume`, `stop`, `return_to_dock` | public capability | yes | no | partly | keep/change | Valetudo basic control fits; `resume` is intentionally unsupported. Need current-state safety. |
| `dock_state` | public capability | yes | no | under-modeled | change | Capability is present, snapshot field missing. |
| `segment_cleaning` | public capability/command | partly | no | deferred | change | Public name is too vendor-shaped; prefer `room_cleaning`/`area_cleaning` public concepts, keep segment IDs backend-private or diagnostic. |
| `fan_speed`, `water_usage` | public capability | yes | no | deferred | keep/defer | Detected-but-unsupported is correct until presets/current values exist. |
| `battery` | public capability | yes | no | yes | keep | Basic battery mapping works. |
| `consumables` | public capability | yes | no | deferred | keep/defer | Detected-but-unsupported is correct. |
| `events` | public capability | partly | no | under-modeled | change | No public events/diagnostics surface yet. |
| `fault_state` | public capability | yes | no | partly | keep/change | Needs normalized fault codes/classes. |

## Command Semantics Table

| current field/command/capability | current owner | backend-neutral? | TurtleBot4/Nav2-shaped? | Valetudo-ready? | keep/change/defer/remove | notes |
|---|---|---:|---:|---:|---|---|
| `start_cleaning` | public command | yes | unsupported | yes | keep/change | Maps to Valetudo basic start; should be unavailable when already cleaning or source stale. |
| `pause` | public command | yes | unsupported | partly | keep/change | Maps to Valetudo basic pause; should be unavailable unless active clean/return/go-to can pause. |
| `resume` | public command | yes | unsupported | no | keep/defer | Unsupported until runtime confirms resume semantics. |
| `stop` | public command | yes | unsupported | yes | keep/change | Maps to Valetudo stop; distinguish stop cleaning vs cancel mission terminal result. |
| `return_to_dock` | public command | yes | unsupported | yes | keep/change | Maps to Valetudo home; should expose dock/returning activity even without Nav2 mission. |
| `go_to_location` | public command | yes | direct Nav2-shaped payload | deferred | keep/change | Coordinates are neutral, but frame/map requirements must be explicit. |
| `start_navigation` | public command | yes | VM mission-shaped | no | keep | Runtime-owned navigation mission; Valetudo unsupported for now. |
| `cancel_navigation` | public command | yes | direct Nav2 cancel | no | keep/change | Should be unavailable if no active navigation, not just service-supported. |
| `manual_control` | public command | partly | yes | no | change | It is a streaming/publisher behavior, not a one-shot `sendCommand` command. |
| `start_coverage` | public command | yes | VM coverage-shaped | deferred | keep/change | Area payload is product-level; route/coverage parameters are simulation-heavy but useful. |
| `pause_mission`, `resume_mission`, `cancel_mission` | public command | yes | no | partly | keep/change | For Valetudo, pause/cancel map to basic pause/stop only when active hardware activity exists. |
| `retry_mission_step`, `skip_mission_step` | public command | yes | VM mission-shaped | no | defer | Multi-step recovery commands belong to runtimes with step semantics. |
| `start_mapping`, `pause_mapping`, `resume_mapping`, `finish_mapping`, `discard_mapping`, `accept_map`, `load_map` | public commands | yes | yes | no | keep optional/defer | Product-level map lifecycle is valid, but current payload and persistence are VM-shaped. |
| `save_map_annotation`, `delete_map_annotation` | public commands | yes | partly | deferred | keep/change | Product annotations are neutral; Valetudo needs adapter-owned or vendor-mapped strategy. |
| `start_room_cleaning`, `start_zone_cleaning` | public commands | yes | VM coverage-shaped | deferred | keep/change | Good public commands if annotation/segment/zone resolution stays backend-private. |
| `segment_cleaning` | public command | no | no | deferred | remove or replace later | Vendor-shaped and has no payload; prefer backend-private mapping under room/zone cleaning. |
| `zone_cleaning` | public command | partly | no | deferred | remove or replace later | Duplicates `start_zone_cleaning` and has no geometry payload. |
| `set_fan_speed`, `set_water_usage` | public commands | yes | unsupported | deferred | keep/defer | Need preset descriptors and state-aware availability before enabling. |
| `VacuumCommandResult.ok` | public result | yes | no | yes | keep/change | Needs richer status enum. |
| `VacuumCommandErrorCode` | public result | partly | no | partly | change | Missing unavailable, invalid_state, stale_source, runtime_offline, source_unreachable, timeout, malformed_response, degraded_runtime, needs_assistance. |

## Snapshot and State Gap Table

| current field/command/capability | current owner | backend-neutral? | TurtleBot4/Nav2-shaped? | Valetudo-ready? | keep/change/defer/remove | notes |
|---|---|---:|---:|---:|---|---|
| `VacuumAvailabilityStatus` | public state | yes | no | yes | keep/change | Add runtime/source split later. |
| `VacuumReadinessState` | public state | yes | no | yes | keep | Good for per-surface readiness. |
| `VacuumNavigationState` | public state | yes | partly | deferred | keep/change | Should support simpler hardware go-to without Nav2 action details. |
| `VacuumMissionState` | public state | yes | no | partly | keep/change | Coarse activity is useful; legacy naming should become `activity` or `robotActivity`. |
| `VacuumMissionType` | public state | yes | partly | partly | keep/change | `hardware_cleaning` is a useful bridge but too vague for long term. |
| `VacuumMissionStatus` | public state | yes | no | partly | keep/change | Add `unavailable`? Terminal/result reason should carry more detail. |
| `VacuumMissionAction` | public state | yes | partly | partly | keep/change | Good state-aware action list; should be primary UI control source. |
| `VacuumMissionSnapshot.target: unknown` | public state | no | no | partly | change | Needs discriminated target shapes for location, area, room, zone, map, hardware activity. |
| `VacuumPoseCoordinates`, `VacuumGoalCoordinates` | public state | yes | partly | partly | keep/change | Coordinates need map/frame contract when map exists. |
| `VacuumMapGrid` | public state | yes | partly | deferred | keep optional | Occupancy grid is generic enough, but not every vacuum has one. |
| `VacuumMapMetadata` | public state | yes | partly | partly | keep | Works for no-map via `hasMap: false`. |
| `VacuumMapAnnotation` | public state | yes | partly | deferred | keep/change | Room/zone annotations are neutral; source of truth needs clarity. |
| `VacuumNavigationProgress` | public state | partly | yes | no | change | Unknown time fields and recovery count are Nav2-shaped. |
| `VacuumMappingStatus` | public state | partly | yes | no | make optional/gated | Frontier counts, saved map paths, YAML/image paths are simulation/runtime diagnostics. |
| `VacuumSavedMapSummary.yamlPath`, `imagePath`, `poseGraphPath` | public state | no | yes | no | move diagnostic/private | File paths are backend/runtime implementation details. |
| `VacuumFaultState.faults: string[]` | public state | yes | no | partly | change | Strings are not enough for product UI; add codes, severity, assistance/recoverability. |
| `VacuumBatteryState` | public state | yes | no | yes | keep/change | Add voltage/charging source only if product needs it. |
| no `dock` state | missing | yes | no | no | add | Needed for docked/returning/charging distinction. |
| no `maintenance` state | missing | yes | no | no | defer/add later | Needed before consumables become supported. |
| no explicit `diagnostics` state | missing | yes | no | no | add | Keeps raw backend data available without product leakage. |
| no explicit stale/source state | missing | yes | no | no | add | Stale source should be first-class and command-blocking. |

## Mission Model Recommendation

Keep runtime-owned missions for long-running product workflows, especially simulation navigation, mapping, coverage, and room/zone cleaning. Do not force every Valetudo hardware state to pretend it is a Nav2-style mission.

Recommended split:

| current field/command/capability | current owner | backend-neutral? | TurtleBot4/Nav2-shaped? | Valetudo-ready? | keep/change/defer/remove | notes |
|---|---|---:|---:|---:|---|---|
| Coarse robot activity | currently `mission.state` | yes | no | yes | change | Rename/replace legacy mission with product activity: idle, cleaning, paused, returning, charging, faulted. |
| Runtime-owned mission | `activeMission`, `missions` | yes | no | partly | keep | Use for explicit product workflows with lifecycle, target, actions, result. |
| Valetudo basic cleaning | currently `hardware_cleaning` active mission | partly | no | partly | change | Represent as activity by default; create mission only if product initiated and runtime can track lifecycle/result. |
| Valetudo pause | command maps to basic pause and `pause_mission` | yes | no | partly | change | Availability should depend on active activity/mission. |
| Valetudo stop | command maps to basic stop and `cancel_mission` | yes | no | partly | change | Stop should yield activity idle/stopped; mission cancel only if a product mission exists. |
| Return to dock | command/activity/mission | yes | no | partly | change | Always activity `returning`; active mission only if product created a return-to-dock mission. |

## Valetudo Compatibility Gap List

| Valetudo concept | classification | decision | notes |
|---|---|---|---|
| Basic control | public supported capability now | keep supported when runtime commands available | Start, pause, stop, home are mapped through normalized commands. |
| Battery state | public supported capability now | keep | Battery percentage/charging maps cleanly. |
| Dock/charging state | missing/under-modeled concept | add public dock snapshot | Runtime has `dock`, adapter ignores it except charging. |
| Current robot state | public product concept | change | Currently normalized to `missionState`; should become activity plus optional mission. |
| Connection/source health | missing/under-modeled concept | add public health/source state | Runtime has source reachable/stale; public snapshot only exposes availability/fault strings. |
| Stale source state | missing/under-modeled concept | add first-class stale status | Should block unsafe commands distinctly from offline. |
| Map presence/absence | public unsupported/deferred | keep unsupported for now | No-map snapshot is valid today. |
| Map rendering | detected/deferred | keep unsupported with notes | Needs renderer/state model before UI activation. |
| Segment/room support | detected-but-not-product-ready | keep unsupported | Do not expose raw segments as public flags. |
| Zone support | detected-but-not-product-ready | keep unsupported | Needs geometry mapping and payload. |
| Go-to-location | detected-but-not-product-ready | keep unsupported | Current behavior is correct. |
| Fan speed presets | detected-but-not-product-ready | keep unsupported | Need preset list/current value. |
| Water usage presets | detected-but-not-product-ready | keep unsupported | Need preset list/current value. |
| Consumables | detected-but-not-product-ready | keep unsupported | Need maintenance schema. |
| Errors/faults | public under-modeled | change | Map raw errors to normalized codes/severity. |
| Events/diagnostics | diagnostics only | add explicit diagnostics boundary | Raw capability names and transport details should stay diagnostic. |
| Unsupported operations | public concept | change | Result codes need unsupported vs unavailable vs invalid state. |

## TurtleBot4/Nav2 Regression Risk List

| risk | classification | mitigation |
|---|---|---|
| Moving map/pose/navigation/mapping to optional fields could break UI assumptions. | regression risk | Keep compatibility defaults until UI is migrated to capability-gated optional reads. |
| Moving `backendCapability`, `map.topic`, `navigation.backendGoalState`, and saved map file paths to diagnostics could break debug UI/tests. | diagnostic boundary risk | Add diagnostics field first, duplicate temporarily, then deprecate public fields. |
| Replacing `mission` legacy state could break existing panel logic. | compatibility risk | Keep `mission` as obsolete compatibility until activity/mission split is adopted. |
| State-aware command availability may disable commands currently enabled by static service discovery. | behavior risk | Add `availableActions`/command availability alongside current capabilities before enforcement. |
| Removing `segment_cleaning`/`zone_cleaning` public commands could break compile-time command coverage tests. | compatibility risk | Mark obsolete first; remove after replacement payloads exist. |
| Richer error codes may require UI result handling updates. | behavior risk | Extend union without removing existing codes initially. |

## Diagnostics Boundary Checklist

| check | current status | action |
|---|---|---|
| Raw Valetudo capability names absent from public adapter files | pass | Regression harness checks public files. |
| Raw Valetudo capability names absent from Vacuum Control UI | pass | Regression harness checks component files. |
| UI does not branch on backend names | pass | Regression harness checks `VacuumControlPanel.tsx`. |
| Nav2/ROS topic/action names absent from public adapter files | partial | Public types avoid imports, but public snapshot has `map.topic`; capability `backendCapability` exposes ROS names. |
| Backend-specific diagnostics available | partial | Valetudo runtime has diagnostics, but public snapshot does not expose a normalized diagnostics container. |
| Product behavior driven by normalized capabilities | mostly pass | UI branches on `snapshot.capabilities.*.supported`; command state availability still weak. |
| Tests prevent diagnostic strings becoming product behavior | partial | Raw Valetudo names are checked; ROS/Nav2 leak checks should include public capability values or diagnostic-only policy. |

## Test Gap List

| test area | current status | gap |
|---|---|---|
| Valetudo snapshot mapping | present | Add dock state, stale source, runtime degraded once fields exist. |
| TurtleBot4/Nav2 snapshot mapping | present | Add no-map/no-pose optional compatibility tests after contract change. |
| Capability mapping | present | Add state-aware availability and detected/product-ready status tests. |
| Detected-but-unsupported Valetudo capabilities | present | Good coverage for go-to, segment, zone, fan, water, consumables. |
| Unsupported command results | present | Need unavailable/invalid-state/stale/offline distinctions. |
| Runtime unavailable results | partial | Valetudo offline snapshot exists; command result mapping needs more cases. |
| Stale source mapping | partial | Currently string fault only. |
| Malformed runtime payloads | partial | Shape guard exists; add explicit mapper/client test for malformed payload result. |
| No-map/no-pose snapshots | partial | Valetudo no-map/no-pose exists; make this contract-level expectation. |
| Command state transitions | partial | Basic dispatch tests exist; add state-aware command availability. |
| Raw backend names not leaking into UI | present | Good baseline. |
| TurtleBot4/Nav2 regression | present | Keep `bun run test:vacuum-adapter` as the harness. |

## Proposed Implementation Milestones

| milestone | scope | notes |
|---|---|---|
| B1: Contract-only status enrichment | Extend capability/result types with availability/status reasons while preserving old fields. | Smallest safe next step. No backend behavior rewrite. |
| B2: Snapshot health split | Add `health`, `source`, `dock`, and `diagnostics` fields; keep old fields for compatibility. | Makes Valetudo runtime/source state product-visible without backend leakage. |
| B3: Activity vs mission split | Add explicit product activity state and keep runtime missions for lifecycle workflows. | Prevents Valetudo basic cleaning from being forced into Nav2 mission semantics. |
| B4: Optional advanced surfaces | Make map/pose/navigation/mapping capability-gated in UI and then optional in contract. | Validates no-map/no-pose hardware path. |
| B5: Command availability | Add per-command availability derived from capability plus current state. | UI can safely show enabled/disabled actions without backend checks. |
| B6: Defer or replace vendor-shaped commands | Deprecate `segment_cleaning` and payload-less `zone_cleaning`; route vendor segments through room/zone product commands. | Keeps Valetudo concepts backend-private. |
| B7: Diagnostics hardening | Move backendCapability/topic/action/file path details under diagnostics and update tests. | Preserves development visibility while protecting product behavior. |

## Smallest Safe Next Step

Add a non-breaking status layer to the public contract:

- `CapabilitySupport.status`: `supported | unsupported | unavailable | detected_not_ready`.
- `CapabilitySupport.availabilityReason` or structured `reasons`.
- Expanded `VacuumCommandErrorCode` for `unavailable`, `invalid_state`, `stale_source`, `runtime_offline`, `source_unreachable`, `backend_timeout`, `malformed_backend_response`, `degraded_runtime`, and `needs_assistance`.
- Optional public `dock`, `source`, and `diagnostics` fields with compatibility defaults.

This keeps TurtleBot4/Nav2 working, preserves Valetudo detected-but-unimplemented gating, and gives the UI a path to branch only on product-level capability and availability semantics.
