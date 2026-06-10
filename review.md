# UI Review A - Valetudo Operator Surface Inventory

Current review date: 2026-06-09.

## Summary

The Valetudo mock-backed operator path has a useful no-map Layer 6A surface. Vacuum Control reserves the main canvas for a map-unavailable placeholder, replaces map/pose-centric status text with basic robot state, and renders Basic Cleaning, Cleaning Settings, Maintenance, and compact unavailable-workflows cards from normalized adapter snapshots.

For a Valetudo-backed mock with no map, no pose, no navigation, and no coverage, the current UI can show identity, availability, source/runtime health, activity, dock, battery, state-aware basic controls, fan speed, water usage, and maintenance/consumables display. Product UI still consumes `useVacuumAdapter`, adapter snapshots, and normalized commands; raw Valetudo HTTP/MQTT endpoints and capability names remain runtime/adapter diagnostics only.

Recommended direction: keep the existing simulation-heavy components intact for TurtleBot4/Nav2, and make the next Valetudo implementation milestone diagnostics or hardware-readiness validation after the no-map path stays stable. Map rendering, pose/go-to/navigation, Clean Area, segment/zone cleaning, consumable reset actions, diagnostics drawer work, and hardware validation remain deferred.

## Component Inventory

| component/section | current purpose | current data dependencies | capability gates | Valetudo Layer 6A behavior | recommended action | notes |
|---|---|---|---|---|---|---|
| `VacuumControlPanel` shell/header | Overall Vacuum Control layout, adapter selection, connection pill | `backend`, `snapshot.identity`, `snapshot.availability` | Backend selection is explicit composition | Valetudo-ready for identity label and connection state | Keep, then enrich | Add model/source/runtime health; backend id use is acceptable only in adapter selection. |
| Adapter selector | Switches Simulation / Valetudo runtime | `VacuumAdapterBackendId`, local storage | None | Backend-neutral composition boundary | Keep | Product behavior should still branch on capabilities, not selected backend. |
| Status strip | Shows connected and either map/pose workflow state or no-map basic robot state | `availability`, `map.receiving`, `pose.available`, `readiness.ready`, target state, no-map status fields | Implicit state checks plus no-map fallback | Valetudo-ready for Layer 6A | Keep | No-map Valetudo no longer presents Map Live / Localized / Target Selected as primary state. |
| Map surface switch | Chooses `MapCanvas` or no-map placeholder | `capabilities.map.supported`, `map.detail`, `identity.label` | `capabilities.map.supported` | Valetudo-ready for Layer 6A | Keep | Correctly avoids rendering map when unsupported and leaves basic robot cards in the sidebar. |
| No-map fallback | Reserves the main canvas and shows map unavailable detail | `identity.label`, `map.detail`, `capabilities.map.supported` | `!capabilities.map.supported` | Valetudo-ready for Layer 6A | Keep | Correctly avoids mounting `MapCanvas` when no map is supported. |
| `BasicControlsCard` | Start/pause/stop/return-to-dock basic controls | `capabilities.start_cleaning/pause/stop/return_to_dock`, command result error | `supported` and `available !== false` | Valetudo-ready core | Keep | Shows readable disabled reasons for state/source/runtime blocks. |
| `CleaningSettingsCard` | Fan speed and water usage current values/options/setters | `snapshot.cleaningSettings`, `capabilities.fan_speed`, `capabilities.water_usage` | Supported/current/options plus command availability | Valetudo-ready for mock-backed Layer 6A | Keep | Routes `set_fan_speed` and `set_water_usage` through normalized adapter commands. |
| `MaintenanceCard` | Display maintenance/consumable status | `snapshot.maintenance`, `capabilities.consumables` | Supported only when normalized consumable entries exist | Valetudo-ready display path | Keep | Main brush, side brush, filter, sensor cleaning, and mop pad render when present; reset actions are not implemented. |
| `MissionLifecycleCard` | Legacy mission/dock/battery summary plus dock command in map-supported workflows | `mission`, `battery`, `capabilities.dock_state`, `capabilities.return_to_dock` | `return_to_dock.supported && available !== false` | Not primary for no-map Valetudo | Keep for simulation modes | No-map Valetudo status uses the basic robot surface instead. |
| Mode switcher | Exposes Mapping, Navigate, Clean Area, Rooms when simulation-heavy workflows are relevant | `mapping`, `navigation`, coverage, room/zone capability booleans | `supported` booleans plus local lock state | Collapsed behind unavailable workflows for no-map Valetudo | Keep | No-map Valetudo should continue opening into the basic robot surface. |
| Mapping card/section | Start/pause/finish/save/load/improve maps | `snapshot.mapping`, map metadata, saved maps | `mapping_session` or `auto_mapping` | Valetudo-inappropriate for Layer 6A | Keep hidden unless capability exists | Current copy assumes exploration and maps; should not appear as active Valetudo control. |
| Navigate mode | Select map destination, send/cancel navigation | `pose`, `navigation`, `readiness`, `activeMission`, route progress | `start_navigation` or `go_to_location` supported | Valetudo-inappropriate for Layer 6A | Keep hidden unless capability exists | Correctly disabled when unsupported, but should not dominate mode default for basic robots. |
| Clean Area mode | Draw rectangle, preview route/coverage, start coverage | `map.grid`, `pose`, `start_coverage`, coverage mission state | `start_coverage` or `coverage_mission` | Valetudo-inappropriate for Layer 6A | Keep hidden unless capability exists | Area validation requires live map cells; no-map Valetudo should not see this as a normal mode. |
| Rooms / Zones mode | Draw/save room/zone annotations and clean selected annotation | `map.annotations`, `map.grid`, room/zone capabilities, coverage runtime | room/zone semantics or cleaning supported | Valetudo-inappropriate for Layer 6A | Keep hidden unless capability exists | Product room/zone workflow can return later after adapter maps segments/zones into normalized targets. |
| Compact unavailable workflows | Disabled-workflow explanation | Capability notes/state reason | Rendered in no-map/basic sidebar | Valetudo-ready for Layer 6A | Keep | Summarizes deferred map/navigation/coverage/room workflows without making them the primary UI. |
| `MapCanvas` | Occupancy map, target selection, paths, coverage overlays, annotations, sensor overlays | adapter map/pose/navigation props plus ROS fallback subscriptions | Parent gates only on `capabilities.map.supported` | TurtleBot4/Nav2 simulation/operator UI | Keep hidden unless map capability exists | Internally subscribes to ROS `/map` and costmaps as fallback; not a pure adapter renderer. |
| `CameraOverlay` | Floating ROS image feed | `ros2Bridge`, image topics | Not currently mounted here in reviewed render path | Valetudo-inappropriate for Layer 6A | Keep separate/debug only | Camera topic discovery is ROS-specific; do not expose for Valetudo basic robot. |
| `TeleopCard` | Manual directional velocity streaming | `ros2Bridge`, `/cmd_vel_raw`, manual control capability in parent | `capabilities.manual_control.supported` | Valetudo-inappropriate for Layer 6A | Keep hidden unless capability exists | Correctly gated in parent, but it is not a normalized adapter command surface. |
| Recent missions card | Shows terminal mission history | `missions.recent`, mission progress/result | Non-empty terminal missions | Valetudo-compatible with minor changes | Reuse later | Useful if Valetudo runtime records product-initiated cleaning/dock missions; otherwise activity history should be separate. |
| Command error banners | Show failed command messages | Local command error state from `sendCommand` result | Command-specific render paths | Valetudo-ready | Keep | Disabled and failed command reasons are readable in the no-map/basic cards. |

## Valetudo Layer 6A UI Fit

| Layer 6A need | current UI fit | current source | remaining gap | recommended action |
|---|---|---|---|---|
| Robot identity/name/model/source | Implemented for basic surface | `snapshot.identity.label`, adapter selector, normalized source fields | Model remains optional | Keep normalized identity/source copy. |
| Connection/availability | Implemented | connection pill, no-map status strip, status cards | Full health endpoint detail is not a product card | Keep runtime/source split visible. |
| Runtime health | Implemented summary | `snapshot.health` | No diagnostics drawer | Keep summary; defer detailed diagnostics drawer. |
| Source reachable/stale/unreachable | Implemented summary | `snapshot.source` | Raw transport details hidden | Keep product summary; raw details stay diagnostics. |
| Activity status | Implemented | `snapshot.activity` | None for Layer 6A | Keep as primary no-map state. |
| Battery | Implemented | `snapshot.battery` | None for Layer 6A | Keep in no-map/basic surface. |
| Charging/dock state | Implemented | `snapshot.battery`, `snapshot.dock` | None for Layer 6A | Keep normalized dock/battery display. |
| Basic controls | Implemented | `BasicControlsCard` | Resume remains unsupported | Keep state-aware disabled reasons. |
| Fan speed | Implemented | `snapshot.cleaningSettings`, `capabilities.fan_speed` | Real hardware validation deferred | Keep normalized current/options/setter path. |
| Water usage | Implemented | `snapshot.cleaningSettings`, `capabilities.water_usage` | Real hardware validation deferred | Keep normalized current/options/setter path. |
| Segment cleaning | Diagnostics/deferred | `capabilities.segment_cleaning` | Map geometry, target normalization, room editor, zone cleaning, and hardware validation deferred | Keep unavailable/detected-not-ready until a later implementation normalizes targets. |
| Maintenance/consumables display | Implemented display path | `snapshot.maintenance`, `capabilities.consumables` | Reset commands deferred | Keep display-only surface. |
| Useful no-map fallback | Implemented | map-unavailable canvas plus no-map sidebar cards | No map rendering by design | Keep `MapCanvas` unmounted when map unsupported. |
| Diagnostics without product behavior | Still deferred | `snapshot.diagnostics` | No diagnostics drawer/card | Add later only as passive diagnostics. |

## Capability-Gating Audit

Passes:

- Basic controls use normalized capability fields: `start_cleaning`, `pause`, `stop`, and `return_to_dock` check `supported` and `available !== false`.
- Cleaning Settings uses normalized `snapshot.cleaningSettings`,
  `capabilities.fan_speed`, `capabilities.water_usage`, and normalized
  `set_fan_speed` / `set_water_usage` commands.
- Maintenance uses normalized `snapshot.maintenance` and
  `capabilities.consumables` for display-only consumable status.
- Map rendering is gated by `capabilities.map.supported`.
- Mapping, navigation, Clean Area, Rooms/Zones, and teleop are gated by normalized `snapshot.capabilities.*.supported` booleans.
- UI submits normalized commands for product operations: `start_cleaning`, `pause`, `stop`, `return_to_dock`, `set_fan_speed`, `set_water_usage`, `start_navigation`, `start_coverage`, mapping commands, mission controls, and room/zone commands.
- Backend id is only used for adapter selection/composition, which is acceptable.
- Raw Valetudo capability names are not used by the reviewed UI.

Gaps and risks:

- Several controls only check `supported`, not `status`, `available`, or `availabilityReason`; this is most visible in mode visibility and simulation-heavy command gates.
- `canSendRun`, `canStartCleanArea`, and room/zone gates combine normalized support with UI-local/readiness state, but do not yet use per-command `available` or `availabilityReason`.
- Some simulation-heavy controls still do not use per-command `available` or `availabilityReason`.
- Detailed diagnostics remain absent from the UI.
- `MapCanvas`, `TeleopCard`, and `CameraOverlay` directly consume `ros2Bridge` and ROS topic names; they are simulation/operator surfaces and must remain hidden for Valetudo Layer 6A.
- `MappingCard` still renders compatibility fields such as saved/loaded map paths when mapping is supported. That is acceptable for simulation/debug now, but product UI should move those to diagnostics before real hardware map workflows.

Boundary rule check:

- Product UI mostly renders normalized adapter state and submits normalized commands.
- Backend adapter owns mapping backend runtime state into `vacuum_adapter`.
- VM runtime owns backend connection, state cache, and command routing.
- Current violation risk is not Valetudo raw names; it is that simulation-heavy product surfaces still read ROS bridge streams directly and dominate the layout when capabilities are present.

## No-Map UI Assessment

The current no-map Valetudo UI is useful enough for the mock-backed Layer 6A operator surface.

What works:

- It does not crash when no map is supported.
- It shows robot identity and map-unavailable detail.
- It shows runtime/source health, availability, activity, dock, battery, and charging summaries.
- It shows Basic Cleaning controls with readable state/source disabled reasons.
- It shows Cleaning Settings for fan speed and water usage when normalized values/options exist.
- It shows Segments when normalized targets and segment cleaning support exist.
- It shows Maintenance when normalized consumable entries exist.
- It collapses unsupported simulation-heavy workflows into a compact unavailable-workflows card.

What is missing:

- Diagnostics drawer/details remain absent.
- Zone cleaning, map rendering, pose/go-to/navigation, Clean Area, room editor, and consumable resets remain unsupported.

Recommendation: keep the reduced no-map segment/room surface stable before diagnostics or real-hardware validation.

## Diagnostics UI Assessment

Diagnostics are now well modeled in the adapter contract, but the current UI does not expose them.

Recommended diagnostic behavior:

- Add a compact diagnostics drawer/card for runtime health, source state, capability diagnostics, raw backend warnings, and fault details.
- Diagnostics must never decide whether product controls render or enable.
- Product controls should render from `capabilities`, `activity`, `health`, `source`, `dock`, `battery`, `map`, `pose`, `navigation`, and `mapping`.
- Keep raw Valetudo capability names and ROS/Nav2 topics/actions inside diagnostics/debug surfaces only.

## Simulation-Specific UI Risk List

| risk | why it matters for Valetudo Layer 6A | mitigation |
|---|---|---|
| Segment raw IDs leaking into UI behavior | Backend target IDs would couple product UI to source internals | Keep raw segment IDs out of product controls until a later implementation defines normalized target state. |
| Status strip regressions to map/pose language | Basic Valetudo no-map/no-pose state would look unhealthy even when valid | Keep the no-map status strip based on source/activity/dock/battery state. |
| `MapCanvas` subscribes to `/map` and costmaps | Product map surface can bypass adapter state with ROS fallback | Treat as simulation map component; later split adapter-only map renderer. |
| Teleop publishes `/cmd_vel_raw` | Not a normalized command path and not Valetudo-safe | Keep gated behind `manual_control`; do not show for Valetudo Layer 6A. |
| Camera discovers ROS image topics | Not part of Valetudo basic vacuum contract | Keep separate/debug only. |
| Diagnostics absent | Source stale/unreachable and raw capability debugging require code inspection | Add diagnostics drawer while keeping it behavior-passive. |

## Recommended Implementation Milestones

| milestone | scope | notes |
|---|---|---|
| Later: Segment target normalization | Normalize segment targets and selected-target commands through mock/runtime/adapter/UI | Keep raw map segmentation diagnostics passive until this is implemented deliberately. |
| Later: Diagnostics drawer/card | Render `snapshot.diagnostics`, health/source warnings, and capability diagnostics | Diagnostics are inspectable but never drive product behavior. |
| Later: Hardware validation | Validate normalized state and commands against a Valetudo-compatible robot | Keep mock-backed behavior stable first. |

## Validation

This was a review-only milestone. No product code changes were made, so adapter tests and panel build were not run.

Manual live webview validation was not run.

# Valetudo Milestone 1 Inventory - Mock/Runtime/Adapter Mapping

Current review date: 2026-06-08.

## Scope

This pass inventoried the local Valetudo mock source, the VM-managed Valetudo integration runtime, the extension runtime client, the Valetudo backend adapter, the shared `vacuum_adapter` contract, command mapping, capability descriptors, UI consumption points, and existing regression/runtime tests.

No UI behavior changes were implemented. This is a mapping foundation for later adapter and UI hardening.

## Runtime/API Shapes Found

| shape | source / endpoint | fields | adapter interpretation | normalized output | UI visibility | classification |
|---|---|---|---|---|---|---|
| Runtime health | `GET /api/v1/valetudo/health` or VM-manager proxy `/vms/self/tensorfleet/api/v1/valetudo/health` | `runtime.{id,version,status}`, `source.{kind,status,stale,lastSeenAt}`, `updatedAt` | Runtime liveness and source freshness are distinct concepts | `snapshot.health`, `snapshot.source` after snapshot mapping; client exposes `getHealth` but UI path currently hydrates from snapshot | Health/source shown in current no-map cards; `/health` itself is not directly used by UI | Product-facing only after normalized snapshot mapping; endpoint details diagnostics/runtime-only |
| Fixed mock snapshot | `GET /api/v1/valetudo/snapshot` with `VALETUDO_RUNTIME_SOURCE_MODE=fixed_mock` | Runtime, robot identity, fixed source health, connectivity, state, battery, dock, command availability, capability diagnostics, raw diagnostics | Stable VM-owned mock robot state | Full `VacuumAdapterSnapshot` with no map/pose/navigation, basic controls, battery, dock, activity, fault/readiness | Visible: identity, availability, source, activity, dock, battery, basic controls, unavailable workflows. Diagnostics not visible | Product-facing normalized fields; raw capability names diagnostics-only |
| HTTP mock source snapshot | Runtime reads Valetudo mock source `GET /api/v2/robot`, `/api/v2/robot/state/attributes`, `/api/v2/robot/capabilities` | Robot info, `StatusStateAttribute`, `BatteryStateAttribute`, capabilities array | Runtime normalizes Valetudo HTTP source into the same runtime snapshot shape | Same adapter shape as fixed mock; `source.kind=valetudo_mock`, diagnostics mode `valetudo_mock_http` | Same as fixed mock after adapter mapping | Product-facing normalized fields; raw HTTP payloads diagnostics-only |
| MQTT cached snapshot | Runtime MQTT cache from `$name`, `$nodes`, `StatusStateAttribute/*`, `BatteryStateAttribute/*`, `DockStatusStateAttribute/*` | Robot name, status, status flag, battery status/level, dock status, raw capability names, transport diagnostics | MQTT is internal runtime transport; snapshot shape remains unchanged | Same adapter shape, diagnostics mode `valetudo_mock_mqtt`, transport diagnostics include HTTP and MQTT | Same as fixed mock after adapter mapping | Product-facing normalized fields; MQTT topics diagnostics-only |
| Future HTTP source readiness snapshot | Runtime mode `VALETUDO_RUNTIME_SOURCE_MODE=valetudo_http` with `VALETUDO_SOURCE_URL` | Same Valetudo HTTP source fields as mock HTTP, with transport-level source labels | Runtime can be pointed at a future robot without code changes or a hardcoded robot address | Same normalized adapter shape; missing/unreachable/malformed source remains unavailable/stale | UI remains stable and command-gated through normalized state | Product-facing normalized readiness; robot URL and raw payload errors diagnostics-only |
| Missing HTTP source config | `valetudo_http` with no source URL | Fallback robot identity, `source.kind=valetudo_http`, `source.status=unknown`, `source.stale=true`, diagnostics stale reason `missing_source_config` | Adapter treats the source as unavailable and blocks basic commands | Offline/unavailable no-map snapshot with faults/reasons instead of a crash | Visible as unavailable source/runtime state | Product-facing safe unavailable state; missing env var detail diagnostics-only |
| Unreachable HTTP source snapshot | Runtime fails fetching Valetudo mock source | Robot fallback identity, `source.status=unreachable`, `source.stale=true`, `connectivity.online=false`, state `unavailable`, empty raw capability names, diagnostics error | Adapter treats source/runtime unavailable as offline-like product state with preserved snapshot shape | `availability.offline`, `source.reason=runtime_offline` or `source_unreachable`, activity unavailable, faults populated, commands unavailable/unsupported | Visible as disconnected/unavailable status; source/fault reason visible in no-map status card | Product-facing normalized unavailability; raw error diagnostics-only |
| Stale MQTT snapshot | MQTT connected/no messages or messages older than timeout | `source.stale=true`, stale reasons `no_messages_received`, `source_stale_after_timeout`, or `transport_unreachable` | Adapter maps stale source to `source.status=stale` and blocks basic commands through normalized availability | Commands get `status=unavailable`, `availabilityReason=stale_source`; activity can remain state-derived but source reason is present | Source stale visible; command reasons visible when controls disabled | Product-facing stale state; exact MQTT transport details diagnostics-only |
| Runtime unavailable client state | Extension runtime client cannot fetch snapshot or receives malformed snapshot | Client stores `lastError`; adapter maps synthetic unavailable boundary | `identity` fallback, `availability.offline`, `health.runtimeStatus=offline`, `source.kind=unknown`, map/pose/navigation unavailable, commands unsupported/unavailable | Visible as offline no-map surface instead of crash | Product-facing normalized offline state; HTTP/proxy error details diagnostics-only |
| Command success result | `POST /api/v1/valetudo/command` with `start_cleaning`, `pause`, `stop`, `return_to_dock` when supported/reachable | `{ok:true,status:"success",command,message,updatedAt,diagnostics}` | Adapter returns `{ok:true, command, message}` and refreshes snapshot | Command result message plus new snapshot activity/state | Basic controls submit normalized commands; resulting state visible after refresh | Product-facing command result; diagnostics-only transition/source details |
| Unsupported command result | Runtime receives unknown command or BasicControl capability absent | `{ok:false,status:"unsupported",reason/code:"unsupported_command" or "capability_unavailable"}` | Adapter maps known unsupported status to `VacuumCommandError.code="unsupported"` unless command is pre-blocked by capability mapping | Unsupported normalized command result | UI avoids most unsupported submissions through capability gating; errors can show if submitted | Product-facing as normalized error; backend capability name remains diagnostics-only |
| Unavailable command result | Runtime source unreachable/MQTT disconnected | `{ok:false,status:"unavailable",reason/code:"source_unreachable"}` | Adapter maps to `source_unreachable` command error; snapshot refresh carries unavailable state | Command capabilities unavailable with structured reason | Disabled controls show readable reason; error visible if command attempted | Product-facing normalized unavailability |
| Invalid-state command result | Fixed mock `start_cleaning`, `pause`, `stop`, and `return_to_dock` now return `status:"failed"`, `reason/code:"invalid_state"` when the robot state makes the command unsafe or impossible | Runtime and adapter both use normalized invalid-state semantics for basic commands | Adapter maps `invalid_state` to `VacuumCommandError.code="invalid_state"` and still accepts legacy `command_invalid_state` as an alias | Disabled command reasons such as robot not cleaning/nothing running/already docked | Visible in no-map controls as readable disabled reasons | Product-facing normalized state gate |
| Failed command result | Invalid JSON/missing command/source command failure/MQTT publish failure/malformed command response | Runtime returns specific backend reason/code values such as `invalid_json`, `missing_command`, `source_command_failed`, or `stale_source` | Adapter collapses runtime-only aliases to shared errors: malformed/unknown backend failures become `backend_error` or `malformed_backend_response`; malformed requests become `invalid_request` | Normalized failed `VacuumCommandResult` error with readable message preserved | Visible only if command path hits failure; diagnostics not used for control gating | Product-facing normalized error; raw failure detail diagnostics-only |

## Field Mapping Inventory

| domain | mock source field | VM runtime field / endpoint response | backend adapter interpretation | normalized `vacuum_adapter` field | current UI visibility | classification |
|---|---|---|---|---|---|---|
| Robot identity | Valetudo mock `getManufacturer()`, `getModelName()`, implementation; fixed mock hardcoded ID/name; MQTT `$name` | `robot.id`, `robot.name`; fixed mock `valetudo-fixed-mock-001` / `Valetudo Fixed Mock`; HTTP stable ID from manufacturer/model/implementation; MQTT `valetudo-mqtt-robot` | Identity is trusted if snapshot shape is valid; fallback identity used only when runtime unavailable | `identity.{id,label,source,model?}` | Header/no-map placeholder/sidebar status use label; model not currently populated for Valetudo | Product-facing label/id; raw manufacturer/model details diagnostics/deferred |
| Availability/connectivity | Fixed mock env reachability; HTTP fetch success; MQTT connected/subscribed/fresh | `connectivity.{reachable,online}`, `source.status`, `source.stale` | `connectionStatus=online` only when runtime/source snapshot says online; unavailable client maps offline | `availability.{status,connected,detail}`, `source.status/stale/reason` | Visible in connection pill, no-map status strip/card | Product-facing normalized availability |
| Runtime health | Runtime constant ID/version/status; health endpoint | `runtime.{id,version,status}` in health and snapshot | `online/degraded/offline` mapped into adapter health | `health.{runtimeStatus,updatedAt,detail}` | Visible in no-map status card | Product-facing after normalization; runtime ID/version diagnostics-only |
| Source health/reachability/staleness | Fixed mock env; HTTP poll diagnostics; MQTT cache freshness | `source.{kind,status,stale,lastSeenAt}` and diagnostics source fields | `stale` wins over reachable; source reasons block commands | `source.{kind,status,stale,lastSeenAt,reason}`; faults for stale/unreachable/degraded | Visible in no-map status/card; exact stale reason partly visible through humanized source reason | Product-facing state; transport/source URLs and stale diagnostics diagnostics-only |
| Activity/state | Fixed mock `Value/Label/Started/Paused`; HTTP `StatusStateAttribute.value`; MQTT status | `state.{value,label,started,paused}` | Maps to `mission.state`, `activity.status`, `activeMission` for non-idle states | `activity`, `mission`, `activeMission`, `missions.active` | Activity visible; mission cards visible only in map-supported workflows; active mission not primary in no-map | Product-facing normalized activity; raw state string diagnostics-only |
| Battery | Fixed mock level 82/charging; HTTP `BatteryStateAttribute.level` + `flag=charging`; MQTT battery level/status | Optional `battery.{level,charging}` | If present, adds battery capability and readiness ready; missing battery is waiting/unavailable | `battery.{readiness,percentage,charging,detail}` and `capabilities.battery` | Visible in no-map placeholder/status card and mission lifecycle cards | Product-facing |
| Charging | Fixed mock charging boolean; HTTP battery flag; MQTT battery status | `battery.charging`, dock detail | Charging also maps dock state to `charging` | `battery.charging`, `dock.charging`, `dock.state=charging` | Visible in no-map placeholder/status card | Product-facing |
| Dock state | Fixed mock docked/returning/available; HTTP derived from robot state; MQTT dock status | Optional `dock.{state,docked}` | Charging takes precedence; state strings normalized to docked/undocked/returning/charging/error/unknown | `dock.{supported,state,charging,detail}`, `capabilities.dock_state` | Visible in no-map placeholder/status card; older mission lifecycle shows capability plus battery | Product-facing normalized dock state |
| Faults | Valetudo mock raises events, but runtime does not ingest event endpoint; runtime source stale/unreachable/degraded | Runtime snapshot diagnostics notes/source errors; adapter faults array | Adapter faults contain stale source, source unreachable, degraded runtime, client last error | `fault.{readiness,faults,detail}` | Faults visible in no-map status card if present; Valetudo event details not shown | Runtime/source faults product-facing; raw Valetudo events deferred/diagnostics-only |
| Diagnostics | Raw mock capabilities, raw source mode, HTTP/MQTT transport details, last command audit, capability tiers, notes | `diagnostics`, `rawDiagnostics` | Passed through `snapshot.diagnostics` with map/pose/navigation/mapping diagnostic reasons | `diagnostics.{backend,runtime,source,capabilities,map,pose,navigation,mapping,warnings,raw}` | No dedicated diagnostics drawer; some normalized warnings/faults visible | Diagnostics-only; must not gate product controls |
| Readiness summary | Runtime diagnostics computed from runtime/source/capability/command state | `diagnostics.readiness.{runtimeOnline,sourceReachable,sourceStale,supportedCapabilities,detectedNotProductReady,basicCommandsAvailable,segmentTargetsAvailable}` | Adapter passes it through diagnostics raw data only | Inspectable diagnostics; no product controls branch on it | No dedicated drawer yet | Diagnostics-only |
| Capabilities | Valetudo mock registered capabilities; runtime raw list; HTTP `/api/v2/robot/capabilities`; MQTT `$nodes` | `capabilities.commands`, `capabilities.diagnostics`, `diagnostics.rawCapabilityNames`, `capabilityTiers` | Only known capability subset influences descriptors; raw names remain diagnostics | `capabilities` normalized descriptor record | UI gates on normalized descriptors; raw names not consumed | Product-facing descriptors; raw capability names diagnostics-only |
| Map | Valetudo mock has map state and map endpoint; runtime currently does not fetch map | Runtime sets no map product data; diagnostics marks map unsupported | Map explicitly unavailable | `map.readiness=unavailable`, `map.grid=null`, `metadata.hasMap=false`, `capabilities.map.supported=false` | Main canvas placeholder, MapCanvas unmounted | Deferred product-facing map support; raw map diagnostics-only for now |
| Pose/navigation/go-to | Valetudo mock has GoTo capability and map robot entity; runtime does not normalize pose or go-to workflow | Go-to raw capability may be detected, but no command path exposed product-ready | Navigation/go-to unsupported/detected-not-ready | `pose.available=false`, `navigation.active=false`, `capabilities.go_to_location.status=detected_not_ready` when detected | Navigation unavailable workflow only; no active go-to UI | Deferred |
| Clean Area/coverage | No current Valetudo runtime coverage executor | No product response | Unsupported | `capabilities.start_coverage`, `coverage_mission` unsupported | Unavailable workflow summary | Deferred |
| Rooms/zones/segments | Valetudo mock has map segmentation and zone cleaning capabilities | Raw capability names plus diagnostics tiers; no geometry or target list normalized | Segment, room, and zone cleaning remain unsupported/detected-not-ready | `capabilities.segment_cleaning`, `room_semantics`, `room_cleaning`, and `zone_cleaning` stay unavailable or detected-not-ready | Unavailable workflow summary only | Deferred; raw names stay diagnostics-only |
| Fan speed/water usage | Mock capabilities registered; HTTP properties/presets exist in Valetudo | Runtime normalizes current preset and available options into `cleaningSettings`, and exposes normalized setters | Product-ready only when current value, options, command availability, and command result behavior are present | `capabilities.fan_speed`, `water_usage` supported/unavailable through normalized descriptors; `snapshot.cleaningSettings` carries current/options | Cleaning Settings card in the no-map/basic sidebar | Product-facing for mock/runtime path; raw Valetudo names remain diagnostics-only |
| Consumables | Mock capability and HTTP consumable status/properties exist | Runtime normalizes consumable display status into `maintenance.consumables` | Product-ready for display only when normalized consumable entries exist; reset commands not implemented | `capabilities.consumables` supported/unavailable through normalized descriptor; raw capability alone remains detected-not-ready | Maintenance card in the no-map/basic sidebar | Product-facing display for mock/runtime path; raw Valetudo names remain diagnostics-only |
| Statistics | Mock current/total statistics capabilities exist; runtime only detects raw capability names | Raw diagnostics/capability tier | Current/total stats not mapped to public descriptor | Statistics remain diagnostics-only | Not visible | Deferred/diagnostics-only |

## Command Mapping

| normalized command | runtime command/action | source behavior | adapter gating/result | current UI visibility | status |
|---|---|---|---|---|---|
| `start_cleaning` | Runtime command `start_cleaning`; HTTP action `start`; MQTT payload `START`; fixed mock state `cleaning` | Requires reachable source and `BasicControlCapability`; fixed mock accepts from idle/stopped/docked | Supported when normalized `capabilities.start_cleaning.supported && available`; blocked with `invalid_state`, `stale_source`, `source_unreachable`, `runtime_offline`, `degraded_runtime`, or unsupported | Basic controls | Product-facing |
| `pause` | Runtime command `pause`; HTTP action `pause`; MQTT `PAUSE`; fixed mock state `paused` | Fixed mock rejects outside cleaning with normalized `invalid_state`; HTTP/MQTT defer to source | Adapter pre-gates to available only when cleaning/not returning and maps runtime invalid-state results to shared `invalid_state` | Basic controls | Product-facing |
| `resume` | No runtime command and no explicit mapper | Not implemented | Always unsupported with note that resume must be mapped explicitly | Not shown as basic no-map button | Deferred |
| `stop` | Runtime command `stop`; HTTP action `stop`; MQTT `STOP`; fixed mock state `stopped` | Requires basic control and source reachability | Adapter pre-gates to cleaning/paused/returning only | Basic controls | Product-facing |
| `return_to_dock` | Runtime command `return_to_dock`; HTTP action `home`; MQTT `HOME`; fixed mock transient `returning_to_dock` then `docked` on next read | Requires basic control and source reachability | Adapter pre-gates unavailable when docked/charging/returning | Basic controls and mission lifecycle dock button in some modes | Product-facing |
| `pause_mission` | Adapter maps to runtime `pause` | Same as pause | Supported only when `pause` supported; mapped to backend pause | Mission cards only in map-supported workflows | Product-facing normalized alias but not primary Valetudo no-map control |
| `cancel_mission` | Adapter maps to runtime `stop` | Same as stop | Supported only when `stop` supported | Mission cards only in map-supported workflows | Product-facing normalized alias but not primary Valetudo no-map control |
| `go_to_location` / `start_navigation` | Mapper contains request shape but returns unsupported because public descriptor is unsupported | No runtime command implemented | Unsupported even if raw go-to capability is detected | Navigation unavailable | Deferred |
| `segment_cleaning` | No product command path in this readiness pass | No source routing | Unsupported even when raw `MapSegmentationCapability` is detected | Not shown as a product control | Deferred until segment targets are normalized deliberately |
| `start_zone_cleaning`, `zone_cleaning` | No runtime command implemented | No source routing | Unsupported; zone requires explicit geometry mapping later | Zone workflow unavailable | Deferred |
| `set_fan_speed`, `set_water_usage` | Runtime command `set_fan_speed` / `set_water_usage`; HTTP preset PUT to the matching Valetudo capability; fixed mock updates preset state | Requires reachable/fresh source, matching capability, payload `value`, and value in normalized options | Adapter rejects missing/invalid values as `invalid_request`, maps unsupported/unavailable/source failures to shared errors | Cleaning Settings card | Product-facing for fan/water mock-backed settings |
| Mapping, map annotations, coverage, retry/skip/resume mission | No Valetudo runtime executor | No source routing | Unsupported | Unavailable workflows or hidden in no-map mode | Deferred |

## Capability Classification

Product-facing now:

- `mission_state`, `start_cleaning`, `pause`, `stop`, `return_to_dock`, `pause_mission`, `cancel_mission`, `battery`, `dock_state`, `events`, `fault_state`, `fan_speed`, `water_usage`, and `consumables` as normalized descriptors/state for the implemented mock/runtime path.

Detected or known but deferred:

- Map rendering/snapshot, pose, go-to/navigation, mapping sessions, map annotations, zone semantics, zone cleaning, Clean Area/coverage, room editor, consumable reset commands, current/total statistics, manual control, locate, auto-empty dock actions, carpet/carpet sensor settings, obstacle/pet/collision avoidance settings, operation mode, pending map changes, persistent maps, mop dock clean/dry actions, speaker/voice pack controls, Wi-Fi configuration/scan, key lock, and do-not-disturb.

Diagnostics-only fields that must not drive product behavior:

- Raw Valetudo capability names, capability tiers, Valetudo HTTP endpoints and payloads, MQTT topics/payloads, transport diagnostics, source URLs/broker URLs, raw Valetudo state strings before adapter normalization, runtime ID/version, last command diagnostic details, raw map/pose/navigation/mapping diagnostic notes, and Valetudo event payloads until normalized into product state.

## Adapter/UI Gaps

- The adapter exposes `diagnostics`, including raw capability names, capability tiers, transports, warnings, raw source state, and last command audit, but the UI has no dedicated diagnostics drawer.
- The Valetudo mock source has map data and robot pose-like entities, but the runtime/adapter intentionally expose no product map/pose yet.
- Valetudo mock events exist, but the runtime does not inventory/fetch them into normalized `fault`/`events` product state.
- Statistics, zones, go-to, and segment geometry are detected/known but not normalized into usable commands, current values, or geometry. Consumable reset commands are not implemented.
- `resume` exists in the shared command/capability contract, but the Valetudo backend marks it unsupported and the basic UI does not expose it.
- Runtime invalid-state command responses are normalized to `invalid_state` for fixed mock basic commands. The adapter still accepts legacy `command_invalid_state` as a compatibility alias.
- The adapter has `snapshot.health` and `snapshot.source`; UI shows summarized normalized values, but not the full health endpoint result or source diagnostics.

## Existing Validation Coverage Found

- VM runtime tests cover fixed mock snapshot identity, raw capability diagnostics, capability tiers, last command audit, unknown command unsupported results, malformed command requests, source-unreachable unavailable results, fixed mock command state transitions, invalid-state behavior for basic commands, missing BasicControl capability, HTTP mock source snapshot/command routing, HTTP source command failure, HTTP source unavailable/stale diagnostics, MQTT snapshot hydration, MQTT stale diagnostics, stale MQTT command unavailability, and MQTT disconnected command unavailability.
- Hardware-readiness runtime tests cover missing HTTP source config, unreachable HTTP source, malformed HTTP source response, and reachable source without `BasicControlCapability`.
- Adapter regression covers Valetudo capability mapping, command mapping, runtime command result mapping, malformed backend command-response aliasing, state-aware command availability, missing/unreachable/stale source command blocking, no segment-target support, runtime unavailable mapping, no-map product surface safety, readable disabled reasons, diagnostics pass-through, and raw capability privacy from UI behavior.

## Boundary Hardening Closure

Milestone 1 inventory is complete. Milestone 2A command/result hardening is complete. The final boundary lock pass confirmed focused runtime and adapter coverage for stale source command blocking, invalid-state results, unsupported/capability-unavailable results, malformed command requests, source command failures, malformed backend command-response aliases, readable disabled reasons, and raw Valetudo capability privacy.

Diagnostics remain intentionally passive. Raw Valetudo capability names, transport details, source URLs, last-command audit data, and raw runtime/source details stay diagnostics-only and must not gate product controls. Product behavior continues to branch on normalized adapter state and capability descriptors.

Deferred Valetudo capabilities remain deferred: map rendering, go-to/navigation, Clean Area, segments, zones, room semantics, room cleaning, room editor, segment geometry, consumable reset commands, statistics, OpenClaw, MQTT production hardening beyond this feature path, production hardware support, and diagnostics drawer work are outside this closed regression/diagnostics-focused thread. Fan speed, water usage, and consumable display are no longer deferred for the mock/runtime/adapter/UI path.

## Next Product Direction

Recommended next milestone: diagnostics or first real-hardware readiness after the mock segment/room path remains stable.

Focus:

1. Keep the normalized mock segment/room target path stable.
2. Add diagnostics or first real-hardware reachability validation next.
3. Keep the simulation layout intact.
4. Do not add map rendering, robot pose visualization, navigation/go-to, zone cleaning, Clean Area, a room editor, consumable reset commands, or production hardware behavior before the readiness checkpoint.

## Validation

Milestone 2A updated runtime command behavior, adapter command result mapping, capability reason text, and focused regression coverage. The final boundary lock pass added only the missing malformed backend command-response alias regression and documentation closure notes.

Focused checks for the final wrap-up:

```sh
bun run test:vacuum-adapter
go test ./...
bun run --cwd panels-standalone build
git diff --check
```

Manual live webview validation was not run. Real hardware validation was not run. TurtleBot4/Nav2 live simulation validation was not run.
