# UI Review A - Valetudo Operator Surface Inventory

Current review date: 2026-06-05.

## Summary

The hardened adapter contract is ahead of the current Vacuum Control UI. The UI now has a usable Layer 6A foothold through the no-map fallback and `BasicControlsCard`, but the primary surface is still organized around map, pose, navigation, mapping, coverage, rooms/zones, ROS overlays, and manual control.

For a Valetudo-backed robot with no map, no pose, no navigation, no coverage, and no rooms/zones, the current UI is functional but thin: it can show identity and basic cleaning controls when `capabilities.map.supported` is false, and mode tabs disable unsupported workflows. It does not yet clearly surface runtime health, source reachability/staleness, activity, real dock state, command availability reasons, or a diagnostic summary.

Recommended direction: keep the existing simulation-heavy components, but split out a backend-neutral Basic Robot / Vacuum Status surface that renders normalized identity, availability, health, source, activity, dock, battery, basic controls, command reasons, and a compact diagnostics drawer. Unsupported simulation-heavy modes should be hidden or collapsed for Layer 6A instead of being the dominant operator frame.

## Component Inventory

| component/section | current purpose | current data dependencies | capability gates | Valetudo Layer 6A behavior | recommended action | notes |
|---|---|---|---|---|---|---|
| `VacuumControlPanel` shell/header | Overall Vacuum Control layout, adapter selection, connection pill | `backend`, `snapshot.identity`, `snapshot.availability` | Backend selection is explicit composition | Valetudo-ready for identity label and connection state | Keep, then enrich | Add model/source/runtime health; backend id use is acceptable only in adapter selection. |
| Adapter selector | Switches Simulation / Valetudo runtime | `VacuumAdapterBackendId`, local storage | None | Backend-neutral composition boundary | Keep | Product behavior should still branch on capabilities, not selected backend. |
| Status strip | Shows connected, map live, localized, ready, target selected | `availability`, `map.receiving`, `pose.available`, `readiness.ready`, target state | Implicit state checks | Valetudo-compatible with minor changes | Replace later | For no-map Valetudo it emphasizes missing map/pose instead of source/activity/dock/battery. |
| Map surface switch | Chooses `MapCanvas` or empty fallback | `capabilities.map.supported`, `map.detail`, `identity.label` | `capabilities.map.supported` | Valetudo-ready baseline | Keep, improve fallback | Correctly avoids rendering map when unsupported, but fallback is too sparse for Layer 6A. |
| No-map fallback | Shows robot label and map unavailable detail | `identity.label`, `map.detail` | `!capabilities.map.supported` | Valetudo-compatible with minor changes | Reuse as status surface host | Should show useful robot status, not just map absence. |
| `BasicControlsCard` | Start/pause/stop/return-to-dock basic controls | `capabilities.start_cleaning/pause/stop/return_to_dock`, command result error | `supported` and `available !== false` | Valetudo-ready core | Keep and split | Needs availability reasons, status labels, activity context, source/health disabled detail. |
| `MissionLifecycleCard` | Legacy mission/dock/battery summary plus dock command | `mission`, `battery`, `capabilities.dock_state`, `capabilities.return_to_dock` | `return_to_dock.supported && available !== false` | Valetudo-compatible with minor changes | Replace later | It displays dock capability status, not `snapshot.dock.state`; "Mission Lifecycle" is the wrong product language for basic hardware cleaning. |
| Mode switcher | Exposes Mapping, Navigate, Clean Area, Rooms | `mapping`, `navigation`, coverage, room/zone capability booleans | `supported` booleans plus local lock state | Valetudo-inappropriate as primary Layer 6A UI | Hide/collapse later | Disabled tabs explain unsupported modes but make a basic robot look broken/simulation-first. |
| Mapping card/section | Start/pause/finish/save/load/improve maps | `snapshot.mapping`, map metadata, saved maps | `mapping_session` or `auto_mapping` | Valetudo-inappropriate for Layer 6A | Keep hidden unless capability exists | Current copy assumes exploration and maps; should not appear as active Valetudo control. |
| Navigate mode | Select map destination, send/cancel navigation | `pose`, `navigation`, `readiness`, `activeMission`, route progress | `start_navigation` or `go_to_location` supported | Valetudo-inappropriate for Layer 6A | Keep hidden unless capability exists | Correctly disabled when unsupported, but should not dominate mode default for basic robots. |
| Clean Area mode | Draw rectangle, preview route/coverage, start coverage | `map.grid`, `pose`, `start_coverage`, coverage mission state | `start_coverage` or `coverage_mission` | Valetudo-inappropriate for Layer 6A | Keep hidden unless capability exists | Area validation requires live map cells; no-map Valetudo should not see this as a normal mode. |
| Rooms / Zones mode | Draw/save room/zone annotations and clean selected annotation | `map.annotations`, `map.grid`, room/zone capabilities, coverage runtime | room/zone semantics or cleaning supported | Valetudo-inappropriate for Layer 6A | Keep hidden unless capability exists | Product room/zone workflow can return later after adapter maps segments/zones into normalized targets. |
| `UnsupportedFeatureCard` | Disabled-mode explanation | Capability notes/state reason | Rendered when mode selected but unsupported | Diagnostic/debug-adjacent | Keep but demote | Good for development, but Layer 6A operator UI should avoid presenting unavailable workflows as tabs. |
| `MapCanvas` | Occupancy map, target selection, paths, coverage overlays, annotations, sensor overlays | adapter map/pose/navigation props plus ROS fallback subscriptions | Parent gates only on `capabilities.map.supported` | TurtleBot4/Nav2 simulation/operator UI | Keep hidden unless map capability exists | Internally subscribes to ROS `/map` and costmaps as fallback; not a pure adapter renderer. |
| `CameraOverlay` | Floating ROS image feed | `ros2Bridge`, image topics | Not currently mounted here in reviewed render path | Valetudo-inappropriate for Layer 6A | Keep separate/debug only | Camera topic discovery is ROS-specific; do not expose for Valetudo basic robot. |
| `TeleopCard` | Manual directional velocity streaming | `ros2Bridge`, `/cmd_vel_raw`, manual control capability in parent | `capabilities.manual_control.supported` | Valetudo-inappropriate for Layer 6A | Keep hidden unless capability exists | Correctly gated in parent, but it is not a normalized adapter command surface. |
| Recent missions card | Shows terminal mission history | `missions.recent`, mission progress/result | Non-empty terminal missions | Valetudo-compatible with minor changes | Reuse later | Useful if Valetudo runtime records product-initiated cleaning/dock missions; otherwise activity history should be separate. |
| Command error banners | Show failed command messages | Local command error state from `sendCommand` result | Command-specific render paths | Valetudo-ready but incomplete | Keep, enrich | Current disabled reasons are often hidden in button `title`; Layer 6A needs visible reasons. |

## Valetudo Layer 6A UI Fit

| Layer 6A need | current UI fit | current source | gap | recommended action |
|---|---|---|---|---|
| Robot identity/name/model/source | Partial | `snapshot.identity.label`, adapter selector | Model/source not prominent; source is not shown as normalized source health | Add Basic Status card with identity and source kind/status. |
| Connection/availability | Partial | connection pill, status strip | Runtime/source split not shown | Show `availability`, `health.runtimeStatus`, and `source.status/stale` together. |
| Runtime health | Missing | `snapshot.health` exists in contract | UI does not consume it | Add health row/card. |
| Source reachable/stale/unreachable | Missing | `snapshot.source` exists in contract | UI does not consume it | Add visible source state and stale reason. |
| Activity status | Missing | `snapshot.activity` exists in contract | UI does not consume it | Add activity badge/status text as primary state. |
| Battery | Partial | `MissionLifecycleCard`, `battery` | Only shown in Clean/Rooms mode, not no-map basic surface | Move battery into Basic Status surface. |
| Charging | Partial | `battery.charging` | Not shown in no-map basic surface | Show with battery/dock. |
| Dock state | Weak | `capabilities.dock_state`, optional `snapshot.dock` unused | Current UI displays capability availability instead of actual dock state | Render `snapshot.dock.state` and `snapshot.dock.detail`. |
| Basic controls | Good baseline | `BasicControlsCard` | Only appears when map unsupported; no visible availability reasons | Keep; split reusable; show reason per command. |
| Command availability reasons | Weak | `capabilities.*.availabilityReason` exists | Buttons disabled without visible reason | Add small reason text under disabled actions. |
| Fault/diagnostic summary | Partial | `fault` exists, diagnostics exists | UI does not show diagnostics/faults in no-map state | Add compact fault summary plus diagnostics drawer. |
| Useful no-map fallback | Partial | `vacuum-map-empty` plus `BasicControlsCard` | Too sparse; mode switcher still simulation-heavy | Replace fallback area with Basic Robot surface. |
| Diagnostics without product behavior | Missing UI | `snapshot.diagnostics` | No diagnostics drawer/card | Add explicit diagnostics drawer that never gates controls. |

## Capability-Gating Audit

Passes:

- Basic controls use normalized capability fields: `start_cleaning`, `pause`, `stop`, and `return_to_dock` check `supported` and `available !== false`.
- Map rendering is gated by `capabilities.map.supported`.
- Mapping, navigation, Clean Area, Rooms/Zones, and teleop are gated by normalized `snapshot.capabilities.*.supported` booleans.
- UI submits normalized commands for product operations: `start_cleaning`, `pause`, `stop`, `return_to_dock`, `start_navigation`, `start_coverage`, mapping commands, mission controls, and room/zone commands.
- Backend id is only used for adapter selection/composition, which is acceptable.
- Raw Valetudo capability names are not used by the reviewed UI.

Gaps and risks:

- Several controls only check `supported`, not `status`, `available`, or `availabilityReason`; this is most visible in mode visibility and simulation-heavy command gates.
- `canSendRun`, `canStartCleanArea`, and room/zone gates combine normalized support with UI-local/readiness state, but do not yet use per-command `available` or `availabilityReason`.
- Disabled command reasons are mostly in `title` attributes or generic text; operators may not see why actions are blocked.
- The status strip is state-driven but map/pose-centric, so normalized health/source/activity/dock fields are effectively invisible.
- `MapCanvas`, `TeleopCard`, and `CameraOverlay` directly consume `ros2Bridge` and ROS topic names; they are simulation/operator surfaces and must remain hidden for Valetudo Layer 6A.
- `MappingCard` still renders compatibility fields such as saved/loaded map paths when mapping is supported. That is acceptable for simulation/debug now, but product UI should move those to diagnostics before real hardware map workflows.

Boundary rule check:

- Product UI mostly renders normalized adapter state and submits normalized commands.
- Backend adapter owns mapping backend runtime state into `vacuum_adapter`.
- VM runtime owns backend connection, state cache, and command routing.
- Current violation risk is not Valetudo raw names; it is that simulation-heavy product surfaces still read ROS bridge streams directly and dominate the layout when capabilities are present.

## No-Map UI Assessment

The current no-map Valetudo UI is useful enough for a developer smoke test, but not enough for an operator surface.

What works:

- It does not crash when no map is supported.
- It shows robot identity and map-unavailable detail.
- It shows Basic Cleaning controls when basic command capabilities are supported.
- Unsupported modes are disabled when their normalized capabilities are unsupported.

What is missing:

- Runtime health, source reachability/staleness, activity, dock state, battery, and charging are not presented together.
- Basic command availability reasons are not visible enough.
- The operator sees an empty map region plus disabled mode tabs, which frames Valetudo as a missing simulation robot instead of a valid basic vacuum.
- It does not clearly explain that map/navigation/coverage/rooms are intentionally unsupported/deferred for Layer 6A.

Recommendation: UI-B1 should create a reduced Basic Robot / Vacuum Status surface inside Vacuum Control. It should replace the empty map area when `capabilities.map.supported` is false, and the mode switcher should either hide unsupported tabs or collapse them behind an "Unavailable workflows" affordance.

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
| Navigation remains the default active mode | A no-map basic robot opens into a disabled simulation workflow | Default to Basic Robot surface when no map/navigation/coverage/rooms capabilities exist. |
| Status strip says Map Live and Localized | Basic Valetudo no-map/no-pose state looks unhealthy even when valid | Replace with source/activity/dock/battery status for basic backends. |
| `MapCanvas` subscribes to `/map` and costmaps | Product map surface can bypass adapter state with ROS fallback | Treat as simulation map component; later split adapter-only map renderer. |
| Teleop publishes `/cmd_vel_raw` | Not a normalized command path and not Valetudo-safe | Keep gated behind `manual_control`; do not show for Valetudo Layer 6A. |
| Camera discovers ROS image topics | Not part of Valetudo basic vacuum contract | Keep separate/debug only. |
| Disabled mode tabs remain visible | Operator sees unsupported features as broken controls | Hide/collapse unsupported simulation-heavy modes for basic robot capability profile. |
| Dock UI shows capability status, not dock state | Operator cannot tell docked/returning/charging | Render `snapshot.dock.state` and `battery.charging`. |
| Diagnostics absent | Source stale/unreachable and raw capability debugging require code inspection | Add diagnostics drawer while keeping it behavior-passive. |

## Recommended Implementation Milestones

| milestone | scope | notes |
|---|---|---|
| UI-B1: Improve reduced Valetudo no-map operator surface | Show Basic Robot / Vacuum Status when map/navigation/coverage/rooms are unsupported | Include identity, availability, health, source, activity, dock, battery, faults, and basic controls. |
| UI-B2: Split Basic Status / Basic Controls into reusable components | Extract current basic controls and new status rows from `VacuumControlPanel` | Makes Valetudo and future no-map hardware paths reusable without touching simulation modes. |
| UI-B3: Hide or collapse unsupported simulation-heavy modes | Replace always-visible disabled tabs with a compact unavailable-workflows area for basic robots | Keep existing modes for TurtleBot4/Nav2 and any backend that supports the capability. |
| UI-B4: Add diagnostics drawer/card | Render `snapshot.diagnostics`, health/source warnings, and capability diagnostics | Diagnostics are inspectable but never drive product behavior. |
| UI-B5: Add live webview smoke validation checklist | Document manual close/reopen and runtime-online/offline checks | Especially important for VM runtime hydration. |
| UI-B6: Future capability-specific UI for fan/water/consumables | Wait until adapter exposes presets/current state and state-aware command availability | Do not add Layer 6A controls for detected-but-unsupported surfaces. |

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
| Capabilities | Valetudo mock registered capabilities; runtime raw list; HTTP `/api/v2/robot/capabilities`; MQTT `$nodes` | `capabilities.commands`, `capabilities.diagnostics`, `diagnostics.rawCapabilityNames`, `capabilityTiers` | Only known capability subset influences descriptors; raw names remain diagnostics | `capabilities` normalized descriptor record | UI gates on normalized descriptors; raw names not consumed | Product-facing descriptors; raw capability names diagnostics-only |
| Map | Valetudo mock has map state and map endpoint; runtime currently does not fetch map | Runtime sets no map product data; diagnostics marks map unsupported | Map explicitly unavailable | `map.readiness=unavailable`, `map.grid=null`, `metadata.hasMap=false`, `capabilities.map.supported=false` | Main canvas placeholder, MapCanvas unmounted | Deferred product-facing map support; raw map diagnostics-only for now |
| Pose/navigation/go-to | Valetudo mock has GoTo capability and map robot entity; runtime does not normalize pose or go-to workflow | Go-to raw capability may be detected, but no command path exposed product-ready | Navigation/go-to unsupported/detected-not-ready | `pose.available=false`, `navigation.active=false`, `capabilities.go_to_location.status=detected_not_ready` when detected | Navigation unavailable workflow only; no active go-to UI | Deferred |
| Clean Area/coverage | No current Valetudo runtime coverage executor | No product response | Unsupported | `capabilities.start_coverage`, `coverage_mission` unsupported | Unavailable workflow summary | Deferred |
| Rooms/zones/segments | Valetudo mock has map segmentation and zone cleaning capabilities; runtime only detects names | Diagnostics tiers; no segment IDs/zone geometry normalized | Segment/zone detected-not-ready when raw capabilities present | `capabilities.segment_cleaning`, `zone_cleaning` detected_not_ready/unsupported; `room_*`, `zone_semantics` unsupported | Unavailable workflow summary | Deferred |
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
| `segment_cleaning`, `start_room_cleaning`, `start_zone_cleaning`, `zone_cleaning` | No runtime command implemented | No source routing | Unsupported; segment/zone requires explicit IDs/geometry mapping later | Rooms/zones unavailable | Deferred |
| `set_fan_speed`, `set_water_usage` | Runtime command `set_fan_speed` / `set_water_usage`; HTTP preset PUT to the matching Valetudo capability; fixed mock updates preset state | Requires reachable/fresh source, matching capability, payload `value`, and value in normalized options | Adapter rejects missing/invalid values as `invalid_request`, maps unsupported/unavailable/source failures to shared errors | Cleaning Settings card | Product-facing for fan/water mock-backed settings |
| Mapping, map annotations, coverage, retry/skip/resume mission | No Valetudo runtime executor | No source routing | Unsupported | Unavailable workflows or hidden in no-map mode | Deferred |

## Capability Classification

Product-facing now:

- `mission_state`, `start_cleaning`, `pause`, `stop`, `return_to_dock`, `pause_mission`, `cancel_mission`, `battery`, `dock_state`, `events`, `fault_state`, `fan_speed`, `water_usage`, and `consumables` as normalized descriptors/state for the implemented mock/runtime path.

Detected or known but deferred:

- Map rendering/snapshot, pose, go-to/navigation, mapping sessions, map annotations, room semantics, zone semantics, room cleaning, segment cleaning, zone cleaning, Clean Area/coverage, consumable reset commands, current/total statistics, manual control, locate, auto-empty dock actions, carpet/carpet sensor settings, obstacle/pet/collision avoidance settings, operation mode, pending map changes, persistent maps, mop dock clean/dry actions, speaker/voice pack controls, Wi-Fi configuration/scan, key lock, and do-not-disturb.

Diagnostics-only fields that must not drive product behavior:

- Raw Valetudo capability names, capability tiers, Valetudo HTTP endpoints and payloads, MQTT topics/payloads, transport diagnostics, source URLs/broker URLs, raw Valetudo state strings before adapter normalization, runtime ID/version, last command diagnostic details, raw map/pose/navigation/mapping diagnostic notes, and Valetudo event payloads until normalized into product state.

## Adapter/UI Gaps

- The adapter exposes `diagnostics`, including raw capability names, capability tiers, transports, warnings, raw source state, and last command audit, but the UI has no dedicated diagnostics drawer.
- The Valetudo mock source has map data and robot pose-like entities, but the runtime/adapter intentionally expose no product map/pose yet.
- Valetudo mock events exist, but the runtime does not inventory/fetch them into normalized `fault`/`events` product state.
- Statistics, zones, segments, and go-to are detected/known but not normalized into usable commands, current values, or target IDs/geometry. Consumable reset commands are not implemented.
- `resume` exists in the shared command/capability contract, but the Valetudo backend marks it unsupported and the basic UI does not expose it.
- Runtime invalid-state command responses are normalized to `invalid_state` for fixed mock basic commands. The adapter still accepts legacy `command_invalid_state` as a compatibility alias.
- The adapter has `snapshot.health` and `snapshot.source`; UI shows summarized normalized values, but not the full health endpoint result or source diagnostics.

## Existing Validation Coverage Found

- VM runtime tests cover fixed mock snapshot identity, raw capability diagnostics, capability tiers, last command audit, unknown command unsupported results, malformed command requests, source-unreachable unavailable results, fixed mock command state transitions, invalid-state behavior for basic commands, missing BasicControl capability, HTTP mock source snapshot/command routing, HTTP source command failure, HTTP source unavailable/stale diagnostics, MQTT snapshot hydration, MQTT stale diagnostics, stale MQTT command unavailability, and MQTT disconnected command unavailability.
- Adapter regression covers Valetudo capability mapping, command mapping, runtime command result mapping, malformed backend command-response aliasing, state-aware command availability, runtime unavailable mapping, no-map product surface safety, readable disabled reasons, and raw capability privacy from UI behavior.

## Boundary Hardening Closure

Milestone 1 inventory is complete. Milestone 2A command/result hardening is complete. The final boundary lock pass confirmed focused runtime and adapter coverage for stale source command blocking, invalid-state results, unsupported/capability-unavailable results, malformed command requests, source command failures, malformed backend command-response aliases, readable disabled reasons, and raw Valetudo capability privacy.

Diagnostics remain intentionally passive. Raw Valetudo capability names, transport details, source URLs, last-command audit data, and raw runtime/source details stay diagnostics-only and must not gate product controls. Product behavior continues to branch on normalized adapter state and capability descriptors.

Deferred Valetudo capabilities remain deferred: map rendering, go-to/navigation, Clean Area, rooms/zones/segments, consumable reset commands, statistics, OpenClaw, MQTT production hardening beyond this feature path, real hardware support, and diagnostics drawer work are outside this closed regression/diagnostics-focused thread. Fan speed, water usage, and consumable display are no longer deferred for the mock/runtime/adapter/UI path.

## Next Product Direction

Recommended next milestone: `Valetudo Milestone 3 - Basic Operator Surface Forward Pass`.

Focus:

1. Improve product/operator UX for the existing supported no-map Valetudo path.
2. Keep the simulation layout intact.
3. Use only normalized adapter fields.
4. Do not add a diagnostics drawer unless explicitly requested later.
5. Do not add advanced controls until the runtime normalizes the required state, presets, and targets.

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
