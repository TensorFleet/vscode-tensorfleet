# Layer 6A Plan - Valetudo Mock Through VM

Current plan date: 2026-05-29.

## Context

We are freezing new TurtleBot4/Nav2 product work and keeping the
TurtleBot4/Nav2 path as the simulation and regression backend while we build
the Valetudo backend path.

The current product-facing architecture remains:

```text
VS Code extension / product UI
  -> vacuum_adapter contract
     -> TurtleBot4/Nav2 backend adapter
        -> VM TurtleBot4 simulation runtime

     -> Valetudo backend adapter
        -> VM-managed Valetudo integration runtime
           -> Valetudo mock source first
           -> real Valetudo-compatible vacuum later
```

The immediate goal is:

```text
Valetudo mock-shaped data
-> VM Valetudo integration runtime
-> small runtime API
-> Valetudo backend adapter
-> vacuum_adapter
-> existing VS Code extension UI
```

The goal is not real hardware yet. The goal is to prove that Valetudo-shaped
state, capabilities, and basic commands can travel through the same
product-facing adapter boundary that the current UI already uses.

## Current Decision

Freeze new TurtleBot4/Nav2 product expansion.

Keep TurtleBot4/Nav2 as the simulation and regression backend.

Build the Valetudo backend path next.

Do not add ROS2 bridge support for Valetudo in Layer 6A.

Do not add OpenClaw support in Layer 6A.

Do not connect real robot hardware in Layer 6A.

Do not add Valetudo map rendering in Layer 6A.

Do not refactor MapCanvas beyond minimum guards needed to avoid broken UI.

## Deployment Meaning

Use this wording consistently:

```text
Valetudo runs on the robot.
The VM runs our Valetudo integration runtime.
```

For the mock milestone:

```text
Valetudo mock source runs locally or in a local development environment.
The VM Valetudo integration runtime connects to that mock source.
The adapter connects to the VM runtime API.
```

For the later real hardware path:

```text
Real robot runs Valetudo.
The same VM Valetudo integration runtime connects to the real robot over the local network.
The same adapter connects to the VM runtime API.
```

## Architecture Rules

The product-facing boundary remains:

```text
Product UI
-> vacuum_adapter contract
-> backend-specific adapter
-> backend runtime
```

The UI must not call Valetudo HTTP, MQTT, or runtime endpoints directly.

The UI must not know Valetudo capability class names.

The UI must not branch on backend names for product behavior.

The UI must branch on normalized capability descriptors and normalized adapter
state.

The Valetudo backend adapter may know about Valetudo concepts.

The VM Valetudo integration runtime may know about Valetudo HTTP, MQTT, mock
source details, state caching, stale-state handling, health checks, and command
routing.

Raw Valetudo capability names may exist in runtime diagnostics, but they must
not become product UI flags.

Public product behavior must be based on `vacuum_adapter` capability
descriptors, normalized state, and explicit unsupported command results.

## Global Constraints

1. Keep product UI backend-neutral.
2. Branch on capabilities, not backend names.
3. Keep Valetudo capability names private to the Valetudo backend/runtime diagnostics.
4. Do not expose raw Valetudo HTTP/MQTT payloads to product UI.
5. Do not expose raw ROS topics to Valetudo product UI.
6. Do not make Valetudo pretend to be Nav2.
7. Do not fake `/tf`, `/odom`, `/cmd_vel`, costmaps, or `NavigateToPose`.
8. Preserve existing `vacuum_adapter` surfaces where possible.
9. Unsupported operations must fail explicitly.
10. Map-dependent UI must not crash when the Valetudo backend has no map.
11. Keep TurtleBot4/Nav2 details inside the TurtleBot4/Nav2 backend/runtime layer.
12. Keep Valetudo details inside the Valetudo backend/runtime layer.
13. Each milestone must be runtime-testable.
14. Update `progress_report.md` after each completed milestone.
15. Record contract changes after each milestone.

## Non-Goals For Layer 6A

Do not implement:

```text
ROS2 bridge for Valetudo
OpenClaw integration
real robot hardware support
robot model selection or purchase recommendation
Valetudo map rendering
Valetudo robot movement visualization
Clean Area support for Valetudo
room/segment cleaning execution
zone cleaning execution
go-to-location execution
fan/water setters as product-ready controls
consumables UI polish
voice pack / Wi-Fi / speaker / vendor extra controls
scheduling
MQTT production hardening
multi-robot fleet management
cloud/Kamal deployment strategy
major product dashboard redesign
full MapCanvas refactor
```

## Revised Milestone Order

```text
0. Freeze TurtleBot4/Nav2 product expansion and protect regression path.
1. Runtime API contract + fixed mock runtime.
2. Runtime client + Valetudo adapter selectable + minimum no-map UI safety.
3. Basic command routing + mutable mock state.
4. Capability mapper hardening + adapter tests.
5. Actual Valetudo mock source connection.
6. Optional MQTT behind runtime boundary.
7. Expanded diagnostics only.
8. Documentation + Layer 6A summary.
```

Do not move to real hardware until Milestones 1 through 5 are stable.

Do not move to ROS2/OpenClaw until the mock-through-VM path is stable.

Do not move to Valetudo map support until basic state, capabilities, commands,
and no-map UI safety are stable.

## Milestone 0 - Freeze TurtleBot4/Nav2 Product Expansion

### Feature Goal

Preserve the current TurtleBot4/Nav2 path as the simulation and regression
backend while preventing new TurtleBot4/Nav2 product scope from entering Layer
6A.

### Required Behavior

The current TurtleBot4/Nav2 adapter and UI workflows should remain available.

No new TurtleBot4/Nav2 product features should be added as part of Valetudo
Layer 6A work.

The existing `vacuum_adapter` public contract should remain valid unless a
small, explicit, documented contract change is needed for Valetudo.

### Acceptance Checks

```text
Existing TurtleBot4/Nav2 regression harness still passes.
No new TurtleBot4/Nav2 product behavior is added.
Existing vacuum_adapter public contract remains valid or changes are documented.
Valetudo work is isolated to runtime/client/adapter/UI safety changes.
```

### Ownership Requirements

```text
TurtleBot4/Nav2 backend:
continues to own simulation and regression behavior.

Valetudo backend:
owns new Layer 6A integration work.

Product UI:
continues to consume vacuum_adapter only.
```

## Milestone 1 - Valetudo Runtime API Contract + Fixed Mock Runtime

### Feature Goal

Create the smallest VM-side Valetudo runtime API boundary and make it the
source of truth for adapter integration.

This milestone should not connect to the actual Valetudo mock source yet.

### Runtime API

Expose:

```text
GET /health
GET /snapshot
POST /command
```

Define exact runtime contract types before adapter work:

```ts
ValetudoRuntimeHealth
ValetudoRuntimeSnapshot
ValetudoRuntimeCommandRequest
ValetudoRuntimeCommandResult
ValetudoRuntimeCapabilityDiagnostic
```

The runtime API is private to the integration path. The public product contract
remains `vacuum_adapter`.

### Snapshot Shape

`GET /snapshot` should return one fixed mock robot and include:

```text
runtime id/version
backend = valetudo
robot id
robot name
source kind = fixed_mock | valetudo_mock | real_robot
runtime status
source status
reachable / online state
stale flag
last successful source poll timestamp if available
updatedAt
current robot state
battery level if available
charging state if available
dock state if available
raw Valetudo capability names as diagnostics only
normalized command availability
diagnostics
optional raw diagnostics field
```

Use distinct runtime and source health concepts:

```ts
runtime: {
  status: "online" | "degraded" | "offline";
}

source: {
  kind: "fixed_mock" | "valetudo_mock" | "real_robot";
  status: "reachable" | "unreachable" | "unknown";
  stale: boolean;
  lastSeenAt: number | null;
}
```

Fixed mock capability diagnostics should include enough Valetudo names to test
mapping, for example:

```text
BasicControlCapability
BatteryStateCapability
FanSpeedControlCapability
WaterUsageControlCapability
MapSegmentationCapability
ZoneCleaningCapability
GoToLocationCapability
ConsumableMonitoringCapability
```

These raw names are runtime diagnostics only.

### Command Shape

`POST /command` should accept normalized command names for this slice:

```text
start_cleaning
pause
stop
return_to_dock
```

Command results should include:

```text
ok
status = success | unsupported | unavailable | failed
command
message
reason/code
updatedAt
diagnostics optional
```

Unsupported commands must return structured unsupported results.

Commands must be rejected or marked unavailable when the source is unreachable.

### Runtime-Testable Acceptance Checks

```text
Start the VM Valetudo integration runtime.
Call GET /health and confirm runtime alive.
Call GET /snapshot and confirm one fixed mock Valetudo robot.
Call POST /command with start_cleaning and confirm structured mock success.
Call POST /command with unsupported command and confirm structured unsupported.
Simulate source reachable = false.
Call GET /snapshot and confirm identity + updatedAt remain available when possible.
Call POST /command while source unreachable and confirm unavailable/rejected result.
```

### Ownership Requirements

```text
VM runtime:
owns fixed mock Valetudo runtime state, health, stale/source status, and command response behavior.

Extension:
does not call this runtime yet unless Milestone 2 is also implemented.

Product UI:
unchanged.

vacuum_adapter:
unchanged unless contract gaps are explicitly documented.
```

### Do Not Do

Do not connect to the actual Valetudo mock source.

Do not add MQTT.

Do not add real Valetudo robot HTTP calls.

Do not modify MapCanvas.

Do not modify TurtleBot4/Nav2 behavior.

Do not add ROS2 bridge support.

Do not add OpenClaw.

## Milestone 2 - Runtime Client + Valetudo Adapter Selectable + No-Map UI Safety

### Feature Goal

Make `useVacuumAdapter({ backend: "valetudo" })` stop throwing and return a
safe Valetudo-backed snapshot through the existing product boundary.

The flow becomes:

```text
VM fixed Valetudo mock snapshot
-> Valetudo runtime client
-> Valetudo backend adapter
-> VacuumAdapterSnapshot
-> existing UI
```

### Required Behavior

Add a runtime client that fetches:

```text
GET /health
GET /snapshot
POST /command
```

Add the Valetudo backend adapter implementation behind the existing
`vacuum_adapter` backend selection path.

The adapter should:

```text
1. Connect to the VM Valetudo integration runtime API.
2. Poll or fetch GET /snapshot.
3. Convert the runtime snapshot into VacuumAdapterSnapshot.
4. Implement sendCommand for supported normalized commands.
5. Return explicit unsupported results for unsupported commands.
6. Expose public vacuum_adapter capability descriptors.
7. Avoid exposing Valetudo raw capability names as product UI flags.
8. Report unavailable/offline when the runtime is stopped.
9. Recover on polling or refresh when the runtime restarts.
```

The UI must still consume `useVacuumAdapter` only.

The UI must not call VM runtime endpoints directly.

### Capability Defaults

Supported in this milestone:

```text
identity
availability
mission_state/basic_activity
battery if present
dock_state if present
start_cleaning if BasicControlCapability exists
pause if BasicControlCapability exists
stop if BasicControlCapability exists
return_to_dock if BasicControlCapability exists
fault_state if runtime reports diagnostics/faults
```

Unsupported in this milestone:

```text
map
pose
start_navigation
go_to_location
cancel_navigation
start_coverage
coverage_mission
map_annotations
mapping_session
auto_mapping
room_semantics
zone_semantics
room_cleaning
zone_cleaning
segment_cleaning
manual_control
camera
```

If the runtime includes detected map/zone/segment/go-to capabilities as
diagnostics, the product capability should remain unsupported until the product
workflow exists.

### Minimum UI Safety

The current `Vacuum Control` surface contains map, navigation, mapping, Clean
Area, Rooms/Zones, teleop, camera, and runtime actions in one panel. Therefore,
Valetudo selectability must include enough UI safety to avoid broken controls.

Minimum behavior:

```text
If map is unsupported:
  do not mount MapCanvas in a broken state.

If navigation is unsupported:
  hide, disable, or show unavailable navigation controls.

If Clean Area / coverage is unsupported:
  hide, disable, or show unavailable Clean Area controls.

If mapping is unsupported:
  hide, disable, or show unavailable Mapping controls.

If Rooms / Zones are unsupported:
  hide, disable, or show unavailable Rooms / Zones controls.

If manual_control is unsupported:
  hide, disable, or show unavailable Teleop controls.

If camera is unavailable:
  hide CameraOverlay or show an unavailable diagnostics state.
```

Do not branch on backend name for product behavior. Use capabilities and
snapshot availability.

### Runtime-Testable Acceptance Checks

```text
Start VM Valetudo integration runtime with fixed mock data.
Start the extension.
Select or configure the Valetudo backend.
Confirm useVacuumAdapter returns a Valetudo-backed snapshot.
Confirm UI shows robot name, connection state, activity, battery/dock if present.
Confirm MapCanvas is not mounted in a broken state when map is unsupported.
Confirm navigation/Clean Area/Rooms/Zones/Teleop/Camera are hidden, disabled, or unavailable based on capabilities.
Confirm unsupported commands return explicit unsupported results.
Stop the VM runtime.
Confirm adapter reports unavailable/offline state.
Confirm UI does not crash.
Restart the VM runtime.
Confirm adapter recovers on polling or refresh.
```

### Ownership Requirements

```text
VM runtime:
owns runtime snapshot, health, stale/source status, and command route.

Valetudo backend adapter:
owns runtime client usage and mapping into vacuum_adapter state.

Product UI:
continues to consume useVacuumAdapter only.

UI:
branches only on normalized capabilities and state.
```

### Do Not Do

Do not connect to a real robot.

Do not add MQTT.

Do not add Valetudo map rendering.

Do not redesign the UI.

Do not add ROS2 bridge support.

Do not add OpenClaw.

## Milestone 3 - Basic Command Routing + Mutable Mock State

### Feature Goal

Prove the command loop through the actual product boundary.

The flow should be:

```text
UI action
-> adapter.sendCommand
-> Valetudo backend adapter validates capability
-> runtime POST /command
-> runtime command router
-> mutable mock state transition
-> next /snapshot
-> adapter maps snapshot
-> UI updates
```

### Required Behavior

Support these normalized commands:

```text
start_cleaning
pause
stop
return_to_dock
```

The runtime command router should maintain enough mock state to show command
effects.

Simple deterministic transitions:

```text
idle/docked -> cleaning
cleaning -> paused
cleaning/paused -> idle or stopped
cleaning/paused/idle -> returning_to_dock -> docked
```

Exact timing may be simple, but it must be deterministic and easy to test.

Unsupported commands must return explicit unsupported results.

If `BasicControlCapability` is absent, basic commands must be disabled or
rejected as unsupported.

### Runtime-Testable Acceptance Checks

```text
Start runtime with fixed mock Valetudo state.
Open extension using Valetudo backend.
Confirm start_cleaning is available when BasicControlCapability is present.
Trigger start_cleaning.
Confirm command returns success.
Confirm next snapshot shows cleaning.
Trigger pause.
Confirm snapshot shows paused.
Trigger stop.
Confirm snapshot shows idle/stopped.
Trigger return_to_dock.
Confirm snapshot shows returning or docked.
Remove BasicControlCapability from mock capabilities.
Confirm the same commands are hidden, disabled, or return unsupported.
Confirm unsupported command path is tested.
```

### Ownership Requirements

```text
UI:
submits normalized commands only.

Valetudo backend adapter:
validates public capabilities and maps command result.

VM runtime:
owns command routing and mutable mock state transition.
```

### Do Not Do

Do not implement room cleaning commands.

Do not implement zone cleaning commands.

Do not implement go-to-location.

Do not implement fan/water setters unless basic command routing is already
complete and explicitly scoped.

Do not add ROS2.

Do not add OpenClaw.

## Milestone 4 - Capability Mapper Hardening + Adapter Tests

### Feature Goal

Make Valetudo capability mapping honest, private, and regression-tested.

Valetudo capability names are backend-private input.

The product UI should only see public capability descriptors.

### Mapping Rules

At minimum:

```text
BasicControlCapability
-> start_cleaning
-> pause
-> stop
-> return_to_dock

BatteryStateCapability
-> battery

Dock / charging state when runtime has actual state
-> dock_state

FanSpeedControlCapability
-> fan_speed detected, but product unsupported unless setter path exists

WaterUsageControlCapability
-> water_usage detected, but product unsupported unless setter path exists

ConsumableMonitoringCapability
-> consumables detected/diagnostic first

MapSegmentationCapability
-> segment_cleaning detected, product unsupported for now

ZoneCleaningCapability
-> zone_cleaning detected, product unsupported for now

GoToLocationCapability
-> go_to_location detected, product unsupported for now
```

Detected-but-unimplemented should usually be represented as:

```ts
{
  supported: false,
  source: "valetudo",
  notes: "Valetudo capability detected, but product workflow is not implemented in this slice."
}
```

This is safer than reporting `supported: true` without a working command path
and UI workflow.

If a public capability model needs diagnostics, keep that diagnostic data from
driving product UI logic.

### Runtime-Testable Acceptance Checks

```text
Provide a mock capability list containing BasicControlCapability.
Confirm start_cleaning, pause, stop, and return_to_dock are supported.
Provide a list without BasicControlCapability.
Confirm those commands are unsupported.
Provide BatteryStateCapability and battery state.
Confirm battery capability/state are represented correctly.
Provide FanSpeedControlCapability.
Confirm fan_speed is detected but not product-ready unless setter path exists.
Provide ZoneCleaningCapability.
Confirm zone_cleaning is detected but does not activate broken UI controls.
Provide GoToLocationCapability.
Confirm go_to_location is detected but remains product unsupported in Layer 6A.
Confirm product UI does not branch on BasicControlCapability or other raw Valetudo names.
Confirm missing capabilities produce unsupported descriptors.
Confirm unsupported command behavior matches capability descriptors.
```

### Ownership Requirements

```text
Raw Valetudo capability names:
private to Valetudo backend/runtime diagnostics.

Public capability descriptors:
owned by vacuum_adapter contract shape.

Product UI:
branches only on public capability descriptors.
```

### Do Not Do

Do not implement full zone cleaning.

Do not implement full segment cleaning.

Do not implement map rendering.

Do not add UI controls for every capability.

Do not expose raw Valetudo capability names as product flags.

## Milestone 5 - Connect Runtime To Actual Valetudo Mock Source

### Feature Goal

Replace fixed internal runtime data with data read from an actual
Valetudo-shaped mock source while keeping the adapter-facing API stable.

The flow becomes:

```text
Valetudo mock source
-> VM Valetudo integration runtime
-> GET /snapshot and POST /command
-> Valetudo backend adapter
-> vacuum_adapter
-> UI
```

### Required Behavior

Run the Valetudo mock source locally or in the local development environment.

Add a runtime source client that can read state and capabilities from the mock
source.

Normalize source data into the existing `ValetudoRuntimeSnapshot` shape.

Route supported basic commands to the mock source where possible.

If the mock source cannot actually perform a command, return a clear mock
limitation result.

If the mock source exposes only static data, document that clearly.

HTTP should be implemented first if available.

MQTT should remain deferred unless the mock source makes MQTT straightforward
and useful.

The runtime API consumed by the adapter must remain stable:

```text
GET /health
GET /snapshot
POST /command
```

### Runtime-Testable Acceptance Checks

```text
Start the Valetudo mock source.
Start the VM Valetudo integration runtime.
Confirm runtime can reach the mock source.
Call GET /health.
Confirm health reports runtime alive and source reachable.
Call GET /snapshot.
Confirm snapshot data is based on the mock source.
Open extension with Valetudo backend.
Confirm UI shows mock robot state from runtime.
Trigger start_cleaning if supported by mock source.
Confirm state changes if mock source supports command state changes.
Stop mock source.
Confirm runtime reports source unreachable/stale.
Confirm adapter/UI do not crash.
Restart mock source.
Confirm runtime reconnects or recovers.
```

### Ownership Requirements

```text
Valetudo mock source:
provides raw mock robot state/capabilities.

VM runtime:
owns source connection, normalization, state cache, health, stale behavior, and command routing.

Valetudo backend adapter:
continues to consume only the VM runtime API.

Product UI:
still consumes only vacuum_adapter.
```

### Do Not Do

Do not add real robot support.

Do not add production MQTT unless required by the mock source.

Do not add map rendering.

Do not add ROS2.

Do not add OpenClaw.

## Milestone 6 - Optional MQTT Prototype Behind Runtime Boundary

### Feature Goal

Add MQTT support only if it provides useful mock state updates or prepares the
runtime for later real robot integration.

MQTT must remain behind the VM runtime boundary.

The adapter and UI should not know whether the runtime used HTTP, MQTT, or both.

### Required Behavior

If implemented, MQTT is internal to the VM Valetudo integration runtime.

The runtime may connect to a broker and subscribe to Valetudo-style topics.

The runtime should update its internal state cache from MQTT messages.

The runtime should continue exposing the same adapter-facing API:

```text
GET /health
GET /snapshot
POST /command
```

Commands may still use HTTP if that is simpler.

A hybrid model is acceptable:

```text
HTTP:
  commands and fallback status

MQTT:
  live state updates and events
```

### Runtime-Testable Acceptance Checks

```text
Start MQTT broker if needed.
Start Valetudo mock source configured for MQTT if supported.
Start VM Valetudo integration runtime.
Confirm runtime subscribes successfully.
Confirm state cache updates from MQTT messages.
Confirm GET /snapshot reflects latest MQTT-derived state.
Stop MQTT source or broker.
Confirm runtime marks state stale or source unreachable.
Confirm adapter/UI do not crash.
Confirm adapter/UI code did not change to know about MQTT.
```

### Ownership Requirements

```text
VM runtime:
owns MQTT broker/client integration.

Adapter:
still consumes stable runtime API.

UI:
does not know about MQTT.

Product contract:
unchanged.
```

### Do Not Do

Do not require MQTT for the first working Valetudo mock path.

Do not expose MQTT topics to product UI.

Do not make the extension talk to MQTT directly.

Do not add real robot MQTT configuration yet.

## Milestone 7 - Expanded Diagnostics, Not Expanded Product UI

### Feature Goal

Expose useful Valetudo runtime/source diagnostics without pretending detected
features are product-ready.

This milestone is about observability and future readiness, not adding product
controls.

### Required Behavior

Add diagnostics such as:

```text
raw Valetudo capability names in diagnostics only
source health
last successful poll
last command result
last command timestamp
stale source reason
runtime version
source kind
optional raw payload sample behind diagnostics
```

Capability tiers should be visible internally:

```text
Tier 1 - implemented:
  basic control
  battery if available
  dock/charging if available
  current state
  availability/connectivity

Tier 2 - detected but not product-ready unless command path exists:
  fan speed
  water usage
  consumables
  maintenance/statistics

Tier 3 - detected but not product-implemented:
  map
  map segments
  zone cleaning
  go-to-location
  native room/segment cleaning

Tier 4 - diagnostics/deferred:
  Wi-Fi configuration
  speaker/voice packs
  timers/schedules
  advanced restrictions
  vendor-specific extras
```

### Runtime-Testable Acceptance Checks

```text
Provide mock capability sets with different combinations.
Confirm diagnostics preserve raw capability names.
Confirm public capabilities remain conservative.
Confirm detected-but-unimplemented capabilities do not expose broken UI controls.
Confirm last poll and stale reason update correctly.
Confirm last command result is visible in diagnostics.
Confirm no product UI code branches on raw Valetudo names.
```

### Ownership Requirements

```text
VM runtime:
owns source diagnostics and raw source details.

Valetudo adapter:
owns public capability normalization and optional diagnostics pass-through.

Product UI:
does not use diagnostics as product behavior flags.
```

### Do Not Do

Do not add Valetudo map rendering.

Do not add rooms/segments execution.

Do not add zone cleaning execution.

Do not add go-to-location execution.

Do not add fan/water setters as product-ready controls.

Do not add consumables UI polish.

## Milestone 8 - Documentation + Layer 6A Summary

### Feature Goal

Document the implemented Layer 6A path clearly so future implementation work
does not confuse:

```text
Valetudo running on robot
```

with:

```text
VM Valetudo integration runtime
```

Documentation should describe what was implemented, what remains mock-only, and
what is explicitly out of scope.

### Required Behavior

Update relevant docs:

```text
VACUUM_STACK_PLAN.md
extension.md if needed
progress_report.md
plan.md if the plan changes
```

Documentation must state:

```text
Valetudo itself runs on the robot in the real hardware path.
The VM runs our Valetudo integration runtime.
Current Layer 6A uses fixed/mock Valetudo source first.
The adapter consumes the VM runtime API.
The UI consumes vacuum_adapter.
TurtleBot4/Nav2 remains the simulation/regression backend.
ROS2 bridge is out of scope for Layer 6A.
OpenClaw is out of scope for Layer 6A.
Real hardware is out of scope for Layer 6A.
Maps, rooms, zones, and go-to-location are out of scope for Layer 6A product behavior.
```

Document first supported behavior:

```text
identity
availability
state
battery if available
dock/charging if available
capabilities
start_cleaning
pause
stop
return_to_dock
explicit unsupported operations
runtime/source health
stale source handling
```

Document unsupported/deferred behavior:

```text
Valetudo map rendering
robot movement visualization
room/segment cleaning
zone cleaning
go-to-location
ROS2 bridge
OpenClaw
real hardware
MQTT production hardening
```

### Acceptance Checks

```text
Docs describe the actual implemented path.
Docs do not claim real hardware support.
Docs do not claim ROS2/OpenClaw support.
Docs do not imply Valetudo runs in the VM in production.
Docs distinguish mock source, VM integration runtime, adapter, and UI.
progress_report.md summarizes each completed milestone.
```

### Ownership Requirements

```text
Architecture docs:
explain the Layer 6A mock path.

Progress report:
summarizes implemented behavior, validation, contract changes, and remaining risks.

Implementation:
matches documented architecture.
```

### Do Not Do

Do not document future features as already implemented.

Do not conflate mock validation with real hardware validation.

Do not overpromise map/room/zone support.

## Progress Report Requirement

After each milestone, update or create:

```text
progress_report.md
```

The report must be feature-oriented, not a low-level commit log.

Mention files only in the "Files changed" section.

Use this structure:

```md
# Progress Report - <Milestone Name>

Current report date: <date>.

## 1. What changed

Describe the concrete product/runtime behavior added or changed in this pass.

Keep it feature-oriented.

## 2. Which mode this affects

- Mapping:
- Navigation:
- Clean Area:
- Rooms / Zones:
- Valetudo backend:
- Shared adapter/runtime architecture:

## 3. Ownership check

For each changed behavior, answer:

- Is this owned by React/webview state?
- Is this owned by the VM runtime?
- Is this owned by the Valetudo backend adapter?
- What state is the UI only rendering?
- What command does the UI submit?

Explicitly call out whether the change follows this rule:

Product UI:
renders normalized adapter state and submits normalized commands.

Backend adapter:
maps backend runtime state into vacuum_adapter.

VM runtime:
owns backend connection, state cache, and command routing.

## 4. Webview close/reopen behavior

Explain what happens if the webview closes during:

- idle Valetudo mock state
- unavailable VM runtime
- reachable mock runtime
- active mock cleaning state
- paused mock cleaning state
- terminal or stopped mock state

Then explain how the UI hydrates after reopening.

## 5. Real hardware compatibility check

Answer:

- Does this assume the robot is TurtleBot4/Nav2?
- Does this expose TurtleBot4/Nav2 specifics to product UI?
- Does this expose Valetudo raw capability names to product UI?
- Can the same VM runtime API later connect to a real Valetudo robot?
- What capability flags decide whether controls are shown/enabled?
- What operations are explicitly unsupported?

## 6. Feature behavior changed

List the user-visible or operator-visible behavior changes.

Examples:

- Valetudo mock robot appears online in the extension.
- Basic Valetudo capabilities are mapped into vacuum_adapter.
- Start cleaning sends a normalized command through the VM runtime.
- Unsupported map/navigation/coverage actions no longer appear as usable controls.

## 7. Files changed

List changed files and describe why each file changed.

Keep this section implementation-aware but concise.

## 8. Tests / validation run

List commands and manual runtime checks performed.

Include both automated checks and live operator validation when relevant.

Example:

```sh
bun run test:vacuum-adapter
bun run --cwd panels-standalone build
git diff --check
```

Also include manual checks such as:

```text
Started VM Valetudo runtime, called /health, confirmed alive.
Called /snapshot, confirmed mock Valetudo state.
Opened extension, confirmed Valetudo backend snapshot renders.
Triggered start_cleaning, confirmed command routes through runtime.
Stopped runtime, confirmed UI shows unavailable state instead of crashing.
```

## 9. Remaining risks

List anything still:

- frontend-owned
- mock-only
- not yet durable
- not yet connected to real Valetudo
- not yet using MQTT
- not yet compatible with real hardware
- not yet capability-gated
- still TurtleBot4/Nav2-shaped
- not compatible with future Valetudo implementation

## 10. Next recommended step

Give the next smallest architecture-aligned step.

## 11. Contract changes

- Runtime API changed?
- vacuum_adapter public contract changed?
- Capability descriptors changed?
- UI behavior changed?
- Backward compatibility impact?
```

## Final Acceptance Criteria For Layer 6A

Layer 6A is complete when:

```text
1. Valetudo mock source can run locally or be simulated by fixed VM runtime data.
2. VM Valetudo integration runtime exposes /health, /snapshot, and /command.
3. Runtime snapshot includes identity, connectivity, runtime/source health, stale state, capabilities, battery/dock when available, and updated timestamp.
4. Runtime command endpoint returns structured success, unsupported, unavailable, and failed results.
5. Valetudo backend adapter fetches runtime snapshot.
6. Adapter maps snapshot into VacuumAdapterSnapshot.
7. Existing extension UI can render Valetudo robot identity and state.
8. Basic controls route through adapter.sendCommand into VM runtime:
   - start_cleaning
   - pause
   - stop
   - return_to_dock
9. Valetudo capability mapper converts raw Valetudo capabilities into conservative public vacuum_adapter capabilities.
10. Product UI does not branch on Valetudo raw capability names.
11. Product UI does not call Valetudo runtime endpoints directly.
12. Product UI does not crash when map/navigation/coverage/mapping are unsupported.
13. Detected-but-unimplemented capabilities do not activate broken UI controls.
14. TurtleBot4/Nav2 simulation backend still works.
15. ROS2 and OpenClaw remain explicitly out of scope.
16. Real hardware remains explicitly out of scope.
17. progress_report.md is updated after each completed milestone.
18. Contract changes are documented after each completed milestone.
```

## Questions To Resolve During Implementation

Answer these before or during implementation:

```text
1. Should the Valetudo runtime live inside existing tensorfleet-mgr, or as a sibling guest service?
2. Should the extension reach it through vm-manager /vms/self/tensorfleet/... or a new route?
3. How should the extension discover the VM runtime base URL?
4. Should the first runtime use fixed data before connecting to the actual Valetudo mock source?
5. Which Valetudo mock source will be used?
6. Does the mock source expose HTTP, MQTT, or both?
7. Should MQTT be implemented now or deferred?
8. How should partial capability support be represented?
9. Should detected-but-unimplemented Valetudo capabilities be marked unsupported with diagnostics?
10. What is the minimum UI change needed so Valetudo backend does not mount broken map/navigation controls?
11. Which verification commands are reliable for the current repo?
12. Should runtime diagnostics include raw payload samples, and where should they be hidden?
13. What is the expected stale-state timeout for mock and later real sources?
14. Should command availability be computed by the runtime, the adapter, or both?
```

## Recommended First Implementation Task

Start with the smallest possible slice:

```text
Create Valetudo runtime contract types.
Create fixed mock VM Valetudo integration runtime.
Expose GET /health, GET /snapshot, POST /command.
Return fixed mock snapshot, structured command success, unsupported, and unavailable.
Validate runtime endpoints manually.
Update progress_report.md for Milestone 1.
```

Then:

```text
Implement the Valetudo runtime client and adapter skeleton.
Make backend = valetudo selectable.
Return a safe snapshot with no map, no pose, no navigation, no coverage.
Show only identity, availability, state, battery/dock if present, and basic controls.
Update progress_report.md for Milestone 2.
```

Do not start with MQTT.

Do not start with MapCanvas refactoring.

Do not start with rooms/zones.

Do not start with ROS2.

Do not start with OpenClaw.

Do not start with real hardware.
