# TensorFleet MCP Architecture Reference

Current document date: 2026-06-18.

This document is the durable reference for Model Context Protocol work in
`vscode-tensorfleet`. It records the current project structure, the existing
MCP scaffold, the recommended architecture for real MCP tools, and the
vacuum-first implementation direction. It is intentionally broader than a task
plan so future work can reference this file alone.

## 1. Goal

TensorFleet should expose a small, safe, product-level MCP surface that lets
agents inspect and operate the extension-backed robotics environment without
learning raw implementation details.

The first real slice should focus on vacuum behavior. Later slices can add VM,
Gazebo, Nav2, telemetry, map, mission, and broader extension tools using the
same design.

Target mental model:

```text
Agent host
  -> MCP client
    -> TensorFleet MCP server
      -> TensorFleet client/domain modules
        -> vm-manager and extension bridge
          -> VM runtime services
            -> robot, simulator, or integration runtime
```

The MCP server should be an action and context facade. It should not become a
raw shell, raw HTTP proxy, raw ROS bridge, raw MQTT bridge, or direct Valetudo
client surface for agents.

## 2. MCP Concepts

MCP gives an agent host a structured way to discover and call capabilities.
For TensorFleet, the relevant MCP primitives are:

- Tools: executable operations such as `vacuum_get_snapshot` or
  `vacuum_return_to_dock`.
- Resources: read-only context objects such as `tensorfleet://vacuum/snapshot`
  or `tensorfleet://vm/status`.
- Prompts: reusable workflows such as "diagnose why the vacuum cannot start".

The first TensorFleet MCP milestone should primarily use tools. Resources can
follow once the tool results and domain client contracts are stable. Prompts are
useful later when workflows become repeatable.

## 3. Current Project Structure

Important top-level files:

- `src/mcp-server.ts`: standalone MCP server entrypoint. Built as a Node
  process and used by MCP-capable hosts over stdio.
- `src/mcp-bridge.ts`: local Unix socket bridge from the MCP server into the
  active VS Code extension process.
- `src/extension.ts`: VS Code extension activation and command registration.
  Starts the MCP bridge on activation and registers commands to start/stop/show
  MCP configuration.
- `packages/tensorfleet-auth/src/vm-manager-client.ts`: shared Node-side VM
  Manager HTTP client. Already owns VM status/start/stop/restart and Gazebo
  world/preset helper calls.
- `packages/tensorfleet-auth/src/regions.ts`: platform-agnostic region
  configuration for backend and VM Manager URLs.
- `src/regions.ts`: VS Code-specific selected-region access, including current
  VM Manager URL.
- `src/auth.ts`: VS Code SecretStorage-backed authentication.
- `MCP_SETUP.md`: existing MCP user setup guide.
- `VSCODE_MCP_INTEGRATION.md`: existing MCP server to VS Code bridge guide.
- `mcp-config-example.json`: existing MCP config example.
- `package.json`: build scripts, package metadata, and `tensorfleet-mcp` binary
  declaration.
- `vite.config.ts`: extension and MCP server bundling configuration.

Vacuum-specific files:

- `panels-standalone/src/vacuum-adapter/adapter.ts`: public adapter shape:
  `snapshot` plus `sendCommand`.
- `panels-standalone/src/vacuum-adapter/state.ts`: normalized vacuum snapshot
  types.
- `panels-standalone/src/vacuum-adapter/capabilities.ts`: normalized vacuum
  capability descriptors.
- `panels-standalone/src/vacuum-adapter/commands.ts`: normalized vacuum command
  types.
- `panels-standalone/src/vacuum-adapter/backends/valetudo/runtimeContract.ts`:
  Valetudo integration runtime JSON contract used by the panel adapter.
- `panels-standalone/src/vacuum-adapter/backends/valetudo/runtimeClient.ts`:
  browser-side client for the VM-managed Valetudo runtime.
- `panels-standalone/src/vacuum-adapter/backends/valetudo/stateMapper.ts`:
  maps runtime snapshots into the normalized vacuum adapter snapshot.
- `panels-standalone/src/vacuum-adapter/backends/valetudo/commandMapper.ts`:
  validates normalized commands against normalized capabilities before
  dispatch.
- `panels-standalone/src/vacuum-adapter/backends/valetudo/capabilityMapper.ts`:
  maps runtime capability diagnostics and command availability into product
  capabilities.
- `panels-standalone/src/vacuum-adapter/backends/turtlebot4-nav2/`: simulation
  and Nav2 backend adapter.
- `panels-standalone/src/components/VacuumControl/VacuumControlPanel.tsx`:
  current operator UI consumer of the vacuum adapter.

External runtime repositories inspected for context:

- `/home/shane/firecracker-vm/tensorfleet-mgr`: Go service that runs inside the
  VM and exposes TensorFleet runtime APIs, including Valetudo.
- `/home/shane/vm-manager`: Go service that manages VMs and proxies
  `/vms/self/tensorfleet/...` requests into the active VM.

## 4. Current MCP Surface

The current MCP server in `src/mcp-server.ts` uses
`@modelcontextprotocol/sdk` with stdio transport:

```text
MCP host
  -> node dist/mcp-server.js
    -> src/mcp-server.ts
```

The production MCP surface currently advertises vacuum product tools:

- `vacuum_get_health`
- `vacuum_get_snapshot`
- `vacuum_get_capabilities`
- `vacuum_get_map_targets`
- `vacuum_get_pose`
- `vacuum_get_map_summary`
- `vacuum_get_mission_state`
- `vacuum_get_navigation_state`
- `vacuum_start_cleaning`
- `vacuum_pause`
- `vacuum_resume`
- `vacuum_stop`
- `vacuum_return_to_dock`
- `vacuum_set_fan_speed`
- `vacuum_set_water_usage`

Current behavior facts:

- Placeholder drone, ROS2, Gazebo, AI inference, QGC mission, install, and
  telemetry tools are no longer advertised as production MCP capabilities.
- Runtime reads route through the selected vacuum backend from MCP config or
  the VS Code bridge. Known backends are `valetudo` and `turtlebot4_nav2`.
- Valetudo reads and commands use typed VM Manager client helpers for the
  `/vms/self/tensorfleet/api/v1/valetudo/*` proxy paths.
- TurtleBot4/Nav2 simulation reads use product-level
  `/vms/self/tensorfleet/api/v1/vacuum/*` proxy paths when the VM runtime
  exposes them. If those routes are not present yet, tools return structured
  `unavailable` results rather than calling raw ROS/Nav2 surfaces.
- Tool results use the shared TensorFleet MCP success/error envelope.
- Command tools fetch current snapshot state before dispatch and refuse
  unsupported, unavailable, stale, unreachable, offline, invalid-state, and
  invalid-setting requests with structured results.
- Simulation write tools remain deferred. Valetudo-only settings such as fan
  speed and water usage return `unsupported` when `turtlebot4_nav2` is selected.

The existing bridge in `src/mcp-bridge.ts` listens on:

```text
/tmp/tensorfleet-mcp-bridge.sock
```

Bridge commands currently include:

- `getRuntimeConfig`
- `openGazeboPanel`
- `openQGCPanel`
- `openAIPanel`
- `openROS2Panel`
- `openAllPanels`
- `showMessage`
- `createTerminal`

The bridge is useful for UI-oriented commands, auth/region discovery, and
extension-only state. It should not be the only path for runtime APIs. Real MCP
tools that read or command the VM should call typed client modules directly.

## 5. MCP Build Output

Current `vite.config.ts` builds the MCP server to:

```text
dist/mcp-server.js
```

`package.json` agrees:

```json
{
  "scripts": {
    "mcp": "node dist/mcp-server.js"
  },
  "bin": {
    "tensorfleet-mcp": "./dist/mcp-server.js"
  }
}
```

The extension MCP start/config helpers and MCP setup docs now use
`dist/mcp-server.js`. Any remaining `out/mcp-server.js` references are legacy
documentation references and should not be copied into host config.

## 6. Existing VM And Runtime HTTP Path

The VM Manager proxy path is already the right transport for extension-facing
runtime APIs:

```text
{VM_MANAGER_URL}/vms/self/tensorfleet/...
```

vm-manager behavior:

- Authenticates the external caller with the user's JWT.
- Finds the user's VM for the selected region.
- Proxies to the VM service at `http://<vm-ip>:9090`.
- Trims `/vms/self/tensorfleet` from the path.
- Injects the internal TensorFleet runtime bearer token.

The VM runtime service `tensorfleet-mgr` exposes:

```text
GET  /api/v1/valetudo/health
GET  /api/v1/valetudo/snapshot
POST /api/v1/valetudo/command
GET  /api/v1/vacuum/health              # simulation product route, when available
GET  /api/v1/vacuum/snapshot            # simulation product route, when available
```

Through VM Manager, those become:

```text
GET  /vms/self/tensorfleet/api/v1/valetudo/health
GET  /vms/self/tensorfleet/api/v1/valetudo/snapshot
POST /vms/self/tensorfleet/api/v1/valetudo/command
GET  /vms/self/tensorfleet/api/v1/vacuum/health
GET  /vms/self/tensorfleet/api/v1/vacuum/snapshot
```

The MCP server should use those proxied paths by default. Direct VM access is a
development path only.

## 7. Existing Vacuum Boundary

The product-facing vacuum boundary is the `vacuum_adapter` contract:

```ts
type VacuumAdapter = {
  snapshot: VacuumAdapterSnapshot;
  sendCommand: (command: VacuumCommand) => Promise<VacuumCommandResult>;
};
```

The architectural rule is:

```text
Product/UI/agent code
  -> normalized snapshot/capabilities/commands
    -> backend adapter/runtime
      -> backend-specific APIs
```

Do not expose raw backend details to agents as the primary interface:

- Do not expose raw Valetudo capability class names as product behavior.
- Do not expose raw Valetudo HTTP routes as tools.
- Do not expose MQTT topics as tools.
- Do not expose raw ROS topics/services as vacuum tools.
- Do not expose VM private IPs as agent-facing inputs.
- Do not let agents call arbitrary runtime URLs.

The MCP server should follow the same normalized boundary as the UI. For the
first slice, it can talk to the Valetudo runtime contract directly, but the
tool names and results should remain product-level and capability-gated.

## 8. Vacuum Runtime Capabilities Today

The Valetudo integration runtime currently has real support for:

- Runtime health.
- Source reachability/staleness.
- Robot identity.
- Robot activity state.
- Connectivity.
- Battery.
- Dock state.
- Cleaning settings.
- Maintenance/consumables.
- Current statistics.
- Attachments.
- Dock components.
- Read-only map metadata.
- Read-only layered map preview.
- Read-only map targets.
- Basic commands:
  - `start_cleaning`
  - `pause`
  - `resume`
  - `stop`
  - `return_to_dock`
  - `set_fan_speed`
  - `set_water_usage`

Deferred or intentionally disabled today:

- Full interactive map support.
- Segment cleaning.
- Zone cleaning.
- Room cleaning.
- Go-to location.
- Clean Area.
- User-created zone drawing.
- Map SSE/live streaming.
- Consumable reset commands.
- Dock action commands.
- Hardware validation beyond current runtime source support.

MCP must preserve those gates. If a capability is detected but not product-ready,
MCP should report it as unsupported or detected-not-ready, not expose a command.

## 9. Recommended MCP Architecture

Split the MCP implementation into layers:

```text
src/mcp-server.ts
  - MCP protocol setup
  - tool/resource registration
  - argument schema
  - result formatting

src/mcp/
  - shared MCP result helpers
  - auth/config resolution
  - tool registration helpers

packages/tensorfleet-auth/src/vm-manager-client.ts
  - shared VM Manager HTTP client
  - reusable by extension and MCP server

domain clients
  - vacuum runtime client
  - VM client
  - Gazebo client
  - future Nav2/telemetry clients
```

The key design rule: MCP tools should be thin, but not dumb. They should:

1. Parse and validate arguments.
2. Resolve auth and VM Manager URL.
3. Read current state/capabilities when needed.
4. Refuse unsupported or unavailable actions before dispatch.
5. Dispatch only product-approved commands.
6. Return structured results.

Avoid putting large domain logic inside `src/mcp-server.ts`. That file should
be a composition/registration layer.

## 10. Auth And Configuration Strategy

MCP runs as a separate Node process. It cannot directly read VS Code
SecretStorage unless the active extension process provides a bridge command.

Support both auth paths:

### Environment path

The MCP host config can pass:

```text
TENSORFLEET_VM_MANAGER_URL
TENSORFLEET_JWT
```

This works without the VS Code extension running, as long as the token and URL
are valid.

### VS Code bridge path

When the extension is active, the MCP server can ask the bridge for runtime
configuration:

```text
getRuntimeConfig
  -> vmManagerUrl
  -> token presence or token
  -> selected vacuum backend
  -> selected region
  -> current VM state
```

Because the bridge runs inside VS Code, it can call `auth.getToken(context)` and
`regions.getVmManagerUrl()`.

Security note: if the bridge returns a token to the MCP server, that token is
available to the local MCP process and the MCP host. That is acceptable for a
local trusted setup, but it should be documented. A future safer bridge can
proxy requests directly instead of returning the token.

Recommended initial precedence:

1. Explicit MCP env vars.
2. VS Code bridge config.
3. Region defaults only for non-secret values.
4. Structured not-authenticated result if no token is available.

## 11. Standard MCP Result Shape

Use one envelope for all TensorFleet tools:

```json
{
  "ok": true,
  "status": "success",
  "message": "Vacuum snapshot fetched.",
  "data": {}
}
```

For non-success:

```json
{
  "ok": false,
  "status": "unavailable",
  "reason": "stale_source",
  "message": "Robot state is stale.",
  "data": null
}
```

Recommended status values:

- `success`
- `unsupported`
- `unavailable`
- `invalid_request`
- `invalid_state`
- `not_authenticated`
- `runtime_offline`
- `source_unreachable`
- `stale_source`
- `backend_error`
- `failed`

Normal business failures should be returned as structured tool results, not as
uncaught exceptions. Reserve thrown MCP errors for malformed tool invocation or
unexpected server failures.

## 12. Naming Rules

Use stable, product-level names.

Prefer underscore names for compatibility with current project style and MCP
hosts:

- `vacuum_get_health`
- `vacuum_get_snapshot`
- `vacuum_get_capabilities`
- `vacuum_get_map_targets`
- `vacuum_get_pose`
- `vacuum_get_map_summary`
- `vacuum_get_mission_state`
- `vacuum_get_navigation_state`
- `vacuum_start_cleaning`
- `vacuum_pause`
- `vacuum_resume`
- `vacuum_stop`
- `vacuum_return_to_dock`
- `vacuum_set_fan_speed`
- `vacuum_set_water_usage`

Avoid raw backend names:

- Do not name tools `valetudo_post_command`.
- Do not name tools `call_runtime_endpoint`.
- Do not name tools after Valetudo class names.
- Do not name tools after ROS topics.

The prefix groups the tools while keeping host compatibility.

## 13. Vacuum MCP Tools

### `vacuum_get_health`

Purpose:

- Fetch runtime and source health.
- Let an agent distinguish runtime offline from source stale/unreachable.

Arguments:

```json
{}
```

Returns:

- Runtime id/version/status.
- Source kind/status/staleness/lastSeenAt.
- Updated timestamp.

Relevant endpoint:

```text
GET /vms/self/tensorfleet/api/v1/valetudo/health
GET /vms/self/tensorfleet/api/v1/vacuum/health
```

### `vacuum_get_snapshot`

Purpose:

- Fetch the current compact product-level vacuum snapshot.
- Main read tool for the vacuum agent.

Arguments:

```json
{
  "include_diagnostics": true,
  "include_raw_diagnostics": false,
  "include_map_preview": false
}
```

Recommended behavior:

- Include common operator state by default.
- Include diagnostics only when requested or when failures occur.
- Do not include huge map preview payloads unless explicitly requested.
- Include map metadata and target inventory by default if compact.
- Include simulation readiness evidence from normalized state: selected backend,
  runtime/source status, source freshness, map and pose availability, mission
  state, navigation state, movement capability availability, and blocking
  reasons.
- Do not expose ROS topic names, Nav2 action names, helper service names,
  Foxglove routes, private VM addresses, or raw backend diagnostics unless the
  caller explicitly asks for raw diagnostics.

Relevant endpoint:

```text
GET /vms/self/tensorfleet/api/v1/valetudo/snapshot
GET /vms/self/tensorfleet/api/v1/vacuum/snapshot
```

### `vacuum_get_capabilities`

Purpose:

- Return current command availability and capability diagnostics.
- Let the agent decide what it can safely do.

Arguments:

```json
{}
```

Returns:

- `commands` for currently advertised MCP write tools.
- `features` for normalized backend capabilities.
- `settings` for normalized setting options.
- `readiness` with movement-readiness evidence and blockers.
- Current availability reasons.

Implementation:

- Can derive from `vacuum_get_snapshot`.
- Should hide raw capability names by default unless diagnostics are requested.
- For the simulation backend, expose normalized capabilities such as `map`,
  `pose`, `navigation_status`, `mission_state`, `start_navigation`,
  `go_to_location`, `start_coverage`, mission action capabilities,
  `mapping_session`, `auto_mapping`, map annotation/room/zone semantics, and
  room/zone cleaning readiness.
- Mark a capability `available: true` only when the normalized descriptor says
  it is currently available. If a capability is supported but not currently
  available, return the normalized reason when present.
- Keep Valetudo-only settings such as `fan_speed` and `water_usage` unsupported
  for simulation unless the normalized simulation adapter explicitly supports
  them.

### `vacuum_get_map_targets`

Purpose:

- Return read-only segment/room/zone target inventory.
- Useful for future planning and explanation.
- Does not enable cleaning commands.

Arguments:

```json
{
  "include_geometry": false
}
```

Recommended behavior:

- Return labels, ids, kind, availability, and geometry summaries by default.
- Return exact geometry only when `include_geometry` is true.
- Make clear that targets are read-only until command support exists.

### `vacuum_get_pose`

Purpose:

- Return normalized robot pose readiness and coordinates when the selected
  backend exposes pose state.
- Keep source details out of the default response.
- If pose is absent, return a structured `available: false` response with
  readiness, timestamp when available, and a normalized reason.

### `vacuum_get_map_summary`

Purpose:

- Return compact map readiness and metadata without large preview/grid payloads.
- Prefer normalized map concepts over backend transport details.

Arguments:

```json
{
  "include_grid": false,
  "include_geometry": false
}
```

Recommended behavior:

- Include compact identity, dimensions/resolution, cell summary, annotation
  counts, target counts, and whether the map is usable for navigation or
  coverage when those normalized fields exist.
- Do not include full occupancy grids or large geometry by default.
- Treat `include_grid` and `include_geometry` as explicit large-payload
  requests.

### `vacuum_get_mission_state`

Purpose:

- Return active and recent mission state through normalized product concepts.
- Surface available mission actions as read-only state in this phase.
- Include active mission id, type, status, phase, progress, available actions,
  terminal result, and compact recent mission summaries when available.
- Do not expose raw Nav2 goal internals.

### `vacuum_get_navigation_state`

Purpose:

- Return normalized navigation status, current target, terminal state, and
  progress.
- Do not expose Nav2 action names or direct goal dispatch as MCP tools.
- Include compact path summary, related mission state, and cancel/pause/resume
  availability when normalized mission actions expose those controls.
- If navigation state is absent but mission state exists, link to the mission
  state rather than inventing navigation details.

### `vacuum_start_cleaning`

Purpose:

- Start a basic whole-home/current-mode cleaning run.

Arguments:

```json
{}
```

Gate:

- Current snapshot must report command availability for `start_cleaning`.

Dispatch:

```json
{
  "command": "start_cleaning"
}
```

### `vacuum_pause`

Purpose:

- Pause active cleaning when available.

Gate:

- Current snapshot must report `pause` available.

Dispatch:

```json
{
  "command": "pause"
}
```

### `vacuum_resume`

Purpose:

- Resume paused cleaning when available.

Gate:

- Current snapshot must report `resume` available.

Dispatch:

```json
{
  "command": "resume"
}
```

### `vacuum_stop`

Purpose:

- Stop active cleaning.

Gate:

- Current snapshot must report `stop` available.

Dispatch:

```json
{
  "command": "stop"
}
```

### `vacuum_return_to_dock`

Purpose:

- Send the robot to dock.

Gate:

- Current snapshot must report `return_to_dock` available.

Dispatch:

```json
{
  "command": "return_to_dock"
}
```

### `vacuum_set_fan_speed`

Purpose:

- Set fan speed preset.

Arguments:

```json
{
  "value": "standard"
}
```

Gate:

- Current snapshot must report `set_fan_speed` or `fan_speed` support.
- Requested value must be present in current `cleaningSettings.fanSpeed.options`
  when options exist.

Dispatch:

```json
{
  "command": "set_fan_speed",
  "params": {
    "value": "standard"
  }
}
```

### `vacuum_set_water_usage`

Purpose:

- Set water usage preset.

Arguments:

```json
{
  "value": "medium"
}
```

Gate:

- Current snapshot must report `set_water_usage` or `water_usage` support.
- Requested value must be present in current
  `cleaningSettings.waterUsage.options` when options exist.

Dispatch:

```json
{
  "command": "set_water_usage",
  "params": {
    "value": "medium"
  }
}
```

## 14. Vacuum MCP Resources

Resources should be read-only and compact by default.

Recommended resources:

- `tensorfleet://vacuum/health`
- `tensorfleet://vacuum/snapshot`
- `tensorfleet://vacuum/capabilities`
- `tensorfleet://vacuum/map-targets`
- `tensorfleet://vacuum/deferred-features`

Resource behavior:

- Resources can call the same underlying client functions as tools.
- Resources should not dispatch commands.
- Resources should avoid large map preview geometry unless a separate explicit
  resource is added.

Possible later map resources:

- `tensorfleet://vacuum/map/metadata`
- `tensorfleet://vacuum/map/preview`
- `tensorfleet://vacuum/map/targets`

Keep map preview separate because dense run data can become large.

## 15. Vacuum MCP Prompts

Prompts should come after tools and resources. Useful prompt candidates:

- `vacuum_diagnose_not_ready`: use health, snapshot, and capabilities to
  explain why cleaning cannot start.
- `vacuum_start_safe_cleaning`: check readiness, settings, and then ask for
  confirmation before starting.
- `vacuum_summarize_status`: produce a concise operator summary.
- `vacuum_map_inventory_review`: summarize targets and deferred map features.

Prompts should never bypass tool gates. They should guide the agent through the
same tools a human could call.

## 16. Command Safety Rules

Apply these rules before every command:

1. Fetch current snapshot or capability state.
2. Check runtime health.
3. Check source reachable and not stale when the command depends on live robot
   state.
4. Check command is supported.
5. Check command is currently available.
6. Validate arguments against current options.
7. Dispatch through the normalized runtime command endpoint.
8. Refresh state after command if practical.
9. Return structured success or structured refusal.

Do not expose these until product workflows are implemented and tested:

- Segment cleaning.
- Zone cleaning.
- Room cleaning.
- Go-to.
- Clean Area.
- Map editing.
- Raw target execution.
- Raw coordinate commands.
- Arbitrary ROS publish/service calls.
- Arbitrary VM shell commands.

## 17. Direct Runtime Contract Vs Adapter Contract

The React panel already maps Valetudo runtime snapshots into
`VacuumAdapterSnapshot`. The MCP server has two possible implementation
choices:

### Option A: call Valetudo runtime contract directly

Pros:

- Fastest first slice.
- Reuses existing VM-managed runtime API.
- Does not require React hooks or panel runtime.

Cons:

- MCP result shape must not leak backend details.
- Some adapter-level normalization may need to be duplicated or moved.

### Option B: extract shared vacuum adapter mapping to a Node-safe package

Pros:

- MCP and UI share the same normalized product contract.
- Lower long-term drift.

Cons:

- More refactoring.
- Current mapper files live under `panels-standalone`.
- Current `useValetudoAdapter` is a React hook and cannot be reused directly.

Recommended path:

1. Start with Option A for the first real MCP slice.
2. Keep tool names/results product-level.
3. Move or share pure mapper/client code later if duplication appears.

## 18. Broader Extension MCP Plan

Once vacuum MCP is working, extend by domain.

Recommended namespaces:

- `vm_*`
- `gazebo_*`
- `nav2_*`
- `telemetry_*`
- `map_*`
- `mission_*`
- `panel_*`
- `project_*`

Possible VM tools:

- `vm_get_status`
- `vm_start`
- `vm_stop`
- `vm_restart`
- `vm_get_info`

Possible Gazebo tools:

- `gazebo_get_status`
- `gazebo_list_presets`
- `gazebo_get_world_selection`
- `gazebo_set_preset`
- `gazebo_reset_world_selection`
- `gazebo_restart`

Possible Nav2 tools:

- `nav2_get_status`
- `nav2_get_map_summary`
- `nav2_get_pose`
- `nav2_get_plan`

Nav2 write tools should be added cautiously:

- `nav2_send_goal`
- `nav2_cancel_goal`

Those require stronger safety checks because they move a simulated or real
robot.

Possible panel/UI tools:

- `panel_open_vacuum_control`
- `panel_open_gazebo`
- `panel_open_nav2`
- `panel_open_raw_messages`
- `panel_open_all`

Panel tools can use the existing VS Code bridge.

Possible project tools:

- `project_get_tensorfleet_config`
- `project_detect_template`
- `project_create_robotic_project`

Creation tools should be separate from runtime operation tools.

## 19. What Not To Build

Avoid generic tools that erase the product boundary:

- `run_shell`
- `call_http`
- `call_vm_endpoint`
- `publish_ros_topic`
- `call_ros_service`
- `call_valetudo_endpoint`
- `mqtt_publish`
- `write_runtime_file`
- `sudo_in_vm`

Those might be useful for internal debugging in a separate admin-only MCP
server, but they should not be part of the normal TensorFleet agent surface.

## 20. Security Model

Initial assumptions:

- MCP server runs locally over stdio.
- The VS Code bridge uses a local Unix socket.
- The VM Manager API still enforces user JWT authentication.
- The VM runtime still enforces its internal bearer token through vm-manager.

Risks:

- MCP hosts can ask tools to execute robot commands.
- If the MCP process receives a JWT, the host process can access it.
- Placeholder tools can mislead agents if they look real.
- Generic bridge commands such as `createTerminal` are more powerful than
  product-level tools.

Mitigations:

- Prefer narrow product tools.
- Return explicit unsupported/unavailable results.
- Keep high-risk commands out until product-ready.
- Add dry-run or confirmation semantics for future movement commands.
- Keep diagnostics separate from feature flags.
- Document token handling clearly.
- Consider splitting operator MCP from admin/debug MCP later.

## 21. Testing Strategy

Use focused validation per layer.

Static/build checks:

```sh
bun run typecheck
bun run build:extension
```

MCP server smoke:

```sh
node dist/mcp-server.js
npx @modelcontextprotocol/inspector node dist/mcp-server.js
```

Vacuum client smoke:

```sh
curl -H "Authorization: Bearer $TENSORFLEET_JWT" \
  "$TENSORFLEET_VM_MANAGER_URL/vms/self/tensorfleet/api/v1/valetudo/snapshot"

curl -H "Authorization: Bearer $TENSORFLEET_JWT" \
  "$TENSORFLEET_VM_MANAGER_URL/vms/self/tensorfleet/api/v1/vacuum/snapshot"
```

Runtime checks when firecracker-vm changes:

```sh
cd /home/shane/firecracker-vm/tensorfleet-mgr
go test ./...
```

For this repo, prefer adding a small MCP/domain-client regression script once
the first tools exist. It should verify:

- tool names are registered
- schemas are valid
- missing auth returns structured `not_authenticated`
- unavailable VM returns structured failure
- command tools check capability availability before dispatch
- setting tools validate options before dispatch

## 22. Documentation Strategy

Keep docs separated by purpose:

- `mcp.md`: durable architecture and implementation reference.
- `progress_report.md`: phase-by-phase implementation notes.
- `MCP_SETUP.md`: user setup instructions.
- `VSCODE_MCP_INTEGRATION.md`: VS Code bridge behavior.

When a phase changes behavior, append to `progress_report.md` using the same
format as the vacuum progress reports:

1. What changed.
2. Product behavior.
3. Still deferred.
4. Validation.

Do not rewrite old progress sections unless correcting a factual error.

## 23. Recommended First Implementation Phase

First implementation phase:

1. Add shared Valetudo runtime client helpers to
   `packages/tensorfleet-auth/src/vm-manager-client.ts` or a nearby client file.
2. Add MCP config resolution for VM Manager URL and JWT.
3. Add bridge command for extension-provided MCP runtime config if needed.
4. Add read tools:
   - `vacuum_get_health`
   - `vacuum_get_snapshot`
   - `vacuum_get_capabilities`
   - `vacuum_get_map_targets`
5. Add basic command tools:
   - `vacuum_start_cleaning`
   - `vacuum_pause`
   - `vacuum_resume`
   - `vacuum_stop`
   - `vacuum_return_to_dock`
   - `vacuum_set_fan_speed`
   - `vacuum_set_water_usage`
6. Add structured result helpers.
7. Reconcile `dist/mcp-server.js` vs `out/mcp-server.js`.
8. Update setup docs and progress report.
9. Run type/build validation.

This phase should not add segment/zone/room/go-to/Clean Area support.

## 24. Open Questions

- Should the bridge return JWTs to the MCP server, or should it proxy
  authenticated requests on behalf of MCP?
- Should the MCP server keep existing placeholder drone tools while vacuum tools
  are added, or should placeholder tools be hidden/renamed as demo tools?
- Should MCP tools return raw Valetudo runtime snapshots for diagnostics, or
  only normalized compact summaries?
- Should command tools require a `confirm` argument for actions that move the
  robot, even if they are currently low-risk basic commands?
- Should vacuum MCP live in this extension package permanently, or should a
  future `@tensorfleet/mcp` package own the server?
- Should the server support HTTP MCP later for remote agent hosts, or remain
  local stdio-only for the extension workflow?

## 25. Summary

TensorFleet already has the pieces needed for a real MCP integration:

- A standalone MCP server.
- A VS Code bridge.
- VM Manager auth/proxy paths.
- VM runtime APIs.
- A normalized vacuum product contract.
- A Valetudo integration runtime with read state and basic commands.

The current MCP server is mostly placeholder scaffolding. The next step is not
to expose more raw internals. The next step is to add a vacuum-first,
capability-gated MCP domain surface that reads real runtime state and dispatches
only currently supported product commands.
