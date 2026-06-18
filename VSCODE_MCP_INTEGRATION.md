# VS Code + MCP Integration Guide

TensorFleet MCP runs as a separate stdio process and can optionally ask the active VS Code extension for runtime configuration over a local Unix socket bridge.

## Architecture

```text
MCP host
  -> node dist/mcp-server.js
    -> TensorFleet MCP tools
      -> VM Manager proxy routes

Optional local config fallback:

node dist/mcp-server.js
  -> /tmp/tensorfleet-mcp-bridge.sock
    -> TensorFleet VS Code extension
      -> selected region, selected vacuum backend, stored auth token
```

The bridge is for local extension state and UI convenience. MCP runtime calls still go through VM Manager proxy routes; the MCP server does not call raw Valetudo endpoints, MQTT topics, ROS topics, private VM IPs, shell commands, or arbitrary HTTP URLs.
The vacuum MCP surface is backend-aware and currently distinguishes `valetudo` from the TurtleBot4/Nav2 simulation backend (`turtlebot4_nav2`).

## Components

### MCP Server

`src/mcp-server.ts` registers the real TensorFleet MCP surface. The built entrypoint is:

```text
dist/mcp-server.js
```

### MCP Bridge

`src/mcp-bridge.ts` listens on:

```text
/tmp/tensorfleet-mcp-bridge.sock
```

The server uses bridge command `getRuntimeConfig` only when environment config is missing.

### Extension Helpers

`src/extension.ts` starts the bridge on activation and shows MCP config that points to `dist/mcp-server.js`.

## Bridge Commands

| Command | Purpose |
| --- | --- |
| `getRuntimeConfig` | Return VM Manager URL, token availability/token, selected region, and selected vacuum backend |
| `openGazeboPanel` | Legacy extension UI bridge command |
| `openQGCPanel` | Legacy extension UI bridge command |
| `openAIPanel` | Legacy extension UI bridge command |
| `openROS2Panel` | Legacy extension UI bridge command |
| `openAllPanels` | Legacy extension UI bridge command |
| `showMessage` | Show a VS Code notification |
| `createTerminal` | Create a VS Code terminal |

The production MCP tool list no longer advertises the legacy drone, ROS2, Gazebo, AI inference, QGC mission, install, or telemetry placeholder tools.

## MCP Config

```json
{
  "mcpServers": {
    "tensorfleet": {
      "command": "node",
      "args": ["/path/to/vscode-tensorfleet/dist/mcp-server.js"],
      "env": {
        "TENSORFLEET_VM_MANAGER_URL": "https://eu.vm.tensorfleet.net",
        "TENSORFLEET_JWT": "YOUR_TOKEN",
        "TENSORFLEET_VACUUM_BACKEND": "turtlebot4_nav2"
      }
    }
  }
}
```

If the VS Code extension is active, the `env` object may be empty because the MCP server can request config from the bridge, including the selected vacuum backend. Without a selected backend from env or bridge, vacuum tools return structured `invalid_state` rather than guessing.

## Vacuum Tool Flow

For Valetudo `vacuum_pause`:

1. MCP host calls `vacuum_pause`.
2. MCP server resolves VM Manager URL, token, and selected backend from env or bridge.
3. MCP server fetches `/vms/self/tensorfleet/api/v1/valetudo/snapshot`.
4. MCP server checks runtime/source availability and `pause` command support/current availability.
5. MCP server posts `{ "command": "pause" }` to `/vms/self/tensorfleet/api/v1/valetudo/command`.
6. MCP server returns a structured TensorFleet result envelope.

For simulation reads, MCP uses product-level `/vms/self/tensorfleet/api/v1/vacuum/health` and `/vms/self/tensorfleet/api/v1/vacuum/snapshot` routes when the VM runtime exposes them. Simulation is the current MCP development focus. Simulation state is mapped to normalized vacuum concepts such as pose, compact map summary, active/recent mission state, navigation state, readiness evidence, and capability availability. MCP does not expose ROS topics, Nav2 actions, helper services, Foxglove endpoints, or direct VM addresses as tools.

Simulation readiness reports include selected backend, runtime/source availability, source freshness when timestamped, map and pose availability, localization evidence from pose state, active mission state, navigation state, movement/coverage capability availability, and blocking reasons from normalized snapshot data.

Simulation write tools remain deferred in this phase. Valetudo-only settings such as `vacuum_set_fan_speed` and `vacuum_set_water_usage` return structured `unsupported` when `turtlebot4_nav2` is selected.

## Troubleshooting

### MCP Server Not Found

Run:

```bash
bun run build:extension
```

Verify:

```bash
ls -la dist/mcp-server.js
```

### Bridge Config Is Unavailable

Set explicit env vars in the MCP host config:

```json
{
  "TENSORFLEET_VM_MANAGER_URL": "https://eu.vm.tensorfleet.net",
  "TENSORFLEET_JWT": "YOUR_TOKEN",
  "TENSORFLEET_VACUUM_BACKEND": "turtlebot4_nav2"
}
```

### Authentication Fails

The tools return `status: "not_authenticated"` when no usable token is available or VM Manager rejects the token. Re-authenticate in TensorFleet VS Code or pass a fresh `TENSORFLEET_JWT`.

## Security Notes

- MCP communicates with the host over stdio.
- The bridge socket is local to the machine and used only for TensorFleet extension state.
- Runtime calls use VM Manager product proxy routes.
- The production MCP surface is product-level and vacuum-scoped.
