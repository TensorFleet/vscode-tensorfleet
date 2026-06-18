# TensorFleet MCP Server Setup

This extension exposes a Model Context Protocol (MCP) server for product-level TensorFleet tools. The current production MCP surface is the vacuum domain only.

The server is built to:

- read vacuum runtime state through VM Manager proxy routes
- return structured TensorFleet MCP result envelopes
- dispatch only normalized, currently supported vacuum commands
- avoid raw robot, Valetudo, MQTT, ROS, shell, VM private IP, or generic proxy tools
- route vacuum tools by the selected backend (`turtlebot4_nav2` or `valetudo`)

## Build

```bash
cd /path/to/vscode-tensorfleet
bun install
bun run build:extension
```

The MCP entrypoint is:

```text
dist/mcp-server.js
```

## Runtime Config

The MCP server resolves runtime config in this order:

1. Environment variables:
   - `TENSORFLEET_VM_MANAGER_URL`
   - `TENSORFLEET_JWT`
   - `TENSORFLEET_VACUUM_BACKEND` (`turtlebot4_nav2`, `simulation`, or `valetudo`)
2. VS Code MCP bridge command `getRuntimeConfig`, when the TensorFleet extension is active.

If no token is available, tools return a structured `not_authenticated` result instead of throwing.
If no vacuum backend is known, tools return a structured `invalid_state` result instead of guessing.

## Available Tools

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

Backend-neutral read tools use the selected backend. Valetudo reads use `/vms/self/tensorfleet/api/v1/valetudo/*`; simulation reads use product-level `/vms/self/tensorfleet/api/v1/vacuum/*` routes when the VM runtime exposes them. If the simulation snapshot route is not present yet, MCP returns structured `unavailable` rather than raw ROS/Nav2 data.

`vacuum_set_fan_speed` and `vacuum_set_water_usage` are currently Valetudo-limited and return `unsupported` for the TurtleBot4/Nav2 simulation backend. Basic command tools remain gated by the selected backend's normalized capabilities and current availability.

Deferred vacuum features such as segment cleaning, zone cleaning, room cleaning, go-to, map editing, live map streaming, consumable resets, dock actions, and hardware validation are not exposed as MCP commands.

## Cursor Config

Edit `~/.cursor/mcp.json`:

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

When the TensorFleet VS Code extension is active, `env` may omit the token and VM Manager URL because the server can ask the local MCP bridge for runtime config.

## Claude Desktop Config

Edit `~/Library/Application Support/Claude/claude_desktop_config.json` on macOS:

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

## Testing

```bash
bun run typecheck
bun run build:extension
bun run test:mcp-vacuum
node dist/mcp-server.js
npx @modelcontextprotocol/inspector node dist/mcp-server.js
```

Optional live vacuum runtime smoke:

```bash
curl -H "Authorization: Bearer $TENSORFLEET_JWT" \
  "$TENSORFLEET_VM_MANAGER_URL/vms/self/tensorfleet/api/v1/valetudo/snapshot"

curl -H "Authorization: Bearer $TENSORFLEET_JWT" \
  "$TENSORFLEET_VM_MANAGER_URL/vms/self/tensorfleet/api/v1/vacuum/snapshot"
```

## Result Shape

Successful tools return JSON text shaped like:

```json
{
  "ok": true,
  "status": "success",
  "message": "Vacuum snapshot fetched.",
  "data": {}
}
```

Failures also return JSON text:

```json
{
  "ok": false,
  "status": "not_authenticated",
  "reason": "missing_token",
  "message": "TensorFleet authentication is not configured for MCP.",
  "data": null
}
```

## Troubleshooting

- Use `dist/mcp-server.js`, not `out/mcp-server.js`.
- Run `bun run build:extension` if `dist/mcp-server.js` is missing.
- Set `TENSORFLEET_VM_MANAGER_URL`, `TENSORFLEET_JWT`, and `TENSORFLEET_VACUUM_BACKEND` when using MCP without the VS Code extension bridge.
- Check that the TensorFleet extension is active if relying on bridge-provided runtime config.
