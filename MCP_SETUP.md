# TensorFleet MCP Server Setup

This extension exposes a **Model Context Protocol (MCP) server** that allows AI assistants like Cursor and Claude to interact with your TensorFleet drone tooling.

## What is MCP?

The Model Context Protocol (MCP) is an open protocol that enables AI assistants to securely access context from your tools, systems, and data sources. The TensorFleet MCP server exposes drone operations, ROS2, Gazebo simulation, and AI ops capabilities.

## 🎯 VS Code Integration

**NEW:** When the TensorFleet extension is running in VS Code, MCP tools will automatically open the corresponding panels!

For example:

- Ask Cursor: **"Start a Gazebo simulation with iris model"**
- The MCP server executes the tool AND opens the Gazebo panel in VS Code
- You see immediate visual feedback in your workspace

See [VSCODE_MCP_INTEGRATION.md](./VSCODE_MCP_INTEGRATION.md) for details on how this works.

## Available Tools

The TensorFleet MCP server provides the following tools:

1. **get_drone_status** - Get current drone status (battery, GPS, mode, readiness)
2. **launch_ros2_environment** - Launch ROS2 packages for drone operations
3. **start_gazebo_simulation** - Start Gazebo simulation with world and model
4. **run_ai_inference** - Run AI model inference on drone video feeds
5. **configure_qgc_mission** - Configure QGroundControl missions
6. **install_tensorfleet_tools** - Install bundled TensorFleet tools
7. **get_telemetry_data** - Get real-time telemetry (position, velocity, sensors)

## Available Resources

The server exposes these resources for context:

- `tensorfleet://drone/config` - Drone configuration and parameters
- `tensorfleet://ros2/topics` - Active ROS2 topics list
- `tensorfleet://gazebo/models` - Available Gazebo models and worlds
- `tensorfleet://ai/models` - Available AI models for analysis
- `tensorfleet://qgc/missions` - Saved QGroundControl missions

---

## Setup for Cursor

### 1. Build the Extension

First, compile the TypeScript code:

```bash
cd /Users/hyper/projects/drone/vscode-tensorfleet
bun install
bun run compile
```

### 2. Configure Cursor Settings

Open Cursor Settings (JSON) and add the MCP server configuration:

**Location:** `~/.cursor/mcp.json` or Cursor Settings → Features → Model Context Protocol

```json
{
  "mcpServers": {
    "tensorfleet-drone": {
      "command": "node",
      "args": [
        "/Users/hyper/projects/drone/vscode-tensorfleet/out/mcp-server.js"
      ],
      "env": {}
    }
  }
}
```

### 3. Restart Cursor

Restart Cursor to load the MCP server. You should now see "TensorFleet" in your MCP servers list.

### 4. Using TensorFleet Tools in Cursor

In Cursor's AI chat, you can now ask questions like:

- "What's the current drone status?"
- "Launch the ROS2 sensor listener"
- "Start a Gazebo simulation with the iris model in the warehouse world"
- "Show me the available AI models"
- "Get telemetry data from the drone"

Cursor will automatically use the TensorFleet MCP tools to answer these questions.

---

## Setup for Claude Desktop

### 1. Build the Extension

```bash
cd /Users/hyper/projects/drone/vscode-tensorfleet
bun install
bun run compile
```

### 2. Configure Claude Desktop

Edit the Claude Desktop configuration file:

**Location:** `~/Library/Application Support/Claude/claude_desktop_config.json` (macOS)

```json
{
  "mcpServers": {
    "tensorfleet-drone": {
      "command": "node",
      "args": [
        "/Users/hyper/projects/drone/vscode-tensorfleet/out/mcp-server.js"
      ]
    }
  }
}
```

### 3. Restart Claude Desktop

Quit and restart Claude Desktop. The TensorFleet MCP server will now be available.

### 4. Using TensorFleet in Claude

You can ask Claude:

- "Can you check the drone battery status?"
- "Start a Gazebo simulation for me"
- "What ROS2 topics are available?"
- "Run YOLOv8 inference on the camera feed"
- "Show me the saved QGC missions"

Claude will use the MCP server to access your TensorFleet tools.

---

## Setup for Other AI Tools (OpenAI, Codex)

For tools that support MCP over stdio, you can run the server directly:

```bash
node /Users/hyper/projects/drone/vscode-tensorfleet/out/mcp-server.js
```

The server communicates via stdin/stdout using the MCP protocol.

---

## Testing the MCP Server

### Test with Direct Execution

You can test the server manually:

```bash
cd /Users/hyper/projects/drone/vscode-tensorfleet
bun run compile
node out/mcp-server.js
```

The server will start and listen on stdio. You should see:

```
TensorFleet MCP Server running on stdio
```

### Test with MCP Inspector

Use the official MCP inspector tool:

```bash
npx @modelcontextprotocol/inspector node out/mcp-server.js
```

This opens a web UI where you can:

- View available tools and resources
- Execute tools with test parameters
- Inspect responses

---

## Extending the MCP Server

The MCP server is defined in `src/mcp-server.ts`. You can add more tools by:

1. **Add a new tool executor** in the `setupTools()` method:

```typescript
this.tools.set("your_tool_name", {
  execute: async (args) => {
    // Your implementation
    return {
      content: [{ type: "text", text: "Result" }],
    };
  },
});
```

2. **Add tool metadata** in the `ListToolsRequestSchema` handler:

```typescript
{
  name: "your_tool_name",
  description: "What your tool does",
  inputSchema: {
    type: "object",
    properties: {
      param1: { type: "string", description: "..." }
    }
  }
}
```

3. **Recompile and restart**:

```bash
bun run compile
# Restart Cursor or Claude Desktop
```

---

## Troubleshooting

### MCP Server Not Appearing

- Ensure the path in config is absolute and correct
- Check that `out/mcp-server.js` exists (run `bun run compile`)
- Verify Node.js is in your PATH
- Check Cursor/Claude logs for errors

### Tools Not Working

- Check the MCP server is running (inspect Cursor/Claude logs)
- Verify the tool name matches exactly
- Ensure input parameters match the schema

### Permission Issues

Make sure the compiled JavaScript is executable:

```bash
chmod +x /Users/hyper/projects/drone/vscode-tensorfleet/out/mcp-server.js
```

---

## Security Notes

- The MCP server runs locally and doesn't expose network ports
- All communication is via stdio (standard input/output)
- The server only has access to what your VS Code extension has access to
- No external network requests are made by default

---

## Learn More

- [Model Context Protocol Documentation](https://modelcontextprotocol.io)
- [MCP SDK for TypeScript](https://github.com/modelcontextprotocol/typescript-sdk)
- [Cursor MCP Documentation](https://docs.cursor.com/context/model-context-protocol)
- [Claude Desktop MCP Guide](https://modelcontextprotocol.io/quickstart)
