# TensorFleet MCP Quick Start Guide

Get your AI-powered drone development environment running in 5 minutes! 🚀

## What You Get

When configured, you can ask Cursor or Claude things like:

> "Start a Gazebo simulation with the iris drone"

And the **Gazebo panel will automatically open in VS Code** with your simulation ready!

## Installation Steps

### 1. Build the Extension

```bash
cd /Users/hyper/projects/drone/vscode-tensorfleet
bun install
bun run compile
```

### 2. Run in VS Code

**Option A - Development Mode** (recommended for testing):

```bash
code --extensionDevelopmentPath="$(pwd)"
```

**Option B - Install as Extension**:

```bash
bunx vsce package
code --install-extension tensorfleet-drone-0.0.1.vsix
```

### 3. Configure Cursor

Create or edit `~/.cursor/mcp.json`:

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

### 4. Restart Cursor

Close and reopen Cursor to load the MCP server.

### 5. Test It! ✨

Open Cursor and try these commands:

**Basic Tools:**

- "What's the current drone status?"
- "Show me available ROS2 topics"
- "What Gazebo models are available?"

**Panel Opening (with VS Code integration):**

- "Start a Gazebo simulation with the iris model"
- "Launch the ROS2 sensor listener"
- "Run YOLOv8 object detection on the camera"
- "Open the QGroundControl panel"

**Watch the magic happen:**

- AI assistant calls the MCP tool
- VS Code panel opens automatically
- You see visual feedback immediately!

## How It Works

```
You ask Cursor  →  Cursor calls MCP tool  →  MCP opens VS Code panel  →  You see results!
```

1. **You ask:** "Start Gazebo simulation"
2. **Cursor** calls the `start_gazebo_simulation` MCP tool
3. **MCP Server** sends command to VS Code via MCP Bridge
4. **VS Code** opens the Gazebo panel
5. **Cursor** responds: "✓ Gazebo panel opened in VS Code"

## Available Commands

### Drone Operations

- "Get drone status" - Check battery, GPS, mode
- "Get telemetry data" - Position, velocity, sensors

### Simulation

- "Start Gazebo simulation with [model] in [world]"
  - Models: iris, typhoon_h480, plane
  - Worlds: empty, warehouse, outdoor

### ROS2

- "Launch ROS2 [package] [launch_file]"
- "Show available ROS2 topics"

### AI/ML

- "Run [model] inference on [source]"
  - Models: yolov8, detectron2, depth_anything
  - Sources: camera, video_file

### Mission Planning

- "Configure a [type] mission with waypoints"
- "Show saved QGC missions"

## Testing Without AI Assistant

You can test the MCP server directly:

### Test MCP Server Only

```bash
node test-mcp.js
```

### Test VS Code Bridge (requires VS Code running with extension)

```bash
node test-bridge.js
```

## Troubleshooting

### "MCP Server not found"

```bash
# Recompile
bun run compile

# Verify it exists
ls -la out/mcp-server.js
```

### "Bridge not available"

- Make sure VS Code is running
- Check that TensorFleet extension is activated
- Look for "TensorFleet MCP Bridge started" in VS Code output

### "Panels don't open"

- Verify VS Code extension is running (not just compiled)
- Check Developer Console in VS Code for errors
- Run `node test-bridge.js` to verify bridge connectivity

## Full Documentation

- **[README.md](./README.md)** - Extension overview and features
- **[MCP_SETUP.md](./MCP_SETUP.md)** - Detailed MCP setup for all platforms
- **[VSCODE_MCP_INTEGRATION.md](./VSCODE_MCP_INTEGRATION.md)** - How the VS Code integration works

## Pro Tips

1. **Open VS Code First**: Start VS Code with the extension before using Cursor
2. **Use Full Paths**: Always use absolute paths in MCP config
3. **Check Logs**: VS Code Developer Console shows bridge activity
4. **Multiple Tools**: You can ask for multiple operations at once!

Example:

> "Set up my complete drone workspace: start Gazebo with iris, launch ROS2, and open AI ops"

All three panels will open automatically! 🎉

---

Happy drone development! 🚁✨
