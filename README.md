# TensorFleet Drone Suite VS Code Extension

TensorFleet Drone Suite delivers a flexible control room for drone development,
simulation, and AI operations. The extension surfaces multiple webview-based
dashboards that can be tiled inside VS Code or launched as dedicated windows,
and bundles a simple installer for shipping external tooling such as ROS 2.

## Features

- QGroundControl view with launch button for a dedicated workspace window.
- Gazebo simulation dashboard for world visualization and sensor overlays.
- AI Ops view for running TensorFleet AI models against live or recorded feeds.
- ROS 2 & Stable Baselines lab with tabbed terminals for middleware and RL
  workflows.
- Tooling panel with a guided installer that copies bundled binaries (e.g.,
  ROS 2) to a user-selected directory.
- **MCP Server**: Expose TensorFleet drone tools to AI assistants like Cursor, Claude, an OpenAI Codex via the Model Context Protocol.

Each dashboard is implemented as a webview view so panes can be arranged in the
side bar or panel area. Commands exposed in the command palette open equivalent
content in standalone webview panels for focused workflows.

## Getting Started

1. Install dependencies with Bun:

   ```bash
   bun install
   ```

2. Compile the extension:

   ```bash
   bun run compile
   ```

   or just continually build the code

   ```bash
      bun watch
   ```

3. Launch a VS Code Extension Development Host:

   ```bash
   code --extensionDevelopmentPath="$(pwd)"
   ```

4. In the development host, open the **TensorFleet** activity bar item to access
   the dashboards, or search for `TensorFleet` commands in the command palette
   (`Cmd/Ctrl + Shift + P`).

   - Use `TensorFleet: Open QGroundControl Workspace`, `TensorFleet: Open Gazebo
Workspace`, or `TensorFleet: Open AI Ops Workspace` to open each dashboard
     as a full-sized webview panel in the main editor area.
   - Run `TensorFleet: Open All Dashboards in Main Area` (or press the **Open
     All Dashboards** button in any TensorFleet view) to arrange all four
     dashboards—including the ROS 2 & Stable Baselines lab—in a quad layout.

## Bundled Tool Installer

The command `TensorFleet: Install Bundled Tools` prompts for a destination
folder and copies all files from `resources/tools` into a `tensorfleet-tools`
directory at the chosen location. Replace the placeholder files in
`resources/tools/` with your actual ROS 2 archives and supporting binaries
before packaging the extension.

## Packaging

To produce a `.vsix` package (requires `vsce`):

```bash
bunx vsce package
```

The resulting package can be installed in VS Code via the Extensions view
("Install from VSIX...").

## MCP Server Integration

TensorFleet includes a **Model Context Protocol (MCP) server** that exposes drone operations, ROS2, Gazebo simulation, and AI ops tools to AI assistants.

> **✨ NEW:** MCP tools automatically open VS Code panels! Ask Cursor "Start a Gazebo simulation" and watch the panel open automatically.

📚 **[Quick Start Guide →](./QUICK_START.md)** - Get set up in 5 minutes!

### Quick Setup

1. **Compile the extension** (if not already done):

   ```bash
   bun run compile
   ```

2. **Get MCP configuration** - Run command:

   ```
   TensorFleet: Show MCP Configuration
   ```

   This displays the JSON config to add to Cursor or Claude Desktop.

3. **Configure your AI assistant**:

   - **Cursor**: Add config to `~/.cursor/mcp.json`
   - **Claude Desktop**: Add config to `~/Library/Application Support/Claude/claude_desktop_config.json`

4. **Restart** Cursor or Claude Desktop to load the TensorFleet MCP server.

### Available MCP Tools

The MCP server provides these tools to AI assistants:

- `get_drone_status` - Get drone battery, GPS, mode, and readiness
- `launch_ros2_environment` - Launch ROS2 packages for drone operations
- `start_gazebo_simulation` - Start Gazebo with world and model
- `run_ai_inference` - Run AI models on drone video feeds
- `configure_qgc_mission` - Configure QGroundControl missions
- `install_tensorfleet_tools` - Install bundled tools
- `get_telemetry_data` - Get real-time telemetry data

### MCP Resources

Contextual resources available to AI assistants:

- `tensorfleet://drone/config` - Drone configuration
- `tensorfleet://ros2/topics` - Active ROS2 topics
- `tensorfleet://gazebo/models` - Available models and worlds
- `tensorfleet://ai/models` - Available AI models
- `tensorfleet://qgc/missions` - Saved missions

### VS Code Integration

When the TensorFleet extension is running in VS Code, MCP tools will **automatically open the corresponding panels**!

For example, when an AI assistant calls `start_gazebo_simulation`, the Gazebo panel opens in VS Code automatically. This creates a seamless experience where your AI assistant controls your workspace.

### Full Documentation

- [MCP_SETUP.md](./MCP_SETUP.md) - Basic MCP setup for Cursor and Claude
- [VSCODE_MCP_INTEGRATION.md](./VSCODE_MCP_INTEGRATION.md) - How MCP tools open VS Code panels automatically
