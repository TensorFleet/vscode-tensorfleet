# TensorFleet Drone Suite VS Code Extension

TensorFleet Drone Suite delivers a flexible control room for drone development,
simulation, and AI operations. The extension surfaces multiple webview-based
dashboards that can be tiled inside VS Code or launched as dedicated windows,
and bundles a simple installer for shipping external tooling such as ROS 2.

## Features

- **🚀 Project Scaffolding**: Create new drone projects with pre-configured templates, example code, and best practices in one click.
- **📊 Smart Status Bar**: Auto-appearing ROS version selector and drone status monitor in the lower right corner when TensorFleet projects are open.
- QGroundControl view with launch button for a dedicated workspace window.
- Gazebo simulation dashboard for world visualization and sensor overlays.
- AI Ops view for running TensorFleet AI models against live or recorded feeds.
- ROS 2 & Stable Baselines lab with tabbed terminals for middleware and RL
  workflows.
- Tooling panel with a guided installer that copies bundled binaries (e.g.,
  ROS 2) to a user-selected directory.
- **MCP Server**: Expose TensorFleet drone tools to AI assistants like Cursor, Claude, an OpenAI Codex via the Model Context Protocol.
- **🖥️ VM Manager Console**: Start/stop the Go-based `vm-manager` service, monitor VM records, and submit stop requests directly from VS Code.

Each dashboard is implemented as a webview view so panes can be arranged in the
side bar or panel area. Commands exposed in the command palette open equivalent
content in standalone webview panels for focused workflows.

## Getting Started

### For Extension Development

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

### For Drone Development (Using the Extension)

1. **Create a New Project**:

   - Open the TensorFleet Tools panel (bottom panel)
   - Click the **🚀 New Project** button
   - Enter a project name and select a location
   - Get a complete project with templates, examples, and configuration!

   📚 **[Project Scaffolding Guide →](./PROJECT_SCAFFOLDING.md)** - Learn about project templates

2. **Configure Your Environment**: Edit `config/drone_config.yaml` in your new project

3. **Launch Tools**: Use TensorFleet panels to start Gazebo, ROS 2, and AI operations

4. **Start Coding**: Modify `src/main.py` to implement your drone control logic

## Bundled Tool Installer

The command `TensorFleet: Install Bundled Tools` prompts for a destination
folder and copies all files from `resources/tools` into a `tensorfleet-tools`
directory at the chosen location. Replace the placeholder files in
`resources/tools/` with your actual ROS 2 archives and supporting binaries
before packaging the extension.

## VM Manager Integration

TensorFleet ships with a dedicated VM Manager console that talks to the Go service living in `../vm-manager`.

1. **Configure locations (optional)**  
   - `tensorfleet.vmManager.repoPath`: path to the Go repo (defaults to `../vm-manager` relative to the extension/workspace). This is only required when running the Go binary locally for development.  
   - `tensorfleet.vmManager.apiBaseUrl`: HTTP endpoint used by the webview (defaults to `http://localhost:8080`).

2. **Select an environment & log in**  
   - Define one or more entries within `tensorfleet.vmManager.environments` (local, staging, production, etc.) and point `tensorfleet.vmManager.activeEnvironment` at the default target.  
   - Use the command palette (`TensorFleet: Select VM Manager Environment`) or the **Environment & Login** card (now surfaced first in the panel) to switch environments without editing JSON.  
   - Authenticate from that same card or run `TensorFleet: Log In to VM Manager` / `TensorFleet: Log Out of VM Manager`; tokens are stored in VS Code Secret Storage per-environment, the associated user UUID is inferred automatically, and everything is cleared on logout or after an expired session (401).  

3. **Start the service (developers only)**  
   Most users can stop here—remote environments work as soon as you are logged in. If you do need to run the Go process locally, run `TensorFleet: Start VM Manager Service`. The extension spawns `go run ./cmd/vm-manager` with `LOCAL_DEV=1`, streams logs to the *TensorFleet VM Manager* output channel, and exposes convenience commands to stop the service or reopen the logs later.

4. **Open the console**  
   Use `TensorFleet: Open VM Manager Console` (or click the VM Manager tile in the TensorFleet view) to launch a full dashboard. It surfaces:
   - Service status + API URL indicator
   - One-click refresh of VM records from `/dev/vms`
   - Inline buttons to mark VMs as stopping
   - A guided form to create development VMs (auto-fills the UUID from your login token, with an optional generator for overrides)

5. **Stop or inspect**  
   Commands `TensorFleet: Stop VM Manager Service` and `TensorFleet: Show VM Manager Logs` remain available in the Command Palette. Any open VM Manager panels stay in sync when the process state changes thanks to live status broadcasts.

### How the VM Manager workflow fits together

1. **Pick an environment** – Each environment entry describes an API surface (URL + login endpoint) and an optional default email hint. Use `TensorFleet: Select VM Manager Environment` when you want to jump between Local, Staging, or Production without editing JSON.
2. **Authenticate once per environment** – The login form (or `TensorFleet: Log In to VM Manager`) captures email + password, exchanges them for a bearer token, and saves that token in VS Code Secret Storage alongside your inferred user UUID. Tokens and UUIDs are scoped per-environment so you can simultaneously stay logged into staging and production with different identities.
3. **Manage cloud VMs immediately** – After logging in you can refresh, stop, or create VMs against the selected environment—even if you never run the Go binary locally. When you open the VM creation form, the UUID that was extracted from your login is auto-filled so new VMs are attributed to the right user record. You can override the UUID (for service accounts or mock users) and the helper text will confirm whether the field is synced to your login or a manual value.
4. **Optionally run the Go service locally** – Developers who have the `vm-manager` repo cloned can point `tensorfleet.vmManager.repoPath` at it and use the Start/Stop buttons (or the matching commands) to run `go run ./cmd/vm-manager` under VS Code. The panel shows whether a local repo was found, which ports the API is listening on, and streams logs to the *TensorFleet VM Manager* output channel.

### Why email and UUID are both tracked

- **Email** – Required to authenticate. It is stored (per environment) so the login prompt can be pre-populated the next time you log in.
- **UUID** – Represents the authenticated user ID returned by the API response or embedded in the JWT. The VM creation form uses it as the default `user_id` owner for new microVMs, and the UI surfaces it next to the email so you can confirm which account is active. Clearing tokens or logging out wipes the cached UUID alongside the email.

### Environments at a glance

| Environment | Typical use case | Default config | Differences |
|-------------|------------------|----------------|-------------|
| `local`     | Iterate on the Go service with a repo on disk | `apiBaseUrl`: `http://localhost:8080`, `authEndpoint`: `http://localhost:8080/login` | Start/Stop buttons become available when a repo is detected, and the login form usually uses a dev/test email. |
| `staging`   | Validate new features against shared infra | Example URL: `https://staging.vm.tensorfleet.dev` | Usually points to a staging Supabase/REST stack; often uses seeded staging accounts and may enforce stricter RBAC than local. |
| `production`| Operate real customer workloads | Example URL: `https://vm.tensorfleet.com` | Same UI, but everything is read/write against production services; you typically keep the service controls disabled and just use the console as a remote client. |

You can add as many custom environments as needed (QA, demo, customer-specific endpoints, etc.). Every environment entry can declare its own URLs, default email hint, and even a completely different login endpoint (for example, a Supabase REST hook vs. an OAuth proxy).

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

- [QUICK_START.md](./QUICK_START.md) - Get started with TensorFleet in 5 minutes
- [PROJECT_SCAFFOLDING.md](./PROJECT_SCAFFOLDING.md) - Create new drone projects with templates
- [STATUS_BAR_FEATURE.md](./STATUS_BAR_FEATURE.md) - ROS version and drone status in the status bar
- [MCP_SETUP.md](./MCP_SETUP.md) - Basic MCP setup for Cursor and Claude
- [VSCODE_MCP_INTEGRATION.md](./VSCODE_MCP_INTEGRATION.md) - How MCP tools open VS Code panels automatically
