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
