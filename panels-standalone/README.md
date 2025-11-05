# TensorFleet Panels - Standalone Development

Standalone React app for developing Lichtblick-style visualization panels for ROS2/drone applications.

**Location:** `/home/shane/vscode-tensorfleet/panels-standalone/`

## Quick Start

```bash
# Install dependencies
bun install  # or: npm install

# Run dev server
bun run dev  # or: npm run dev
# Opens http://localhost:5173

# Build for production
bun run build  # or: npm run build
# Outputs to dist/
```

## Features

- 🚀 **Fast iteration**: Edit → instant browser refresh
- 🔧 **Real ROS2 data**: Connects to rosbridge or Foxglove Bridge
- 🎨 **Full DevTools**: React DevTools, console, network inspector
- 📦 **Easy deployment**: Build once, use in browser or VS Code extension

## Available Panels

### ✅ Image Panel
- Display camera feeds (compressed images)
- Brightness/contrast controls
- Flip horizontal/vertical
- Rotation (0-360°)
- Pan/zoom (coming soon)

### 🚧 Coming Soon
- Teleoperation Panel
- Plot Panel
- 3D View Panel
- Map Panel
- Log Panel

## ROS2 Connection

Connects to your ROS2 system via:

**ROS Bridge** (default)
- WebSocket: `ws://172.16.0.2:9091`
- Protocol: rosbridge_suite
- Install: `sudo apt install ros-${ROS_DISTRO}-rosbridge-suite`
- Launch: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`

**Foxglove Bridge**
- WebSocket: `ws://172.16.0.2:8765`
- Protocol: Foxglove WebSocket
- Install: `sudo apt install ros-${ROS_DISTRO}-foxglove-bridge`
- Launch: `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`

## Project Structure

```
tensorfleet-panels-standalone/
├── index.html              # Panel selector (home page)
├── image.html              # Image panel entry
├── src/
│   ├── image.tsx           # Image panel React entry
│   ├── ros2-bridge.ts      # ROS2 WebSocket connection
│   ├── global.css          # Global styles
│   └── components/
│       ├── ImagePanel.tsx  # Image panel component
│       └── ImagePanel.css  # Image panel styles
├── package.json
├── vite.config.ts
└── tsconfig.json
```

## Development Workflow

1. **Start ROS2 bridge** (rosbridge or Foxglove)
2. **Run dev server**: `bun run dev`
3. **Open browser**: http://localhost:5173
4. **Select panel** from home page
5. **Edit components** → instant refresh!

## Adding New Panels

1. Create HTML entry: `new-panel.html`
2. Create React entry: `src/new-panel.tsx`
3. Create component: `src/components/NewPanel.tsx`
4. Add to `vite.config.ts`:
   ```typescript
   input: {
     main: resolve(__dirname, 'index.html'),
     image: resolve(__dirname, 'image.html'),
     'new-panel': resolve(__dirname, 'new-panel.html') // Add this
   }
   ```
5. Add to `index.html` panel grid

## VS Code Extension Integration

To use these panels in a VS Code extension:

```bash
# Build
bun run build

# Copy to extension
cp -r dist/* /path/to/extension/out/webviews/panels/
```

The panels will work exactly the same in VS Code!

## Technologies

- **React 18** - UI framework
- **TypeScript** - Type safety
- **Vite** - Fast dev server & bundler
- **Canvas API** - Image rendering
- **WebSocket** - ROS2 connection

## License

MIT

