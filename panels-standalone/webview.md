# TensorFleet Webview Panels

**Goal:** Build Lichtblick-style visualization panels for ROS2 drones.

**Approach:** Standalone-first development → fast iteration → VS Code integration.

---

## Architecture

```
Standalone React App (develop here)
  ↓ bun run build
VS Code Extension (hosts the build)
```

---

## What's Built

### Panel Selector (`index.html`)
- Lists all available panels
- Connection test button (rosbridge only)
- Clean navigation UI

### Image Panel (`image.html`)
- Camera feed display (CompressedImage)
- Topic selector
- Connection mode selector (rosbridge/Foxglove)
- Brightness/contrast sliders
- Flip horizontal/vertical
- Rotation (0-360°)
- Pause/resume
- Auto-reconnect (3s delay)

---

## File Structure

```
panels-standalone/
├── index.html              # Panel selector
├── image.html              # Image panel entry
├── src/
│   ├── image.tsx           # React entry point
│   ├── ros2-bridge.ts      # WebSocket connection (rosbridge + Foxglove)
│   ├── global.css          # VS Code theme variables
│   └── components/
│       ├── ImagePanel.tsx  # Panel logic (~200 lines)
│       └── ImagePanel.css  # Panel styling
├── package.json
├── vite.config.ts          # Multi-page build config
└── tsconfig.json
```

**Total:** 9 source files.

---

## Dev Workflow

```bash
# 1. Start dev server
cd panels-standalone
bun run dev
# → http://localhost:5173

# 2. Edit code
# Edit src/components/ImagePanel.tsx
# Browser auto-refreshes instantly

# 3. Test with real ROS2
# Connects to rosbridge (9091) or Foxglove (8765)
```

---

## ROS2 Connection

### ROS Bridge ✅ (Ready)
- WebSocket: `ws://172.16.0.2:9091`
- Protocol: rosbridge JSON
- Message handling: Implemented
- Install: `sudo apt install ros-${ROS_DISTRO}-rosbridge-suite`
- Launch: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`

### Foxglove Bridge ⚠️ (Partial)
- WebSocket: `ws://172.16.0.2:8765`
- Protocol: Foxglove WebSocket
- Message handling: TODO (line 152 in ros2-bridge.ts)
- Install: `sudo apt install ros-${ROS_DISTRO}-foxglove-bridge`
- Launch: `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`

**Supported Message Types:**
- `sensor_msgs/CompressedImage` ✅ (JPEG/PNG via rosbridge)
- `sensor_msgs/Image` ❌ (raw formats - not implemented)

---

## Adding New Panels

**3 files per panel:**

1. `new-panel.html` (copy `image.html`, change script src)
2. `src/new-panel.tsx` (React entry point)
3. `src/components/NewPanel.tsx` (your panel logic)

**Update `vite.config.ts` input:**
```typescript
input: {
  main: resolve(__dirname, 'index.html'),
  image: resolve(__dirname, 'image.html'),
  'new-panel': resolve(__dirname, 'new-panel.html')  // Add this
}
```

**Update `index.html` panel grid:**
```html
<a href="/new-panel.html" class="panel-card">
  <div class="panel-icon">🎮</div>
  <div class="panel-name">New Panel</div>
  <div class="panel-desc">Description here</div>
</a>
```

Total: 3 new files + 2 edits.

---

## VS Code Integration (Later)

```bash
# 1. Build
cd panels-standalone
bun run build  # → dist/

# 2. Copy to extension
cp -r dist/* ../out/webviews/panels/

# 3. Register in extension.ts
# Add to DRONE_VIEWS or similar array
```

Extension serves static files. No React logic in extension.

---

## Tech Stack

- **React 18** - UI framework
- **TypeScript** - Type safety
- **Vite 5** - Dev server + bundler
- **Canvas API** - Image rendering
- **WebSocket (native)** - ROS2 connection

**Dependencies:** react, react-dom, vite, typescript, @vitejs/plugin-react

Bundle output: `dist/` (optimized for production)

