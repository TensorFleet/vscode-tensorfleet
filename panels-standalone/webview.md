# TensorFleet Webview Panels - Technical Notes

**Architecture:** Standalone React → build → VS Code Extension

See `README.md` for user docs, quick start, and features.

---

## Key Implementation Details

### Direct ROS2 Connection

Standalone panels connect directly to ROS2 via WebSocket (rosbridge or Foxglove):

```typescript
// ros2-bridge.ts
ros2Bridge.connect('rosbridge'); // ws://172.16.0.10:9091
ros2Bridge.subscribe('/camera/image_raw');
ros2Bridge.publish('/cmd_vel', 'geometry_msgs/Twist', twistMsg);
```

Foxglove requires subprotocol `foxglove.websocket.v1`.

### Supported Messages

**Image Panel** - `sensor_msgs/Image` (subscribe)
- Parses rosbridge JSON: `{op: 'publish', msg: {...}}`
- Converts raw data to RGBA → JPEG data URI
- Encodings: rgb8, rgba8, bgr8, bgra8, mono8, mono16

**Teleops Panel** - `geometry_msgs/Twist` (publish)
- Publishes via rosbridge: `{op: 'publish', topic: '/cmd_vel', type: 'geometry_msgs/Twist', msg: {...}}`
- Keyboard-driven (WASD/arrows)
- Configurable rates & speeds

### Auto-Reconnect

3-second delay on disconnect (line 65):
```typescript
this.reconnectTimeout = window.setTimeout(() => {
  this.connect(this.currentMode);
}, 3000);
```

### Vite Multi-Page Config

Each panel = separate HTML entry (vite.config.ts):
```typescript
input: {
  main: resolve(__dirname, 'index.html'),
  image: resolve(__dirname, 'image.html'),
  teleops: resolve(__dirname, 'teleops.html')
}
```

Outputs independent bundles per panel.

---

## VS Code Integration

Build output (`dist/`) gets served by extension:

```typescript
// extension.ts
const htmlPath = path.join(context.extensionPath, 'out/webviews/panels', 'image.html');
```

No React code runs in extension process - just static file serving.

---

## Message Types (Quick Reference)

**Image:** `sensor_msgs/Image` (subscribe)
- Encodings: rgb8, rgba8, bgr8, bgra8, mono8, mono16
- Topic: `/camera/image_raw`

**Teleops:** `geometry_msgs/Twist` (publish)
- Fields: linear.{x,y,z}, angular.{x,y,z}
- Topic: `/cmd_vel`

