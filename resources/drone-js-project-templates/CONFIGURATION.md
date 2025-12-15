# Configuration Guide

This document explains how TensorFleet handles configuration, connection management, and proxy setup for drone operations.

## Configuration Sources

TensorFleet loads configuration from multiple sources with the following precedence:

### 1. Environment Variables (Highest Priority)

Environment variables can be set in `.env` files, VS Code tasks, or shell environment:

```bash
# Connection settings
ROSBRIDGE_URL=ws://localhost:9091
TENSORFLEET_BASE_URL=https://api.tensorfleet.com
TENSORFLEET_NODE_ID=my-drone-vm
TENSORFLEET_JWT=your-jwt-token

# Flight settings
SETPOINT_FRAME_ID=map
ROSBRIDGE_PORT=9091
```

### 2. YAML Configuration File

`config/drone_config.yaml` provides workspace-specific defaults:

```yaml
network:
  vm_ip: ""              # Auto-filled when VM is running
  rosbridge_url: ""      # Auto-filled from environment
  rosbridge_port: 9091
  setpoint_frame: "map"

simulation:
  world: "empty"
  headless: false
  enable_gui: true

flight:
  max_altitude: 100.0
  max_speed: 10.0
```

### 3. TensorFleet Metadata

`.tensorfleet` file contains VM-specific metadata:

```json
{
  "env": {
    "baseUrl": "https://api.tensorfleet.com",
    "nodeId": "drone-vm-123",
    "rosbridgeUrl": "ws://10.0.0.1:9091",
    "frameId": "map"
  }
}
```

### 4. Hardcoded Defaults (Lowest Priority)

Safe fallback values when no configuration is provided.

## Connection Methods

TensorFleet supports two connection methods to ROS Bridge:

### Direct Connection

Connect directly to ROS Bridge WebSocket server:

- **URL**: `ws://localhost:9091` (default)
- **Use Case**: Local development, same machine
- **Configuration**: Set `ROSBRIDGE_URL` or use auto-detection

### Proxy Connection (TensorFleet Cloud)

Route traffic through TensorFleet VM Manager proxy:

- **Use Case**: Remote VMs, cloud deployment
- **Requirements**: `TENSORFLEET_BASE_URL`, `TENSORFLEET_NODE_ID`, `TENSORFLEET_JWT`
- **Security**: JWT authentication required

## Proxy Architecture

### VM Manager Proxy

The TensorFleet VM Manager provides a WebSocket proxy that:

1. **Authentication**: Validates JWT tokens
2. **Routing**: Routes traffic to specific VM instances
3. **Security**: Isolates VM network access
4. **Monitoring**: Tracks connection health

### Proxy URL Construction

Proxy URLs are automatically derived from base URLs:

```javascript
// Input: https://api.tensorfleet.com
// Output: wss://api.tensorfleet.com/ws

function toProxyWebSocketUrl(baseUrl) {
  // Convert HTTP(S) to WS(S) and add /ws path
  return baseUrl.replace(/^http/, 'ws') + '/ws';
}
```

### Authentication Handshake

Proxy connections perform a login handshake:

```javascript
// Client sends login message
{
  type: "login",
  token: "jwt-token-here",
  nodeId: "target-vm-id",
  targetPort: 9091
}

// Server responds with success/failure
{
  type: "loginResponse",
  success: true
}
```

## Configuration Resolution

The `getTensorfleetSettings()` function resolves configuration in order:

```javascript
function getTensorfleetSettings() {
  // 1. Load sources (.env, YAML, .tensorfleet)
  const sources = loadConfigSources();

  // 2. Resolve connection method
  const useProxy = determineConnectionMethod(sources);

  // 3. Build final configuration object
  return {
    rosbridgeUrl: resolveRosbridgeUrl(sources),
    useProxy,
    proxyUrl: useProxy ? resolveProxyUrl(sources) : null,
    nodeId: sources.nodeId,
    token: sources.token,
    // ... other settings
  };
}
```

## Environment Variables Reference

### Connection Settings

| Variable | Description | Default |
|----------|-------------|---------|
| `ROSBRIDGE_URL` | Direct WebSocket URL to ROS Bridge | `ws://127.0.0.1:9091` |
| `TENSORFLEET_BASE_URL` | VM Manager base URL | - |
| `TENSORFLEET_VM_MANAGER_URL` | Alternative VM Manager URL | - |
| `TENSORFLEET_NODE_ID` | Target VM identifier | - |
| `TENSORFLEET_JWT` | JWT authentication token | - |
| `TENSORFLEET_PROXY_URL` | Explicit proxy WebSocket URL | Auto-derived |

### ROS Settings

| Variable | Description | Default |
|----------|-------------|---------|
| `ROSBRIDGE_PORT` | ROS Bridge port number | `9091` |
| `SETPOINT_FRAME_ID` | Coordinate frame for setpoints | `map` |

## YAML Configuration Reference

### Network Section

```yaml
network:
  vm_ip: ""              # VM IP address (auto-filled)
  rosbridge_url: ""      # ROS Bridge URL (auto-filled)
  rosbridge_port: 9091   # ROS Bridge port
  setpoint_frame: "map"  # Coordinate frame
```

### Simulation Section

```yaml
simulation:
  world: "empty"         # World file: empty, warehouse, outdoor
  headless: false        # Run without GUI
  enable_gui: true       # Enable Gazebo GUI
```

### Flight Section

```yaml
flight:
  max_altitude: 100.0    # Maximum allowed altitude (m)
  max_speed: 10.0        # Maximum speed (m/s)
  waypoint_hold_ms: 5000 # Hold time at waypoints (ms)
```

## Troubleshooting Configuration

### Common Issues

1. **"Could not load config at config/drone_config.yaml"**
   - Ensure the file exists and has valid YAML syntax
   - Check file permissions

2. **"Missing proxyUrl for VM Manager WebSocket proxy"**
   - Verify `TENSORFLEET_BASE_URL` is set
   - Check that URL is reachable

3. **"VM Manager proxy login failed"**
   - Verify `TENSORFLEET_JWT` token is valid
   - Check `TENSORFLEET_NODE_ID` matches target VM
   - Ensure VM is running and accessible

4. **Connection timeouts**
   - Check firewall settings
   - Verify ROS Bridge is running on target VM
   - Test network connectivity

### Debug Configuration

Enable debug logging to troubleshoot configuration issues:

```bash
DEBUG=tensorfleet:* bun run your-script.js
```

Or check resolved settings programmatically:

```javascript
const { getTensorfleetSettings } = require('./src/lib/tensorfleet_config');
console.log('Resolved config:', getTensorfleetSettings());
```

## VS Code Integration

The TensorFleet VS Code extension automatically:

- Creates `.tensorfleet` metadata files
- Populates `.env` files with connection details
- Updates `config/drone_config.yaml` with VM information
- Manages JWT tokens for authentication

## Security Considerations

- **JWT Tokens**: Store securely, never commit to version control
- **Proxy URLs**: Use WSS (secure WebSocket) in production
- **Network Access**: Limit VM access to authorized networks
- **Token Rotation**: Regularly rotate authentication tokens
