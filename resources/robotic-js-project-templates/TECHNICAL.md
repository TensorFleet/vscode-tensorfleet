# Technical Reference Guide

This document contains detailed technical information about TensorFleet's internal workings, ROS/MAVROS integration, and low-level implementation details.

## MAVROS Service Calls

### Arming/Disarming Service

The DroneController's `arm()` and `disarm()` methods internally call the MAVROS `/mavros/cmd/arming` service with a `CommandBool` request:

```typescript
// Arming request
{ value: true }

// Disarming request
{ value: false }
```

This service communicates directly with the flight controller (PX4/ArduPilot) to change the arming state. The service returns a `CommandBool_Response` indicating success or failure.

### Other Common Services

- **Set Mode**: `/mavros/set_mode` - Changes flight mode (STABILIZE, GUIDED, OFFBOARD, etc.)
- **Takeoff**: `/mavros/cmd/takeoff` - Commands autonomous takeoff to specified altitude
- **Land**: `/mavros/cmd/land` - Commands autonomous landing
- **Command Long**: `/mavros/cmd/command` - General MAVLink command interface

## ROS Topic Structure

### State Topics

- **`/mavros/state`**: Core vehicle state (armed, connected, mode, guided)
  ```javascript
  {
    connected: boolean,
    armed: boolean,
    mode: string,      // "STABILIZE", "GUIDED", etc.
    guided: boolean
  }
  ```

### Position Topics

- **`/mavros/local_position/pose`**: Local position in ENU coordinates
- **`/mavros/global_position/global`**: GPS coordinates (latitude/longitude)
- **`/mavros/altitude`**: Comprehensive altitude data (AMSL, relative, AGL)

### Sensor Topics

- **`/mavros/battery`**: Battery voltage, current, percentage
- **`/mavros/vfr_hud`**: Flight instruments (airspeed, groundspeed, heading)
- **`/mavros/imu/data`**: Raw IMU accelerometer/gyroscope data

## Automatic State Transitions

### Arming During Takeoff

When `droneController.takeoff()` is called, the flight controller automatically arms the drone if possible. This is implemented as a MAV_CMD_NAV_TAKEOFF command:

```typescript
{
  command: 22,        // MAV_CMD_NAV_TAKEOFF
  param1: 0,          // min_pitch (fixed-wing)
  param4: yaw_deg,    // yaw angle in degrees
  param5: lat_deg,    // latitude
  param6: lon_deg,    // longitude
  param7: alt_meters, // altitude in meters
}
```

### Disarming After Landing

After successful landing detection, PX4 automatically disarms after a 2-3 second delay. This is controlled by the `COM_DISARM_LAND` parameter.

## Safety Mechanisms

### Airborne Disarm Prevention

The flight controller firmware includes hardcoded safety checks that prevent disarming while airborne:

- **Altitude Check**: Must be below minimum altitude threshold
- **Ground Contact**: Landing gear sensors (if equipped)
- **Landed State**: Internal state estimation confirms vehicle is on ground

### Pre-Arm Checks

Before allowing arming, multiple safety checks are performed:

- **GPS Lock**: Required for navigation modes
- **Battery Level**: Minimum voltage/current checks
- **Sensor Health**: IMU, magnetometer calibration
- **RC Connection**: Remote controller link status
- **Geofence**: Position within allowed boundaries

### Failsafe Behaviors

- **RC Loss**: Return to launch or land immediately
- **Battery Low**: Warning beeps, forced landing
- **GPS Loss**: Position hold or emergency landing
- **Communication Loss**: Autonomous return procedures

## Coordinate Systems

### ENU (East-North-Up)

Local coordinate system used for precise positioning:
- **X-axis**: East (positive) / West (negative)
- **Y-axis**: North (positive) / South (negative)
- **Z-axis**: Up (positive) / Down (negative)
- **Origin**: Takeoff location (home position)

### GPS/WGS84

Global coordinate system:
- **Latitude**: -90° to +90° (North/South)
- **Longitude**: -180° to +180° (East/West)
- **Altitude**: Meters above mean sea level

## Update Frequencies

Different sensors publish at varying rates:
- **GPS/IMU**: 50-400 Hz
- **Position**: 30-100 Hz
- **Battery**: 1 Hz
- **Vehicle State**: 1-10 Hz (changes only)

## ROS Bridge Implementation

### ROSLibBridgeWrapper

The wrapper implements the ROS2BridgeApi interface using ROSLIB:

```javascript
class ROSLibBridgeWrapper {
  // Core methods
  subscribe(topic, handler)    // Subscribe to ROS topic
  publish(topic, type, message) // Publish to ROS topic
  callService(name, request)   // Call ROS service
  waitForConnection()          // Wait for bridge connection
}
```

### Connection Handling

- **Direct Connection**: WebSocket to ROS Bridge server
- **Proxy Connection**: Via TensorFleet proxy for remote VMs
- **Auto-Reconnection**: Handles connection drops gracefully

## State Model Architecture

### DroneStateModel

Aggregates multiple ROS topics into unified state:

```typescript
interface DroneState {
  vehicle: {
    connected: boolean;
    armed: boolean;
    mode: string;
    guided: boolean;
  };
  local: {
    position: { x, y, z };
    velocity: { x, y, z };
  };
  global_position_int: {
    lat: number;  // degrees * 1e7
    lon: number;  // degrees * 1e7
    alt: number;  // mm
  };
  // ... more fields
}
```

### Update Mechanism

- **Topic Subscriptions**: Automatic subscription to relevant topics
- **State Aggregation**: Combines related data into coherent state
- **Change Detection**: Efficiently detects and reports state changes
- **Thread Safety**: Handles concurrent updates from multiple topics

## Error Handling

### Service Call Timeouts

All service calls include configurable timeouts:
- Default: 5000ms for most operations
- Extended: 15000ms for takeoff/landing operations

### Connection Recovery

- **Graceful Degradation**: Continues with cached state during outages
- **Auto-Reconnection**: Attempts to restore connection automatically
- **State Synchronization**: Re-syncs state after reconnection

## Performance Considerations

### Message Filtering

- **Change-Only Updates**: Only publish when values actually change
- **Throttling**: Rate-limit high-frequency updates
- **Buffering**: Accumulate rapid changes before publishing

### Memory Management

- **Topic Cleanup**: Properly unsubscribe when no longer needed
- **Event Listener Limits**: Prevent memory leaks from accumulating listeners
- **State History**: Optional state history with configurable retention
