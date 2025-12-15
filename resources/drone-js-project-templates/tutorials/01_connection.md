# Tutorial 01: Connection & State Monitoring

## Overview

This tutorial demonstrates fundamental concepts for connecting to a drone via ROS Bridge and monitoring its state in real-time. It covers both direct ROS topic subscriptions for raw data access and the use of a managed state model for simplified state management.

## Learning Objectives

- Establish a WebSocket connection to ROS Bridge using ROSLibBridgeWrapper
- Subscribe directly to MAVROS ROS topics for low-level state monitoring
- Utilize DroneStateModel for aggregated, managed drone state handling
- Compare raw ROS processing vs. managed state utilities
- Monitor key drone parameters: connection status, arming state, flight mode, and guided mode

## Key Concepts

- **ROS Bridge**: Enables remote WebSocket-based communication with ROS systems
- **MAVROS**: MAVLink extendable communication library for ROS drone integration
- **State Aggregation**: Combining multiple ROS topics into a unified state representation

## Prerequisites

- Active TensorFleet VM with ROS Bridge running
- Simulation restarted (as described in [Preparation](00_preparation.md))

## Running the Tutorial

First we will go through the basics of connecting to the VM through javascript.
There are many ways to connect to a drone. The most basic ways are with a ROS Bridge or directly via ROS.

For the start we will use ROS Bridge for basic operations. The advantage of ROS Bridge is that it's suitable for websocket connections which are easier to establish remotely.
We already have utilities that connect you to the drone's environment using an authenticated proxy and the "roslib" javascript library.

Here we have an example of how to connect to a drone and conduct basic telemetry of the states (armed, flight mode, etc.)

To run the 01_connect tutorial, use the following bun command:

```bash
bun run src/tutorials/01_connect.js
```

This will start the tutorial script, which connects to the drone via MAVROS and begins monitoring state changes.

## Expected Output Example

When running the tutorial, you should see output similar to the following (this is the result of a successful connection and state monitoring and only prints again when changes occur):

```
[INFO] Listening to drone state...

[INFO] Showing both raw ROS subscription via wrapper and managed DroneStateModel

[DEBUG] DroneStateModel.connect() called
[DEBUG] Subscribing to topics: [
  {
    topic: "/mavros/global_position/raw/fix",
    type: "sensor_msgs/msg/NavSatFix",
  }, {
    topic: "/mavros/global_position/compass_hdg",
    type: "std_msgs/msg/Float64",
  }, {
    topic: "/mavros/state",
    type: "mavros_msgs/msg/State",
  }, {
    topic: "/mavros/extended_state",
    type: "mavros_msgs/msg/ExtendedState",
  }, {
    topic: "/mavros/battery",
    type: "sensor_msgs/msg/BatteryState",
  }, {
    topic: "/mavros/vfr_hud",
    type: "mavros_msgs/msg/VFR_HUD",
  }, {
    topic: "/mavros/local_position/pose",
    type: "geometry_msgs/msg/PoseStamped",
  }, {
    topic: "/mavros/local_position/velocity_local",
    type: "geometry_msgs/msg/TwistStamped",
  }, {
    topic: "/mavros/imu/data",
    type: "sensor_msgs/msg/Imu",
  }, {
    topic: "/mavros/altitude",
    type: "mavros_msgs/msg/Altitude",
  }, {
    topic: "/mavros/home_position/home",
    type: "mavros_msgs/msg/HomePosition",
  }
]
[DEBUG] DroneStateModel.connect() completed
Press Ctrl+C to exit

=== RAW ROS SUBSCRIPTION RECEIVED CHANGED DATA ===
Drone State:
  Connected: true
  Armed:     false
  Mode:      AUTO.LOITER
  Guided:    true

=== MANAGED DRONE STATE MODEL UPDATE ===
vehicle :
 {
  time_boot_ms: 1765832026405,
  connected: true,
  armed: false,
  guided: true,
  manual_input: false,
  mode: "AUTO.LOITER",
  system_status: 3,
}
status:
 {
  time_boot_ms: 1765832026439,
  connected: true,
  gcs_link: true,
  faults: [],
  armable: true,
  arm_reasons: [],
}
=== MANAGED DRONE STATE MODEL UPDATE ===
vehicle :
 {
  time_boot_ms: 1765832028302,
  connected: true,
  armed: false,
  guided: true,
  manual_input: false,
  mode: "AUTO.LOITER",
  system_status: 3,
}
status:
 {
  time_boot_ms: 1765832028358,
  connected: true,
  gcs_link: true,
  faults: [],
  armable: true,
  arm_reasons: [],
}
=== RAW ROS SUBSCRIPTION RECEIVED CHANGED DATA ===
Drone State:
  Connected: true
  Armed:     true
  Mode:      AUTO.TAKEOFF
  Guided:    true

=== MANAGED DRONE STATE MODEL UPDATE ===
vehicle :
 {
  time_boot_ms: 1765832051711,
  connected: true,
  armed: true,
  guided: true,
  manual_input: false,
  mode: "AUTO.TAKEOFF",
  system_status: 4,
}
status:
 {
  time_boot_ms: 1765832051789,
  connected: true,
  gcs_link: true,
  faults: [],
  armable: true,
  arm_reasons: [],
}
=== MANAGED DRONE STATE MODEL UPDATE ===
vehicle :
 {
  time_boot_ms: 1765832052734,
  connected: true,
  armed: true,
  guided: true,
  manual_input: false,
  mode: "AUTO.TAKEOFF",
  system_status: 4,
}
status:
 {
  time_boot_ms: 1765832052799,
  connected: true,
  gcs_link: true,
  faults: [],
  armable: false,
  arm_reasons: [ "not.on.ground" ],
}
=== MANAGED DRONE STATE MODEL UPDATE ===
vehicle :
 {
  time_boot_ms: 1765832056829,
  connected: true,
  armed: true,
  guided: true,
  manual_input: false,
  mode: "AUTO.TAKEOFF",
  system_status: 4,
}
status:
 {
  time_boot_ms: 1765832057647,
  connected: true,
  gcs_link: true,
  faults: [],
  armable: false,
  arm_reasons: [ "not.on.ground" ],
}
=== RAW ROS SUBSCRIPTION RECEIVED CHANGED DATA ===
Drone State:
  Connected: true
  Armed:     true
  Mode:      AUTO.LOITER
  Guided:    true

=== MANAGED DRONE STATE MODEL UPDATE ===
vehicle :
 {
  time_boot_ms: 1765832057848,
  connected: true,
  armed: true,
  guided: true,
  manual_input: false,
  mode: "AUTO.LOITER",
  system_status: 4,
}
status:
 {
  time_boot_ms: 1765832057849,
  connected: true,
  gcs_link: true,
  faults: [],
  armable: false,
  arm_reasons: [ "not.on.ground" ],
}
```

## How It Works

The tutorial demonstrates two approaches to accessing drone state:

1. **Direct MAVROS Usage**: You can subscribe directly to ROS topics using the MAVROS library. This gives you raw access to individual sensor and state messages, allowing for fine-grained control over what data you receive and how you process it.

2. **Managed State Utility**: The `DroneStateModel` provides an automatically managed state utility that handles subscriptions to multiple ROS topics internally. It aggregates and processes the data into a structured state object, making it easier to work with drone state without manually managing each subscription.

## Update Detection

Updates are detected through ROS topic subscriptions:

- For the raw approach, each topic subscription triggers a callback whenever a new message is published on that topic.
- For the managed approach, the `DroneStateModel` subscribes to all relevant topics and emits updates when the aggregated state changes, reducing the need for individual topic handlers.

This allows real-time monitoring of the drone's connection status, arming state, flight mode, and other critical parameters.

## Code Analysis

### Connection Setup

```javascript
// Create ROS bridge using the wrapper
const bridge = new ROSLibBridgeWrapper();
await bridge.waitForConnection();
```

The `ROSLibBridgeWrapper` handles the complex connection setup including authentication and proxy configuration.

### Raw State Tracking

```javascript
// Subscribe to state topic via wrapper (educational: raw ROS processing)
const unsubscribeRaw = bridge.subscribe(
    { topic: "/mavros/state", type: "mavros_msgs/State" },
    (msg) => {
        // Process raw MAVROS message
        const currentConnected = msg.connected;
        const currentArmed = msg.armed;
        // ... handle state changes
    }
);
```

Direct topic subscription gives you complete control over message processing.

### Managed State Tracking

```javascript
// Create drone state model for comparison (shows utility)
const droneState = new DroneStateModel();
droneState.connect(bridge);

droneState.onStatusUpdate((state) => {
    console.log("=== MANAGED DRONE STATE MODEL UPDATE ===");
    if (state.vehicle) {
        console.log("vehicle :\n", state.vehicle);
    }
    // ... display aggregated state
});
```

The `DroneStateModel` automatically manages subscriptions and provides structured state data.

## Navigation

- **Previous**: [Tutorial 00: Preparation](00_preparation.md)
- **Next**: [Tutorial 02: Telemetry](02_telemetry.md)

---

*Tip: Use the Map panel to trigger state changes (arming, mode changes) while the tutorial runs to see live updates.*
