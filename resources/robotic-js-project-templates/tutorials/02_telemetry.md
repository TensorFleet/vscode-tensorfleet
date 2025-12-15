# Tutorial 02: Comprehensive Telemetry Monitoring

## Overview

This tutorial demonstrates advanced telemetry data collection and processing using MAVROS ROS topics. It showcases both raw ROS topic subscriptions for fine-grained control and the managed DroneStateModel for simplified, aggregated state handling. You'll learn about key telemetry sources, data formats, and real-time monitoring techniques essential for drone applications.

## Learning Objectives

- Subscribe to multiple MAVROS ROS topics for comprehensive telemetry data
- Process raw ROS messages from state, position, GPS, altitude, and battery topics
- Utilize DroneStateModel for unified, managed telemetry aggregation
- Understand MAVROS data structures and update frequencies
- Compare raw vs. managed telemetry processing approaches
- Monitor critical flight parameters in real-time

## Key Concepts

- **MAVROS Telemetry**: MAVLink protocol extension providing drone sensor and state data via ROS
- **ROS Topics**: Publish-subscribe messaging system for real-time data distribution
- **Data Aggregation**: Combining multiple raw topics into a coherent state representation
- **Update Frequencies**: Different sensors publish at varying rates (GPS: 1-10Hz, IMU: 50-400Hz)
- **Coordinate Systems**: Understanding local (ENU) vs global (GPS) positioning

## Prerequisites

- Active TensorFleet VM with MAVROS running
- Simulation restarted (as described in [Preparation](00_preparation.md))
- Understanding of basic ROS Bridge connection from [Tutorial 01](01_connection.md)

## Running the Tutorial

After understanding basic connection and state monitoring from Tutorial 01, the telemetry tutorial builds upon this foundation by demonstrating comprehensive data collection from the drone's sensors and systems.

To run the telemetry tutorial, use the following bun command:

```bash
bun run src/tutorials/02_telemetry.js
```

This will start the telemetry monitoring script, which connects to multiple MAVROS topics and begins collecting comprehensive flight data.

## Expected Output Example

When running the tutorial, you should see output similar to the following (this is the result of successful telemetry collection and displays both raw topic data and managed state model data):

```
[INFO] Connected to ROS Bridge - monitoring comprehensive telemetry...

[INFO] Demonstrating both raw MAVROS topic subscriptions and managed DroneStateModel

Press Ctrl+C to exit

=== COMPREHENSIVE DRONE TELEMETRY ===

RAW MAVROS TOPIC DATA:
----------------------
Vehicle State (/mavros/state):
  Connected: true
  Armed:     false
  Mode:      AUTO.LOITER
  Guided:    true

Local Position (/mavros/local_position/pose) - ENU coordinates:
  East (X):  -0.01 m
  North (Y): -0.05 m
  Up (Z):    0.01 m

GPS Position (/mavros/global_position/raw/fix):
  Latitude:  47.3979707°
  Longitude: 8.5461639°
  Altitude:  47.39 m

Altitude Breakdown (/mavros/altitude):
  Above Mean Sea Level: 0.25 m
  Relative to Home:     0.03 m
  Above Ground Level:   undefined m

Battery Status (/mavros/battery):
  Voltage:    16.20 V
  Current:    1.00 A
  Percentage: 1%

MANAGED STATE MODEL (Aggregated Data):
---------------------------------------
Vehicle State:
  Connected: true
  Armed:     false
  Mode:      AUTO.LOITER
  Guided:    true

Local Position (ENU):
  East:  -0.01 m
  North: -0.05 m
  Up:    0.01 m

Global Position:
  Lat: 47.3979707°
  Lon: 8.5461639°
  Alt: 47.39 m

Altitude:
  AMSL:     0.25 m
  Relative: 0.03 m

Battery:
  Voltage:    16.20 V
  Percentage: 1%
```

## How It Works

The tutorial demonstrates comprehensive telemetry collection through multiple approaches:

1. **Raw MAVROS Topic Subscriptions**: Direct subscriptions to individual ROS topics provide fine-grained access to specific sensor data. This approach gives you complete control over data processing and allows for custom filtering, transformation, or analysis of individual data streams.

2. **Managed State Model Aggregation**: The `DroneStateModel` automatically subscribes to all relevant telemetry topics and combines them into a unified state representation. This simplifies development by handling topic management, data synchronization, and providing a consistent interface for accessing drone telemetry.

## Key MAVROS Topics Covered

- **`/mavros/state`**: Core vehicle state including connection, arming, mode, and guidance status
- **`/mavros/local_position/pose`**: Local position in ENU (East-North-Up) coordinates relative to the home position
- **`/mavros/global_position/raw/fix`**: Global GPS position with latitude, longitude, and altitude
- **`/mavros/altitude`**: Comprehensive altitude information including AMSL, relative, and AGL measurements
- **`/mavros/battery`**: Battery status including voltage, current draw, and charge percentage
- **`/mavros/vfr_hud`**: Virtual flight instrument data including airspeed, groundspeed, heading, throttle, and climb rate

## Understanding Coordinate Systems

The tutorial demonstrates two important coordinate systems used in drone telemetry:

- **ENU (East-North-Up)**: Local coordinate system where X points East, Y points North, and Z points Up. Positions are relative to the home/takeoff location.
- **GPS/WGS84**: Global coordinate system using latitude, longitude, and altitude above mean sea level.

## Data Update Frequencies

Different sensors and systems publish data at varying frequencies:

- GPS data: Typically 1-10 Hz
- IMU/Position: 50-400 Hz
- Battery: 1 Hz
- Vehicle state: 1-10 Hz depending on changes

The managed state model handles these different update rates and provides a unified view of the current drone state.

## Code Analysis

### Connection and Setup

```javascript
// Establish ROS Bridge connection using our wrapper
const bridge = new ROSLibBridgeWrapper();
await bridge.waitForConnection();

// Raw telemetry data storage
let rawTelemetry = {
    state: null,
    pose: null,
    fix: null,
    altitude: null,
    battery: null,
    vfr_hud: null,
};
```

The tutorial sets up both raw data storage and connection to the ROS Bridge.

### Raw Topic Subscriptions

```javascript
// Subscribe to vehicle state (/mavros/state)
rawSubscriptions.push(
    bridge.subscribe(
        { topic: "/mavros/state", type: "mavros_msgs/State" },
        (msg) => {
            rawTelemetry.state = msg;
        }
    )
);
```

Each telemetry source is subscribed to individually, giving direct access to raw MAVROS messages.

### Managed State Model

```javascript
// Initialize managed state model for comparison
const droneState = new DroneStateModel();
droneState.connect(bridge);

// Variables to track managed state for display
let managedState = {};

droneState.onUpdate((state) => {
    managedState = state;
});
```

The `DroneStateModel` handles all subscriptions internally and provides aggregated state updates.

### Display Logic

The tutorial displays both raw and managed data side-by-side, updating every second to show real-time telemetry changes.

## Navigation

- **Previous**: [Tutorial 01: Connection](01_connection.md)
- **Next**: [Tutorial 03: Arm/Disarm](03_arm.md)

---

*This comprehensive telemetry monitoring is essential for building robust drone applications that require awareness of position, status, and environmental conditions.*
