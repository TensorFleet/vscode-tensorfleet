# Tutorial 03: Arm / Disarm the Drone

## Overview

This tutorial demonstrates fundamental drone safety and control operations using MAVROS services. It showcases the use of our ROSLibBridgeWrapper and DroneController from tensorfleet-util to safely arm and disarm the drone, with proper state monitoring and error handling.

## Learning Objectives

- Use ROSLibBridgeWrapper for ROS Bridge communication
- Utilize DroneController for high-level arm/disarm operations
- Monitor drone state changes during arming/disarming
- Understand MAVROS service calls and their internal workings
- Learn about automatic arming/disarming behavior during flight operations
- Understand safety mechanisms that prevent unsafe disarming

## Key Concepts

- **MAVROS Services**: ROS service calls for drone control commands
- **Arming State**: Critical safety state that enables motor control
- **Service Calls**: Synchronous communication pattern vs. topic publishing
- **Safety Mechanisms**: Built-in protections against unsafe operations
- **Automatic Transitions**: How takeoff and landing affect arming state

## Prerequisites

- Active TensorFleet VM with MAVROS running
- Simulation restarted (as described in [Preparation](00_preparation.md))
- Understanding of connection and telemetry from [Tutorial 01](01_connection.md) and [Tutorial 02](02_telemetry.md)

## Running the Tutorial

After learning about connection and telemetry monitoring, this tutorial introduces drone control operations. The arm/disarm functionality is fundamental to safe drone operation and must be mastered before attempting flight control.

To run the arm/disarm tutorial, use the following bun command:

```bash
bun run src/tutorials/03_arm.js
```

This will connect to the drone, check its current arming state, and toggle it (arming if disarmed, disarming if armed).

## Expected Output Example

When running the tutorial, you should see output similar to the following:

```
[INFO] Connected to ROS Bridge - initializing drone control...

[INFO] Waiting for drone state...
[INFO] Drone connected. Current state: armed=false, mode=STABILIZE

[INFO] Drone is currently disarmed. Arming...

[SUCCESS] Drone arm command sent successfully!
Waiting for arm confirmation...
[SUCCESS] Drone is now armed!

[INFO] Arming/Disarming operation completed successfully!

[EXIT] Disconnected from drone state monitoring.
```

## Understanding Arming

Arming a drone enables its motors and prepares it for flight. This is a critical safety mechanism that prevents accidental motor activation. The drone will only respond to flight commands when armed.

## What You'll Learn

- **Safety First**: Why arming/disarming is essential for drone safety
- **State Monitoring**: How to verify arming state changes
- **Automatic Behavior**: When drones arm/disarm automatically
- **Safety Mechanisms**: Built-in protections against unsafe operations

## Key Safety Concepts

### Manual vs Automatic Arming

**Manual Arming**: You explicitly call `droneController.arm()` before flight operations.

**Automatic Arming**: Some commands (like takeoff) will arm the drone automatically if needed.

### When Drones Auto-Disarm

Drones typically disarm automatically after landing to prevent:
- Accidental motor activation on the ground
- Battery drain from idle motors
- Safety risks during handling

## Safety Mechanisms

Flight controllers include multiple safety features:

- **Airborne Protection**: Cannot disarm while flying (prevents crashes)
- **Pre-Arm Checks**: Validates GPS, battery, sensors before arming
- **Emergency Protocols**: Automatic failsafe responses

## Code Walkthrough

### Setup Phase

```javascript
// Connect to ROS Bridge
const bridge = new ROSLibBridgeWrapper();
await bridge.waitForConnection();

// Monitor drone state
const droneState = new DroneStateModel();
droneState.connect(bridge);

// Control the drone
const droneController = new DroneController(droneState, bridge);
```

### Arming Operation

```javascript
// Check current state
const state = droneState.getState();
if (!state.vehicle?.armed) {
    // Arm the drone
    await droneController.arm();

    // Wait for confirmation using low-latency listener
    await new Promise((resolve, reject) => {
        const timeout = setTimeout(() => reject(new Error("Timeout")), 5000);

        // Listen for vehicle state changes (armed, mode, etc.)
        const unsubscribe = droneState.onSectionChange('vehicle', (oldVal, newVal) => {
            if (newVal.armed && !oldVal.armed) {
                clearTimeout(timeout);
                unsubscribe();
                resolve();
            }
        });
    });
}
```

### Low-Latency State Monitoring

Instead of polling every 100ms, this tutorial uses the new `onSectionChange()` API:

- **`onSectionChange(section, listener)`**: Registers a listener for changes in a specific state section
- **Immediate notifications**: Fires when ROS messages update state, not on a timer
- **Selective listening**: Only listens to relevant state sections (excludes time updates)
- **Automatic cleanup**: Listeners are unsubscribed when no longer needed

The code waits for state confirmation because:
- Commands are sent asynchronously
- Network delays may occur
- Verifies the flight controller accepted the command
- Provides immediate feedback when state actually changes

## Expected Behavior

- **Arming**: Takes 1-2 seconds, may include pre-arm checks
- **Disarming**: Usually immediate (unless airborne)
- **Auto-Disarm**: 2-3 seconds after landing detection

## Common Issues

- **Arming Fails**: Check battery, GPS, sensor calibration
- **Cannot Disarm**: Drone may still be airborne (safety feature)
- **Timeouts**: Network issues or flight controller busy

## Next Steps

Once you understand arming/disarming, you're ready for:
- Takeoff and landing operations
- Flight control commands
- Mission planning

Remember: **Always verify arming state before flight operations!**

## Important Safety Notes

- **Never attempt to disarm while airborne** - this will be rejected by safety mechanisms
- **Always verify arming state** before takeoff attempts
- **Monitor battery levels** - low battery can prevent arming
- **Check GPS lock** if your flight controller requires it for arming
- **Be aware of automatic disarming** after landing

## Navigation

- **Previous**: [Tutorial 02: Telemetry](02_telemetry.md)
- **Next**: Takeoff and Landing (`bun run src/tutorials/04_takeoff_land.js`)

---

*Mastering arm/disarm operations is crucial for safe drone programming. Always verify state changes and understand the safety mechanisms protecting against dangerous operations.*
