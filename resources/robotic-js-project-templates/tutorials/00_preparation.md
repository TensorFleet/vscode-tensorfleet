# Tutorial 00: Preparation & Setup

## Overview

Before starting the TensorFleet tutorials, you need to set up your development environment and ensure the simulation is properly configured. This tutorial covers the essential preparation steps.

## Learning Objectives

- Understand the TensorFleet development environment
- Set up simulation monitoring tools
- Learn simulation restart procedures
- Prepare for consistent tutorial execution

## Prerequisites

- TensorFleet account access
- Basic understanding of drone operations (optional)

## Step 1: Access the TensorFleet Platform

1. **Login to your TensorFleet account**
   - Visit the TensorFleet web interface
   - Log in with your credentials

2. **Start your VM**
   - Navigate to the VM section
   - Start your allocated virtual machine
   - Wait for the VM to fully boot up

3. **Open the Map Panel**
   - Once the VM is running, open the Map panel interface
   - This provides the primary interface for drone monitoring and control

## Step 2: Understanding Monitoring Tools

### Simulation Views vs Map Panel

- **Simulation Views**: Provide 3D visualization of the drone and environment
- **Map Panel**: Offers 2D map view with control interfaces

**Recommendation**: Use the Map panel for basic functionality and monitoring - it's more suitable for the tutorials.

### Available Controls

The Map panel provides several essential controls:
- **Arm/Disarm**: Enable or disable drone motors
- **Mission Control**: Execute pre-programmed flight missions
- **Manual Control**: Direct drone operation

## Step 3: Simulation Management

### Why Restart Simulation?

For consistent tutorial results, you must restart the simulation before each tutorial run. This ensures:
- Clean initial state
- Predictable drone behavior
- Consistent telemetry data

### Restart Procedure

1. **Locate the Restart Button**
   - In the Map panel interface
   - Look for "Restart simulation" on the right side

2. **Execute Restart**
   - Click "Restart simulation"
   - Wait 10-15 seconds for completion
   - The simulation will reset to its initial state

3. **Verify Restart**
   - Drone should return to ground state
   - All telemetry should reset to default values

## Step 4: Testing State Changes

### Debugging Telemetry Scripts

When developing or debugging your telemetry scripts, you'll need to observe state changes. Since tutorials run continuously, you need ways to trigger state changes during execution.

### Methods to Change Drone State

1. **Basic Takeoff/Land Operations**
   - Use the Map panel controls
   - Perform manual takeoff and landing
   - Observe how telemetry data changes

2. **Mission Execution**
   - Run pre-configured missions
   - Monitor state transitions during flight

3. **Arming/Disarming**
   - Toggle drone arming state
   - Watch connection and safety indicators

## Step 5: Environment Verification

### Check Your Setup

Before proceeding to tutorials, verify:

- [ ] TensorFleet VM is running
- [ ] Map panel is accessible
- [ ] Simulation has been restarted recently
- [ ] You can see drone status indicators

### Common Issues

- **Connection Problems**: Ensure VM is fully started
- **Map Panel Not Loading**: Check browser compatibility
- **Simulation Not Responding**: Try restarting the simulation again

## Next Steps

Once your environment is prepared:

1. **Proceed to Tutorial 01**: [Connection & State Monitoring](01_connection.md)
2. **Run tutorials in order**: Each builds upon the previous
3. **Restart simulation**: Before each tutorial execution

## Navigation

- **Previous**: [Tutorial Overview](../tutorials/README.md)
- **Next**: [Tutorial 01: Connection](01_connection.md)

---

*Note: Always restart the simulation before running any tutorial script for consistent results.*
