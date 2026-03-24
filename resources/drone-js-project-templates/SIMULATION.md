# Simulation Management Guide

## Overview

The TensorFleet drone simulation runs on PX4 and provides a virtual environment for testing drone scripts and tutorials. Consistent simulation state is crucial for reliable testing, so restarting the simulation between runs ensures predictable behavior.

For the current `vision:yolo` pass, use the default drone VM/world as-is.

## Methods to Restart Simulation

### Method 1: Manual Restart via Map Panel (Recommended for Beginners)

This is the primary method for most users and tutorials.

#### Prerequisites
- Active TensorFleet account
- Running VM instance

#### Steps
1. **Log in to TensorFleet**: Access your TensorFleet account through the web interface.

2. **Start your VM**: Launch your virtual machine instance if it's not already running.

3. **Open the Map Panel**: Navigate to the Map panel in the TensorFleet interface.

4. **Restart Simulation**:
   - Look for the "Restart simulation" button on the right side of the interface
   - Click the button to initiate the restart
   - Wait 10-15 seconds for the process to complete

5. **Verify**: The simulation should now be in a clean state with the drone ready for testing.

#### When to Use
- Following tutorial instructions
- Before running any drone scripts
- When troubleshooting inconsistent behavior
- When starting a new development session

### Method 2: Programmatic Restart via Script

For automated workflows or advanced users, you can restart the simulation programmatically.

#### Prerequisites (These should be automatically setup)
- Node.js/Bun environment set up
- Proper `.env` configuration with TensorFleet settings
- Active ROS Bridge connection

#### Usage
```bash
bun run src/restart_sim.js
```

#### Vision demo setup

No extra MMDS scene composition is required for `vision:yolo` in this first pass.
Use the current default drone VM configuration and verify the front camera topic
is available after restart.

The expected image topics in that world are:

- `/drone_camera/image_raw`
- `/drone_camera/image_raw/compressed`
- `/drone_camera/down/image_raw/compressed`

#### What it does
- Initializes ROS bridge connection using ROSLibBridgeWrapper
- Automatically handles proxy vs direct connection based on TensorFleet config
- Calls the `/simulation_manager/start_simulation` ROS service
- Waits for confirmation of successful restart
- Provides console output for status monitoring

#### Output Example
```
bun ./src/restart_sim.js
[SIM] Initializing ROS bridge connection...
ROSLib uses utf8 encoding by default. It would be more efficient to use ascii (if possible).
[SIM] Connected to ROS bridge
[SIM] Requesting simulation restart...
[SIM] Restart succeeded
[SIM] Message: gazebo-headless@harmonic restarted and simulation connected
```

#### Integration
The restart functionality is also available programmatically:

```javascript
const { restartSimulation } = require('./src/restart_sim.js');

async function prepareTest() {
    const success = await restartSimulation();
    if (success) {
        // Proceed with your drone operations
    }
}
```

## Testing the simulation restart script

To verify that the simulation restart works correctly:

1. **Open the Map Panel**: Navigate to the TensorFleet Map panel in your browser.

2. **Change Drone State**: Ensure the drone is not in its default state. Press the "Take Off" button to see the drone automatically arm and take off.

3. **Run the Restart Script**:
   ```bash
   bun run src/restart_sim.js
   ```

4. **Wait and Observe**: After about 10 seconds, you should see the drone reconnect with a disarmed state in its initial position. The simulation will have reset to its clean starting condition.

This test confirms that the restart process properly resets the drone's flight state, position, and arming status.

## Troubleshooting

### Connection Issues
- Ensure your VM is running and accessible
- Check ROS Bridge connectivity
- Verify TensorFleet token and proxy settings in `.env`

### Restart Failures
- Wait for the full restart process to complete (10-15 seconds)
- Check simulation manager service availability
- Try manual restart from the map panel if programmatic restart fails

### Inconsistent State
- Always restart simulation between test runs
- Ensure no other processes are interfering with the simulation
- If image topics are missing, verify the VM bridge config and the x500 camera model were updated together

## Best Practices

1. **Always restart** before running tutorials or scripts
2. **Wait for completion** before proceeding with operations
3. **Use manual method** for initial setup and learning
4. **Use programmatic method** for automated testing pipelines
5. **Monitor output** for any error messages or warnings

## Related Documentation

- [TUTORIAL.md](TUTORIAL.md) - Tutorial preparation steps
- [TENSORFLEET_UTIL.md](TENSORFLEET_UTIL.md) - Core utility library documentation
