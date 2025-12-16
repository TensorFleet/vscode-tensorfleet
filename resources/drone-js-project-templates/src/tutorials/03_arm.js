#!/usr/bin/env -S bun run
/**
 * Tutorial 03: Arm / Disarm the Drone with Auto State Management
 *
 * This tutorial demonstrates:
 * - Manual arming and disarming operations
 * - Enabling automatic state management that maintains target states
 * - Using setRequestedState() with auto-enforcement
 * - Proper waiting for state transitions to complete
 *
 * Sequence:
 * 1. Arm the drone if not already armed
 * 2. Wait 6 seconds to observe armed state
 * 3. Disarm the drone
 * 4. Wait 5 seconds to observe disarmed state
 * 5. Enable automatic state management (maintains target state automatically)
 * 6. Command drone to become airborne at 3m altitude
 * 7. Wait 10 seconds while drone maintains altitude
 * 8. Command drone to land and disarm
 * 9. Wait for landing and disarming to complete
 * 10. Disable auto state management and clean up
 */

import { DroneStateModel, DroneController } from "tensorfleet-util";
import { ROSLibBridgeWrapper } from "../lib/roslib-bridge-wrapper.js";

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

/**
 * Waits for the drone to establish connection with the flight controller
 * @param {DroneStateModel} droneState - The drone state model instance
 */
async function waitUntilConnected(droneState) {
  console.log("[INFO] Waiting for drone state connection...");
  while (true) {
    const state = await droneState.getState();
    if (state.vehicle?.connected) {
      console.log(`[INFO] Drone connected. Current state: armed=${state.vehicle.armed}, mode=${state.vehicle.mode}\n`);
      return;
    }
    await sleep(100);
  }
}

/**
 * Main tutorial execution function
 * Demonstrates the complete arm/disarm cycle with automatic state management
 */
async function main() {
  // Initialize ROS bridge connection
  const bridge = new ROSLibBridgeWrapper();
  await bridge.waitForConnection();

  console.log("\n[INFO] Connected to ROS Bridge - initializing drone control...\n");

  // Create drone state model for monitoring vehicle state
  const droneState = new DroneStateModel();
  droneState.connect(bridge);

  // Create drone controller for high-level commands
  const droneController = new DroneController(droneState, bridge);
  await droneController.initialize();

  // Wait for drone to be connected and ready
  await waitUntilConnected(droneState);

  // Step 1: Arm the drone if it's not already armed
  // This prepares the drone for flight operations
  {
    const st = await droneState.getState();
    console.log(`[STEP 1] Current state before arming: armed=${st.vehicle?.armed}, mode=${st.vehicle?.mode}`);
    if (!st.vehicle?.armed) {
      console.log("[STEP 1] Arming drone...");
      await droneController.arm();
      console.log("[STEP 1] Arm command sent successfully");
    } else {
      console.log("[STEP 1] Drone already armed - skipping arm command");
    }
  }

  // Step 2: Wait to observe the armed state
  console.log("[STEP 2] Waiting 6 seconds to observe armed state...");
  await sleep(6000);
  const armedState = await droneState.getState();
  console.log(`[STEP 2] State after 6 seconds: armed=${armedState.vehicle?.armed}, mode=${armedState.vehicle?.mode}`);

  // Step 3: Disarm the drone
  // Returns drone to safe state on ground
  console.log("[STEP 3] Disarming drone...");
  await droneController.disarm();
  console.log("[STEP 3] Disarm command sent successfully");

  // Step 4: Wait to observe the disarmed state
  console.log("[STEP 4] Waiting 5 seconds to observe disarmed state...");
  await sleep(5000);
  const disarmedState = await droneState.getState();
  console.log(`[STEP 4] State after 5 seconds: armed=${disarmedState.vehicle?.armed}, mode=${disarmedState.vehicle?.mode}`);

  // Step 5: Enable automatic state management
  // This enables the controller to automatically maintain target states
  // The system will derive initial target from current drone state
  console.log("[STEP 5] Enabling automatic state management...");
  await droneController.enableAutoStateManagement(true);
  console.log("[STEP 5] Auto state management enabled - controller will now maintain target states automatically");

  // Step 6: Command drone to become airborne at 3m altitude
  // Auto state management will arm, takeoff, and maintain altitude
  console.log("[STEP 6] Setting target state: airborne at 3m altitude...");
  droneController.setRequestedState({ kind: "airborne", altMeters: 3 });
  console.log("[STEP 6] Target state set to", droneController.requestedState , " - auto management will handle arming and takeoff");

  // Step 7: Wait while drone maintains altitude
  console.log("[STEP 7] Waiting while drone transitions to and maintains altitude...");
  const startTime = Date.now();
  while (true) {
    
    const currentState = await droneState.getState();
    const alt = currentState.altitude?.relative || 0;
    if (await droneController.isInRequestedState()) {
      console.log('[STEP 7] target state reached reached');
      break
    }
    await sleep(1000); // Note : if we don't sleep here we may block the process any further processing of the websocket packets and never get a state change.
  }

  let currentState = await droneState.getState();
  const alt = currentState.altitude?.relative || 0;

  console.log(`[STEP 7] FINISHED. Current altitude: ${alt.toFixed(2)}m, armed=${currentState.vehicle?.armed}, mode=${currentState.vehicle?.mode}`);

  // Step 8: Command drone to land and disarm
  // Auto state management will land the drone and then disarm it
  console.log("[STEP 8] Setting target state: landed and disarmed...");
  droneController.setRequestedState({ kind: "landed", armed: false });
  console.log("[STEP 8] Target state set - auto management will handle landing and disarming");

  // Step 9: Wait for the landing and disarming process to complete
  // This ensures the script doesn't exit prematurely
  console.log("[STEP 9] Waiting for landing and disarming to complete...");
  await droneController.waitForRequestedState();
  console.log("[STEP 9] Landing and disarming completed successfully");
  console.log("\n[SUCCESS] Drone has landed and disarmed!");

  // Step 10: Disable auto state management
  // Clean shutdown of automatic control systems
  console.log("[STEP 10] Disabling auto state management...");
  await droneController.enableAutoStateManagement(false);
  console.log("[STEP 10] Auto state management disabled");

  console.log("\n[INFO] Sequence completed successfully.\n");

  // Clean up connections
  currentState = await droneState.getState();
  console.log(`[EXIT] Final drone state : armed=${currentState.vehicle?.armed}, mode=${currentState.vehicle?.mode}`);
  droneState.disconnect();
  console.log("[EXIT] Disconnected from drone state monitoring.");
}

if (require.main === module) {
  main().catch((err) => {
    console.error("[ERROR]", err.message || err);
    process.exit(1);
  });
}
