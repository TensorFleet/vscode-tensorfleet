#!/usr/bin/env -S bun run
/**
 * Tutorial 03: Arm / Disarm the Drone with Auto State Management
 *
 * This tutorial demonstrates:
 * - Manual arming and disarming operations in landed state
 * - Automatic state management that maintains target states
 * - Using requestAutoState() with auto-enforcement and built-in waiting
 * - Proper state transitions with automatic completion detection
 * - No takeoff operations - drone remains landed throughout
 *
 * Manual Sequence:
 * 1. Arm the drone if not already armed (in landed state)
 * 2. Wait 6 seconds to observe armed state. Retry if needed.
 * 3. Disarm the drone (returns to landed disarmed state)
 * 4. Wait 0.5 seconds to observe disarmed state. Retry if needed.
 * 
 * Automated Sequence (full async):
 * 1. Command drone to landed armed state (auto-waits for completion)
 * 2. Command drone to landed disarmed state (auto-waits for completion)
 */

import { DroneStateModel, DroneController } from "tensorfleet-util";
import { initializeDroneControl } from "../lib/drone_utils.js";

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

/**
 * Manual arm and disarm sequence
 * Demonstrates basic manual control of arming and disarming
 * @param {DroneController} droneController - The drone controller instance
 * @param {DroneStateModel} droneState - The drone state model instance
 */
async function manualArmDisarmSequence(droneController, droneState) {
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
  let armedState = await droneState.getState();

  while(!armedState.vehicle?.armed) {
    console.log("[STEP 2] Retrying. drone not armed...");
    await droneController.arm();
    await sleep(1000);
    armedState = await droneState.getState();
  }

  console.log(`[STEP 2] State after arming : armed=${armedState.vehicle?.armed}, mode=${armedState.vehicle?.mode}`);
  

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
}

/**
 * Automated state management sequence
 * Demonstrates automatic state management for takeoff and landing
 * @param {DroneController} droneController - The drone controller instance
 * @param {DroneStateModel} droneState - The drone state model instance
 */
async function automatedStateManagementSequence(droneController, droneState) {
  // Step 5: Automatic state management is running
  // The controller will automatically maintain target states once we set one
  console.log("[STEP 1] Automatic state management is running - will activate once we set a target state");

  console.log("[STEP 1] Waiting 5 seconds before setting target state to landed and armed...");
  await sleep(5000)

  // Step 6: Command drone to landed armed state
  // Auto state management will handle the transition and wait for completion
  console.log("[STEP 1] Setting target state: landed and armed...");
  await droneController.requestAutoState({ kind: "landed", armed: true });
  console.log("[STEP 1] Target state reached - drone is now landed and armed");

  // Step 7: Observe the final state
  let currentState = await droneState.getState();
  const alt = currentState.altitude?.relative || 0;
  console.log(`[STEP 1] Current state: altitude=${alt.toFixed(2)}m, armed=${currentState.vehicle?.armed}, mode=${currentState.vehicle?.mode}`);

  console.log("[STEP 2] Waiting 6 seconds before setting target state: landed and disarmed...");

  await sleep(6000);

  // Step 8: Command drone to landed and disarmed state
  // Auto state management will handle the transition and wait for completion
  console.log("[STEP 2] Setting target state: landed and disarmed...");
  await droneController.requestAutoState({ kind: "landed", armed: false });
  console.log("[STEP 2] Target state reached - drone is now landed and disarmed");
  console.log("\n[SUCCESS] Drone has landed and disarmed!");
}

/**
 * Main tutorial execution function
 * Demonstrates the complete arm/disarm cycle with automatic state management
 */
async function main() {
  const { bridge, droneState, droneController } = await initializeDroneControl();

  // Execute manual arm/disarm sequence
  // This will only work if the drone is on land.
  await manualArmDisarmSequence(droneController, droneState);

  console.log("\n[INFO] Manual sequence finished. \nNow, we wait 5 seconds before we initiate the automated sequence...");
  await sleep(5000);

  // Execute automated state management sequence
  // This can work even if the drone is in mid air. The automatic state management takes care of the transition.
  await automatedStateManagementSequence(droneController, droneState);

  console.log("\n[INFO] Sequence completed successfully.\n");

  // Clean up connections
  const currentState = await droneState.getState();
  console.log(`[EXIT] Final drone state : armed=${currentState.vehicle?.armed}, mode=${currentState.vehicle?.mode}`);
  droneState.disconnect();
  console.log("[EXIT] Disconnected from drone state monitoring.");

  process.exit(0);
}

if (require.main === module) {
  main().catch((err) => {
    console.error("[ERROR]", err.message || err);
    process.exit(1);
  });
}
