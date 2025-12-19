#!/usr/bin/env -S bun run
/**
 * Tutorial 05: OFFBOARD Hover with Manual Control
 *
 * Learn: OFFBOARD mode basics using DroneController
 *
 * This tutorial demonstrates:
 * - Manual takeoff and landing operations
 * - Using DroneController for offboard operations
 * - Setting velocity targets for hovering (zero velocity = hover)
 * - Automatic OFFBOARD mode switching and setpoint streaming
 * - Manual sequence without automatic state management
 *
 * Sequence:
 * 1. Manual arm and takeoff to target altitude
 * 2. Enter OFFBOARD mode by setting zero velocity target (hover)
 * 3. Maintain hover for specified duration with status monitoring
 * 4. Clear offboard target to exit OFFBOARD mode
 * 5. Manual landing and disarm
 *
 * Run: bun src/tutorials/05_offboard_hover.js
 */

import { DroneStateModel, DroneController } from "tensorfleet-util";
import { ROSLibBridgeWrapper } from "../lib/roslib-bridge-wrapper.js";

const TARGET_ALTITUDE = 3.0; // meters
const HOVER_DURATION = 10.0; // seconds

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
 * Manual arm and takeoff sequence
 * @param {DroneController} droneController - The drone controller instance
 * @param {DroneStateModel} droneState - The drone state model instance
 */
async function manualArmAndTakeoff(droneController, droneState, targetAltitude) {
  console.log("[STEP 1] Manual arm and takeoff sequence...\n");

  // Arm the drone
  console.log("[STEP 1] Arming drone...");
  await droneController.arm();
  console.log("[STEP 1] Arm command sent");

  // Wait for arming to complete
  while (!(await droneState.isArmed())) {
    console.log("[STEP 1] Waiting for drone to arm...");
    await sleep(1000);
  }
  console.log("[STEP 1] Drone armed successfully\n");

  // Takeoff to target altitude
  console.log(`[STEP 1] Taking off to ${targetAltitude}m...`);
  await droneController.takeoff(targetAltitude);
  console.log("[STEP 1] Takeoff command sent");

  // Wait for takeoff to complete
  while (!(await droneState.isAirborne())) {
    console.log("[STEP 1] Waiting for takeoff to complete...");
    await sleep(1000);
  }

  const takeoffState = await droneState.getState();
  const takeoffAlt = takeoffState.altitude?.relative || 0;
  console.log(`[STEP 1] Takeoff complete. Current altitude: ${takeoffAlt.toFixed(2)}m, mode=${takeoffState.vehicle?.mode}\n`);
}

/**
 * Main tutorial execution function
 */
async function main() {
  // Create ROS bridge using the wrapper
  const bridge = new ROSLibBridgeWrapper();
  await bridge.waitForConnection();

  console.log(`\n[INFO] Target altitude: ${TARGET_ALTITUDE}m, hover duration: ${HOVER_DURATION}s\n`);

  // Create managed state model for state monitoring
  const droneState = new DroneStateModel();
  droneState.connect(bridge);

  // Create drone controller for high-level operations
  const droneController = new DroneController(droneState, bridge);
  await droneController.initialize();

  // Wait for drone to be connected and ready
  await waitUntilConnected(droneState);

  // Step 1: Manual arm and takeoff
  await manualArmAndTakeoff(droneController, droneState, TARGET_ALTITUDE);

  // Step 2: Enter OFFBOARD mode by setting zero velocity target (hover)
  console.log("[STEP 2] Entering OFFBOARD mode with zero velocity (hover)...");
  await droneController.requestAutoState({
    kind: "offboard",
    target: {
      kind: "velocity_local",
      vx: 0.0,
      vy: 0.0,
      vz: 0.0
    }
  });
  console.log("[STEP 2] Offboard target set to zero velocity - controller will automatically switch to OFFBOARD mode and stream setpoints\n");

  console.log("[STEP 2] OFFBOARD mode active\n");

  // Step 3: Maintain hover for specified duration
  console.log(`[STEP 3] Hovering for ${HOVER_DURATION} seconds...`);

  const hoverStart = Date.now();
  while (Date.now() - hoverStart < HOVER_DURATION * 1000) {
    const currentState = await droneState.getState();
    const currentAlt = currentState.altitude?.relative || 0;
    const currentMode = currentState.vehicle?.mode || "unknown";
    const isOffboard = await droneState.isOffboard();

    // Log status every 2 seconds
    if (Math.floor((Date.now() - hoverStart) / 2000) > Math.floor((Date.now() - hoverStart - 2000) / 2000)) {
      const elapsed = ((Date.now() - hoverStart) / 1000).toFixed(1);
      console.log(`[HOVER] t=${elapsed}s: alt=${currentAlt.toFixed(2)}m, mode=${currentMode}, offboard=${isOffboard}`);
    }

    await sleep(1000); // Check status every 500ms
  }

  console.log(`[STEP 3] Hover duration complete\n`);

  // Step 4: Clear offboard target to exit OFFBOARD mode
  console.log("[STEP 4] Clearing offboard target to exit OFFBOARD mode...");
  droneController.clearAutoState();
  console.log("[STEP 4] Offboard target cleared");

  // Wait for exit from OFFBOARD mode
  while (await droneState.isOffboard()) {
    console.log("[STEP 4] OFFBOARD target boradcast has ended.\n[STEP 4] Waiting for exit from OFFBOARD mode due to timeout...");
    await sleep(500);
  }

  const exitState = await droneState.getState();
  console.log(`[STEP 4] Exited OFFBOARD mode. Current mode( POSCTL expected): ${exitState.vehicle?.mode}\n`);
  // Note : if we leave the drone in POSCTL with no manual input given (not covered in tutorials yet) then the drone will automatically land.
  // In this case when drone is disarmed after land it will switch to a disarmed OFFBOARD mode.

  // Step 5: Manual landing and disarm
  console.log("[STEP 5] Landing drone...");
  await droneController.land();
  console.log("[STEP 5] Land command sent");

  // Wait for landing to complete
  while (!(await droneState.isLanded())) {
    console.log("[STEP 5] Waiting for landing to complete...");
    await sleep(1000);
  }

  // Disarm if still armed
  if (await droneState.isArmed()) {
    console.log("[STEP 5] Disarming drone...");
    await droneController.disarm();
    console.log("[STEP 5] Disarm command sent");

    // Wait for disarming
    while (await droneState.isArmed()) {
      console.log("[STEP 5] Waiting for disarm...");
      await sleep(1000);
    }
  }

  const finalState = await droneState.getState();
  console.log(`[STEP 5] Landing and disarming complete. armed=${finalState.vehicle?.armed}, mode=${finalState.vehicle?.mode}\n`);

  // Clean up connections
  console.log("[EXIT] Cleaning up connections...");
  droneState.disconnect();
  console.log("[EXIT] Disconnected from drone state monitoring.");

  console.log("\n[SUCCESS] OFFBOARD hover tutorial completed successfully!");
}

if (require.main === module) {
  main().catch((err) => {
    console.error("[ERROR]", err.message || err);
    process.exit(1);
  });
}
