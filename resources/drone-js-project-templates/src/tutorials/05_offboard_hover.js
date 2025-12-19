#!/usr/bin/env -S bun run
/**
 * Tutorial 05: OFFBOARD Hover with Manual and Automated Control
 *
 * Learn: OFFBOARD mode basics using DroneController
 *
 * This tutorial demonstrates:
 * - Manual takeoff and landing operations
 * - Manual OFFBOARD mode switching and setpoint streaming (no auto state management)
 * - Automatic OFFBOARD mode switching and setpoint streaming
 * - Setting velocity targets for hovering (zero velocity = hover)
 *
 * Manual Sequence:
 * 1. Manual arm and takeoff to target altitude
 * 2. Manually set mode to OFFBOARD and broadcast zero velocity target continuously
 * 3. Maintain hover for specified duration with status monitoring
 * 4. Stop broadcasting target to exit OFFBOARD mode (returns to POSCTL)
 * 5. Manual landing and disarm
 *
 * Automated Sequence:
 * 1. Manual arm and takeoff to target altitude
 * 2. Use requestAutoState() to enter OFFBOARD mode with zero velocity target
 * 3. Maintain hover for specified duration with status monitoring
 * 4. Clear offboard target to exit OFFBOARD mode
 * 5. Manual landing and disarm
 *
 * Run: bun src/tutorials/05_offboard_hover.js
 */

import { DroneStateModel, DroneController } from "tensorfleet-util";
import { initializeDroneControl } from "../lib/drone_utils.js";

const TARGET_ALTITUDE = 3.0; // meters
const HOVER_DURATION = 10.0; // seconds

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

/**
 * Main tutorial execution function
 */
async function main() {
  // Create ROS bridge using the wrapper
  let { bridge, droneState, droneController, currentState } = await initializeDroneControl();

  // Execute manual OFFBOARD hover sequence
  await manualOffboardHoverSequence(droneController, droneState);

  console.log("\n[INFO] Manual sequence finished.\nNow, we will try out the automated sequence after 5 seconds...");
  await sleep(5000);

  // Execute automated OFFBOARD hover sequence
  await automatedOffboardHoverSequence(droneController, droneState);

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
 * Manual OFFBOARD hover sequence
 * Demonstrates manual OFFBOARD mode switching and setpoint streaming
 * @param {DroneController} droneController - The drone controller instance
 * @param {DroneStateModel} droneState - The drone state model instance
 */
async function manualOffboardHoverSequence(droneController, droneState) {
  console.log("[INFO] Executing Manual OFFBOARD hover sequence...\n");

  // Step 1: Manual arm and takeoff
  await manualArmAndTakeoff(droneController, droneState, TARGET_ALTITUDE);

  // Step 2: Continuously send OFFBOARD mode command and broadcast targets
  console.log("[STEP 2] Starting continuous OFFBOARD mode commands and setpoint broadcast...");

  const hoverStart = Date.now();
  let setpointInterval = setInterval(async () => {
    // Continuously send OFFBOARD mode command
    await droneController.setMode("OFFBOARD", 0, false); // Silent mode setting

    // Broadcast zero velocity target
    droneController.publishOffboardTarget({
      kind: "velocity_local",
      vx: 0.0,
      vy: 0.0,
      vz: 0.0
    });
  }, 50); // Broadcast at 20Hz as per PX4 requirements

  // Wait for OFFBOARD mode to be active
  while (!(await droneState.isOffboard())) {
    console.log("[STEP 2] Waiting for OFFBOARD mode to activate...");
    await sleep(500);
  }
  console.log("[STEP 2] OFFBOARD mode active - continuing continuous commands and setpoints\n");

  // Step 3: Maintain hover for specified duration
  console.log(`[STEP 3] Maintaining continuous OFFBOARD commands and hover setpoints for ${HOVER_DURATION} seconds...`);

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

    await sleep(1000);
  }

  // Step 4: Stop broadcasting setpoints
  console.log("[STEP 4] Stopping setpoint broadcast to exit OFFBOARD mode...");
  clearInterval(setpointInterval);
  console.log("[STEP 4] Setpoint broadcast stopped\n");

  // Wait for exit from OFFBOARD mode (should return to POSCTL)
  while (await droneState.isOffboard()) {
    console.log("[STEP 4] Waiting for exit from OFFBOARD mode due to timeout...");
    await sleep(500);
  }

  const exitState = await droneState.getState();
  console.log(`[STEP 4] Exited OFFBOARD mode. Current mode (POSCTL expected): ${exitState.vehicle?.mode}\n`);

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

  console.log("\n[SUCCESS] Manual OFFBOARD hover sequence finished!");
}

/**
 * Automated OFFBOARD hover sequence
 * Demonstrates automatic OFFBOARD mode switching and setpoint streaming
 * @param {DroneController} droneController - The drone controller instance
 * @param {DroneStateModel} droneState - The drone state model instance
 */
async function automatedOffboardHoverSequence(droneController, droneState) {
  console.log("[INFO] Executing Automated OFFBOARD hover sequence...\n");

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

    await sleep(1000);
  }

  console.log(`[STEP 3] Hover duration complete\n`);

  // Step 4: Clear offboard target to exit OFFBOARD mode
  console.log("[STEP 4] Clearing offboard target to exit OFFBOARD mode...");
  droneController.clearAutoState();
  console.log("[STEP 4] Offboard target cleared");

  // Wait for exit from OFFBOARD mode
  while (await droneState.isOffboard()) {
    console.log("[STEP 4] OFFBOARD target broadcast has ended.\n[STEP 4] Waiting for exit from OFFBOARD mode due to timeout...");
    await sleep(500);
  }

  const exitState = await droneState.getState();
  console.log(`[STEP 4] Exited OFFBOARD mode. Current mode (POSCTL expected): ${exitState.vehicle?.mode}\n`);

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

  console.log("\n[SUCCESS] Automated OFFBOARD hover sequence finished!");
}
