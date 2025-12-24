#!/usr/bin/env -S bun run
/**
 * Tutorial 06: Move Forward with Manual and Automated Control
 *
 * Learn: Velocity control and timing-based movement using DroneController
 *
 * This tutorial demonstrates:
 * - Manual takeoff and landing operations
 * - Manual OFFBOARD mode switching and setpoint streaming (no auto state management)
 * - Automatic OFFBOARD mode switching and setpoint streaming
 * - Setting forward velocity targets for movement
 * - Time-based movement with stopping
 *
 * Manual Sequence:
 * 1. Manual arm and takeoff to target altitude
 * 2. Manually set mode to OFFBOARD and broadcast forward velocity target continuously
 * 3. Maintain forward movement for specified duration with status monitoring
 * 4. Stop broadcasting target to exit OFFBOARD mode (returns to POSCTL)
 * 5. Manual landing and disarm
 *
 * Automated Sequence:
 * 1. Use requestAutoState() for automated arm and takeoff to target altitude
 * 2. Use requestAutoState() to enter OFFBOARD mode with forward velocity target
 * 3. Maintain forward movement for specified duration with status monitoring
 * 4. Clear offboard target to exit OFFBOARD mode
 * 5. Use requestAutoState() for automated landing and disarm
 *
 * Run: bun src/tutorials/06_move_forward.js
 */

import { DroneStateModel, DroneController } from "tensorfleet-util";
import { initializeDroneControl } from "../lib/drone_utils.js";

const TARGET_ALTITUDE = 3.0; // meters
const FORWARD_VELOCITY = 3.0; // m/s
const MOVE_DURATION = 5.0; // seconds

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

/**
 * Main tutorial execution function
 */
async function main() {
  // Create ROS bridge using the wrapper
  let { bridge, droneState, droneController, currentState } = await initializeDroneControl();

  // Execute manual OFFBOARD move forward sequence
  await manualOffboardMoveForwardSequence(droneController, droneState);

  console.log("\n[INFO] Manual sequence finished.\nNow, we will try out the automated sequence after 5 seconds...");
  await sleep(5000);

  // // Execute automated OFFBOARD move forward sequence
  await automatedOffboardMoveBackwardsSequence(droneController, droneState);

  // Clean up connections
  console.log("[EXIT] Cleaning up connections...");
  droneState.disconnect();
  console.log("[EXIT] Disconnected from drone state monitoring.");

  console.log("\n[SUCCESS] Move forward tutorial completed successfully!");
  process.exit(0);
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
 * Manual OFFBOARD move forward sequence
 * Demonstrates manual OFFBOARD mode switching and setpoint streaming
 * @param {DroneController} droneController - The drone controller instance
 * @param {DroneStateModel} droneState - The drone state model instance
 */
async function manualOffboardMoveForwardSequence(droneController, droneState) {
  console.log("[INFO] Executing Manual OFFBOARD move forward sequence...\n");

  // Step 1: Manual arm and takeoff
  await manualArmAndTakeoff(droneController, droneState, TARGET_ALTITUDE);

  // Step 2: Move forward - continuously send OFFBOARD mode command and broadcast targets
  console.log("[STEP 2] Starting continuous OFFBOARD mode commands and forward velocity setpoint broadcast...");
  console.log(`[STEP 2] Moving forward at ${FORWARD_VELOCITY} m/s for ${MOVE_DURATION} seconds`);
  console.log(`[STEP 2] Expected distance: ~${FORWARD_VELOCITY * MOVE_DURATION} meters\n`);

  const moveStart = Date.now();
  let setpointInterval = setInterval(async () => {
    // Continuously send OFFBOARD mode command
    await droneController.setMode("OFFBOARD", 0, false); // Silent mode setting

    // Broadcast forward velocity target
    droneController.publishOffboardTarget({
      kind: "velocity_local",
      vx: FORWARD_VELOCITY,
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

  // Step 3: Maintain forward movement for specified duration
  console.log(`[STEP 3] Maintaining continuous OFFBOARD commands and forward velocity setpoints for ${MOVE_DURATION} seconds...`);

  await sleep(MOVE_DURATION * 1000);

  // Step 4: Stop - broadcast zero velocity to stop movement
  console.log("[STEP 4] Stopping movement by broadcasting zero velocity...");
  clearInterval(setpointInterval);

  setpointInterval = setInterval(async () => {
    // Continuously send OFFBOARD mode command
    await droneController.setMode("OFFBOARD", 0, false); // Silent mode setting

    // Broadcast zero velocity target to stop
    droneController.publishOffboardTarget({
      kind: "velocity_local",
      vx: 0.0,
      vy: 0.0,
      vz: 0.0
    });
  }, 50);

  // Stop for 2 seconds to ensure complete stop
  await sleep(2000);

  // Step 5: Stop broadcasting setpoints
  console.log("[STEP 5] Stopping setpoint broadcast to exit OFFBOARD mode...");
  clearInterval(setpointInterval);
  console.log("[STEP 5] Setpoint broadcast stopped\n");

  // Wait for exit from OFFBOARD mode (should return to POSCTL)
  while (await droneState.isOffboard()) {
    console.log("[STEP 5] Waiting for exit from OFFBOARD mode due to timeout...");
    await sleep(500);
  }

  const exitState = await droneState.getState();
  console.log(`[STEP 5] Exited OFFBOARD mode. Current mode (POSCTL expected): ${exitState.vehicle?.mode}\n`);

  // Step 6: Manual landing and disarm
  console.log("[STEP 6] Landing drone...");
  await droneController.land();
  console.log("[STEP 6] Land command sent");

  // Wait for landing to complete
  while (!(await droneState.isLanded())) {
    console.log("[STEP 6] Waiting for landing to complete...");
    await sleep(1000);
  }

  // Disarm if still armed
  if (await droneState.isArmed()) {
    console.log("[STEP 6] Disarming drone...");
    await droneController.disarm();
    console.log("[STEP 6] Disarm command sent");

    // Wait for disarming
    while (await droneState.isArmed()) {
      console.log("[STEP 6] Waiting for disarm...");
      await sleep(1000);
    }
  }

  const finalState = await droneState.getState();
  console.log(`[STEP 6] Landing and disarming complete. armed=${finalState.vehicle?.armed}, mode=${finalState.vehicle?.mode}\n`);

  console.log("\n[SUCCESS] Manual OFFBOARD move forward sequence finished!");
}

/**
 * Automated OFFBOARD move forward sequence
 * Demonstrates automatic OFFBOARD mode switching and setpoint streaming
 * @param {DroneController} droneController - The drone controller instance
 * @param {DroneStateModel} droneState - The drone state model instance
 */
async function automatedOffboardMoveBackwardsSequence(droneController, droneState) {
  console.log("[INFO] Executing Automated OFFBOARD move forward sequence...\n");

  // Step 1: Automated arm and takeoff using requestAutoState
  console.log("[STEP 1] Requesting automated takeoff to target altitude...");
  await droneController.requestAutoState({
    kind: "airborne",
    altMeters: TARGET_ALTITUDE
  });
  console.log("[STEP 1] Automated takeoff requested - controller will handle arming and takeoff\n");

  // Step 2: Enter OFFBOARD mode by setting forward velocity target
  console.log("[STEP 2] Entering OFFBOARD mode with forward velocity...");
  console.log(`[STEP 2] Moving forward at ${FORWARD_VELOCITY} m/s for ${MOVE_DURATION} seconds`);
  console.log(`[STEP 2] Expected distance: ~${FORWARD_VELOCITY * MOVE_DURATION} meters`);

  // No await. In this tutorial we want to count the acceleration time as time passed. In the real world it's wrong to do this but we just want to demonstrate it.
  droneController.requestAutoState({
    kind: "offboard",
    target: {
      kind: "velocity_local",
      vx: -FORWARD_VELOCITY,
      vy: 0.0,
      vz: 0.0
    }
  });
  console.log("[STEP 2] Offboard target set to forward velocity - controller will automatically switch to OFFBOARD mode and stream setpoints\n");

  console.log("[STEP 2] OFFBOARD mode active\n");

  // Step 3: Maintain forward movement for specified duration
  console.log(`[STEP 3] Moving forward for ${MOVE_DURATION} seconds...`);

  await sleep(MOVE_DURATION*1000);
  console.log(`[STEP 3] Forward movement duration complete\n`);

  // Step 4: Stop by setting zero velocity target
  console.log("[STEP 4] Stopping movement by setting zero velocity target...");
  await droneController.requestAutoState({
    kind: "airborne",
    altMeters: TARGET_ALTITUDE
  });
  console.log("[STEP 4] Offboard target set to zero velocity\n");

  // Maintain zero velocity for 2 seconds to ensure complete stop
  await sleep(2000);

  // Step 5: Clear offboard target to exit OFFBOARD mode
  console.log("[STEP 5] Clearing offboard target to exit OFFBOARD mode...");
  droneController.clearAutoState();
  console.log("[STEP 5] Offboard target cleared");

  // Wait for exit from OFFBOARD mode
  while (await droneState.isOffboard()) {
    console.log("[STEP 5] OFFBOARD target broadcast has ended.\n[STEP 5] Waiting for exit from OFFBOARD mode due to timeout...");
    await sleep(500);
  }

  const exitState = await droneState.getState();
  console.log(`[STEP 5] Exited OFFBOARD mode. Current mode (POSCTL expected): ${exitState.vehicle?.mode}\n`);

  // Step 6: Automated landing and disarm using requestAutoState
  console.log("[STEP 6] Requesting automated landing and disarm...");
  await droneController.requestAutoState({
    kind: "landed",
    armed: false
  });
  console.log("[STEP 6] Automated landing and disarm requested - controller will handle landing and disarming\n");

  console.log("\n[SUCCESS] Automated OFFBOARD move forward sequence finished!");
}
