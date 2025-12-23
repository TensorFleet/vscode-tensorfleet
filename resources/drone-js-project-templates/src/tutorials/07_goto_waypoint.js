#!/usr/bin/env -S bun run
/**
 * Tutorial 07: Go to Waypoint with Velocity Control
 *
 * Learn: Position-based navigation and closed-loop control using DroneController
 *
 * This tutorial demonstrates:
 * - Manual takeoff and landing operations
 * - Manual OFFBOARD mode switching and setpoint streaming (no auto state management)
 * - Automatic OFFBOARD mode switching and setpoint streaming
 * - Setting velocity targets for waypoint navigation
 * - Closed-loop control (feedback)
 * - Waypoint arrival detection
 *
 * Manual Sequence:
 * 1. Manual arm and takeoff to target altitude
 * 2. Manually set mode to OFFBOARD and broadcast velocity targets continuously
 * 3. Maintain waypoint navigation for specified duration with status monitoring
 * 4. Stop broadcasting target to exit OFFBOARD mode (returns to POSCTL)
 * 5. Manual landing and disarm
 *
 * Automated Sequence:
 * 1. Use requestAutoState() for automated arm and takeoff to target altitude
 * 2. Use requestAutoState() to enter OFFBOARD mode with position target to opposite waypoint
 * 3. Maintain waypoint navigation with status monitoring
 * 4. Clear offboard target to exit OFFBOARD mode
 * 5. Use requestAutoState() for automated landing and disarm
 *
 * Run: bun src/tutorials/07_goto_waypoint_with_velocity.js
 */

import { DroneStateModel, DroneController } from "tensorfleet-util";
import { initializeDroneControl } from "../lib/drone_utils.js";

const TARGET_ALTITUDE = 3.0; // meters
const WAYPOINT_OFFSET_X = 10.0; // meters forward
const WAYPOINT_OFFSET_Y = 10.0; // meters right
const WAYPOINT_RADIUS = 0.5; // meters
const MAX_VELOCITY = 2.0; // m/s

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

/**
 * Main tutorial execution function
 */
async function main() {
  // Create ROS bridge using the wrapper
  let { bridge, droneState, droneController, currentState } = await initializeDroneControl();

  // Execute manual OFFBOARD waypoint navigation sequence
  await manualOffboardWaypointSequence(droneController, droneState);

  console.log("\n[INFO] Manual sequence finished.\nNow, we will try out the automated sequence after 5 seconds...");
  await sleep(5000);

  // Execute automated OFFBOARD waypoint navigation sequence
  await automatedOffboardWaypointSequence(droneController, droneState);

  // Clean up connections
  console.log("[EXIT] Cleaning up connections...");
  droneState.disconnect();
  console.log("[EXIT] Disconnected from drone state monitoring.");

  console.log("\n[SUCCESS] Go to waypoint tutorial completed successfully!");
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
 * Manual OFFBOARD waypoint navigation sequence
 * Demonstrates manual OFFBOARD mode switching and setpoint streaming
 * @param {DroneController} droneController - The drone controller instance
 * @param {DroneStateModel} droneState - The drone state model instance
 */
async function manualOffboardWaypointSequence(droneController, droneState) {
  console.log("[INFO] Executing Manual OFFBOARD waypoint navigation sequence...\n");

  // Step 1: Manual arm and takeoff
  await manualArmAndTakeoff(droneController, droneState, TARGET_ALTITUDE);

  // Step 2: Record home position and calculate target
  const homeState = await droneState.getState();
  const home = {
    x: homeState.pose?.position?.x || 0,
    y: homeState.pose?.position?.y || 0
  };

  const target = {
    x: home.x + WAYPOINT_OFFSET_X,
    y: home.y + WAYPOINT_OFFSET_Y
  };

  console.log(`[STEP 2] Home: (${home.x.toFixed(2)}, ${home.y.toFixed(2)})`);
  console.log(`[STEP 2] Target: (${target.x.toFixed(2)}, ${target.y.toFixed(2)})\n`);

  // Step 3: Start waypoint navigation - continuously send OFFBOARD mode command and broadcast velocity targets
  console.log("[STEP 3] Starting continuous OFFBOARD mode commands and velocity setpoint broadcast...");

  let arrived = false;
  let lastLogTime = 0;
  let setpointInterval = setInterval(async () => {
    // Continuously send OFFBOARD mode command
    await droneController.setMode("OFFBOARD", 0, false); // Silent mode setting

    if (!arrived) {
      // Get current position
      const currentState = await droneState.getState();
      const pos = currentState.local?.position;
      if (!pos) {
        console.log("Pose not defined");
        return;
      }

      const dx = target.x - pos.x;
      const dy = target.y - pos.y;
      const distance = Math.sqrt(dx * dx + dy * dy);

      // Check if arrived at waypoint
      if (distance < WAYPOINT_RADIUS) {
        console.log("\n[NAV] Arrived at waypoint!\n");
        arrived = true;
        // Broadcast zero velocity to stop
        droneController.publishOffboardTarget({
          kind: "velocity_local",
          vx: 0.0,
          vy: 0.0,
          vz: 0.0
        });
        return;
      }

      // Calculate velocity (proportional control)
      const speed = Math.min(MAX_VELOCITY, distance * 0.5);
      const vx = (dx / distance) * speed;
      const vy = (dy / distance) * speed;

      // Broadcast velocity target
      droneController.publishOffboardTarget({
        kind: "velocity_local",
        vx: vx,
        vy: vy,
        vz: 0.0
      });

      // Log progress occasionally
      const now = Date.now();
      if (now - lastLogTime > 2000) {
        console.log(`[NAV] Distance to target: ${distance.toFixed(2)}m, velocity: (${vx.toFixed(2)}, ${vy.toFixed(2)})`);
        lastLogTime = now;
      }
    } else {
      // Already arrived, broadcast zero velocity
      droneController.publishOffboardTarget({
        kind: "velocity_local",
        vx: 0.0,
        vy: 0.0,
        vz: 0.0
      });
    }
  }, 50); // Broadcast at 20Hz as per PX4 requirements

  // Wait for OFFBOARD mode to be active
  while (!(await droneState.isOffboard())) {
    console.log("[STEP 3] Waiting for OFFBOARD mode to activate...");
    await sleep(500);
  }
  console.log("[STEP 3] OFFBOARD mode active - continuing continuous commands and setpoints\n");

  // Step 4: Wait for arrival at waypoint
  console.log(`[STEP 4] Navigating to waypoint...`);

  while (!arrived) {
    await sleep(100);
  }

  // Wait a bit for the drone to fully stop
  console.log("[STEP 4] Waiting for drone to stabilize at waypoint...");
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

  console.log("\n[SUCCESS] Manual OFFBOARD waypoint navigation sequence finished!");
}

/**
 * Automated OFFBOARD waypoint navigation sequence
 * Demonstrates automatic OFFBOARD mode switching and setpoint streaming
 * @param {DroneController} droneController - The drone controller instance
 * @param {DroneStateModel} droneState - The drone state model instance
 */
async function automatedOffboardWaypointSequence(droneController, droneState) {
  console.log("[INFO] Executing Automated OFFBOARD waypoint navigation sequence...\n");

  // Step 1: Automated arm and takeoff using requestAutoState
  console.log("[STEP 1] Requesting automated takeoff to target altitude...");
  await droneController.requestAutoState({
    kind: "airborne",
    altMeters: TARGET_ALTITUDE
  });
  console.log("[STEP 1] Automated takeoff requested - controller will handle arming and takeoff\n");

  // Step 2: Record home position and calculate opposite target
  const homeState = await droneState.getState();
  const home = {
    x: homeState.pose?.position?.x || 0,
    y: homeState.pose?.position?.y || 0
  };

  const target = {
    x: home.x - WAYPOINT_OFFSET_X,  // Opposite horizontally (multiply by -1)
    y: home.y - WAYPOINT_OFFSET_Y,  // Multiply by -1 as requested
    z: TARGET_ALTITUDE
  };

  console.log(`[STEP 2] Home: (${home.x.toFixed(2)}, ${home.y.toFixed(2)})`);
  console.log(`[STEP 2] Opposite Target: (${target.x.toFixed(2)}, ${target.y.toFixed(2)}, ${target.z.toFixed(2)})\n`);

  // Step 3: Enter OFFBOARD mode by setting position target to opposite waypoint
  console.log("[STEP 3] Entering OFFBOARD mode with position target to opposite waypoint...");
  await droneController.requestAutoState({
    kind: "offboard",
    target: {
      kind: "position_local",
      x: target.x,
      y: target.y,
      z: target.z
    }
  });
  console.log("[STEP 3] Offboard position target set - controller will automatically switch to OFFBOARD mode and stream setpoints\n");

  console.log("[STEP 3] OFFBOARD mode active\n");

  // Step 4: Wait for arrival at waypoint (requestAutoState will wait until position reached)
  console.log(`[STEP 4] Navigating to opposite waypoint...`);

  // The requestAutoState already waits for the position to be reached due to isInRequestedAutoState check
  console.log(`[STEP 4] Arrived at opposite waypoint!\n`);

  // Wait a bit for the drone to stabilize
  console.log("[STEP 4] Waiting for drone to stabilize at opposite waypoint...");
  await sleep(2000);

  // Step 5: Clear offboard target to exit OFFBOARD mode
  console.log("[STEP 5] Clearing offboard target to exit OFFBOARD mode...");
  droneController.clearAutoState();
  console.log("[STEP 5] Offboard target cleared\n");

  // Step 6: Automated landing and disarm using requestAutoState
  console.log("[STEP 6] Requesting automated landing and disarm...");
  await droneController.requestAutoState({
    kind: "landed",
    armed: false
  });
  console.log("[STEP 6] Automated landing and disarm requested - controller will handle landing and disarming\n");

  console.log("\n[SUCCESS] Automated OFFBOARD waypoint navigation sequence finished!");
}
