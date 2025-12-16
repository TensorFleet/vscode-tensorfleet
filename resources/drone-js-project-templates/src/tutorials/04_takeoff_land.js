#!/usr/bin/env -S bun run
/**
 * Tutorial 04: Takeoff and Land with Auto State Management
 *
 * This tutorial demonstrates:
 * - Manual takeoff and landing operations
 * - Enabling automatic state management that maintains target states
 * - Using setRequestedState() with auto-enforcement for takeoff and landing
 * - Proper waiting for state transitions to complete
 *
 * Sequence:
 * 1. Manual sequence: Arm, takeoff to 3m, land, and disarm
 * 2. Enable automatic state management (maintains target state automatically)
 * 3. Command drone to become airborne at 3m altitude using setRequestedState
 * 4. Wait for takeoff to complete with automatic state management
 * 5. Wait while drone maintains altitude
 * 6. Command drone to land and disarm using setRequestedState
 * 7. Wait for landing and disarming to complete
 * 8. Disable auto state management and clean up
 *
 * Run: bun src/tutorials/04_takeoff_land.js
 */

import { DroneStateModel, DroneController } from "tensorfleet-util";
import { ROSLibBridgeWrapper } from "../lib/roslib-bridge-wrapper.js";
import { LANDED } from "tensorfleet-util";

const TARGET_ALTITUDE = 3.0; // meters

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

async function performManualSequence(droneState, droneController, TARGET_ALTITUDE) {
    let currentState;
    console.log("[INFO] Executing Manual arm/land sequence...\n");

    // while state.vehicle?.armed is false
    while(!await droneState.isArmed()) {
        console.log("[INFO] Drone is not armed. Arming...\n");

        // Arm the drone using DroneController
        
        await droneController.arm();
        console.log("Waiting for arm confirmation...");

        // Use low-latency selective change listener for immediate notification
        await sleep(1500);
    }

    console.log("[INFO] Manual drone arm check finished\n");

    // Takeoff
    console.log("[INFO] Initiating takeoff...");
    await droneController.takeoff(TARGET_ALTITUDE);

    console.log("[INFO] Takeoff command sent, waiting for altitude...");

    // this.state.vehicle?.armed && this.state.vehicle?.mode === "AUTO.TAKEOFF"
    while(await droneState.isTakingOff()) {
        console.log("[INFO] waiting for takeoff to finish...");
        await sleep(1500);
    }

    currentState = await droneState.getState();

    console.log(`[INFO] Takeoff sequence finished. armed=${currentState.vehicle.armed}, mode=${currentState.vehicle.mode}, landed=${currentState.extended?.landed_state}\n`);

    const relAlt = currentState.altitude?.relative || 0;

    console.log("[INFO] Altitude :", relAlt);

    console.log("\n[SUCCESS] Takeoff complete! Drone is hovering at altitude.");

    // Wait a moment before landing
    console.log("[INFO] Waiting 2 seconds before landing...\n");
    await new Promise(resolve => setTimeout(resolve, 2000));

    // Land
    console.log("[INFO] Landing drone...\n");
    await droneController.land();

    console.log("[INFO] Landing command sent, waiting for disarm...");

    while(!await droneState.isLanded()) {
        console.log("[INFO] waiting for landing to finish...");
        await sleep(1500);
    }

    currentState = await droneState.getState();

    console.log(`[INFO] Drone has landed succesfully. armed=${currentState.vehicle.armed}, mode=${currentState.vehicle.mode}, landed=${currentState.extended?.landed_state}\n`);

    

    if(await droneState.isArmed()) {
        console.log("[INFO] Vehicle still armed. Disarming drone");

        await droneController.disarm();

        await sleep(1500);

        while(await droneState.isArmed()) {
            console.log("[INFO] Vehicle still armed. Waiting for disarm sequence to finish");
            await sleep(1500)
        }
    }

    console.log("\n[SUCCESS] Drone has landed and disarmed!");

    console.log("\n[SUCCESS] !!! MANUAL SEQUENCE FINISHED !!!");
}

async function performAutomaticSequence(droneState, droneController, TARGET_ALTITUDE) {
    let currentState;
    // Step 1: Enable automatic state management
    // This enables the controller to automatically maintain target states
    // The system will derive initial target from current drone state
    console.log("[STEP 1] Enabling automatic state management...");
    await droneController.enableAutoStateManagement(true);
    console.log("[STEP 1] Auto state management enabled - controller will now maintain target states automatically");

    // Step 2: Command drone to become airborne at 3m altitude
    // Auto state management will arm, takeoff, and maintain altitude
    console.log("[STEP 2] Setting target state: airborne at 3m altitude...");
    droneController.setRequestedState({ kind: "airborne", altMeters: TARGET_ALTITUDE });
    console.log("[STEP 2] Target state set to", droneController.requestedState, " - auto management will handle arming and takeoff");

    // Step 3: Wait for takeoff to complete with automatic state management
    console.log("[STEP 3] Waiting for drone to reach and maintain target altitude...");
    await droneController.waitForRequestedState();
    console.log("[STEP 3] Takeoff completed successfully - drone is now airborne at target altitude");

    currentState = await droneState.getState();
    const currentAlt = currentState.altitude?.relative || 0;
    console.log(`[STEP 3] Current altitude: ${currentAlt.toFixed(2)}m, armed=${currentState.vehicle?.armed}, mode=${currentState.vehicle?.mode}`);

    // Step 4: Wait while drone maintains altitude
    console.log("[STEP 4] Maintaining altitude for 5 seconds...");
    await sleep(5000);
    currentState = await droneState.getState();
    const maintainedAlt = currentState.altitude?.relative || 0;
    console.log(`[STEP 4] Altitude maintained: ${maintainedAlt.toFixed(2)}m`);

    // Step 5: Command drone to land and disarm
    // Auto state management will land the drone and then disarm it
    console.log("[STEP 5] Setting target state: landed and disarmed...");
    droneController.setRequestedState({ kind: "landed", armed: false });
    console.log("[STEP 5] Target state set - auto management will handle landing and disarming");

    // Step 6: Wait for the landing and disarming process to complete
    // This ensures the script doesn't exit prematurely
    console.log("[STEP 6] Waiting for landing and disarming to complete...");
    await droneController.waitForRequestedState();
    console.log("[STEP 6] Landing and disarming completed successfully");
    console.log("\n[SUCCESS] Drone has landed and disarmed with automatic state management!");

    // Step 7: Disable auto state management
    // Clean shutdown of automatic control systems
    console.log("[STEP 7] Disabling auto state management...");
    await droneController.enableAutoStateManagement(false);
    console.log("[STEP 7] Auto state management disabled");

    console.log("\n[STEP 7] Automatic state management sequence completed successfully.\n");
}

async function main() {
    // Create ROS bridge using our wrapper
    const bridge = new ROSLibBridgeWrapper();
    await bridge.waitForConnection();

    console.log(`\n[INFO] Target altitude: ${TARGET_ALTITUDE}m\n`);

    // Create managed state model for state monitoring
    const droneState = new DroneStateModel();
    droneState.connect(bridge);

    // Create drone controller for high-level operations
    const droneController = new DroneController(droneState, bridge);

    console.log("[INFO] Waiting for drone state...");

    // This will automatically wait for initial state
    let currentState = await droneState.getState();
    
    console.log(`[INFO] Drone connected. Current state: armed=${currentState.vehicle.armed}, mode=${currentState.vehicle.mode}, landed=${currentState.extended?.landed_state}\n`);

    // await performManualSequence(droneState, droneController, TARGET_ALTITUDE);

    await performAutomaticSequence(droneState, droneController, TARGET_ALTITUDE);


    // Clean up connections
    currentState = await droneState.getState();
    console.log(`[EXIT] Final drone state: armed=${currentState.vehicle?.armed}, mode=${currentState.vehicle?.mode}`);
    droneState.disconnect();
    console.log("[EXIT] Disconnected from drone state monitoring.");
}

if (require.main === module) {
    main().catch((err) => {
        console.error("[ERROR]", err.message || err);
        process.exit(1);
    });
}
