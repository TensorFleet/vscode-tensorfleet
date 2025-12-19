#!/usr/bin/env -S bun run
/**
 * Tutorial 04: Takeoff and Land with Auto State Management
 *
 * This tutorial demonstrates:
 * - Manual takeoff and landing operations
 * - Automatic state management that maintains target states
 * - Using requestAutoState() with auto-enforcement and built-in waiting
 * - Proper state transitions with automatic completion detection
 *
 * Manual Sequence:
 * 1. Arm if needed , land, and disarm
 * 2. Takeoff
 * 3. Wait for takeoff to finish
 * 4. Land
 * 5. Wait for land to finish
 * 6. Disarm
 *
 * Automatic sequence (full async):
 * 1. Command drone to airborne state (auto-waits for completion)
 * 2. Command drone to landed and disarmed state (auto-waits for completion)
 *
 * Run: bun src/tutorials/04_takeoff_land.js
 */

import { DroneStateModel, DroneController } from "tensorfleet-util";
import { initializeDroneControl } from "../lib/drone_utils.js";
import { LANDED } from "tensorfleet-util";

const TARGET_ALTITUDE = 3.0; // meters

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

async function main() {
    // Create ROS bridge using our wrapper
    let { bridge, droneState, droneController, currentState } = await initializeDroneControl();
    
    console.log(`[INFO] Drone connected. Current state: armed=${currentState.vehicle.armed}, mode=${currentState.vehicle.mode}, landed=${currentState.extended?.landed_state}\n`);

    await performManualSequence(droneState, droneController, TARGET_ALTITUDE);

    console.log("[INFO] Manual sequence finished.\nNow, we will try out the automatic sequence after 5 seconds...");
    await sleep(5000);

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

    // This is a combined check. It checks if the drone is armed and is neither landed, landing or taking off.
    // Why is this needed? "AUTO.LOITER" flight mode is possible while being on ground.
    // We also have a delayed state. So when you send the takeoff request, there is no guarantee that the state we have will reflect that.
    while(!await droneState.isAirborne()) {
        console.log("[INFO] waiting for takeoff to finish...");
        await sleep(1500);
    }

    currentState = await droneState.getState();

    console.log(`[INFO] Takeoff sequence finished. armed=${currentState.vehicle.armed}, mode=${currentState.vehicle.mode}, landed=${currentState.extended?.landed_state}\n`);

    const relAlt = currentState.altitude?.relative || 0;

    console.log("[INFO] Altitude :", relAlt);

    console.log("\n[SUCCESS] Takeoff complete! Drone is hovering at altitude.");

    // Wait a moment before landing
    console.log("[INFO] Waiting 5 seconds before landing...\n");
    await sleep(5000);

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
    // Step 1: Automatic state management is running
    // The controller will automatically maintain target states once we set one
    console.log("[STEP 1] Automatic state management is running - will activate once we set a target state");

    console.log("[STEP 1] Waiting 5 seconds before setting target state to airborne...");
    await sleep(5000);

    // Step 1: Command drone to airborne state at target altitude
    // Auto state management will handle the transition and wait for completion
    console.log("[STEP 1] Setting target state: airborne at 3m altitude...");
    await droneController.requestAutoState({ kind: "airborne", altMeters: TARGET_ALTITUDE });
    console.log("[STEP 1] Target state reached - drone is now airborne at target altitude");

    // Step 1: Observe the final state
    let currentState = await droneState.getState();
    const alt = currentState.altitude?.relative || 0;
    console.log(`[STEP 1] Current state: altitude=${alt.toFixed(2)}m, armed=${currentState.vehicle?.armed}, mode=${currentState.vehicle?.mode}`);

    console.log("[STEP 2] Waiting 6 seconds before setting target state: landed and disarmed...");
    await sleep(6000);

    // Step 2: Command drone to landed and disarmed state
    // Auto state management will handle the transition and wait for completion
    console.log("[STEP 2] Setting target state: landed and disarmed...");
    await droneController.requestAutoState({ kind: "landed", armed: false });
    console.log("[STEP 2] Target state reached - drone is now landed and disarmed");
    console.log("\n[SUCCESS] Drone has landed and disarmed!");
}
