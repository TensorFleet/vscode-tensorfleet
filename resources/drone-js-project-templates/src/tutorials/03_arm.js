#!/usr/bin/env -S bun run
/**
 * Tutorial 03: Arm / Disarm the Drone
 *
 * Demonstrates drone arming/disarming with the new low-latency selective change listener system.
 * Instead of periodic polling, this uses immediate notifications when vehicle state changes,
 * providing real-time responsiveness for critical safety operations.
 *
 * Key Features:
 * - MAVROS service calls for arm/disarm operations
 * - Low-latency state monitoring using onSectionChange('vehicle')
 * - Immediate notification of arming state changes
 * - Safety mechanisms and automatic transitions
 *
 * Prerequisites: Active TensorFleet VM, Tutorials 01-02 completed
 * Usage: bun run src/tutorials/03_arm.js
 */

import { DroneStateModel, DroneController } from "tensorfleet-util";
import { ROSLibBridgeWrapper } from "../lib/roslib-bridge-wrapper.js";

async function main() {
    // Create ROS bridge using our wrapper
    const bridge = new ROSLibBridgeWrapper();
    await bridge.waitForConnection();

    console.log("\n[INFO] Connected to ROS Bridge - initializing drone control...\n");

    // Create managed state model for state monitoring
    const droneState = new DroneStateModel();
    droneState.connect(bridge);

    // Create drone controller for high-level operations
    const droneController = new DroneController(droneState, bridge);

    // Wait for initial state
    console.log("[INFO] Waiting for drone state...");
    await new Promise(async resolve => {
        const checkState = async () => {
            const state = await droneState.getState();
            if (state.vehicle?.connected) {
                console.log(`[INFO] Drone connected. Current state: armed=${state.vehicle.armed}, mode=${state.vehicle.mode}\n`);
                resolve();
            } else {
                setTimeout(checkState, 100);
            }
        };
        checkState();
    });

    const currentState = await droneState.getState();

    // Toggle arm/disarm based on current state
    if (!currentState.vehicle?.armed) {
        console.log("[INFO] Drone is currently disarmed. Arming...\n");

        // Arm the drone using DroneController
        await droneController.arm();

        console.log("[SUCCESS] Drone arm command sent successfully!");
        console.log("Waiting for arm confirmation...");

        // Use low-latency selective change listener for immediate notification
        // This provides real-time responsiveness instead of polling every 100ms
        await new Promise(async (resolve, reject) => {
            const timeout = setTimeout(() => {
                unsubscribe(); // Clean up listener on timeout
                reject(new Error("Timeout waiting for armed state"));
            }, 5000);

            // Listen for changes in vehicle state section (armed, mode, etc.)
            const unsubscribe = droneState.onSectionChange('vehicle', (oldVal, newVal) => {
                if (newVal.armed && !oldVal.armed) {
                    clearTimeout(timeout);
                    unsubscribe();
                    console.log("[SUCCESS] Drone is now armed!\n");
                    resolve();
                }
            });
        });

    } else {
        console.log("[INFO] Drone is currently armed. Disarming...\n");

        // Disarm the drone using DroneController
        await droneController.disarm();

        console.log("[SUCCESS] Drone disarm command sent successfully!");
        console.log("Waiting for disarm confirmation...");

        // Use low-latency selective change listener for immediate notification
        await new Promise(async (resolve, reject) => {
            const timeout = setTimeout(() => {
                unsubscribe(); // Clean up listener on timeout
                reject(new Error("Timeout waiting for disarmed state"));
            }, 5000);

            // Listen for changes in vehicle state section (armed, mode, etc.)
            const unsubscribe = droneState.onSectionChange('vehicle', (oldVal, newVal) => {
                if (!newVal.armed && oldVal.armed) {
                    clearTimeout(timeout);
                    unsubscribe();
                    console.log("[SUCCESS] Drone is now disarmed!\n");
                    resolve();
                }
            });
        });
    }

    console.log("[INFO] Arming/Disarming operation completed successfully!\n");

    // Clean up
    droneState.disconnect();
    console.log("[EXIT] Disconnected from drone state monitoring.");
}

if (require.main === module) {
    main().catch((err) => {
        console.error("[ERROR]", err.message || err);
        process.exit(1);
    });
}
