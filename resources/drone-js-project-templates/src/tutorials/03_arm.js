#!/usr/bin/env -S bun run
/**
 * Tutorial 03: Arm / Disarm the Drone
 *
 * This tutorial demonstrates fundamental drone safety and control operations using MAVROS services.
 * It showcases the use of our ROSLibBridgeWrapper and DroneController from tensorfleet-util
 * to safely arm and disarm the drone, with proper state monitoring and error handling.
 *
 * Learning Objectives:
 * - Use ROSLibBridgeWrapper for ROS Bridge communication
 * - Utilize DroneController for high-level arm/disarm operations
 * - Monitor drone state changes during arming/disarming
 * - Understand MAVROS service calls and their internal workings
 * - Learn about automatic arming/disarming behavior during flight operations
 * - Understand safety mechanisms that prevent unsafe disarming
 *
 * Key Concepts:
 * - MAVROS Services: ROS service calls for drone control commands
 * - Arming State: Critical safety state that enables motor control
 * - Service Calls: Synchronous communication pattern vs. topic publishing
 * - Safety Mechanisms: Built-in protections against unsafe operations
 * - Automatic Transitions: How takeoff and landing affect arming state
 *
 * Prerequisites:
 * - Active TensorFleet VM with MAVROS running
 * - Simulation restarted (as described in TUTORIAL.md)
 * - Understanding of connection and telemetry from Tutorials 01-02
 *
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
    await new Promise(resolve => {
        const checkState = () => {
            const state = droneState.getState();
            if (state.vehicle?.connected) {
                console.log(`[INFO] Drone connected. Current state: armed=${state.vehicle.armed}, mode=${state.vehicle.mode}\n`);
                resolve();
            } else {
                setTimeout(checkState, 100);
            }
        };
        checkState();
    });

    const currentState = droneState.getState();

    // Toggle arm/disarm based on current state
    if (!currentState.vehicle?.armed) {
        console.log("[INFO] Drone is currently disarmed. Arming...\n");

        // Arm the drone using DroneController
        await droneController.arm();

        console.log("[SUCCESS] Drone arm command sent successfully!");
        console.log("Waiting for arm confirmation...");

        // Wait for armed state
        await new Promise((resolve, reject) => {
            const timeout = setTimeout(() => reject(new Error("Timeout waiting for armed state")), 5000);
            const checkArmed = () => {
                const state = droneState.getState();
                if (state.vehicle?.armed) {
                    clearTimeout(timeout);
                    console.log("[SUCCESS] Drone is now armed!\n");
                    resolve();
                } else {
                    setTimeout(checkArmed, 100);
                }
            };
            checkArmed();
        });

    } else {
        console.log("[INFO] Drone is currently armed. Disarming...\n");

        // Disarm the drone using DroneController
        await droneController.disarm();

        console.log("[SUCCESS] Drone disarm command sent successfully!");
        console.log("Waiting for disarm confirmation...");

        // Wait for disarmed state
        await new Promise((resolve, reject) => {
            const timeout = setTimeout(() => reject(new Error("Timeout waiting for disarmed state")), 5000);
            const checkDisarmed = () => {
                const state = droneState.getState();
                if (!state.vehicle?.armed) {
                    clearTimeout(timeout);
                    console.log("[SUCCESS] Drone is now disarmed!\n");
                    resolve();
                } else {
                    setTimeout(checkDisarmed, 100);
                }
            };
            checkDisarmed();
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
