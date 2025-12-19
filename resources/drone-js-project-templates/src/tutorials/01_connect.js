#!/usr/bin/env -S bun run
/**
 * Tutorial 01: Connecting to the Drone
 *
 * This tutorial shows how to connect to a drone.
 *
 * First, we need to connect to the ROS environment using rosbridge (websocket) or direct ROS connection.
 *
 * Then, we can access drone information by subscribing to ROS topics and control the drone by ROS service calls and publishing to specific ROS topics.
 *
 * Run: bun src/tutorials/01_connect.js
 */

import { DroneStateModel } from "tensorfleet-util";
import { ROSLibBridgeWrapper } from "../lib/roslib-bridge-wrapper.js";
import ROSLIB from "roslib";

async function main() {
    // Create ROS bridge using the wrapper
    const bridge = new ROSLibBridgeWrapper();
    await bridge.waitForConnection();

    console.log("\n[INFO] Listening to drone state...\n");
    console.log("[INFO] Showing both raw ROS subscription via wrapper and managed DroneStateModel\n");

    // Variables to track previous state for change detection
    let previousConnected = null;
    let previousArmed = null;
    let previousMode = null;
    let previousGuided = null;

    // Subscribe to state topic via wrapper (educational: raw ROS processing)
    const unsubscribeRaw = bridge.subscribe(
        { topic: "/mavros/state", type: "mavros_msgs/State" },
        (msg) => {
            const currentConnected = msg.connected;
            const currentArmed = msg.armed;
            const currentMode = msg.mode;
            const currentGuided = msg.guided;

            // Only print on change or first time
            if (
                previousConnected !== currentConnected ||
                previousArmed !== currentArmed ||
                previousMode !== currentMode ||
                previousGuided !== currentGuided ||
                previousConnected === null
            ) {
                console.log("=== RAW ROS SUBSCRIPTION RECEIVED CHANGED DATA ===");
                console.log("Drone State:");
                console.log(`  Connected: ${currentConnected}`);
                console.log(`  Armed:     ${currentArmed}`);
                console.log(`  Mode:      ${currentMode}`);
                console.log(`  Guided:    ${currentGuided}`);
                console.log("");

                // Update previous values
                previousConnected = currentConnected;
                previousArmed = currentArmed;
                previousMode = currentMode;
                previousGuided = currentGuided;
            }
        }
    );

    // Create drone state model for comparison (shows utility)
    const droneState = new DroneStateModel();
    droneState.connect(bridge);

    droneState.onStatusUpdate((state) => {
        console.log("=== MANAGED DRONE STATE MODEL UPDATE ===");
        if (state.vehicle) {
            console.log("vehicle :\n", state.vehicle);
        }

        if (state.status) {
            console.log("status: \n", state.status);
        }
    });

    // Keep running until Ctrl+C
    console.log("Press Ctrl+C to exit\n");

    process.on("SIGINT", () => {
        console.log("\n[EXIT] Shutting down...");
        unsubscribeRaw();
        droneState.disconnect();
        process.exit(0);
    });
}

if (require.main === module) {
    main().catch((err) => {
        console.error("[ERROR]", err.message || err);
        process.exit(1);
    });
}
