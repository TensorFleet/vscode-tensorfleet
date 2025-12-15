#!/usr/bin/env -S bun run
/**
 * Tutorial 01: Basic Connection
 *
 * Learn: How to connect to rosbridge and read drone state
 *
 * This script demonstrates:
 * - Connecting to rosbridge WebSocket using ROSLibBridgeWrapper
 * - Subscribing to /mavros/state topic directly (raw ROS processing)
 * - Using DroneStateModel for managed state (shows utility of the library)
 * - Reading basic telemetry (connected, armed, mode) from both approaches
 *
 * Run: bun src/tutorials/01_connect.js
 */

import "dotenv/config";
import { DroneStateModel } from "tensorfleet-util";
import { ROSLibBridgeWrapper } from "../lib/roslib-bridge-wrapper.js";
import ROSLIB from "roslib";

async function main() {
    // Create ROS bridge using the wrapper
    const bridge = new ROSLibBridgeWrapper();
    await bridge.waitForConnection();

    console.log("\n[INFO] Listening to drone state...\n");
    console.log("[INFO] Showing both raw ROS subscription via wrapper and managed DroneStateModel\n");

    // Subscribe to state topic via wrapper (educational: raw ROS processing)
    const unsubscribeRaw = bridge.subscribe(
        { topic: "/mavros/state", type: "mavros_msgs/State" },
        (msg) => {
            console.log("=== RAW ROS SUBSCRIPTION VIA WRAPPER ===");
            console.log("Drone State:");
            console.log(`  Connected: ${msg.connected}`);
            console.log(`  Armed:     ${msg.armed}`);
            console.log(`  Mode:      ${msg.mode}`);
            console.log(`  Guided:    ${msg.guided}`);
            console.log("");
        }
    );

    // Create drone state model for comparison (shows utility)
    const droneState = new DroneStateModel();
    droneState.connect(bridge);

    droneState.onUpdate((state) => {
        if (state.vehicle) {
            console.log("=== MANAGED DRONE STATE MODEL ===");
            console.log("Drone State:");
            console.log(`  Connected: ${state.vehicle.connected}`);
            console.log(`  Armed:     ${state.vehicle.armed}`);
            console.log(`  Mode:      ${state.vehicle.mode}`);
            console.log(`  Guided:    ${state.vehicle.guided}`);
            console.log("Note: DroneStateModel provides unified state management and automatic health monitoring");
            console.log("");
        }
    });

    // Keep running until Ctrl+C
    console.log("Press Ctrl+C to exit\n");

    process.on("SIGINT", () => {
        console.log("\n[EXIT] Shutting down...");
        stateSub.unsubscribe();
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
