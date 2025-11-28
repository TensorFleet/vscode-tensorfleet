#!/usr/bin/env node
/**
 * Tutorial 03: Arm / Disarm the Drone
 * 
 * Learn: Service calls and arm/disarm requirements
 * 
 * This script demonstrates:
 * - Calling /mavros/cmd/arming service (arm + disarm)
 * - Waiting for state changes
 * - Checking arm/disarm success
 * 
 * Run: bun src/tutorials/03_arm.js
 */

require("dotenv").config();
const {
    connectToDrone,
    waitForTelemetry,
    armDrone,
    disarmDrone
} = require("../lib/drone_utils");

const R2B_HOST = process.env.R2B_HOST || process.env.ROS_HOST || "172.16.0.10";
const R2B_PORT = process.env.R2B_PORT || process.env.ROS_PORT || "9091";
const url = process.env.ROSBRIDGE_URL || `ws://${R2B_HOST}:${R2B_PORT}`;

async function main() {

    const ros = await connectToDrone(url);
    const { telemetry } = await waitForTelemetry(ros);

    console.log(`\n[INFO] Current state: armed=${telemetry.state.armed}\n`);

    if (!telemetry.state.armed) {
        console.log("[INFO] Drone is currently disarmed. Arming...\n");
        await armDrone(ros, telemetry);
        console.log("\n[SUCCESS] Drone is now armed!");
    } else {
        console.log("[INFO] Drone is currently armed. Disarming...\n");
        await disarmDrone(ros, telemetry);
        console.log("\n[SUCCESS] Drone is now disarmed!");
    }

    console.log("\n[EXIT] Closing connection...");
    ros.close();
}

if (require.main === module) {
    main().catch((err) => {
        console.error("[ERROR]", err.message || err);
        process.exit(1);
    });
}
