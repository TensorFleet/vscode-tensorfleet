#!/usr/bin/env node
/**
 * Tutorial 05: Land Command
 * 
 * Learn: Landing sequence and disarm detection
 * 
 * This script demonstrates:
 * - Checking if drone is flying
 * - Sending AUTO.LAND mode change
 * - Monitoring altitude during descent
 * - Detecting disarm
 * 
 * Run: bun src/tutorials/05_land.js
 */

require("dotenv").config();
const { connectToDrone, waitForTelemetry, landDrone } = require("../lib/drone_utils");

const R2B_HOST = process.env.R2B_HOST || process.env.ROS_HOST || "172.16.0.10";
const R2B_PORT = process.env.R2B_PORT || process.env.ROS_PORT || "9091";
const url = process.env.ROSBRIDGE_URL || `ws://${R2B_HOST}:${R2B_PORT}`;

async function main() {

    const ros = await connectToDrone(url);
    const { telemetry } = await waitForTelemetry(ros);

    console.log(`\n[INFO] Current state: armed=${telemetry.state.armed}, mode=${telemetry.state.mode}\n`);

    if (!telemetry.state.armed) {
        console.log("[INFO] Drone is not armed (already on ground)");
        console.log("[EXIT] Nothing to do.");
        ros.close();
        return;
    }

    console.log("[INFO] Landing drone...\n");
    await landDrone(ros, telemetry.state);

    console.log("\n[SUCCESS] Drone has landed and disarmed!");
    console.log("[EXIT] Closing connection...");
    ros.close();
}

if (require.main === module) {
    main().catch((err) => {
        console.error("[ERROR]", err.message || err);
        process.exit(1);
    });
}
