#!/usr/bin/env node
/**
 * Tutorial 03: Arm the Drone
 * 
 * Learn: Service calls and arming requirements
 * 
 * This script demonstrates:
 * - Calling /mavros/cmd/arming service
 * - Waiting for state changes
 * - Checking arm success
 * 
 * Run: bun src/tutorials/03_arm.js
 */

require("dotenv").config();
const { connectToDrone, waitForTelemetry, armDrone } = require("../lib/drone_utils");

const R2B_HOST = process.env.R2B_HOST || process.env.ROS_HOST || "172.16.0.10";
const R2B_PORT = process.env.R2B_PORT || process.env.ROS_PORT || "9091";
const url = process.env.ROSBRIDGE_URL || `ws://${R2B_HOST}:${R2B_PORT}`;

async function main() {

    const ros = await connectToDrone(url);
    const { telemetry } = await waitForTelemetry(ros);

    console.log(`\n[INFO] Current state: armed=${telemetry.state.armed}\n`);

    if (telemetry.state.armed) {
        console.log("[INFO] Drone is already armed!");
    } else {
        console.log("[INFO] Arming drone...\n");
        await armDrone(ros, telemetry.state);
        console.log("\n[SUCCESS] Drone is now armed!");
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
