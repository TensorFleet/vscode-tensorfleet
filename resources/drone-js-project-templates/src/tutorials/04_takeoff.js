#!/usr/bin/env node
/**
 * Tutorial 04: Takeoff Command
 * 
 * Learn: MAV commands and altitude monitoring
 * 
 * This script demonstrates:
 * - Arming the drone
 * - Sending MAV_CMD_NAV_TAKEOFF
 * - Monitoring altitude until target reached
 * - AUTO.LOITER mode
 * 
 * Run: bun src/tutorials/04_takeoff.js
 */

require("dotenv").config();
const {
    connectToDrone,
    waitForTelemetry,
    armDrone,
    takeoffToAlt
} = require("../lib/drone_utils");

const R2B_HOST = process.env.R2B_HOST || process.env.ROS_HOST || "172.16.0.10";
const R2B_PORT = process.env.R2B_PORT || process.env.ROS_PORT || "9091";
const url = process.env.ROSBRIDGE_URL || `ws://${R2B_HOST}:${R2B_PORT}`;

const TARGET_ALTITUDE = 3.0; // meters

async function main() {

    const ros = await connectToDrone(url);
    const { telemetry } = await waitForTelemetry(ros);

    console.log(`\n[INFO] Target altitude: ${TARGET_ALTITUDE}m\n`);

    // Arm if not already armed
    if (!telemetry.state.armed) {
        await armDrone(ros, telemetry.state);
    } else {
        console.log("[INFO] Drone already armed");
    }

    // Takeoff
    console.log("");
    await takeoffToAlt(
        ros,
        telemetry.state,
        telemetry.fix,
        telemetry.altitude,
        TARGET_ALTITUDE
    );

    console.log("\n[SUCCESS] Takeoff complete! Drone is hovering at altitude.");
    console.log("[INFO] Drone will remain in AUTO.LOITER mode.");
    console.log("[INFO] Use tutorial 05_land.js to land.\n");

    console.log("[EXIT] Closing connection (drone stays in air)...");
    ros.close();
}

if (require.main === module) {
    main().catch((err) => {
        console.error("[ERROR]", err.message || err);
        process.exit(1);
    });
}
