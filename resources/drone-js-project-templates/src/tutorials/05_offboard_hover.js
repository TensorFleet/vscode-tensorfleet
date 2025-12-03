#!/usr/bin/env node
/**
 * Tutorial 05: OFFBOARD Hover
 * 
 * Learn: OFFBOARD mode basics and velocity streaming
 * 
 * This script demonstrates:
 * - Standalone operation: automatically arms and takes off if needed
 * - Entering OFFBOARD mode
 * - Streaming velocity setpoints (zero = hover)
 * - Maintaining OFFBOARD with continuous updates
 * - Returning to land
 * 
 * Run: bun src/tutorials/05_offboard_hover.js
 */

require("dotenv").config();
const R2B_HOST = process.env.R2B_HOST || process.env.ROS_HOST || "172.16.0.10";
const R2B_PORT = process.env.R2B_PORT || process.env.ROS_PORT || "9091";
const url = process.env.ROSBRIDGE_URL || `ws://${R2B_HOST}:${R2B_PORT}`;
const {
    connectToDrone,
    waitForTelemetry,
    armDrone,
    takeoffToAlt,
    enterOffboard,
    landDrone,
    sleep
} = require("../lib/drone_utils");
const ROSLIB = require("roslib");

const TARGET_ALTITUDE = 3.0; // meters
const HOVER_DURATION = 10.0; // seconds
const SETPOINT_HZ = 20;

async function main() {
    const ros = await connectToDrone(url);
    const { telemetry } = await waitForTelemetry(ros);

    // Arm and takeoff
    if (!telemetry.state.armed) {
        await armDrone(ros, telemetry);
    }

    console.log("");
    await takeoffToAlt(ros, telemetry, TARGET_ALTITUDE);

    // Create velocity publisher
    const velPub = new ROSLIB.Topic({
        ros,
        name: "/mavros/setpoint_velocity/cmd_vel",
        messageType: "geometry_msgs/TwistStamped"
    });

    // Enter OFFBOARD mode
    console.log("");
    await enterOffboard(ros, velPub, SETPOINT_HZ);

    // Hover in OFFBOARD for specified duration
    console.log(`\n[HOVER] Hovering in OFFBOARD for ${HOVER_DURATION}s...`);
    console.log("[HOVER] Streaming zero velocities to maintain position\n");

    const zeroVel = new ROSLIB.Message({
        header: { frame_id: "map" },
        twist: {
            linear: { x: 0.0, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: 0.0 }
        }
    });

    const endTime = Date.now() + HOVER_DURATION * 1000;
    const intervalMs = 1000 / SETPOINT_HZ;

    while (Date.now() < endTime) {
        velPub.publish(zeroVel);
        await sleep(intervalMs);
    }

    console.log("[HOVER] Hover complete\n");

    // Land
    await landDrone(ros, telemetry, SETPOINT_HZ);

    console.log("\n[SUCCESS] Mission complete!");
    console.log("[EXIT] Closing connection...");
    velPub.unadvertise();
    ros.close();
}

if (require.main === module) {
    main().catch((err) => {
        console.error("[ERROR]", err.message || err);
        process.exit(1);
    });
}

