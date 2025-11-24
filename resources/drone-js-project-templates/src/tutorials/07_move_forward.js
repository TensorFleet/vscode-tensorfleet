#!/usr/bin/env node
/**
 * Tutorial 07: Move Forward
 * 
 * Learn: Velocity control and timing-based movement
 * 
 * This script demonstrates:
 * - Setting forward velocity (vx)
 * - Time-based movement (5 seconds = ~5 meters)
 * - Stopping with zero velocity
 * - Simple open-loop control
 * 
 * Run: bun src/tutorials/07_move_forward.js
 */

require("dotenv").config();
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

const R2B_HOST = process.env.R2B_HOST || process.env.ROS_HOST || "172.16.0.10";
const R2B_PORT = process.env.R2B_PORT || process.env.ROS_PORT || "9091";
const url = process.env.ROSBRIDGE_URL || `ws://${R2B_HOST}:${R2B_PORT}`;

const TARGET_ALTITUDE = 3.0; // meters
const FORWARD_VELOCITY = 1.0; // m/s
const MOVE_DURATION = 5.0; // seconds
const SETPOINT_HZ = 20;

async function main() {

    const ros = await connectToDrone(url);
    const { telemetry } = await waitForTelemetry(ros);

    // Arm and takeoff
    if (!telemetry.state.armed) {
        await armDrone(ros, telemetry.state);
    }

    console.log("");
    await takeoffToAlt(
        ros,
        telemetry.state,
        telemetry.fix,
        telemetry.altitude,
        TARGET_ALTITUDE
    );

    // Create velocity publisher
    const velPub = new ROSLIB.Topic({
        ros,
        name: "/mavros/setpoint_velocity/cmd_vel",
        messageType: "geometry_msgs/TwistStamped"
    });

    // Enter OFFBOARD mode
    console.log("");
    await enterOffboard(ros, velPub, SETPOINT_HZ);

    // Move forward
    console.log(`\n[MOVE] Moving forward at ${FORWARD_VELOCITY} m/s for ${MOVE_DURATION}s`);
    console.log(`[MOVE] Expected distance: ~${FORWARD_VELOCITY * MOVE_DURATION} meters\n`);

    const forwardVel = new ROSLIB.Message({
        header: { frame_id: "map" },
        twist: {
            linear: { x: FORWARD_VELOCITY, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: 0.0 }
        }
    });

    const endTime = Date.now() + MOVE_DURATION * 1000;
    const intervalMs = 1000 / SETPOINT_HZ;

    while (Date.now() < endTime) {
        velPub.publish(forwardVel);
        await sleep(intervalMs);
    }

    // Stop
    console.log("[MOVE] Stopping...\n");
    const zeroVel = new ROSLIB.Message({
        header: { frame_id: "map" },
        twist: {
            linear: { x: 0.0, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: 0.0 }
        }
    });

    for (let i = 0; i < SETPOINT_HZ; i++) {
        velPub.publish(zeroVel);
        await sleep(intervalMs);
    }

    console.log("[MOVE] Movement complete\n");

    // Land
    await landDrone(ros, telemetry.state, SETPOINT_HZ);

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
