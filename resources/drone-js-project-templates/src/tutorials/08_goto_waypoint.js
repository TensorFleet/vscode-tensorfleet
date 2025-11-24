#!/usr/bin/env node
/**
 * Tutorial 08: Go to Waypoint
 * 
 * Learn: Position-based navigation and closed-loop control
 * 
 * This script demonstrates:
 * - Reading current position
 * - Calculating velocity to target
 * - Closed-loop control (feedback)
 * - Waypoint arrival detection
 * 
 * Run: bun src/tutorials/08_goto_waypoint.js
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
const WAYPOINT_OFFSET_X = 5.0; // meters forward
const WAYPOINT_OFFSET_Y = 5.0; // meters right
const WAYPOINT_RADIUS = 1.0; // meters
const MAX_VELOCITY = 2.0; // m/s
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

    // Record home position
    const home = {
        x: telemetry.pose.pose.position.x,
        y: telemetry.pose.pose.position.y
    };

    const target = {
        x: home.x + WAYPOINT_OFFSET_X,
        y: home.y + WAYPOINT_OFFSET_Y
    };

    console.log(`\n[WAYPOINT] Home: (${home.x.toFixed(2)}, ${home.y.toFixed(2)})`);
    console.log(`[WAYPOINT] Target: (${target.x.toFixed(2)}, ${target.y.toFixed(2)})\n`);

    // Create velocity publisher
    const velPub = new ROSLIB.Topic({
        ros,
        name: "/mavros/setpoint_velocity/cmd_vel",
        messageType: "geometry_msgs/TwistStamped"
    });

    // Enter OFFBOARD mode
    console.log("");
    await enterOffboard(ros, velPub, SETPOINT_HZ);

    // Navigate to waypoint
    console.log("\n[NAV] Navigating to waypoint...\n");

    const intervalMs = 1000 / SETPOINT_HZ;
    let arrived = false;

    while (!arrived) {
        const pos = telemetry.pose.pose.position;
        const dx = target.x - pos.x;
        const dy = target.y - pos.y;
        const distance = Math.sqrt(dx * dx + dy * dy);

        console.log(`[NAV] Distance to target: ${distance.toFixed(2)}m`);

        if (distance < WAYPOINT_RADIUS) {
            console.log("\n[NAV] Arrived at waypoint!\n");
            arrived = true;
            break;
        }

        // Calculate velocity (proportional control)
        const speed = Math.min(MAX_VELOCITY, distance * 0.5);
        const vx = (dx / distance) * speed;
        const vy = (dy / distance) * speed;

        const vel = new ROSLIB.Message({
            header: { frame_id: "map" },
            twist: {
                linear: { x: vx, y: vy, z: 0.0 },
                angular: { x: 0.0, y: 0.0, z: 0.0 }
            }
        });

        velPub.publish(vel);
        await sleep(intervalMs);
    }

    // Stop
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
