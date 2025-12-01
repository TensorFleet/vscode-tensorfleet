#!/usr/bin/env node
/**
 * Tutorial 01: Basic Connection
 * 
 * Learn: How to connect to rosbridge and read drone state
 * 
 * This script demonstrates:
 * - Connecting to rosbridge WebSocket
 * - Subscribing to /mavros/state topic
 * - Reading basic telemetry (connected, armed, mode)
 * 
 * Run: bun src/tutorials/01_connect.js
 */

require("dotenv").config();
const { connectToDrone } = require("../lib/drone_utils");
const ROSLIB = require("roslib");

const R2B_HOST = process.env.R2B_HOST || process.env.ROS_HOST || "172.16.0.10";
const R2B_PORT = process.env.R2B_PORT || process.env.ROS_PORT || "9091";
const url = process.env.ROSBRIDGE_URL || `ws://${R2B_HOST}:${R2B_PORT}`;

async function main() {

    // Connect to rosbridge
    const ros = await connectToDrone(url);

    // Subscribe to state topic
    const stateSub = new ROSLIB.Topic({
        ros,
        name: "/mavros/state",
        messageType: "mavros_msgs/State"
    });

    console.log("\n[INFO] Listening to drone state...\n");

    stateSub.subscribe((msg) => {
        console.log("Drone State:");
        console.log(`  Connected: ${msg.connected}`);
        console.log(`  Armed:     ${msg.armed}`);
        console.log(`  Mode:      ${msg.mode}`);
        console.log(`  Guided:    ${msg.guided}`);
        console.log("");
    });

    // Keep running until Ctrl+C
    console.log("Press Ctrl+C to exit\n");

    process.on("SIGINT", () => {
        console.log("\n[EXIT] Shutting down...");
        stateSub.unsubscribe();
        ros.close();
        process.exit(0);
    });
}

if (require.main === module) {
    main().catch((err) => {
        console.error("[ERROR]", err.message || err);
        process.exit(1);
    });
}
