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
const { getTensorfleetSettings } = require("../lib/tensorfleet_config");
const ROSLIB = require("roslib");

const { rosbridgeUrl } = getTensorfleetSettings();

async function main() {

    const ros = await connectToDrone(rosbridgeUrl);

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
