#!/usr/bin/env node
/**
 * Tutorial 02: Read All Telemetry
 * 
 * Learn: Available topics and message formats
 * 
 * This script demonstrates:
 * - Subscribing to multiple telemetry topics
 * - Reading state, position, GPS, altitude, battery
 * - Formatting and displaying data
 * 
 * Run: bun src/tutorials/02_telemetry.js
 */

require("dotenv").config();
const { connectToDrone, waitForTelemetry } = require("../lib/drone_utils");
const ROSLIB = require("roslib");

const R2B_HOST = process.env.R2B_HOST || process.env.ROS_HOST || "172.16.0.10";
const R2B_PORT = process.env.R2B_PORT || process.env.ROS_PORT || "9091";
const url = process.env.ROSBRIDGE_URL || `ws://${R2B_HOST}:${R2B_PORT}`;

async function main() {

    const ros = await connectToDrone(url);
    const { telemetry, subscriptions } = await waitForTelemetry(ros);

    // Also subscribe to battery
    const batterySub = new ROSLIB.Topic({
        ros,
        name: "/mavros/battery",
        messageType: "sensor_msgs/BatteryState"
    });

    let battery = null;
    batterySub.subscribe((msg) => {
        battery = msg;
    });

    console.log("\n[INFO] Displaying telemetry (updates every 1 second)\n");
    console.log("Press Ctrl+C to exit\n");

    const interval = setInterval(() => {
        console.clear();
        console.log("=== DRONE TELEMETRY ===\n");

        // State
        console.log("STATE:");
        console.log(`  Connected: ${telemetry.state?.connected}`);
        console.log(`  Armed:     ${telemetry.state?.armed}`);
        console.log(`  Mode:      ${telemetry.state?.mode}`);
        console.log("");

        // Position (local)
        const pos = telemetry.pose?.pose?.position;
        console.log("LOCAL POSITION:");
        console.log(`  X: ${pos?.x?.toFixed(2)} m`);
        console.log(`  Y: ${pos?.y?.toFixed(2)} m`);
        console.log(`  Z: ${pos?.z?.toFixed(2)} m`);
        console.log("");

        // GPS
        console.log("GPS:");
        console.log(`  Lat: ${telemetry.fix?.latitude?.toFixed(7)}°`);
        console.log(`  Lon: ${telemetry.fix?.longitude?.toFixed(7)}°`);
        console.log(`  Alt: ${telemetry.fix?.altitude?.toFixed(2)} m`);
        console.log("");

        // Altitude
        console.log("ALTITUDE:");
        console.log(`  Relative: ${telemetry.altitude?.relative?.toFixed(2)} m`);
        console.log(`  AMSL:     ${telemetry.altitude?.amsl?.toFixed(2)} m`);
        console.log("");

        // Battery
        if (battery) {
            console.log("BATTERY:");
            console.log(`  Voltage:    ${battery.voltage?.toFixed(2)} V`);
            console.log(`  Percentage: ${battery.percentage?.toFixed(0)}%`);
            console.log("");
        }

        console.log("Press Ctrl+C to exit");
    }, 1000);

    process.on("SIGINT", () => {
        clearInterval(interval);
        console.log("\n[EXIT] Shutting down...");
        subscriptions.stateSub.unsubscribe();
        subscriptions.poseSub.unsubscribe();
        subscriptions.fixSub.unsubscribe();
        subscriptions.altSub.unsubscribe();
        batterySub.unsubscribe();
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
