#!/usr/bin/env -S bun run
/**
 * Tutorial 02: Telemetry Monitoring
 *
 * This tutorial shows how to monitor drone telemetry data.
 *
 * Telemetry includes information like position, altitude, battery level, and flight status.
 *
 * We access this data by subscribing to various ROS topics published by MAVROS.
 *
 * Run: bun src/tutorials/02_telemetry.js
 */

import { DroneStateModel } from "tensorfleet-util";
import { ROSLibBridgeWrapper } from "../lib/roslib-bridge-wrapper.js";

async function main() {
    // Establish ROS Bridge connection using our wrapper
    const bridge = new ROSLibBridgeWrapper();
    await bridge.waitForConnection();

    console.log("\n[INFO] Connected to ROS Bridge - monitoring comprehensive telemetry...\n");
    console.log("[INFO] Demonstrating both raw MAVROS topic subscriptions and managed DroneStateModel\n");

    // Raw telemetry data storage
    let rawTelemetry = {
        state: null,
        pose: null,
        fix: null,
        altitude: null,
        battery: null,
        vfr_hud: null,
    };

    // Raw MAVROS topic subscriptions for educational purposes
    const rawSubscriptions = [];

    // Subscribe to vehicle state (/mavros/state)
    rawSubscriptions.push(
        bridge.subscribe(
            { topic: "/mavros/state", type: "mavros_msgs/State" },
            (msg) => {
                rawTelemetry.state = msg;
            }
        )
    );

    // Subscribe to local position (/mavros/local_position/pose)
    rawSubscriptions.push(
        bridge.subscribe(
            { topic: "/mavros/local_position/pose", type: "geometry_msgs/PoseStamped" },
            (msg) => {
                rawTelemetry.pose = msg;
            }
        )
    );

    // Subscribe to GPS fix (/mavros/global_position/raw/fix)
    rawSubscriptions.push(
        bridge.subscribe(
            { topic: "/mavros/global_position/raw/fix", type: "sensor_msgs/NavSatFix" },
            (msg) => {
                rawTelemetry.fix = msg;
            }
        )
    );

    // Subscribe to altitude (/mavros/altitude)
    rawSubscriptions.push(
        bridge.subscribe(
            { topic: "/mavros/altitude", type: "mavros_msgs/Altitude" },
            (msg) => {
                rawTelemetry.altitude = msg;
            }
        )
    );

    // Subscribe to battery (/mavros/battery)
    rawSubscriptions.push(
        bridge.subscribe(
            { topic: "/mavros/battery", type: "sensor_msgs/BatteryState" },
            (msg) => {
                rawTelemetry.battery = msg;
            }
        )
    );

    // Subscribe to VFR HUD (/mavros/vfr_hud) for additional flight data
    rawSubscriptions.push(
        bridge.subscribe(
            { topic: "/mavros/vfr_hud", type: "mavros_msgs/VFR_HUD" },
            (msg) => {
                rawTelemetry.vfr_hud = msg;
            }
        )
    );

    // Initialize managed state model for comparison
    const droneState = new DroneStateModel();
    droneState.connect(bridge);

    // Variables to track managed state for display
    let managedState = {};

    droneState.onUpdate((state) => {
        managedState = state;
    });

    console.log("Press Ctrl+C to exit\n");

    // Display telemetry updates every second
    const displayInterval = setInterval(() => {
        console.clear();
        console.log("=== COMPREHENSIVE DRONE TELEMETRY ===\n");

        // Raw MAVROS Topic Data
        console.log("RAW MAVROS TOPIC DATA:");
        console.log("----------------------");

        // State information
        if (rawTelemetry.state) {
            console.log("Vehicle State (/mavros/state):");
            console.log(`  Connected: ${rawTelemetry.state.connected}`);
            console.log(`  Armed:     ${rawTelemetry.state.armed}`);
            console.log(`  Mode:      ${rawTelemetry.state.mode}`);
            console.log(`  Guided:    ${rawTelemetry.state.guided}`);
            console.log("");
        }

        // Local position (ENU coordinates)
        if (rawTelemetry.pose?.pose?.position) {
            const pos = rawTelemetry.pose.pose.position;
            console.log("Local Position (/mavros/local_position/pose) - ENU coordinates:");
            console.log(`  East (X):  ${pos.x?.toFixed(2)} m`);
            console.log(`  North (Y): ${pos.y?.toFixed(2)} m`);
            console.log(`  Up (Z):    ${pos.z?.toFixed(2)} m`);
            console.log("");
        }

        // GPS position
        if (rawTelemetry.fix) {
            console.log("GPS Position (/mavros/global_position/raw/fix):");
            console.log(`  Latitude:  ${rawTelemetry.fix.latitude?.toFixed(7)}°`);
            console.log(`  Longitude: ${rawTelemetry.fix.longitude?.toFixed(7)}°`);
            console.log(`  Altitude:  ${rawTelemetry.fix.altitude?.toFixed(2)} m`);
            console.log("");
        }

        // Altitude breakdown
        if (rawTelemetry.altitude) {
            console.log("Altitude Breakdown (/mavros/altitude):");
            console.log(`  Above Mean Sea Level: ${rawTelemetry.altitude.amsl?.toFixed(2)} m`);
            console.log(`  Relative to Home:     ${rawTelemetry.altitude.relative?.toFixed(2)} m`);
            console.log(`  Above Ground Level:   ${rawTelemetry.altitude.agl?.toFixed(2)} m`);
            console.log("");
        }

        // Battery status
        if (rawTelemetry.battery) {
            console.log("Battery Status (/mavros/battery):");
            console.log(`  Voltage:    ${rawTelemetry.battery.voltage?.toFixed(2)} V`);
            console.log(`  Current:    ${rawTelemetry.battery.current?.toFixed(2)} A`);
            console.log(`  Percentage: ${rawTelemetry.battery.percentage?.toFixed(0)}%`);
            console.log("");
        }

        // VFR HUD (additional flight instruments)
        if (rawTelemetry.vfr_hud) {
            console.log("Flight Instruments (/mavros/vfr_hud):");
            console.log(`  Airspeed:   ${rawTelemetry.vfr_hud.airspeed?.toFixed(2)} m/s`);
            console.log(`  Groundspeed:${rawTelemetry.vfr_hud.groundspeed?.toFixed(2)} m/s`);
            console.log(`  Heading:    ${rawTelemetry.vfr_hud.heading?.toFixed(1)}°`);
            console.log(`  Throttle:   ${rawTelemetry.vfr_hud.throttle?.toFixed(1)}%`);
            console.log(`  Climb Rate: ${rawTelemetry.vfr_hud.climb?.toFixed(2)} m/s`);
            console.log("");
        }

        // Managed State Model Data
        console.log("MANAGED STATE MODEL (Aggregated Data):");
        console.log("---------------------------------------");

        if (managedState.vehicle) {
            console.log("Vehicle State:");
            console.log(`  Connected: ${managedState.vehicle.connected}`);
            console.log(`  Armed:     ${managedState.vehicle.armed}`);
            console.log(`  Mode:      ${managedState.vehicle.mode}`);
            console.log(`  Guided:    ${managedState.vehicle.guided}`);
            console.log("");
        }

        if (managedState.local?.position) {
            console.log("Local Position (ENU):");
            console.log(`  East:  ${managedState.local.position.x?.toFixed(2)} m`);
            console.log(`  North: ${managedState.local.position.y?.toFixed(2)} m`);
            console.log(`  Up:    ${managedState.local.position.z?.toFixed(2)} m`);
            console.log("");
        }

        if (managedState.global_position_int) {
            console.log("Global Position:");
            console.log(`  Lat: ${managedState.global_position_int.lat?.toFixed(7)}°`);
            console.log(`  Lon: ${managedState.global_position_int.lon?.toFixed(7)}°`);
            console.log(`  Alt: ${managedState.global_position_int.alt?.toFixed(2)} m`);
            console.log("");
        }

        if (managedState.altitude) {
            console.log("Altitude:");
            console.log(`  AMSL:     ${managedState.altitude.amsl?.toFixed(2)} m`);
            console.log(`  Relative: ${managedState.altitude.relative?.toFixed(2)} m`);
            console.log("");
        }

        if (managedState.battery) {
            console.log("Battery:");
            console.log(`  Voltage:    ${managedState.battery.voltage?.toFixed(2)} V`);
            console.log(`  Percentage: ${managedState.battery.percentage?.toFixed(0)}%`);
            console.log("");
        }

        if (managedState.vfr_hud) {
            console.log("Flight Data:");
            console.log(`  Airspeed:   ${managedState.vfr_hud.airspeed?.toFixed(2)} m/s`);
            console.log(`  Groundspeed:${managedState.vfr_hud.groundspeed?.toFixed(2)} m/s`);
            console.log(`  Heading:    ${managedState.vfr_hud.heading?.toFixed(1)}°`);
            console.log("");
        }

        console.log("Press Ctrl+C to exit - Updates every 1 second");
    }, 1000);

    // Handle graceful shutdown
    process.on("SIGINT", () => {
        clearInterval(displayInterval);
        console.log("\n[EXIT] Shutting down telemetry monitoring...");

        // Unsubscribe from all raw topics
        rawSubscriptions.forEach(unsub => unsub());

        // Disconnect managed state model
        droneState.disconnect();

        process.exit(0);
    });
}

if (require.main === module) {
    main().catch((err) => {
        console.error("[ERROR]", err.message || err);
        process.exit(1);
    });
}
