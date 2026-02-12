#!/usr/bin/env -S bun run
/**
 * Tutorial 02: Telemetry Monitoring (Enhanced)
 *
 * This tutorial shows how to monitor drone telemetry data.
 *
 * Telemetry includes information like position, altitude, battery level, and flight status.
 *
 * We access this data by subscribing to various ROS topics published by MAVROS.
 *
 * Adds:
 *  - /mavros/vfr_hud (navigation summary)
 *  - /mavros/estimator_status (estimator health summary)
 *  - /mavros/gpsstatus/gps1/raw (GPS fix type + satellites)
 *  - /mavros/statustext/recv (autopilot warnings/info)
 *  - Message "age" display so stale data is visible
 *
 * Run: bun src/tutorials/02_telemetry.js
 */

import { DroneStateModel } from "tensorfleet-util";
import { ROSLibBridgeWrapper } from "../lib/roslib-bridge-wrapper.js";

function nowMs() {
    return Date.now();
}

function ageStr(tsMs) {
    if (!tsMs) return "n/a";
    const ms = nowMs() - tsMs;
    if (ms < 1000) return `${ms} ms`;
    return `${(ms / 1000).toFixed(1)} s`;
}

function fmtNum(x, digits = 2) {
    if (typeof x !== "number" || Number.isNaN(x)) return "n/a";
    return x.toFixed(digits);
}

function fmtPercent(x, digits = 0) {
    if (typeof x !== "number" || Number.isNaN(x)) return "n/a";
    if (x >= 0 && x <= 1) return `${(x * 100).toFixed(digits)}%`;
    return `${x.toFixed(digits)}%`;
}

function gpsFixLabel(fixType) {
    // MAVLink GPS_FIX_TYPE
    switch (fixType) {
        case 0: return "NO_GPS";
        case 1: return "NO_FIX";
        case 2: return "2D_FIX";
        case 3: return "3D_FIX";
        case 4: return "DGPS";
        case 5: return "RTK_FLOAT";
        case 6: return "RTK_FIXED";
        default: return `UNKNOWN(${fixType})`;
    }
}

function isValidLatLon(lat, lon) {
    if (typeof lat !== "number" || typeof lon !== "number") return false;
    if (Number.isNaN(lat) || Number.isNaN(lon)) return false;
    if (lat === 0 && lon === 0) return false;
    if (lat < -90 || lat > 90) return false;
    if (lon < -180 || lon > 180) return false;
    return true;
}

async function main() {
    // Establish ROS Bridge connection using our wrapper
    const bridge = new ROSLibBridgeWrapper();
    await bridge.waitForConnection();

    console.log("\n[INFO] Connected to ROS Bridge - monitoring comprehensive telemetry...\n");
    console.log("[INFO] Enhanced with VFR HUD, GPSRAW, Estimator, StatusText, and message ages\n");
    console.log("[INFO] Demonstrating both raw MAVROS topic subscriptions and managed DroneStateModel\n");

    // Raw telemetry data storage
    let rawTelemetry = {
        state: null,
        pose: null,
        fix: null,
        altitude: null,
        battery: null,
        imu: null,
        vfrHud: null,
        gpsRaw: null,
        estimator: null,
        statusText: [],
    };

    // Last-seen timestamps per telemetry stream
    let rawTelemetryTs = {
        state: 0,
        pose: 0,
        fix: 0,
        altitude: 0,
        battery: 0,
        imu: 0,
        vfrHud: 0,
        gpsRaw: 0,
        estimator: 0,
        statusText: 0,
    };

    // Raw MAVROS topic subscriptions for educational purposes
    const rawSubscriptions = [];

    // Subscribe to vehicle state (/mavros/state)
    rawSubscriptions.push(
        bridge.subscribe(
            { topic: "/mavros/state", type: "mavros_msgs/State" },
            (msg) => {
                rawTelemetry.state = msg;
                rawTelemetryTs.state = nowMs();
            }
        )
    );

    // Subscribe to local position (/mavros/local_position/pose)
    rawSubscriptions.push(
        bridge.subscribe(
            { topic: "/mavros/local_position/pose", type: "geometry_msgs/PoseStamped" },
            (msg) => {
                rawTelemetry.pose = msg;
                rawTelemetryTs.pose = nowMs();
            }
        )
    );

    // Subscribe to GPS fix (/mavros/global_position/raw/fix)
    rawSubscriptions.push(
        bridge.subscribe(
            { topic: "/mavros/global_position/raw/fix", type: "sensor_msgs/NavSatFix" },
            (msg) => {
                rawTelemetry.fix = msg;
                rawTelemetryTs.fix = nowMs();
            }
        )
    );

    // Subscribe to altitude (/mavros/altitude)
    rawSubscriptions.push(
        bridge.subscribe(
            { topic: "/mavros/altitude", type: "mavros_msgs/Altitude" },
            (msg) => {
                rawTelemetry.altitude = msg;
                rawTelemetryTs.altitude = nowMs();
            }
        )
    );

    // Subscribe to battery (/mavros/battery)
    rawSubscriptions.push(
        bridge.subscribe(
            { topic: "/mavros/battery", type: "sensor_msgs/BatteryState" },
            (msg) => {
                rawTelemetry.battery = msg;
                rawTelemetryTs.battery = nowMs();
            }
        )
    );

    // Subscribe to IMU data (/mavros/imu/data)
    rawSubscriptions.push(
        bridge.subscribe(
            { topic: "/mavros/imu/data", type: "sensor_msgs/Imu" },
            (msg) => {
                rawTelemetry.imu = msg;
                rawTelemetryTs.imu = nowMs();
            }
        )
    );

    // Subscribe to VFR HUD (/mavros/vfr_hud)
    rawSubscriptions.push(
        bridge.subscribe(
            { topic: "/mavros/vfr_hud", type: "mavros_msgs/VFR_HUD" },
            (msg) => {
                rawTelemetry.vfrHud = msg;
                rawTelemetryTs.vfrHud = nowMs();
            }
        )
    );

    // Subscribe to raw GPS status (/mavros/gpsstatus/gps1/raw)
    rawSubscriptions.push(
        bridge.subscribe(
            { topic: "/mavros/gpsstatus/gps1/raw", type: "mavros_msgs/GPSRAW" },
            (msg) => {
                rawTelemetry.gpsRaw = msg;
                rawTelemetryTs.gpsRaw = nowMs();
            }
        )
    );

    // Subscribe to estimator status (/mavros/estimator_status)
    rawSubscriptions.push(
        bridge.subscribe(
            { topic: "/mavros/estimator_status", type: "mavros_msgs/EstimatorStatus" },
            (msg) => {
                rawTelemetry.estimator = msg;
                rawTelemetryTs.estimator = nowMs();
            }
        )
    );

    // Subscribe to autopilot status text (/mavros/statustext/recv)
    rawSubscriptions.push(
        bridge.subscribe(
            { topic: "/mavros/statustext/recv", type: "mavros_msgs/StatusText" },
            (msg) => {
                rawTelemetry.statusText.push({
                    ts: nowMs(),
                    severity: msg.severity,
                    text: msg.text,
                });
                rawTelemetry.statusText = rawTelemetry.statusText.slice(-8);
                rawTelemetryTs.statusText = nowMs();
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
        console.log("=== COMPREHENSIVE DRONE TELEMETRY (ENHANCED) ===\n");

        // Raw MAVROS Topic Data
        console.log("RAW MAVROS TOPIC DATA:");
        console.log("----------------------");

        // State information
        if (rawTelemetry.state) {
            console.log(`Vehicle State (/mavros/state) [age: ${ageStr(rawTelemetryTs.state)}]:`);
            console.log(`  Connected: ${rawTelemetry.state.connected}`);
            console.log(`  Armed:     ${rawTelemetry.state.armed}`);
            console.log(`  Mode:      ${rawTelemetry.state.mode}`);
            console.log(`  Guided:    ${rawTelemetry.state.guided}`);
            console.log("");
        } else {
            console.log("Vehicle State (/mavros/state): (no data yet)\n");
        }

        // VFR HUD
        if (rawTelemetry.vfrHud) {
            const v = rawTelemetry.vfrHud;
            console.log(`VFR HUD (/mavros/vfr_hud) [age: ${ageStr(rawTelemetryTs.vfrHud)}]:`);
            console.log(`  Heading:     ${v.heading} deg`);
            console.log(`  Groundspeed: ${fmtNum(v.groundspeed, 3)} m/s`);
            console.log(`  Climb:       ${fmtNum(v.climb, 3)} m/s`);
            console.log(`  Altitude:    ${fmtNum(v.altitude, 2)} m`);
            console.log(`  Throttle:    ${fmtNum(v.throttle, 1)}`);
            console.log(`  Airspeed:    ${fmtNum(v.airspeed, 3)} m/s`);
            console.log("");
        } else {
            console.log("VFR HUD (/mavros/vfr_hud): (no data yet)\n");
        }

        // GPS RAW status
        if (rawTelemetry.gpsRaw) {
            const g = rawTelemetry.gpsRaw;
            console.log(`GPS Status (/mavros/gpsstatus/gps1/raw) [age: ${ageStr(rawTelemetryTs.gpsRaw)}]:`);
            console.log(`  Fix Type:   ${gpsFixLabel(g.fix_type)} (${g.fix_type})`);
            console.log(`  Satellites: ${g.satellites_visible}`);
            console.log(`  EPH/EPV:    ${fmtNum(g.eph, 2)} / ${fmtNum(g.epv, 2)}`);
            console.log("");
        } else {
            console.log("GPS Status (/mavros/gpsstatus/gps1/raw): (no data yet)\n");
        }

        // Local position (ENU coordinates)
        if (rawTelemetry.pose?.pose?.position) {
            const pos = rawTelemetry.pose.pose.position;
            console.log(`Local Position (/mavros/local_position/pose) [age: ${ageStr(rawTelemetryTs.pose)}] - ENU coordinates:`);
            console.log(`  East (X):  ${fmtNum(pos.x, 2)} m`);
            console.log(`  North (Y): ${fmtNum(pos.y, 2)} m`);
            console.log(`  Up (Z):    ${fmtNum(pos.z, 2)} m`);
            console.log("");
        } else {
            console.log("Local Position (/mavros/local_position/pose): (no data yet)\n");
        }

        // GPS position
        if (rawTelemetry.fix) {
            const f = rawTelemetry.fix;
            const validFix = isValidLatLon(f.latitude, f.longitude) && f.status?.status !== -1;
            console.log(`GPS Position (/mavros/global_position/raw/fix) [age: ${ageStr(rawTelemetryTs.fix)}]:`);
            if (!validFix) {
                console.log("  Status:     NO FIX (values invalid)");
                console.log(`  status:     ${f.status?.status}  service: ${f.status?.service}`);
                console.log("  Lat/Lon:    n/a");
                console.log(`  Altitude:   ${fmtNum(f.altitude, 3)} m`);
            } else {
                console.log(`  Latitude:   ${f.latitude.toFixed(7)} deg`);
                console.log(`  Longitude:  ${f.longitude.toFixed(7)} deg`);
                console.log(`  Altitude:   ${fmtNum(f.altitude, 2)} m`);
            }
            console.log("");
        } else {
            console.log("GPS Position (/mavros/global_position/raw/fix): (no data yet)\n");
        }

        // Altitude breakdown
        if (rawTelemetry.altitude) {
            console.log(`Altitude Breakdown (/mavros/altitude) [age: ${ageStr(rawTelemetryTs.altitude)}]:`);
            console.log(`  Above Mean Sea Level: ${fmtNum(rawTelemetry.altitude.amsl, 2)} m`);
            console.log(`  Relative to Home:     ${fmtNum(rawTelemetry.altitude.relative, 2)} m`);
            console.log(`  Above Ground Level:   ${fmtNum(rawTelemetry.altitude.agl, 2)} m`);
            console.log("");
        } else {
            console.log("Altitude Breakdown (/mavros/altitude): (no data yet)\n");
        }

        // Battery status
        if (rawTelemetry.battery) {
            console.log(`Battery Status (/mavros/battery) [age: ${ageStr(rawTelemetryTs.battery)}]:`);
            console.log(`  Voltage:    ${fmtNum(rawTelemetry.battery.voltage, 2)} V`);
            console.log(`  Current:    ${fmtNum(rawTelemetry.battery.current, 2)} A`);
            console.log(`  Percentage: ${fmtPercent(rawTelemetry.battery.percentage, 0)}`);
            console.log("");
        } else {
            console.log("Battery Status (/mavros/battery): (no data yet)\n");
        }

        if (rawTelemetry.imu?.linear_acceleration) {
            const accel = rawTelemetry.imu.linear_acceleration;
            const gyro = rawTelemetry.imu.angular_velocity;
            console.log(`IMU Data (/mavros/imu/data) [age: ${ageStr(rawTelemetryTs.imu)}]:`);
            console.log(`  Accel X/Y/Z: ${fmtNum(accel.x, 3)}, ${fmtNum(accel.y, 3)}, ${fmtNum(accel.z, 3)} m/s^2`);
            console.log(`  Gyro X/Y/Z:  ${fmtNum(gyro.x, 3)}, ${fmtNum(gyro.y, 3)}, ${fmtNum(gyro.z, 3)} rad/s`);
            console.log("");
        } else {
            console.log("IMU Data (/mavros/imu/data): (no data yet)\n");
        }

        // Estimator summary
        if (rawTelemetry.estimator) {
            const e = rawTelemetry.estimator;
            const summary = {};
            for (const k of ["flags", "vel_ratio", "pos_horiz_ratio", "pos_vert_ratio", "mag_ratio", "hagl_ratio", "tas_ratio"]) {
                if (e[k] !== undefined) summary[k] = e[k];
            }
            console.log(`Estimator Status (/mavros/estimator_status) [age: ${ageStr(rawTelemetryTs.estimator)}]:`);
            if (Object.keys(summary).length === 0) {
                console.log("  Fields: (fields present, but unknown schema)");
            } else {
                console.log(`  Fields: ${JSON.stringify(summary)}`);
            }
            console.log("");
        } else {
            console.log("Estimator Status (/mavros/estimator_status): (no data yet)\n");
        }

        // StatusText
        console.log(`StatusText (/mavros/statustext/recv) [last: ${ageStr(rawTelemetryTs.statusText)}]:`);
        if (rawTelemetry.statusText.length === 0) {
            console.log("  (no messages yet - normal until autopilot emits warnings/info)");
            console.log("");
        } else {
            for (const m of rawTelemetry.statusText.slice(-5)) {
                console.log(`  [sev ${m.severity}] ${m.text}`);
            }
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
            console.log(`  East:  ${fmtNum(managedState.local.position.x, 2)} m`);
            console.log(`  North: ${fmtNum(managedState.local.position.y, 2)} m`);
            console.log(`  Up:    ${fmtNum(managedState.local.position.z, 2)} m`);
            console.log("");
        }

        if (managedState.global_position_int) {
            console.log("Global Position:");
            console.log(`  Lat: ${fmtNum(managedState.global_position_int.lat, 7)} deg`);
            console.log(`  Lon: ${fmtNum(managedState.global_position_int.lon, 7)} deg`);
            console.log(`  Alt: ${fmtNum(managedState.global_position_int.alt, 2)} m`);
            console.log("");
        }

        if (managedState.altitude) {
            console.log("Altitude:");
            console.log(`  AMSL:     ${fmtNum(managedState.altitude.amsl, 2)} m`);
            console.log(`  Relative: ${fmtNum(managedState.altitude.relative, 2)} m`);
            console.log("");
        }

        if (managedState.battery) {
            console.log("Battery:");
            console.log(`  Voltage:    ${fmtNum(managedState.battery.voltage, 2)} V`);
            console.log(`  Percentage: ${fmtPercent(managedState.battery.percentage, 0)}`);
            console.log("");
        }

        console.log("Press Ctrl+C to exit - Updates every 1 second");
    }, 1000);

    // Handle graceful shutdown
    process.on("SIGINT", async () => {
        clearInterval(displayInterval);
        console.log("\n[EXIT] Shutting down telemetry monitoring...");

        // Unsubscribe from all raw topics
        rawSubscriptions.forEach(unsub => unsub());

        // Disconnect managed state model
        droneState.disconnect();
        if (typeof bridge.disconnect === "function") {
            await bridge.disconnect();
        }

        process.exit(0);
    });
}

if (require.main === module) {
    main().catch((err) => {
        console.error("[ERROR]", err.message || err);
        process.exit(1);
    });
}
