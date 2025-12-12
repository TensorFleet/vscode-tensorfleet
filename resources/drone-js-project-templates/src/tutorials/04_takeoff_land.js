#!/usr/bin/env -S bun run
/**
 * Tutorial 04: Takeoff and Land
 * 
 * Learn: MAV commands, altitude monitoring, and landing sequence
 * 
 * This script demonstrates:
 * - Standalone operation: automatically arms drone if not already armed
 * - Sending MAV_CMD_NAV_TAKEOFF
 * - Setting GUIDED mode before takeoff (required)
 * - Monitoring altitude until target reached
 * - AUTO.LOITER mode after takeoff
 * - Landing sequence and disarm detection
 * 
 * Run: bun src/tutorials/04_takeoff_land.js
 */

require("dotenv").config();
const {
    connectToDrone,
    waitForTelemetry,
    armDrone,
    takeoffToAlt,
    landDrone
} = require("../lib/drone_utils");
const { getTensorfleetSettings } = require("../lib/tensorfleet_config");

const { rosbridgeUrl } = getTensorfleetSettings();

const TARGET_ALTITUDE = 3.0; // meters

async function main() {

    const ros = await connectToDrone(rosbridgeUrl);
    const { telemetry } = await waitForTelemetry(ros);

    console.log(`\n[INFO] Target altitude: ${TARGET_ALTITUDE}m\n`);

    // Arm if not already armed
    if (!telemetry.state.armed) {
        console.log("[INFO] Drone is not armed. Arming...\n");
        await armDrone(ros, telemetry);
    } else {
        console.log("[INFO] Drone already armed");
    }

    // Takeoff
    console.log("");
    await takeoffToAlt(ros, telemetry, TARGET_ALTITUDE);

    console.log("\n[SUCCESS] Takeoff complete! Drone is hovering at altitude.");
    console.log("[INFO] Drone will remain in AUTO.LOITER mode.");
    
    // Wait a moment before landing
    console.log("[INFO] Waiting 2 seconds before landing...\n");
    await new Promise(resolve => setTimeout(resolve, 2000));

    // Land
    console.log("[INFO] Landing drone...\n");
    await landDrone(ros, telemetry);

    console.log("\n[SUCCESS] Drone has landed and disarmed!");
    console.log("[EXIT] Closing connection...");
    ros.close();
}

if (require.main === module) {
    main().catch((err) => {
        console.error("[ERROR]", err.message || err);
        process.exit(1);
    });
}
