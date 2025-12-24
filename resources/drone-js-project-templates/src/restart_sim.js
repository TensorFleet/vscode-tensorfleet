#!/usr/bin/env -S bun run
/**
 * Restart Simulation
 *
 * Simple utility to restart the PX4 simulation via ROS service call.
 * Uses ROSLibBridgeWrapper for simplified connection handling.
 * Useful for resetting the drone state between test runs.
 *
 * Run:
 *   bun src/restart_sim.js
 */

const { ROSLibBridgeWrapper } = require("./lib/roslib-bridge-wrapper");

async function restartSimulation() {
    console.log("[SIM] Initializing ROS bridge connection...");

    const bridge = new ROSLibBridgeWrapper();

    try {
        // Wait for connection with timeout
        await bridge.waitForConnection(10000);
        console.log("[SIM] Connected to ROS bridge");

        console.log("[SIM] Requesting simulation restart...");

        // Call the simulation restart service
        // We increase the timeout to 30 seconds because it will take a while
        const result = await bridge.callService("/simulation_manager/start_simulation", {}, 30000);

        const success = result?.success || false;
        const message = result?.message || "";

        console.log(`[SIM] Restart ${success ? "succeeded" : "failed"}`);
        if (message) {
            console.log(`[SIM] Message: ${message}`);
        }

        return success;
    } catch (error) {
        console.error("[SIM] Operation failed:", error.message || error);
        throw error;
    }
}

async function main() {
    try {
        const success = await restartSimulation();
        process.exit(success ? 0 : 1);
    } catch (err) {
        console.error("[ERROR]", err.message || err);
        process.exit(1);
    }
}

if (require.main === module) {
    main();
}

module.exports = { restartSimulation };
