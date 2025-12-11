#!/usr/bin/env node
/**
 * Restart Simulation
 *
 * Simple utility to restart the PX4 simulation via ROS service call.
 * Useful for resetting the drone state between test runs.
 *
 * Run:
 *   bun src/restart_sim.js
 */

require("dotenv").config();
const ROSLIB = require("roslib");
const { getTensorfleetSettings } = require("./lib/tensorfleet_config");
const socketAdapter = require("roslib/src/core/SocketAdapter.js");
const { createProxyWebSocket } = require("./lib/proxy_ws_client");

async function restartSimulation() {
    const tf = getTensorfleetSettings();
    const useProxy = tf.useProxy && tf.proxyUrl;

    let ros;

    if (useProxy) {
        console.log(
            `[SIM] Connecting via VM Manager proxy at ${tf.proxyUrl} (nodeId=${tf.nodeId}, targetPort=${tf.targetPort}) ...`
        );
        const proxyWs = createProxyWebSocket({
            proxyUrl: tf.proxyUrl,
            token: tf.token,
            nodeId: tf.nodeId,
            targetPort: tf.targetPort,
        });
        ros = new ROSLIB.Ros({});
        const adaptedSocket = Object.assign(proxyWs, socketAdapter(ros));
        ros.socket = adaptedSocket;
    } else {
        const directUrl = tf.rosbridgeUrl || "ws://localhost:9091";
        console.log(`[SIM] Connecting to rosbridge at ${directUrl}...`);
        ros = new ROSLIB.Ros({ url: directUrl });
    }

    await new Promise((resolve, reject) => {
        const timer = setTimeout(
            () => reject(new Error("Connection timeout")),
            10000
        );

        ros.on("connection", () => {
            clearTimeout(timer);
            resolve();
        });

        ros.on("error", (err) => {
            clearTimeout(timer);
            reject(err instanceof Error ? err : new Error(String(err)));
        });
    });

    console.log("[SIM] Connected to rosbridge");

    const simSrv = new ROSLIB.Service({
        ros,
        name: "/simulation_manager/start_simulation",
        serviceType: "std_srvs/Trigger",
    });

    console.log("[SIM] Requesting simulation restart...");

    return new Promise((resolve, reject) => {
        const timer = setTimeout(
            () => reject(new Error("Service call timeout")),
            20000
        );

        simSrv.callService(
            new ROSLIB.ServiceRequest({}),
            (result) => {
                clearTimeout(timer);
                const success = result?.success || false;
                const message = result?.message || "";

                console.log(`[SIM] Restart ${success ? "succeeded" : "failed"}`);
                if (message) {
                    console.log(`[SIM] Message: ${message}`);
                }

                ros.close();
                resolve(success);
            },
            (err) => {
                clearTimeout(timer);
                console.error("[SIM] Service call failed:", err);
                ros.close();
                reject(err);
            }
        );
    });
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
