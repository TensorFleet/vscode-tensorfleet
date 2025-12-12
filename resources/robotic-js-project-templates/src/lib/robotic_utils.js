/**
 * Shared utilities for robotic control scripts
 *
 * Common functions used across robotic tutorial scripts to reduce duplication
 * and keep individual tutorials focused on one concept.
 */

const ROSLIB = require("roslib");
const { getTensorfleetSettings } = require("./tensorfleet_config");
const { createProxyWebSocket } = require("./proxy_ws_client");
const socketAdapter = require("roslib/src/core/SocketAdapter.js");

function sleep(ms) {
    return new Promise((resolve) => setTimeout(resolve, ms));
}

async function waitFor(checkFn, label, timeoutMs = 10000, intervalMs = 100) {
    const start = Date.now();
    while (Date.now() - start < timeoutMs) {
        if (checkFn()) return true;
        await sleep(intervalMs);
    }
    throw new Error(`Timeout waiting for ${label}`);
}

function makeServiceCall(service, request, timeoutMs = 5000) {
    return new Promise((resolve, reject) => {
        const timer = setTimeout(
            () => reject(new Error("Service call timeout")),
            timeoutMs
        );

        service.callService(
            new ROSLIB.ServiceRequest(request),
            (result) => {
                clearTimeout(timer);
                resolve(result || {});
            },
            (err) => {
                clearTimeout(timer);
                reject(err);
            }
        );
    });
}

/**
 * Attach a proxy WebSocket to a ROSLIB.Ros instance.
 * This wires up the socket adapter so roslib can communicate through the proxy.
 *
 * @param {ROSLIB.Ros} ros - The ROSLIB.Ros instance
 * @param {Object} proxyWs - The proxy WebSocket wrapper
 * @returns {ROSLIB.Ros} The same ros instance (for chaining)
 */
function attachProxySocket(ros, proxyWs) {
    ros.socket = Object.assign(proxyWs, socketAdapter(ros));
    return ros;
}

/**
 * Connect to rosbridge and return ROS client.
 *
 * If TensorFleet proxy settings are available (base/vm-manager URL + node ID + JWT),
 * traffic is routed through the derived proxy socket. Otherwise, we connect directly
 * to the resolved rosbridge URL.
 */
async function connectToRobot(url) {
    const settings = getTensorfleetSettings();

    const useProxy = settings.useProxy && settings.proxyUrl;
    let ros;

    if (useProxy) {
        console.log(`[CONNECT] Connecting via VM Manager proxy at ${settings.proxyUrl} (nodeId=${settings.nodeId}, targetPort=${settings.targetPort})...`);

        const proxyWs = createProxyWebSocket({
            proxyUrl: settings.proxyUrl,
            vmManagerUrl: settings.vmManagerUrl,
            token: settings.token,
            nodeId: settings.nodeId,
            targetPort: settings.targetPort
        });

        // Attach proxy socket to ROSLIB using helper
        ros = attachProxySocket(new ROSLIB.Ros({}), proxyWs);
    } else {
        const directUrl = settings.rosbridgeUrl || url;
        console.log(`[CONNECT] Connecting to rosbridge at ${directUrl}...`);
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

    console.log("[CONNECT] Connected to rosbridge");
    return ros;
}

module.exports = {
    sleep,
    waitFor,
    makeServiceCall,
    connectToRobot,
};
