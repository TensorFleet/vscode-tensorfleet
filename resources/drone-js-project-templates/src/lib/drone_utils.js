/**
 * Shared utilities for drone control tutorials
 *
 * Common functions used across tutorial scripts to reduce duplication
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
async function connectToDrone(url) {
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

/**
 * Subscribe to telemetry topics and wait for initial data
 */
async function waitForTelemetry(ros) {
    const telemetry = {
        state: null,
        pose: null,
        fix: null,
        altitude: null
    };

    const stateSub = new ROSLIB.Topic({
        ros,
        name: "/mavros/state",
        messageType: "mavros_msgs/State"
    });
    stateSub.subscribe((msg) => {
        telemetry.state = msg;
    });

    const poseSub = new ROSLIB.Topic({
        ros,
        name: "/mavros/local_position/pose",
        messageType: "geometry_msgs/PoseStamped"
    });
    poseSub.subscribe((msg) => {
        telemetry.pose = msg;
    });

    const fixSub = new ROSLIB.Topic({
        ros,
        name: "/mavros/global_position/global",
        messageType: "sensor_msgs/NavSatFix"
    });
    fixSub.subscribe((msg) => {
        telemetry.fix = msg;
    });

    const altSub = new ROSLIB.Topic({
        ros,
        name: "/mavros/altitude",
        messageType: "mavros_msgs/Altitude"
    });
    altSub.subscribe((msg) => {
        telemetry.altitude = msg;
    });

    console.log("[TELEMETRY] Waiting for initial telemetry data...");
    await waitFor(() => !!telemetry.state, "/mavros/state");
    await waitFor(() => !!telemetry.pose, "/mavros/local_position/pose");
    await waitFor(
        () => !!telemetry.fix && typeof telemetry.fix.latitude === "number",
        "/mavros/global_position/global",
        15000,
        200
    );

    console.log("[TELEMETRY] Received initial telemetry");

    return {
        telemetry,
        subscriptions: { stateSub, poseSub, fixSub, altSub }
    };
}

/**
 * Arm the drone
 * Note: telemetry parameter should be the container object that holds the state
 */
async function armDrone(ros, telemetry, armWaitSec = 3.0) {
    console.log(`[ARM] Waiting ${armWaitSec.toFixed(1)}s before arming...`);
    await sleep(armWaitSec * 1000);

    const armSrv = new ROSLIB.Service({
        ros,
        name: "/mavros/cmd/arming",
        serviceType: "mavros_msgs/CommandBool"
    });

    console.log("[ARM] Sending arm command...");
    const resp = await makeServiceCall(armSrv, { value: true });
    if (!resp?.success) {
        throw new Error("Arming command rejected");
    }

    // Wait for the state to update
    const start = Date.now();
    while (Date.now() - start < 7000) {
        if (telemetry.state?.armed) {
            console.log("[ARM] Vehicle armed");
            return;
        }
        await sleep(100);
    }
    throw new Error("Vehicle did not arm in time");
}

/**
 * Disarm the drone
 * Note: telemetry parameter should be the container object that holds the state
 */
async function disarmDrone(ros, telemetry, disarmWaitSec = 3.0) {
    console.log(`[DISARM] Waiting ${disarmWaitSec.toFixed(1)}s before disarming...`);
    await sleep(disarmWaitSec * 1000);

    const armSrv = new ROSLIB.Service({
        ros,
        name: "/mavros/cmd/arming",
        serviceType: "mavros_msgs/CommandBool"
    });

    console.log("[DISARM] Sending disarm command...");
    const resp = await makeServiceCall(armSrv, { value: false });
    if (!resp?.success) {
        throw new Error("Disarm command rejected");
    }

    // Wait for the state to update
    const start = Date.now();
    while (Date.now() - start < 7000) {
        if (!telemetry.state?.armed) {
            console.log("[DISARM] Vehicle disarmed");
            return;
        }
        await sleep(100);
    }
    throw new Error("Vehicle did not disarm in time");
}

/**
 * Set flight mode
 * Note: telemetry should be the container object so we can read live updates
 */
async function setMode(ros, telemetry, targetMode, timeoutSec = 10) {
    const modeSrv = new ROSLIB.Service({
        ros,
        name: "/mavros/set_mode",
        serviceType: "mavros_msgs/SetMode"
    });

    const currentMode = (telemetry.state?.mode || "").toUpperCase();
    if (currentMode === targetMode.toUpperCase()) {
        console.log(`[MODE] Already in ${targetMode} mode`);
        return;
    }

    console.log(`[MODE] Switching from ${currentMode} to ${targetMode}...`);
    const resp = await makeServiceCall(modeSrv, {
        base_mode: 0,
        custom_mode: targetMode
    });

    if (!resp?.mode_sent) {
        throw new Error(`Failed to set mode to ${targetMode}`);
    }

    // Wait for mode to actually change
    const start = Date.now();
    while (Date.now() - start < timeoutSec * 1000) {
        // Read live value from telemetry container (updated by subscriptions)
        const mode = (telemetry.state?.mode || "").toUpperCase();
        if (mode === targetMode.toUpperCase()) {
            console.log(`[MODE] Successfully switched to ${targetMode}`);
            return;
        }
        await sleep(100);
    }
    throw new Error(`Timeout waiting for mode ${targetMode} (current: ${telemetry.state?.mode})`);
}

/**
 * Takeoff to specified altitude
 * Note: telemetry should be the container object so we can read live updates
 */
async function takeoffToAlt(ros, telemetry, targetAlt, timeoutSec = 60) {
    // Try to switch to GUIDED mode if not already in a compatible mode
    // Note: NAV_TAKEOFF can work in GUIDED, AUTO, or AUTO.LOITER modes
    const currentMode = (telemetry.state?.mode || "").toUpperCase();
    const compatibleModes = ["GUIDED", "AUTO", "AUTO.LOITER"];

    if (!compatibleModes.includes(currentMode)) {
        console.log(`[TAKEOFF] Current mode ${currentMode} may not support takeoff, attempting to switch to GUIDED...`);
        try {
            await setMode(ros, telemetry, "GUIDED", 10);
            await sleep(500); // Wait for mode to stabilize
        } catch (err) {
            // If GUIDED mode switch fails, try AUTO mode as fallback
            console.log(`[TAKEOFF] GUIDED mode switch failed, trying AUTO mode...`);
            try {
                await setMode(ros, telemetry, "AUTO", 10);
                await sleep(500);
            } catch (err2) {
                // If both fail, log warning but proceed - AUTO.LOITER might still work
                console.warn(`[TAKEOFF] Mode switch failed, proceeding anyway (current: ${currentMode})`);
            }
        }
    } else {
        console.log(`[TAKEOFF] Already in compatible mode: ${currentMode}`);
    }

    const cmdLongSrv = new ROSLIB.Service({
        ros,
        name: "/mavros/cmd/command",
        serviceType: "mavros_msgs/CommandLong"
    });

    console.log(`[TAKEOFF] Sending MAV_CMD_NAV_TAKEOFF to ${targetAlt.toFixed(2)}m AGL`);
    const lat = Number(telemetry.fix.latitude);
    const lon = Number(telemetry.fix.longitude);
    const request = {
        command: 22,
        confirmation: 0,
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: lat,
        param6: lon,
        param7: Number(targetAlt)
    };

    const resp = await makeServiceCall(cmdLongSrv, request, 5000);
    if (!resp?.success) {
        throw new Error(`NAV_TAKEOFF rejected (result=${resp?.result})`);
    }

    console.log("[TAKEOFF] Command accepted, waiting for altitude...");

    const start = Date.now();
    let lastAlt = telemetry.altitude?.relative || 0;
    let stableCount = 0;
    let hasStartedClimbing = false;

    while (true) {
        // Read live values from telemetry container (updated by subscriptions)
        const mode = (telemetry.state?.mode || "").toUpperCase();
        const relAlt = telemetry.altitude?.relative || 0;
        console.log(`[TAKEOFF] mode=${mode} rel_alt=${relAlt.toFixed(2)}`);

        // Check if altitude has started increasing (detect takeoff initiation)
        if (relAlt > lastAlt + 0.1) {
            hasStartedClimbing = true;
        }

        // If we've been in AUTO.LOITER for a while without climbing, something is wrong
        if (mode === "AUTO.LOITER" && !hasStartedClimbing && Date.now() - start > 5000) {
            throw new Error(`Takeoff failed: drone in AUTO.LOITER but not climbing (rel_alt=${relAlt.toFixed(2)}m). Ensure GUIDED mode is available and drone is ready.`);
        }

        // Check if altitude is reached (within 0.4m tolerance)
        if (relAlt >= targetAlt - 0.4) {
            // Check if altitude is stable (not climbing anymore)
            if (Math.abs(relAlt - lastAlt) < 0.1) {
                stableCount++;
                // Require 3 consecutive stable readings (600ms) to confirm
                if (stableCount >= 3) {
                    console.log("[TAKEOFF] Takeoff complete");
                    return;
                }
            } else {
                stableCount = 0; // Reset if still moving
            }
        } else {
            stableCount = 0; // Reset if not at target yet
        }

        lastAlt = relAlt;

        if (Date.now() - start > timeoutSec * 1000) {
            throw new Error(`Timeout waiting for altitude ${targetAlt.toFixed(2)}m (current: ${relAlt.toFixed(2)}m)`);
        }
        if (!telemetry.state?.armed) {
            throw new Error("Vehicle disarmed during takeoff");
        }
        await sleep(200);
    }
}

/**
 * Enter OFFBOARD mode
 */
async function enterOffboard(ros, velPub, setpointHz = 20) {
    console.log("[OFFBOARD] Pre-streaming zero velocities...");
    const intervalMs = 1000 / setpointHz;
    const end = Date.now() + 1500;

    const zeroVel = new ROSLIB.Message({
        header: { frame_id: "map" },
        twist: {
            linear: { x: 0.0, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: 0.0 }
        }
    });

    while (Date.now() < end) {
        velPub.publish(zeroVel);
        await sleep(intervalMs);
    }

    const modeSrv = new ROSLIB.Service({
        ros,
        name: "/mavros/set_mode",
        serviceType: "mavros_msgs/SetMode"
    });

    console.log("[OFFBOARD] Switching to OFFBOARD mode...");
    const resp = await makeServiceCall(modeSrv, {
        base_mode: 0,
        custom_mode: "OFFBOARD"
    });

    if (!resp?.mode_sent) {
        throw new Error("Failed to enter OFFBOARD mode");
    }

    console.log("[OFFBOARD] Entered OFFBOARD mode");
}

/**
 * Land the drone
 * Note: telemetry should be the container object so we can read live updates
 */
async function landDrone(ros, telemetry, setpointHz = 20, timeoutSec = 300) {
    const modeSrv = new ROSLIB.Service({
        ros,
        name: "/mavros/set_mode",
        serviceType: "mavros_msgs/SetMode"
    });

    console.log("[LAND] Setting AUTO.LAND mode...");
    await makeServiceCall(modeSrv, {
        base_mode: 0,
        custom_mode: "AUTO.LAND"
    });

    console.log("[LAND] Waiting for disarm...");
    const start = Date.now();
    while (Date.now() - start < timeoutSec * 1000) {
        // Read live values from telemetry container (updated by subscriptions)
        const armed = !!telemetry.state?.armed;
        const mode = telemetry.state?.mode;
        console.log(`[LAND] mode=${mode} armed=${armed}`);
        if (!armed) {
            console.log("[LAND] Vehicle disarmed, landing complete");
            return;
        }
        await sleep(1000);
    }
    console.warn("[LAND] Disarm not observed within timeout");
}

module.exports = {
    sleep,
    waitFor,
    makeServiceCall,
    connectToDrone,
    waitForTelemetry,
    armDrone,
    disarmDrone,
    setMode,
    takeoffToAlt,
    enterOffboard,
    landDrone
};
