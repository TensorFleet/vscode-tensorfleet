/**
 * Shared utilities for drone control tutorials
 * 
 * Common functions used across tutorial scripts to reduce duplication
 * and keep individual tutorials focused on one concept.
 */

const ROSLIB = require("roslib");

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
 * Connect to rosbridge and return ROS client
 */
async function connectToDrone(url) {
    console.log(`[CONNECT] Connecting to rosbridge at ${url}...`);
    const ros = new ROSLIB.Ros({ url });

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
 */
async function armDrone(ros, state, armWaitSec = 3.0) {
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

    const start = Date.now();
    while (Date.now() - start < 7000) {
        if (state.armed) {
            console.log("[ARM] Vehicle armed");
            return;
        }
        await sleep(100);
    }
    throw new Error("Vehicle did not arm in time");
}

/**
 * Takeoff to specified altitude
 */
async function takeoffToAlt(ros, state, fix, altitude, targetAlt, timeoutSec = 60) {
    const cmdLongSrv = new ROSLIB.Service({
        ros,
        name: "/mavros/cmd/command",
        serviceType: "mavros_msgs/CommandLong"
    });

    console.log(`[TAKEOFF] Sending MAV_CMD_NAV_TAKEOFF to ${targetAlt.toFixed(2)}m AGL`);
    const lat = Number(fix.latitude);
    const lon = Number(fix.longitude);
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

    console.log("[TAKEOFF] Command accepted, waiting for AUTO.LOITER @ altitude...");

    const start = Date.now();
    while (true) {
        const mode = (state?.mode || "").toUpperCase();
        const relAlt = altitude?.relative || 0;
        console.log(`[TAKEOFF] mode=${mode} rel_alt=${relAlt.toFixed(2)}`);

        if (mode === "AUTO.LOITER" && Math.abs(relAlt - targetAlt) < 0.4) {
            console.log("[TAKEOFF] Takeoff complete");
            return;
        }
        if (Date.now() - start > timeoutSec * 1000) {
            throw new Error("Timeout waiting for AUTO.LOITER at altitude");
        }
        if (!state?.armed) {
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
 */
async function landDrone(ros, state, setpointHz = 20, timeoutSec = 300) {
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
        const armed = !!state?.armed;
        const mode = state?.mode;
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
    takeoffToAlt,
    enterOffboard,
    landDrone
};
