#!/usr/bin/env node
/**
 * Simple PX4/MAVROS OFFBOARD velocity mission using roslib + rosbridge.
 * Ports the Python guided mission sample into JavaScript.
 *
 * Flow:
 *  - Connect to rosbridge
 *  - Arm + AUTO.TAKEOFF to target altitude
 *  - Switch to OFFBOARD and stream velocity setpoints to two ENU waypoints
 *  - Command AUTO.LAND and wait for disarm
 *
 * Run:
 *   ROSBRIDGE_URL=ws://172.16.0.10:9091 bun src/drone_mover.js
 */

require("dotenv").config();
const fs = require("fs");
const path = require("path");
const ROSLIB = require("roslib");
const yaml = require("js-yaml");

function loadConfig() {
  const configPath = path.join(process.cwd(), "config", "drone_config.yaml");
  try {
    const contents = fs.readFileSync(configPath, "utf8");
    return yaml.load(contents) || {};
  } catch (err) {
    console.warn(`[CFG] Could not load config at ${configPath}, using defaults. ${err.message}`);
    return {};
  }
}

const config = loadConfig();

const ALT_TARGET = numEnv("ALT_TARGET", config?.offboard?.alt_target ?? 3.0);
const EDGE_M = numEnv("EDGE_M", config?.offboard?.edge_m ?? 200.0);
const RADIUS = numEnv("WAYPOINT_RADIUS", config?.offboard?.waypoint_radius ?? 2.0);
const SLOW_RADIUS = numEnv("SLOW_RADIUS", config?.offboard?.slow_radius ?? 10.0);
const V_FAST = numEnv("V_FAST", config?.offboard?.v_fast ?? 20.0);
const V_MIN = numEnv("V_MIN", config?.offboard?.v_min ?? 1.0);
const ARM_WAIT = numEnv("ARM_WAIT", config?.offboard?.arm_wait ?? 3.0);
const TAKEOFF_TIMEOUT = numEnv("TAKEOFF_TIMEOUT", config?.offboard?.takeoff_timeout ?? 60.0);
const LAND_TIMEOUT = numEnv("LAND_TIMEOUT", config?.offboard?.land_timeout ?? 300.0);
const SETPOINT_HZ = numEnv("SETPOINT_HZ", config?.offboard?.setpoint_hz ?? 20.0);
const FRAME_ID = process.env.SETPOINT_FRAME_ID || config?.network?.setpoint_frame || "map";

const R2B_HOST = process.env.R2B_HOST || config?.network?.vm_ip || "172.16.0.10";
const R2B_PORT = process.env.R2B_PORT || config?.network?.rosbridge_port || "9091";
const rosbridgeUrl =
  process.env.ROSBRIDGE_URL ||
  config?.network?.rosbridge_url ||
  `ws://${R2B_HOST}:${R2B_PORT}`;

function numEnv(key, fallback) {
  const raw = process.env[key];
  if (raw === undefined) return fallback;
  const parsed = Number(raw);
  return Number.isFinite(parsed) ? parsed : fallback;
}

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

class GuidedMissionController {
  constructor(url) {
    this.url = url;
    this.ros = null;

    this.state = null;
    this.pose = null;
    this.fix = null;
    this.altitude = null;
    this.home = null;
    this.homeFix = null;
  }

  async connect() {
    console.log(`[SYS] Connecting to rosbridge at ${this.url} ...`);
    this.ros = new ROSLIB.Ros({ url: this.url });

    await new Promise((resolve, reject) => {
      const timer = setTimeout(
        () => reject(new Error("rosbridge connection timeout")),
        10000
      );
      this.ros.on("connection", () => {
        clearTimeout(timer);
        resolve();
      });
      this.ros.on("error", (err) => {
        clearTimeout(timer);
        reject(err instanceof Error ? err : new Error(String(err)));
      });
    });
    console.log("[SYS] Connected to rosbridge");

    this._initTopicsAndServices();

    await this._setHeartbeatParams().catch((err) =>
      console.warn("[SYS][WARN] Heartbeat param set failed:", err.message || err)
    );

    await this._waitState();
    await this._waitPose();
    await this._waitFix();

    console.log(`[SYS] Initial FCU mode: ${this.state?.mode}`);

    await this._restartSimIfAvailable();

    // Refresh telemetry after sim restart
    this.state = null;
    this.pose = null;
    this.fix = null;
    this.altitude = null;

    await this._waitState();
    await this._waitPose();
    await this._waitFix();

    this.home = { ...this.pose.pose.position };
    this.homeFix = { ...this.fix };
    console.log(
      `[SYS] Home local: x=${Number(this.home.x).toFixed(
        2
      )}, y=${Number(this.home.y).toFixed(2)}, z=${Number(this.home.z).toFixed(
        2
      )}`
    );
    console.log(
      `[SYS] Home GPS : lat=${Number(this.homeFix.latitude).toFixed(
        7
      )}, lon=${Number(this.homeFix.longitude).toFixed(7)}`
    );
  }

  _initTopicsAndServices() {
    this.stateSub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/mavros/state",
      messageType: "mavros_msgs/State"
    });
    this.stateSub.subscribe((msg) => {
      this.state = msg;
    });

    this.poseSub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/mavros/local_position/pose",
      messageType: "geometry_msgs/PoseStamped"
    });
    this.poseSub.subscribe((msg) => {
      this.pose = msg;
    });

    this.fixSub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/mavros/global_position/global",
      messageType: "sensor_msgs/NavSatFix"
    });
    this.fixSub.subscribe((msg) => {
      this.fix = msg;
    });

    this.altSub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/mavros/altitude",
      messageType: "mavros_msgs/Altitude"
    });
    this.altSub.subscribe((msg) => {
      this.altitude = msg;
    });

    this.velPub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/mavros/setpoint_velocity/cmd_vel",
      messageType: "geometry_msgs/TwistStamped"
    });

    this.modeSrv = new ROSLIB.Service({
      ros: this.ros,
      name: "/mavros/set_mode",
      serviceType: "mavros_msgs/SetMode"
    });

    this.armSrv = new ROSLIB.Service({
      ros: this.ros,
      name: "/mavros/cmd/arming",
      serviceType: "mavros_msgs/CommandBool"
    });

    this.cmdLongSrv = new ROSLIB.Service({
      ros: this.ros,
      name: "/mavros/cmd/command",
      serviceType: "mavros_msgs/CommandLong"
    });

    this.paramSrv = new ROSLIB.Service({
      ros: this.ros,
      name: "/mavros/sys/set_parameters",
      serviceType: "rcl_interfaces/srv/SetParameters"
    });

    this.simSrv = new ROSLIB.Service({
      ros: this.ros,
      name: "/simulation_manager/start_simulation",
      serviceType: "std_srvs/Trigger"
    });
  }

  async _waitState() {
    await waitFor(() => !!this.state, "/mavros/state");
  }

  async _waitPose() {
    await waitFor(() => !!this.pose, "/mavros/local_position/pose");
  }

  async _waitFix() {
    await waitFor(
      () =>
        !!this.fix &&
        typeof this.fix.latitude === "number" &&
        typeof this.fix.longitude === "number",
      "/mavros/global_position/global",
      15000,
      200
    );
  }

  async _setHeartbeatParams() {
    console.log("[SYS] Setting MAVROS heartbeat params...");
    const PARAMETER_DOUBLE = 3;
    const PARAMETER_STRING = 4;
    const request = {
      parameters: [
        {
          name: "heartbeat_mav_type",
          value: { type: PARAMETER_STRING, string_value: "GCS" }
        },
        {
          name: "heartbeat_rate",
          value: { type: PARAMETER_DOUBLE, double_value: 2.0 }
        }
      ]
    };
    const resp = await makeServiceCall(this.paramSrv, request, 5000);
    console.log("[SYS] Heartbeat param response:", resp?.results || []);
  }

  async _restartSimIfAvailable() {
    console.log("[SIM] Requesting simulation restart...");
    try {
      const resp = await makeServiceCall(
        this.simSrv,
        {},
        20000 /* allow longer */
      );
      console.log(
        `[SIM] success=${resp?.success ? "true" : "false"} msg=${
          resp?.message || ""
        }`
      );
    } catch (err) {
      console.warn("[SIM][WARN] restart failed or service missing:", err.message);
    }
  }

  async _setMode(customMode) {
    console.log(`[MODE] Setting mode: ${customMode}`);
    const resp = await makeServiceCall(this.modeSrv, {
      base_mode: 0,
      custom_mode: customMode
    });
    if (!resp?.mode_sent) {
      console.warn(`[MODE][WARN] mode_sent=false for ${customMode}`);
      return false;
    }

    const start = Date.now();
    while (Date.now() - start < 5000) {
      const mode = (this.state?.mode || "").toUpperCase();
      if (mode === customMode.toUpperCase()) {
        console.log(`[MODE] Mode is now ${mode}`);
        return true;
      }
      await sleep(100);
    }
    console.warn(
      `[MODE][WARN] Mode did not switch to ${customMode}, current=${this.state?.mode}`
    );
    return false;
  }

  async _arm() {
    console.log(`[ARM] Waiting ${ARM_WAIT.toFixed(1)}s before arming...`);
    await sleep(ARM_WAIT * 1000);
    console.log("[ARM] Sending arm command...");
    const resp = await makeServiceCall(this.armSrv, { value: true });
    if (!resp?.success) {
      throw new Error("Arming command rejected");
    }
    const start = Date.now();
    while (Date.now() - start < 7000) {
      if (this.state?.armed) {
        console.log("[ARM] Vehicle armed");
        return;
      }
      await sleep(100);
    }
    throw new Error("Vehicle did not arm in time");
  }

  _relativeAlt() {
    if (this.altitude && typeof this.altitude.relative === "number") {
      return Number(this.altitude.relative);
    }
    const z = Number(this.pose?.pose?.position?.z || 0);
    const base = Number(this.home?.z || 0);
    return Math.abs(z - base);
  }

  async _takeoffToAlt(alt) {
    console.log(`[TKOFF] Sending MAV_CMD_NAV_TAKEOFF to ${alt.toFixed(2)} m AGL`);
    const lat = Number(this.fix.latitude);
    const lon = Number(this.fix.longitude);
    const request = {
      command: 22,
      confirmation: 0,
      param1: 0.0,
      param2: 0.0,
      param3: 0.0,
      param4: 0.0,
      param5: lat,
      param6: lon,
      param7: Number(alt)
    };
    const resp = await makeServiceCall(this.cmdLongSrv, request, 5000);
    if (!resp?.success) {
      throw new Error(`NAV_TAKEOFF rejected (result=${resp?.result})`);
    }
    console.log("[TKOFF] Command accepted, waiting for AUTO.LOITER @ altitude...");

    const start = Date.now();
    while (true) {
      const mode = (this.state?.mode || "").toUpperCase();
      const rel = this._relativeAlt();
      console.log(`[TKOFF] mode=${mode} rel_alt=${rel.toFixed(2)}`);

      if (mode === "AUTO.LOITER" && Math.abs(rel - alt) < 0.4) {
        console.log("[TKOFF] Takeoff complete, in AUTO.LOITER near target alt");
        return;
      }
      if (Date.now() - start > TAKEOFF_TIMEOUT * 1000) {
        throw new Error("Timeout waiting for AUTO.LOITER at altitude");
      }
      if (!this.state?.armed) {
        throw new Error("Vehicle disarmed during takeoff");
      }
      await sleep(200);
    }
  }

  _publishVelocity(vx, vy, vz = 0.0) {
    const msg = new ROSLIB.Message({
      header: { frame_id: FRAME_ID },
      twist: {
        linear: { x: Number(vx), y: Number(vy), z: Number(vz) },
        angular: { x: 0.0, y: 0.0, z: 0.0 }
      }
    });
    this.velPub.publish(msg);
  }

  async _ensureOffboard() {
    console.log("[OFFB] Pre-streaming zero velocities...");
    const intervalMs = 1000 / SETPOINT_HZ;
    const end = Date.now() + 1500;
    while (Date.now() < end) {
      this._publishVelocity(0.0, 0.0, 0.0);
      await sleep(intervalMs);
    }

    console.log("[OFFB] Switching to OFFBOARD...");
    const ok = await this._setMode("OFFBOARD");
    if (!ok) {
      throw new Error("Failed to enter OFFBOARD mode");
    }
  }

  async _gotoLocalEnu(tx, ty, label = "") {
    console.log(`[LEG] ${label}: target local ENU (${tx.toFixed(1)}, ${ty.toFixed(1)})`);
    const timeoutAt = Date.now() + LAND_TIMEOUT * 1000;

    while (true) {
      const p = this.pose?.pose?.position;
      const cx = Number(p?.x || 0);
      const cy = Number(p?.y || 0);
      const dx = tx - cx;
      const dy = ty - cy;
      const dist = Math.hypot(dx, dy);
      const relAlt = this._relativeAlt();

      console.log(
        `[LEG] ${label}: dist=${dist.toFixed(2)} m, pos=(${cx.toFixed(
          2
        )},${cy.toFixed(2)}), alt=${relAlt.toFixed(2)}`
      );

      if (dist < RADIUS) {
        console.log(`[LEG] ${label}: within radius, leg complete`);
        this._publishVelocity(0.0, 0.0, 0.0);
        return;
      }

      if (Date.now() > timeoutAt) {
        throw new Error(`[LEG] ${label}: timeout reaching target`);
      }

      const v = dist > SLOW_RADIUS ? V_FAST : Math.max(V_MIN, (V_FAST * dist) / SLOW_RADIUS);
      const vx = (dx / dist) * v;
      const vy = (dy / dist) * v;

      const altErr = ALT_TARGET - relAlt;
      const vz =
        Math.abs(altErr) < 0.2
          ? 0.0
          : Math.max(-1.0, Math.min(1.0, altErr));

      this._publishVelocity(vx, vy, vz);
      await sleep(1000 / SETPOINT_HZ);
    }
  }

  async _landAndWaitDisarm() {
    console.log("[LAND] Stopping OFFBOARD velocities before landing");
    for (let i = 0; i < 0.5 * SETPOINT_HZ; i += 1) {
      this._publishVelocity(0.0, 0.0, 0.0);
      await sleep(1000 / SETPOINT_HZ);
    }

    console.log("[LAND] Leaving OFFBOARD to AUTO.LOITER");
    await this._setMode("AUTO.LOITER");
    await sleep(500);

    console.log("[LAND] Setting AUTO.LAND");
    const ok = await this._setMode("AUTO.LAND");
    if (!ok) {
      console.warn("[LAND][WARN] AUTO.LAND not confirmed, still waiting for disarm...");
    }

    const start = Date.now();
    while (Date.now() - start < LAND_TIMEOUT * 1000) {
      const armed = !!this.state?.armed;
      const mode = this.state?.mode;
      console.log(`[LAND] mode=${mode} armed=${armed}`);
      if (!armed) {
        console.log("[LAND] Vehicle disarmed, landing complete");
        return;
      }
      await sleep(1000);
    }
    console.warn("[LAND][WARN] Disarm not observed within timeout");
  }

  async runMission() {
    try {
      await this.connect();

      await this._arm();
      await this._takeoffToAlt(ALT_TARGET);
      await this._ensureOffboard();

      const hx = Number(this.home.x);
      const hy = Number(this.home.y);
      const waypoint = [hx + EDGE_M, hy + EDGE_M];

      await this._gotoLocalEnu(waypoint[0], waypoint[1], "HOP");
      await this._gotoLocalEnu(hx, hy, "HOME");

      await this._landAndWaitDisarm();
    } catch (err) {
      console.error("[ERROR]", err.message || err);
    } finally {
      console.log("[SYS] Shutting down");
      try {
        this.velPub?.unadvertise();
        this.stateSub?.unsubscribe();
        this.poseSub?.unsubscribe();
        this.fixSub?.unsubscribe();
        this.altSub?.unsubscribe();
      } catch (e) {
        // ignore
      }
      try {
        this.ros?.close();
      } catch (e) {
        // ignore
      }
    }
  }
}

async function main() {
  const controller = new GuidedMissionController(rosbridgeUrl);
  await controller.runMission();
}

if (require.main === module) {
  main().catch((err) => {
    console.error(err);
    process.exit(1);
  });
}
