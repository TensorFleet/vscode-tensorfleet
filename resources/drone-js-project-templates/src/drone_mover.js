#!/usr/bin/env node
/**
 * Simple PX4/MAVROS OFFBOARD velocity mission using roslib + rosbridge.
 *
 * Flow:
 *  - Connect to rosbridge
 *  - Arm + AUTO.TAKEOFF to target altitude
 *  - Switch to OFFBOARD and stream velocity setpoints to waypoints
 *  - Command AUTO.LAND and wait for disarm
 *
 * Run (remote VM or local):
 *   - Ensure .env contains ROSBRIDGE_URL or R2B_HOST/R2B_PORT (the VS Code
 *     extension refreshes this automatically from .tensorfleet metadata), or
 *   - Set ROSBRIDGE_URL in environment and run:
 *       bun src/drone_mover.js
 */

require("dotenv").config();
const fs = require("fs");
const path = require("path");
const ROSLIB = require("roslib");
const { getTensorfleetSettings } = require("./lib/tensorfleet_config");
const { connectToDrone, sleep, waitFor, makeServiceCall } = require("./lib/drone_utils");

function loadMissionPlan() {
  const planPath = path.join(process.cwd(), "missions", "example_mission.plan");
  try {
    const contents = fs.readFileSync(planPath, "utf8");
    return JSON.parse(contents);
  } catch (err) {
    console.warn(
      `[CFG] Could not load mission plan at ${planPath}, continuing with defaults. ${err.message}`
    );
    return null;
  }
}

function buildPlanWaypoints(plan, homeLocal) {
  if (!plan || !plan.mission || !Array.isArray(plan.mission.waypoints)) return [];

  const hx = Number(homeLocal?.x || 0);
  const hy = Number(homeLocal?.y || 0);
  return plan.mission.waypoints
    .map((item) => {
      if (!Array.isArray(item.offset) || item.offset.length < 2) return null;
      const [dx, dy] = item.offset;
      if (typeof dx !== "number" || typeof dy !== "number") return null;
      return {
        x: hx + dx,
        y: hy + dy,
        label: item.label || "WP"
      };
    })
    .filter(Boolean);
}

function numEnv(key, fallback) {
  const raw = process.env[key];
  if (raw === undefined) return fallback;
  const parsed = Number(raw);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function buildSettings() {
  const tf = getTensorfleetSettings();

  // Offboard / mission tuning comes from env with fallback to YAML.
  const cfgPath = path.join(process.cwd(), "config", "drone_config.yaml");
  let cfg = {};
  try {
    const contents = fs.readFileSync(cfgPath, "utf8");
    cfg = (require("js-yaml").load(contents) || {});
  } catch (err) {
    console.warn(`[CFG] Could not reload config at ${cfgPath}, using defaults. ${err.message}`);
  }

  const offboard = cfg.offboard || {};

  const altTarget = numEnv("ALT_TARGET", offboard.alt_target ?? 3.0);
  const edgeMeters = numEnv("EDGE_M", offboard.edge_m ?? 200.0);
  const waypointRadius = numEnv("WAYPOINT_RADIUS", offboard.waypoint_radius ?? 2.0);
  const slowRadius = numEnv("SLOW_RADIUS", offboard.slow_radius ?? 10.0);
  const vFast = numEnv("V_FAST", offboard.v_fast ?? 20.0);
  const vMin = numEnv("V_MIN", offboard.v_min ?? 1.0);
  const armWaitSec = numEnv("ARM_WAIT", offboard.arm_wait ?? 3.0);
  const takeoffTimeoutSec = numEnv(
    "TAKEOFF_TIMEOUT",
    offboard.takeoff_timeout ?? 60.0
  );
  const landTimeoutSec = numEnv(
    "LAND_TIMEOUT",
    offboard.land_timeout ?? 300.0
  );
  const setpointHz = numEnv("SETPOINT_HZ", offboard.setpoint_hz ?? 20.0);

  return {
    altTarget,
    edgeMeters,
    waypointRadius,
    slowRadius,
    vFast,
    vMin,
    armWaitSec,
    takeoffTimeoutSec,
    landTimeoutSec,
    setpointHz,
    frameId: tf.frameId,
    rosbridgeUrl: tf.rosbridgeUrl,
  };
}

const SETTINGS = buildSettings();

class GuidedMissionController {
  constructor(settings) {
    this.settings = settings;
    this.ros = null;

    this.state = null;
    this.pose = null;
    this.fix = null;
    this.altitude = null;
    this.home = null;
    this.homeFix = null;
  }

  async connect() {
    const settings = getTensorfleetSettings();

    // Use shared connection logic from drone_utils
    this.ros = await connectToDrone(settings.rosbridgeUrl);

    console.log("[SYS] Connected to rosbridge");

    this._initTopicsAndServices();

    await this._waitState();
    await this._waitPose();
    await this._waitFix();

    console.log(`[SYS] Initial FCU mode: ${this.state?.mode}`);

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
    const armWaitSec = this.settings.armWaitSec;
    console.log(`[ARM] Waiting ${armWaitSec.toFixed(1)}s before arming...`);
    await sleep(armWaitSec * 1000);
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
    const { takeoffTimeoutSec } = this.settings;

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
      if (Date.now() - start > takeoffTimeoutSec * 1000) {
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
      header: { frame_id: this.settings.frameId },
      twist: {
        linear: { x: Number(vx), y: Number(vy), z: Number(vz) },
        angular: { x: 0.0, y: 0.0, z: 0.0 }
      }
    });
    this.velPub.publish(msg);
  }

  async _ensureOffboard() {
    console.log("[OFFB] Pre-streaming zero velocities...");
    const intervalMs = 1000 / this.settings.setpointHz;
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
    const {
      landTimeoutSec,
      waypointRadius,
      slowRadius,
      vFast,
      vMin,
      altTarget,
      setpointHz
    } = this.settings;

    console.log(`[LEG] ${label}: target local ENU (${tx.toFixed(1)}, ${ty.toFixed(1)})`);
    const timeoutAt = Date.now() + landTimeoutSec * 1000;

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

      if (dist < waypointRadius) {
        console.log(`[LEG] ${label}: within radius, leg complete`);
        this._publishVelocity(0.0, 0.0, 0.0);
        return;
      }

      if (Date.now() > timeoutAt) {
        throw new Error(`[LEG] ${label}: timeout reaching target`);
      }

      const v = dist > slowRadius ? vFast : Math.max(vMin, (vFast * dist) / slowRadius);
      const vx = (dx / dist) * v;
      const vy = (dy / dist) * v;

      const altErr = altTarget - relAlt;
      const vz =
        Math.abs(altErr) < 0.2
          ? 0.0
          : Math.max(-1.0, Math.min(1.0, altErr));

      this._publishVelocity(vx, vy, vz);
      await sleep(1000 / setpointHz);
    }
  }

  async _landAndWaitDisarm() {
    const { setpointHz, landTimeoutSec } = this.settings;

    console.log("[LAND] Stopping OFFBOARD velocities before landing");
    for (let i = 0; i < 0.5 * setpointHz; i += 1) {
      this._publishVelocity(0.0, 0.0, 0.0);
      await sleep(1000 / setpointHz);
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
    while (Date.now() - start < landTimeoutSec * 1000) {
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
    const { altTarget, edgeMeters } = this.settings;

    try {
      await this.connect();

      await this._arm();
      await this._takeoffToAlt(altTarget);
      await this._ensureOffboard();

      const hx = Number(this.home.x);
      const hy = Number(this.home.y);

      const plan = loadMissionPlan();
      let waypoints = buildPlanWaypoints(plan, this.home);

      if (!waypoints.length) {
        // Fallback: small circle around home if no mission plan
        const radius = Math.min(edgeMeters, 5.0);
        const steps = 8;
        waypoints = [];
        for (let i = 0; i < steps; i += 1) {
          const angle = (2 * Math.PI * i) / steps;
          const x = hx + radius * Math.cos(angle);
          const y = hy + radius * Math.sin(angle);
          waypoints.push({ x, y, label: `CIRCLE_${i + 1}` });
        }
      }

      for (const wp of waypoints) {
        await this._gotoLocalEnu(wp.x, wp.y, wp.label);
      }

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
  const controller = new GuidedMissionController(SETTINGS);
  await controller.runMission();
}

if (require.main === module) {
  main().catch((err) => {
    console.error(err);
    process.exit(1);
  });
}
