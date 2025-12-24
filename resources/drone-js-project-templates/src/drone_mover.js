#!/usr/bin/env -S bun run
/**
 * Automated PX4/MAVROS mission using DroneController with auto state management.
 *
 * Flow:
 *  - Connect to drone using DroneController
 *  - Generate R-shaped curve waypoints (6 points)
 *  - Automated takeoff to target altitude
 *  - Navigate through R-shaped curve using OFFBOARD position targets
 *  - Automated landing and disarm
 *
 * The R shape is scaled by R_SIZE_METERS constant (default: 30.0m)
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
const { initializeDroneControl } = require("./lib/drone_utils");

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

// Size constant to scale the R shape
const R_SIZE_METERS = 30.0; // Scale factor for the R curve

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

// Generate R-shaped curve waypoints using 6 position points
function generateRShapeWaypoints(home, size) {
  const points = [];
  const half = size * 0.5;

  // 1. The Vertical Spine (Bottom to Top)
  points.push({ x: home.x - half, y: home.y - half, label: "SPINE_BOTTOM" });
  points.push({ x: home.x - half, y: home.y + half, label: "SPINE_TOP" });

  // 2. The Curved Top (6-point resolution)
  // We curve from the top-left, around the right, and back to the middle-left
  const curvePoints = 6;
  for (let i = 0; i <= curvePoints; i++) {
    // Angle goes from 90 degrees (top) to -90 degrees (middle)
    const angle = (Math.PI / 2) - (Math.PI * (i / curvePoints));
    points.push({
      x: (home.x - half) + (half * Math.cos(angle)) + half, // Offset to right of spine
      y: (home.y + half * 0.5) + (half * 0.5 * Math.sin(angle)),
      label: `CURVE_${i}`
    });
  }

  // 3. The Diagonal Leg (From the middle of the spine to bottom-right)
  points.push({ x: home.x - half, y: home.y, label: "LEG_START" });
  points.push({ x: home.x + half, y: home.y - half, label: "LEG_END" });

  return points;
}

function numEnv(key, fallback) {
  const raw = process.env[key];
  if (raw === undefined) return fallback;
  const parsed = Number(raw);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function buildSettings() {
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

  const altTarget = numEnv("ALT_TARGET", offboard.alt_target ?? 6.0);

  return {
    altTarget,
  };
}

const SETTINGS = buildSettings();

async function runMission() {
  const { altTarget } = SETTINGS;

  // Initialize drone control with automated state management
  const { bridge, droneState, droneController, currentState } = await initializeDroneControl();

  console.log(`[SYS] Connected to drone. Current state: armed=${currentState.vehicle?.armed}, mode=${currentState.vehicle?.mode}`);

  try {
    // Record home position
    const homeState = await droneState.getState();
    const home = {
      x: homeState.pose?.position?.x || 0,
      y: homeState.pose?.position?.y || 0,
      z: homeState.pose?.position?.z || 0
    };

    console.log(`[SYS] Home position: (${home.x.toFixed(2)}, ${home.y.toFixed(2)}, ${home.z.toFixed(2)})`);

    // Generate R-shaped curve waypoints using 6 position points
    const waypoints = generateRShapeWaypoints(home, R_SIZE_METERS);

    console.log(`[MISSION] Generated R-shaped path with ${waypoints.length} waypoints, size: ${R_SIZE_METERS}m`);

    // Step 1: Automated takeoff to target altitude
    console.log(`[MISSION] Taking off to ${altTarget}m altitude...`);
    await droneController.requestAutoState({ kind: "airborne", altMeters: altTarget });
    console.log("[MISSION] Takeoff complete");

    // Step 2: Navigate through R-shaped curve using OFFBOARD position targets
    for (const wp of waypoints) {
      console.log(`[MISSION] Navigating to waypoint: ${wp.label} at (${wp.x.toFixed(2)}, ${wp.y.toFixed(2)})`);
      await droneController.requestAutoState({
        kind: "offboard",
        target: {
          kind: "position_local",
          x: wp.x,
          y: wp.y,
          z: altTarget
        }
      });
      console.log(`[MISSION] Reached waypoint: ${wp.label}`);
    }

    // Step 3: Automated landing and disarm (no return home, land after drawing R)
    console.log("[MISSION] R shape complete, landing and disarming...");
    await droneController.requestAutoState({ kind: "landed", armed: false });
    console.log("[MISSION] Mission complete - R shape drawn and drone landed and disarmed");

  } catch (err) {
    console.error("[ERROR]", err.message || err);
  } finally {
    console.log("[SYS] Shutting down");
    droneState.disconnect();
    process.exit(0);
  }
}

async function main() {
  await runMission();
}

if (require.main === module) {
  main().catch((err) => {
    console.error(err);
    process.exit(1);
  });
}
