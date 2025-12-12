/**
 * TensorFleet configuration helper for drone JS template.
 *
 * Responsibility:
 * - Load connection-related settings from environment variables (dotenv),
 *   falling back to config/drone_config.yaml defaults.
 * - Decide whether to connect directly to rosbridge or via VM Manager proxy.
 *
 * This file is copied into new workspaces by the VS Code extension.
 */

require("dotenv").config();

const fs = require("fs");
const path = require("path");
const yaml = require("js-yaml");
const { toProxyWebSocketUrl } = require("./url_utils");

function firstNonEmpty(...values) {
  for (const value of values) {
    if (typeof value === "string" && value.trim() !== "") {
      return value;
    }
  }
  return "";
}

function loadYamlConfig() {
  const configPath = path.join(process.cwd(), "config", "drone_config.yaml");
  try {
    const contents = fs.readFileSync(configPath, "utf8");
    return yaml.load(contents) || {};
  } catch (err) {
    console.warn(
      `[TF-CONFIG] Could not load config at ${configPath}, using defaults. ${err.message}`
    );
    return {};
  }
}

function loadTensorfleetMetadata() {
  const markerPath = path.join(process.cwd(), ".tensorfleet");
  try {
    const raw = fs.readFileSync(markerPath, "utf8").trim();
    if (!raw) return {};
    return JSON.parse(raw);
  } catch (err) {
    if (err?.code !== "ENOENT") {
      console.warn(
        `[TF-CONFIG] Could not parse .tensorfleet at ${markerPath}: ${err.message || err}`
      );
    }
    return {};
  }
}

function numEnv(key, fallback) {
  const raw = process.env[key];
  if (raw === undefined) return fallback;
  const parsed = Number(raw);
  return Number.isFinite(parsed) ? parsed : fallback;
}

/**
 * Build resolved TensorFleet settings for this workspace.
 *
 * Precedence:
 *   1. Environment variables (from .env, tasks, or shell)
 *   2. config/drone_config.yaml values
 *   3. Hardcoded safe defaults
 */
function getTensorfleetSettings() {
  const cfg = loadYamlConfig();
  const network = cfg.network || {};
  const marker = loadTensorfleetMetadata();
  const markerEnv =
    marker && typeof marker === "object" && typeof marker.env === "object"
      ? marker.env
      : {};

  const frameId =
    process.env.SETPOINT_FRAME_ID ||
    markerEnv.frameId ||
    network.setpoint_frame ||
    "map";

  const baseUrl =
    firstNonEmpty(
      process.env.TENSORFLEET_BASE_URL,
      process.env.TENSORFLEET_VM_MANAGER_URL,
      markerEnv.baseUrl,
      markerEnv.vmManagerUrl
    ) || "";

  const portStr =
    process.env.R2B_PORT ||
    process.env.ROSBRIDGE_PORT ||
    markerEnv.rosbridgePort ||
    network.rosbridge_port ||
    "9091";
  const port = Number(portStr) || 9091;

  const host =
    process.env.R2B_HOST ||
    markerEnv.r2bHost ||
    network.vm_ip ||
    "";

  const rosbridgeUrl =
    process.env.ROSBRIDGE_URL ||
    markerEnv.rosbridgeUrl ||
    network.rosbridge_url ||
    (host ? `ws://${host}:${port}` : `ws://127.0.0.1:${port}`);

  const vmManagerUrl =
    process.env.TENSORFLEET_VM_MANAGER_URL ||
    markerEnv.vmManagerUrl ||
    baseUrl ||
    "";
  const nodeId = process.env.TENSORFLEET_NODE_ID || markerEnv.nodeId || "";
  const token = process.env.TENSORFLEET_JWT || "";

  const explicitProxyUrl =
    process.env.TENSORFLEET_PROXY_URL || markerEnv.proxyUrl || "";
  const proxyUrl =
    explicitProxyUrl ||
    (vmManagerUrl ? toProxyWebSocketUrl(vmManagerUrl) : "") ||
    (baseUrl ? toProxyWebSocketUrl(baseUrl) : "");
  const useProxy = Boolean(proxyUrl && nodeId && token);
  const fallbackHost = host || markerEnv.r2bHost || "127.0.0.1";

  return {
    baseUrl: vmManagerUrl || baseUrl || "",
    frameId,
    rosbridgeUrl,
    host: fallbackHost,
    port: Number(port),
    vmManagerUrl,
    nodeId,
    token,
    useProxy,
    proxyUrl,
    targetPort: numEnv(
      "ROSBRIDGE_PORT",
      markerEnv.rosbridgePort ?? network.rosbridge_port ?? 9091
    ),
  };
}

module.exports = {
  getTensorfleetSettings,
};
