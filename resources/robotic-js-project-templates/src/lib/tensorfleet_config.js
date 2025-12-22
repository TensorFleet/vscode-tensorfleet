/**
 * TensorFleet configuration helper for robotic JS template.
 *
 * Responsibility:
 * - Load connection-related settings from environment variables (dotenv),
 *   falling back to config/robot_config.yaml defaults.
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
    const configPath = path.join(process.cwd(), "config", "robot_config.yaml");
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
 * Load configuration sources (YAML config and .tensorfleet metadata)
 */
function loadConfigSources() {
    const cfg = loadYamlConfig();
    const marker = loadTensorfleetMetadata();
    const markerEnv = marker?.env ?? {};
    const network = cfg.network || {};

    return { cfg, marker, markerEnv, network };
}

/**
 * Resolve the base/VM manager URL from various sources
 */
function resolveBaseUrl(markerEnv) {
    return firstNonEmpty(
        process.env.TENSORFLEET_BASE_URL,
        process.env.TENSORFLEET_VM_MANAGER_URL,
        markerEnv.baseUrl,
        markerEnv.vmManagerUrl
    ) || "";
}

/**
 * Resolve rosbridge connection details (host, port, URL)
 */
function resolveRosbridgeConnection(markerEnv, network) {
    const portStr =
        process.env.R2B_PORT ||
        process.env.ROSBRIDGE_PORT ||
        markerEnv.rosbridgePort ||
        network.rosbridge_port ||
        "9091";
    const port = Number(portStr) || 9091;

    const host =
        process.env.R2B_HOST ||
        process.env.ROS_HOST ||
        markerEnv.r2bHost ||
        network.vm_ip ||
        "";

    const rosbridgeUrl =
        process.env.ROSBRIDGE_URL ||
        markerEnv.rosbridgeUrl ||
        network.rosbridge_url ||
        (host ? `ws://${host}:${port}` : `ws://127.0.0.1:${port}`);

    return { host, port, rosbridgeUrl };
}

/**
 * Resolve proxy connection details
 */
function resolveProxyConnection(baseUrl, markerEnv) {
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

    return { vmManagerUrl, nodeId, token, proxyUrl, useProxy };
}

/**
 * Build resolved TensorFleet settings for this workspace.
 *
 * Precedence:
 *   1. Environment variables (from .env, tasks, or shell)
 *   2. config/robot_config.yaml values
 *   3. Hardcoded safe defaults
 */
function getTensorfleetSettings() {
    const { markerEnv, network } = loadConfigSources();

    const baseUrl = resolveBaseUrl(markerEnv);
    const { host, port, rosbridgeUrl } = resolveRosbridgeConnection(markerEnv, network);
    const { vmManagerUrl, nodeId, token, proxyUrl, useProxy } = resolveProxyConnection(baseUrl, markerEnv);

    const fallbackHost = host || markerEnv.r2bHost || "127.0.0.1";

    return {
        baseUrl: vmManagerUrl || baseUrl || "",
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
