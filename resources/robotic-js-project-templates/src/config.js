#!/usr/bin/env node
/**
 * Shared configuration utilities for the robotic JS templates.
 *
 * This module:
 *   - Reads defaults from config/robot_config.yaml when available
 *   - Lets environment variables override host/port and topic names
 *   - Provides small helpers so example scripts share one config surface
 */

const fs = require("fs");
const path = require("path");

const DEFAULT_CONFIG = {
  network: {
    vm_ip: "172.16.0.10",
    rosbridge_url: "ws://172.16.0.10:9091",
    foxglove_bridge_url: "ws://172.16.0.10:8765"
  },
  motion: {
    cmd_vel_topic: "/cmd_vel_raw"
  },
  vision: {
    yolo: {
      // Smallest/default YOLO model name
      model_name: "yolov8n",
      // Optional explicit ONNX model path (overrides name)
      model_path: "",
      // Execution device; JS implementation is CPU‑only for now
      device: "cpu"
    }
  }
};

let cachedConfig = null;

function readRobotConfigFile() {
  const configPath = path.join(__dirname, "..", "config", "robot_config.yaml");
  try {
    return fs.readFileSync(configPath, "utf8");
  } catch {
    return null;
  }
}

function parseYamlValue(text, key) {
  if (!text) return undefined;
  const regex = new RegExp(`${key}:\\s*["']?([^"']+)["']?`);
  const match = text.match(regex);
  return match ? match[1].trim() : undefined;
}

function loadRobotConfig() {
  if (cachedConfig) {
    return cachedConfig;
  }

  const fileText = readRobotConfigFile();
  if (!fileText) {
    cachedConfig = DEFAULT_CONFIG;
    return cachedConfig;
  }

  const vmIp = parseYamlValue(fileText, "vm_ip");
  const rosbridgeUrl = parseYamlValue(fileText, "rosbridge_url");
  const foxgloveBridgeUrl = parseYamlValue(fileText, "foxglove_bridge_url");
  const cmdVelTopic = parseYamlValue(fileText, "cmd_vel_topic");

  cachedConfig = {
    network: {
      vm_ip: vmIp || DEFAULT_CONFIG.network.vm_ip,
      rosbridge_url: rosbridgeUrl || DEFAULT_CONFIG.network.rosbridge_url,
      foxglove_bridge_url:
        foxgloveBridgeUrl || DEFAULT_CONFIG.network.foxglove_bridge_url
    },
    motion: {
      cmd_vel_topic: cmdVelTopic || DEFAULT_CONFIG.motion.cmd_vel_topic
    }
  };

  return cachedConfig;
}

function getEnvNumber(name, defaultValue) {
  const raw = process.env[name];
  if (raw === undefined || raw === "") {
    return defaultValue;
  }
  const value = Number(raw);
  return Number.isFinite(value) ? value : defaultValue;
}

function getRosConnectionDetails() {
  const config = loadRobotConfig();

  const envUrl = process.env.ROSBRIDGE_URL;
  const envHost = process.env.ROS_HOST;
  const envPortRaw = process.env.ROS_PORT;

  let scheme = "ws";
  let host = envHost;
  let port =
    envPortRaw !== undefined && envPortRaw !== ""
      ? Number(envPortRaw)
      : undefined;

  if (envUrl) {
    try {
      const parsed = new URL(envUrl);
      if (parsed.protocol) {
        scheme = parsed.protocol.replace(":", "") || scheme;
      }
      if (!host) host = parsed.hostname;
      if (!Number.isFinite(port) && parsed.port) {
        const p = Number(parsed.port);
        if (Number.isFinite(p)) port = p;
      }
    } catch {
      // Ignore malformed ROSBRIDGE_URL and fall back to other sources.
    }
  }

  if (!host || !Number.isFinite(port)) {
    const cfgUrl = config.network?.rosbridge_url;
    if (cfgUrl) {
      try {
        const parsed = new URL(cfgUrl);
        if (parsed.protocol) {
          scheme = parsed.protocol.replace(":", "") || scheme;
        }
        if (!host) host = parsed.hostname;
        if (!Number.isFinite(port) && parsed.port) {
          const p = Number(parsed.port);
          if (Number.isFinite(p)) port = p;
        }
      } catch {
        // Ignore malformed rosbridge_url and fall back further.
      }
    }
  }

  if (!host) {
    host = config.network?.vm_ip || DEFAULT_CONFIG.network.vm_ip;
  }
  if (!Number.isFinite(port)) {
    port = 9091;
  }

  const url = `${scheme}://${host}:${port}`;
  return { scheme, host, port, url };
}

function getCmdVelTopic() {
  const config = loadRobotConfig();
  return (
    process.env.CMD_VEL_TOPIC ||
    config.motion?.cmd_vel_topic ||
    DEFAULT_CONFIG.motion.cmd_vel_topic
  );
}

function getScanTopic() {
  return process.env.SCAN_TOPIC || "/scan";
}

function getImageTopics() {
  return {
    imageTopic: process.env.IMAGE_TOPIC || "/camera/image_raw",
    annotatedImageTopic:
      process.env.ANNOTATED_IMAGE_TOPIC || "/camera/image_annotated",
    messageType:
      process.env.IMAGE_MESSAGE_TYPE || "sensor_msgs/Image"
  };
}

function getMovementSpeeds() {
  return {
    linearSpeed: getEnvNumber("LINEAR_SPEED", 0.2),
    angularSpeed: getEnvNumber("ANGULAR_SPEED", 0.5)
  };
}

function getObstacleParams() {
  return {
    obstacleDistance: getEnvNumber("OBSTACLE_DISTANCE", 0.5),
    clearDistance: getEnvNumber("CLEAR_DISTANCE", 1.0),
    linearSpeed: getEnvNumber("LINEAR_SPEED", 3.0),
    angularSpeed: getEnvNumber("ANGULAR_SPEED", 4.0)
  };
}

function getYoloConfig() {
  const config = loadRobotConfig();
  const base = (config.vision && config.vision.yolo) || DEFAULT_CONFIG.vision.yolo;

  const modelName =
    process.env.YOLO_MODEL_NAME ||
    process.env.YOLO_MODEL ||
    base.model_name ||
    DEFAULT_CONFIG.vision.yolo.model_name;

  const modelPath =
    process.env.YOLO_MODEL_PATH ||
    base.model_path ||
    DEFAULT_CONFIG.vision.yolo.model_path;

  // We currently only support CPU execution in JS.
  const deviceEnv = (process.env.YOLO_DEVICE || process.env.YOLO_BACKEND || "").toLowerCase();
  const device =
    deviceEnv === "cpu" || deviceEnv === ""
      ? "cpu"
      : "cpu";
  return {
    modelName,
    modelPath,
    device
  };
}

module.exports = {
  loadRobotConfig,
  getRosConnectionDetails,
  getCmdVelTopic,
  getScanTopic,
  getImageTopics,
  getMovementSpeeds,
  getObstacleParams,
  getYoloConfig
};
