#!/usr/bin/env node
/**
 * Shared configuration helpers for the front-camera YOLO demo.
 */

const fs = require("fs");
const path = require("path");
const yaml = require("js-yaml");

const DEFAULT_CONFIG = {
  vision: {
    topics: {
      image_topic: "/drone_camera/image_raw",
      annotated_image_topic: "/drone_camera/image_annotated",
      message_type: "sensor_msgs/Image",
    },
    yolo: {
      model_name: "yolov8n",
      model_path: "",
      confidence_threshold: 0.25,
      iou_threshold: 0.45,
      input_size: 640,
      target_label: "dining table",
    },
  },
};

let cachedConfig = null;

function readDroneConfigFile() {
  const configPath = path.join(process.cwd(), "config", "drone_config.yaml");
  try {
    return fs.readFileSync(configPath, "utf8");
  } catch {
    return null;
  }
}

function deepMerge(base, override) {
  if (!override || typeof override !== "object") {
    return Array.isArray(base) ? [...base] : { ...base };
  }

  const result = Array.isArray(base) ? [...base] : { ...base };
  for (const [key, value] of Object.entries(override)) {
    if (
      value &&
      typeof value === "object" &&
      !Array.isArray(value) &&
      result[key] &&
      typeof result[key] === "object" &&
      !Array.isArray(result[key])
    ) {
      result[key] = deepMerge(result[key], value);
    } else {
      result[key] = value;
    }
  }
  return result;
}

function loadDroneConfig() {
  if (cachedConfig) {
    return cachedConfig;
  }

  const fileText = readDroneConfigFile();
  if (!fileText) {
    cachedConfig = DEFAULT_CONFIG;
    return cachedConfig;
  }

  try {
    const parsed = yaml.load(fileText) || {};
    cachedConfig = deepMerge(DEFAULT_CONFIG, parsed);
  } catch (err) {
    console.warn(`[DRONE-CONFIG] Failed to parse config/drone_config.yaml: ${err.message || err}`);
    cachedConfig = DEFAULT_CONFIG;
  }

  return cachedConfig;
}

function envNumber(name, fallback) {
  const raw = process.env[name];
  if (raw === undefined || raw === "") {
    return fallback;
  }
  const value = Number(raw);
  return Number.isFinite(value) ? value : fallback;
}

function envString(name, fallback) {
  const raw = process.env[name];
  return raw !== undefined && raw !== "" ? raw : fallback;
}

function getImageTopics() {
  const config = loadDroneConfig();
  const topics = config.vision?.topics || DEFAULT_CONFIG.vision.topics;

  return {
    imageTopic: envString("IMAGE_TOPIC", topics.image_topic),
    annotatedImageTopic: envString("ANNOTATED_IMAGE_TOPIC", topics.annotated_image_topic),
    messageType: envString("IMAGE_MESSAGE_TYPE", topics.message_type),
  };
}

function getYoloConfig() {
  const config = loadDroneConfig();
  const base = config.vision?.yolo || DEFAULT_CONFIG.vision.yolo;

  return {
    modelName: envString("YOLO_MODEL_NAME", envString("YOLO_MODEL", base.model_name)),
    modelPath: envString("YOLO_MODEL_PATH", base.model_path || ""),
    confThreshold: envNumber("YOLO_CONFIDENCE_THRESHOLD", base.confidence_threshold),
    iouThreshold: envNumber("YOLO_IOU_THRESHOLD", base.iou_threshold),
    inputSize: envNumber("YOLO_INPUT_SIZE", base.input_size),
    targetLabel: envString("YOLO_TARGET_LABEL", base.target_label),
  };
}

module.exports = {
  loadDroneConfig,
  getImageTopics,
  getYoloConfig,
};
