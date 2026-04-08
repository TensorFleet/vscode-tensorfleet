#!/usr/bin/env node
/**
 * Shared configuration helpers for the drone vision demos.
 */

const fs = require("fs");
const path = require("path");
const yaml = require("js-yaml");

const DEFAULT_CONFIG = {
  vision: {
    topics: {
      image_topic: "/drone_camera/image_raw",
      annotated_image_topic: "/drone_camera/image_annotated",
      landing_image_topic: "/drone_camera/down/image_raw",
      landing_annotated_image_topic: "/drone_camera/down/image_annotated",
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
    landing_demo: {
      takeoff_altitude_m: 4.0,
      xy_gain: 1.3,
      max_xy_speed: 1.0,
      descent_speed_mps: 0.45,
      slow_descent_speed_mps: 0.18,
      centered_tolerance: 0.10,
      descent_tolerance: 0.18,
      min_blob_pixels: 150,
      handoff_area_ratio: 0.16,
      stable_frames_required: 12,
      loop_hz: 10,
      pad_color: "magenta",
      frame_timeout_ms: 1500,
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
    landingImageTopic: envString("LANDING_IMAGE_TOPIC", topics.landing_image_topic),
    landingAnnotatedImageTopic: envString(
      "LANDING_ANNOTATED_IMAGE_TOPIC",
      topics.landing_annotated_image_topic
    ),
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

function getLandingConfig() {
  const config = loadDroneConfig();
  const base = config.vision?.landing_demo || DEFAULT_CONFIG.vision.landing_demo;

  return {
    takeoffAltitudeM: envNumber("LANDING_TAKEOFF_ALTITUDE_M", base.takeoff_altitude_m),
    xyGain: envNumber("LANDING_XY_GAIN", base.xy_gain),
    maxXySpeed: envNumber("LANDING_MAX_XY_SPEED", base.max_xy_speed),
    descentSpeedMps: envNumber("LANDING_DESCENT_SPEED_MPS", base.descent_speed_mps),
    slowDescentSpeedMps: envNumber(
      "LANDING_SLOW_DESCENT_SPEED_MPS",
      base.slow_descent_speed_mps
    ),
    centeredTolerance: envNumber("LANDING_CENTERED_TOLERANCE", base.centered_tolerance),
    descentTolerance: envNumber("LANDING_DESCENT_TOLERANCE", base.descent_tolerance),
    minBlobPixels: envNumber("LANDING_MIN_BLOB_PIXELS", base.min_blob_pixels),
    handoffAreaRatio: envNumber("LANDING_HANDOFF_AREA_RATIO", base.handoff_area_ratio),
    stableFramesRequired: envNumber("LANDING_STABLE_FRAMES_REQUIRED", base.stable_frames_required),
    loopHz: envNumber("LANDING_LOOP_HZ", base.loop_hz),
    padColor: envString("LANDING_PAD_COLOR", base.pad_color),
    frameTimeoutMs: envNumber("LANDING_FRAME_TIMEOUT_MS", base.frame_timeout_ms),
  };
}

module.exports = {
  loadDroneConfig,
  getImageTopics,
  getYoloConfig,
  getLandingConfig,
};
