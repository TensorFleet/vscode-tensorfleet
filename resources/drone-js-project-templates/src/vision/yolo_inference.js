#!/usr/bin/env node
/**
 * Minimal YOLOv8 ONNX inference helper for drone image topics.
 */

const fs = require("fs");
const https = require("https");
const path = require("path");
const jpeg = require("jpeg-js");
const ort = require("onnxruntime-node");
const { getYoloConfig } = require("../config");

const COCO_CLASSES = [
  "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
  "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
  "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
  "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
  "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
  "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
  "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
  "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
  "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
  "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
  "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush",
];

const DEFAULT_MODEL_URLS = {
  yolov8n: "https://huggingface.co/Kalray/yolov8/resolve/main/yolov8n.onnx",
  yolov8s: "https://huggingface.co/Kalray/yolov8/resolve/main/yolov8s.onnx",
  yolov8m: "https://huggingface.co/Kalray/yolov8/resolve/main/yolov8m.onnx",
  yolov8l: "https://huggingface.co/Kalray/yolov8/resolve/main/yolov8l.onnx",
  yolov8x: "https://huggingface.co/Kalray/yolov8/resolve/main/yolov8x.onnx",
};

let sessionPromise = null;

function resolveModelPath() {
  const { modelName, modelPath } = getYoloConfig();
  if (modelPath && modelPath.trim() !== "") {
    return path.resolve(modelPath);
  }
  return path.join(process.cwd(), "models", `${modelName}.onnx`);
}

function getModelDownloadUrl() {
  const { modelName } = getYoloConfig();
  const override = process.env.YOLO_MODEL_URL;
  if (override && override.trim() !== "") {
    return override.trim();
  }
  return DEFAULT_MODEL_URLS[modelName] || null;
}

function downloadFile(url, destination) {
  return new Promise((resolve, reject) => {
    fs.mkdirSync(path.dirname(destination), { recursive: true });
    const request = https.get(url, (response) => {
      if (response.statusCode >= 300 && response.statusCode < 400 && response.headers.location) {
        response.resume();
        downloadFile(response.headers.location, destination).then(resolve).catch(reject);
        return;
      }

      if (response.statusCode !== 200) {
        response.resume();
        reject(new Error(`Failed to download YOLO model (HTTP ${response.statusCode})`));
        return;
      }

      const file = fs.createWriteStream(destination);
      response.pipe(file);
      file.on("finish", () => file.close(resolve));
      file.on("error", (err) => {
        fs.unlink(destination, () => reject(err));
      });
    });

    request.on("error", (err) => {
      fs.unlink(destination, () => reject(err));
    });
  });
}

async function ensureModelFile(modelFile) {
  if (fs.existsSync(modelFile)) {
    return;
  }

  const url = getModelDownloadUrl();
  if (!url) {
    throw new Error(`YOLO model not found at '${modelFile}' and no download URL is configured.`);
  }

  console.log(`[YOLO] Downloading model to ${modelFile}`);
  await downloadFile(url, modelFile);
}

async function getSession() {
  if (!sessionPromise) {
    sessionPromise = (async () => {
      const modelFile = resolveModelPath();
      await ensureModelFile(modelFile);
      console.log(`[YOLO] Loading model from ${modelFile}`);
      return ort.InferenceSession.create(modelFile, {
        executionProviders: ["cpu"],
        graphOptimizationLevel: "all",
      });
    })();
  }
  return sessionPromise;
}

function decodeImageToRgbBuffer(msg) {
  let width = msg.width;
  let height = msg.height;
  const encoding = (msg.encoding || "rgb8").toLowerCase();
  const dataField = msg.data;

  if (!dataField) {
    throw new Error("Invalid image message: missing data field");
  }

  if (typeof dataField === "string" && dataField.startsWith("data:image/")) {
    const commaIndex = dataField.indexOf(",");
    const base64Part = commaIndex >= 0 ? dataField.slice(commaIndex + 1) : dataField;
    const jpegBytes = Buffer.from(base64Part, "base64");
    const decoded = jpeg.decode(jpegBytes, { useTArray: true });
    width = decoded.width;
    height = decoded.height;

    const rgb = Buffer.alloc(width * height * 3);
    for (let i = 0, j = 0; i < decoded.data.length; i += 4, j += 3) {
      rgb[j] = decoded.data[i];
      rgb[j + 1] = decoded.data[i + 1];
      rgb[j + 2] = decoded.data[i + 2];
    }
    return { width, height, buffer: rgb };
  }

  const bytes = typeof dataField === "string" ? Buffer.from(dataField, "base64") : Buffer.from(dataField);
  if (!width || !height) {
    throw new Error("Invalid image message: missing width/height");
  }

  const isMono = encoding.startsWith("mono");
  const isBgr = encoding.startsWith("bgr");
  const hasAlpha = encoding.includes("a8");
  const srcChannels = isMono ? 1 : (hasAlpha ? 4 : 3);
  const rgb = Buffer.alloc(width * height * 3);

  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      const srcIndex = (y * width + x) * srcChannels;
      let r = bytes[srcIndex];
      let g = srcChannels > 1 ? bytes[srcIndex + 1] : bytes[srcIndex];
      let b = srcChannels > 2 ? bytes[srcIndex + 2] : bytes[srcIndex];

      if (isMono) {
        g = r;
        b = r;
      } else if (isBgr) {
        b = bytes[srcIndex];
        g = bytes[srcIndex + 1];
        r = bytes[srcIndex + 2];
      }

      const dstIndex = (y * width + x) * 3;
      rgb[dstIndex] = r;
      rgb[dstIndex + 1] = g;
      rgb[dstIndex + 2] = b;
    }
  }

  return { width, height, buffer: rgb };
}

function imageMsgToTensor(msg, inputSize = 640) {
  const { width, height, buffer } = decodeImageToRgbBuffer(msg);
  const input = new Float32Array(3 * inputSize * inputSize);

  const scale = Math.min(inputSize / width, inputSize / height);
  const scaledWidth = Math.round(width * scale);
  const scaledHeight = Math.round(height * scale);
  const padX = Math.floor((inputSize - scaledWidth) / 2);
  const padY = Math.floor((inputSize - scaledHeight) / 2);

  input.fill(0.5);

  for (let y = 0; y < scaledHeight; y += 1) {
    const srcY = Math.floor((y / scaledHeight) * height);
    for (let x = 0; x < scaledWidth; x += 1) {
      const srcX = Math.floor((x / scaledWidth) * width);
      const dstY = y + padY;
      const dstX = x + padX;
      const dstIndex = dstY * inputSize + dstX;
      const srcIndex = (srcY * width + srcX) * 3;

      input[dstIndex] = buffer[srcIndex] / 255.0;
      input[inputSize * inputSize + dstIndex] = buffer[srcIndex + 1] / 255.0;
      input[2 * inputSize * inputSize + dstIndex] = buffer[srcIndex + 2] / 255.0;
    }
  }

  return {
    tensor: new ort.Tensor("float32", input, [1, 3, inputSize, inputSize]),
    scale,
    padX,
    padY,
  };
}

function computeIoU(a, b) {
  const x1 = Math.max(a.x1, b.x1);
  const y1 = Math.max(a.y1, b.y1);
  const x2 = Math.min(a.x2, b.x2);
  const y2 = Math.min(a.y2, b.y2);

  const inter = Math.max(0, x2 - x1) * Math.max(0, y2 - y1);
  const areaA = Math.max(0, a.x2 - a.x1) * Math.max(0, a.y2 - a.y1);
  const areaB = Math.max(0, b.x2 - b.x1) * Math.max(0, b.y2 - b.y1);
  const union = areaA + areaB - inter;
  return union > 0 ? inter / union : 0;
}

function applyNms(detections, iouThreshold) {
  detections.sort((a, b) => b.score - a.score);
  const byClass = new Map();
  for (const det of detections) {
    if (!byClass.has(det.classId)) {
      byClass.set(det.classId, []);
    }
    byClass.get(det.classId).push(det);
  }

  const selected = [];
  for (const classDetections of byClass.values()) {
    const classSelected = [];
    for (const det of classDetections) {
      if (classSelected.every((existing) => computeIoU(det, existing) <= iouThreshold)) {
        classSelected.push(det);
      }
    }
    selected.push(...classSelected);
  }
  return selected;
}

function decodeDetections(output, tensorInfo, confThreshold, iouThreshold) {
  const dims = output.dims;
  const data = output.data;
  if (dims.length !== 3 || dims[0] !== 1) {
    return [];
  }

  const boxesFirst = dims[1] > dims[2];
  const numBoxes = boxesFirst ? dims[1] : dims[2];
  const attributes = boxesFirst ? dims[2] : dims[1];
  const numClasses = attributes - 4;
  const detections = [];

  for (let i = 0; i < numBoxes; i += 1) {
    const read = (k) => (boxesFirst ? data[i * attributes + k] : data[k * numBoxes + i]);

    let bestClass = -1;
    let bestScore = 0;
    for (let c = 0; c < numClasses; c += 1) {
      const score = read(4 + c);
      if (score > bestScore) {
        bestScore = score;
        bestClass = c;
      }
    }

    if (bestScore < confThreshold) {
      continue;
    }

    const cx = read(0);
    const cy = read(1);
    const w = read(2);
    const h = read(3);
    const x1 = (cx - w / 2 - tensorInfo.padX) / tensorInfo.scale;
    const y1 = (cy - h / 2 - tensorInfo.padY) / tensorInfo.scale;
    const x2 = (cx + w / 2 - tensorInfo.padX) / tensorInfo.scale;
    const y2 = (cy + h / 2 - tensorInfo.padY) / tensorInfo.scale;

    detections.push({
      x1: Math.max(0, x1),
      y1: Math.max(0, y1),
      x2,
      y2,
      score: bestScore,
      classId: bestClass,
      label: COCO_CLASSES[bestClass] || `class_${bestClass}`,
    });
  }

  return applyNms(detections, iouThreshold);
}

async function runYoloOnImageMsg(msg, options = {}) {
  const defaults = getYoloConfig();
  const confThreshold = options.confThreshold ?? defaults.confThreshold;
  const iouThreshold = options.iouThreshold ?? defaults.iouThreshold;
  const inputSize = options.inputSize ?? defaults.inputSize;

  try {
    const session = await getSession();
    const inputName = session.inputNames[0];
    const { tensor, scale, padX, padY } = imageMsgToTensor(msg, inputSize);
    const results = await session.run({ [inputName]: tensor });
    const outputName = session.outputNames[0];
    return decodeDetections(results[outputName], { scale, padX, padY }, confThreshold, iouThreshold);
  } catch (err) {
    console.error("[YOLO] Inference failed:", err.message || err);
    return [];
  }
}

module.exports = {
  COCO_CLASSES,
  decodeImageToRgbBuffer,
  getSession,
  runYoloOnImageMsg,
};
