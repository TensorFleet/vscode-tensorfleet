#!/usr/bin/env node
/**
 * Minimal YOLO inference helper using onnxruntime-node.
 *
 * Design goals:
 *   - CPU-only execution
 *   - Smallest model by default (e.g. yolov8n.onnx)
 *   - Pluggable model selection via config/getYoloConfig
 *   - Automatic model download from HuggingFace
 *
 * This module does not ship any model weights by default. If the
 * expected ONNX file is missing, it will attempt to download it
 * from HuggingFace into ../models/<modelName>.onnx.
 */

const fs = require("fs");
const https = require("https");
const path = require("path");
const jpeg = require("jpeg-js");
const ort = require("onnxruntime-node");
const { getYoloConfig } = require("./config");

// COCO class labels matching the standard YOLOv8 models.
const COCO_CLASSES = [
  "person",
  "bicycle",
  "car",
  "motorcycle",
  "airplane",
  "bus",
  "train",
  "truck",
  "boat",
  "traffic light",
  "fire hydrant",
  "stop sign",
  "parking meter",
  "bench",
  "bird",
  "cat",
  "dog",
  "horse",
  "sheep",
  "cow",
  "elephant",
  "bear",
  "zebra",
  "giraffe",
  "backpack",
  "umbrella",
  "handbag",
  "tie",
  "suitcase",
  "frisbee",
  "skis",
  "snowboard",
  "sports ball",
  "kite",
  "baseball bat",
  "baseball glove",
  "skateboard",
  "surfboard",
  "tennis racket",
  "bottle",
  "wine glass",
  "cup",
  "fork",
  "knife",
  "spoon",
  "bowl",
  "banana",
  "apple",
  "sandwich",
  "orange",
  "broccoli",
  "carrot",
  "hot dog",
  "pizza",
  "donut",
  "cake",
  "chair",
  "couch",
  "potted plant",
  "bed",
  "dining table",
  "toilet",
  "tv",
  "laptop",
  "mouse",
  "remote",
  "keyboard",
  "cell phone",
  "microwave",
  "oven",
  "toaster",
  "sink",
  "refrigerator",
  "book",
  "clock",
  "vase",
  "scissors",
  "teddy bear",
  "hair drier",
  "toothbrush"
];

let sessionPromise = null;

// Default model URLs from HuggingFace (these are stable and reliable)
const DEFAULT_MODEL_URLS = {
  yolov8n: "https://huggingface.co/Kalray/yolov8/resolve/main/yolov8n.onnx",
  yolov8s: "https://huggingface.co/Kalray/yolov8/resolve/main/yolov8s.onnx",
  yolov8m: "https://huggingface.co/Kalray/yolov8/resolve/main/yolov8m.onnx",
  yolov8l: "https://huggingface.co/Kalray/yolov8/resolve/main/yolov8l.onnx",
  yolov8x: "https://huggingface.co/Kalray/yolov8/resolve/main/yolov8x.onnx"
};

function resolveModelPath() {
  const { modelName, modelPath } = getYoloConfig();
  if (modelPath && modelPath.trim() !== "") {
    return path.resolve(modelPath);
  }
  return path.join(__dirname, "..", "models", `${modelName}.onnx`);
}

function getModelDownloadUrl() {
  const { modelName } = getYoloConfig();
  const override = process.env.YOLO_MODEL_URL;
  
  if (override && override.trim() !== "") {
    return override.trim();
  }
  
  const fromDefaults = DEFAULT_MODEL_URLS[modelName];
  return fromDefaults || null;
}

/**
 * Download a file from a URL with redirect support and progress tracking.
 */
function downloadFile(url, destination) {
  return new Promise((resolve, reject) => {
    fs.mkdirSync(path.dirname(destination), { recursive: true });

    const startTime = Date.now();
    let downloadedBytes = 0;
    let totalBytes = 0;
    let lastProgress = 0;

    const request = https.get(url, (response) => {
      // Handle redirects
      if (response.statusCode >= 300 && response.statusCode < 400 && response.headers.location) {
        const redirectedUrl = response.headers.location;
        console.log(`Following redirect to '${redirectedUrl}'`);
        response.resume();
        downloadFile(redirectedUrl, destination).then(resolve).catch(reject);
        return;
      }

      if (response.statusCode !== 200) {
        response.resume();
        reject(
          new Error(
            `Failed to download YOLO model (HTTP ${response.statusCode}) from '${url}'`
          )
        );
        return;
      }

      totalBytes = parseInt(response.headers["content-length"] || "0", 10);
      console.log(`Downloading YOLO model from '${url}'`);
      console.log(`File size: ${(totalBytes / (1024 * 1024)).toFixed(2)} MB`);

      const file = fs.createWriteStream(destination);
      
      response.on("data", (chunk) => {
        downloadedBytes += chunk.length;
        
        // Show progress every 10%
        if (totalBytes > 0) {
          const progress = Math.floor((downloadedBytes / totalBytes) * 100);
          if (progress >= lastProgress + 10 || progress === 100) {
            const elapsed = (Date.now() - startTime) / 1000;
            const speed = downloadedBytes / elapsed / (1024 * 1024);
            console.log(
              `Progress: ${progress}% (${(downloadedBytes / (1024 * 1024)).toFixed(2)}MB / ${(totalBytes / (1024 * 1024)).toFixed(2)}MB) - ${speed.toFixed(2)} MB/s`
            );
            lastProgress = progress;
          }
        }
      });

      response.pipe(file);
      
      file.on("finish", () => {
        file.close(() => {
          try {
            const stats = fs.statSync(destination);
            const elapsed = (Date.now() - startTime) / 1000;
            
            if (!stats.size || stats.size < 100000) {
              throw new Error(
                `Downloaded YOLO model at '${destination}' looks too small (${stats.size} bytes). ` +
                `It may be an HTML error page instead of an ONNX file.`
              );
            }
            
            console.log(
              `✓ Successfully saved YOLO model to '${destination}' ` +
              `(${(stats.size / (1024 * 1024)).toFixed(2)} MB in ${elapsed.toFixed(1)}s)`
            );
            resolve();
          } catch (err) {
            fs.unlink(destination, () => reject(err));
          }
        });
      });

      file.on("error", (err) => {
        fs.unlink(destination, () => reject(err));
      });
    });

    request.on("error", (err) => {
      fs.unlink(destination, () => reject(err));
    });

    request.setTimeout(30000, () => {
      request.destroy();
      fs.unlink(destination, () => {
        reject(new Error("Download timeout after 30 seconds"));
      });
    });
  });
}

async function ensureModelFile(modelFile) {
  if (fs.existsSync(modelFile)) {
    const stats = fs.statSync(modelFile);
    console.log(`Using existing YOLO model: ${modelFile} (${(stats.size / (1024 * 1024)).toFixed(2)} MB)`);
    return;
  }

  const url = getModelDownloadUrl();
  if (!url) {
    const { modelName } = getYoloConfig();
    throw new Error(
      `YOLO model not found at '${modelFile}' and no download URL available for '${modelName}'. ` +
      `Available models: ${Object.keys(DEFAULT_MODEL_URLS).join(", ")}. ` +
      `Set YOLO_MODEL_PATH or YOLO_MODEL_URL environment variable, or place an ONNX model at that path.`
    );
  }

  console.log(`Model not found locally, downloading...`);
  await downloadFile(url, modelFile);
}

async function getSession() {
  if (!sessionPromise) {
    sessionPromise = (async () => {
      const modelFile = resolveModelPath();
      await ensureModelFile(modelFile);

      console.log("Loading ONNX model into inference session...");
      const startTime = Date.now();
      
      // Use default CPU execution provider
      const session = await ort.InferenceSession.create(modelFile, {
        executionProviders: ["cpu"],
        graphOptimizationLevel: "all"
      });
      
      const loadTime = Date.now() - startTime;
      console.log(`✓ Model loaded in ${loadTime}ms`);
      console.log(`  Input: ${session.inputNames[0]} ${JSON.stringify(session.inputNames)}`);
      console.log(`  Output: ${session.outputNames[0]} ${JSON.stringify(session.outputNames)}`);
      
      return session;
    })();
  }
  return sessionPromise;
}

/**
 * Convert an incoming image message into an RGB byte buffer plus geometry.
 *
 * Supports:
 *   - Raw sensor_msgs/Image-style data where msg.data is a byte array or base64
 *   - Data URL strings like "data:image/jpeg;base64,...."
 *
 * Returns { width, height, buffer } where buffer is tightly packed RGB.
 */
function decodeImageToRgbBuffer(msg) {
  let width = msg.width;
  let height = msg.height;
  const encoding = (msg.encoding || "rgb8").toLowerCase();
  const dataField = msg.data;

  if (!dataField) {
    throw new Error("Invalid image message: missing data field");
  }

  // Handle data URLs (commonly used when images are JPEG-compressed).
  if (typeof dataField === "string" && dataField.startsWith("data:image/")) {
    const commaIndex = dataField.indexOf(",");
    const base64Part =
      commaIndex >= 0 ? dataField.slice(commaIndex + 1) : dataField;
    const jpegBytes = Buffer.from(base64Part, "base64");
    const decoded = jpeg.decode(jpegBytes, { useTArray: true });
    if (!decoded || !decoded.data) {
      throw new Error("Failed to decode JPEG image from data URL");
    }
    width = decoded.width;
    height = decoded.height;
    const rgba = decoded.data;

    const rgb = Buffer.alloc(width * height * 3);
    for (let i = 0, j = 0; i < rgba.length; i += 4, j += 3) {
      rgb[j] = rgba[i + 0];
      rgb[j + 1] = rgba[i + 1];
      rgb[j + 2] = rgba[i + 2];
    }

    return { width, height, buffer: rgb };
  }

  // Otherwise, treat as raw sensor_msgs/Image-style bytes.
  let bytes;
  if (typeof dataField === "string") {
    // rosbridge often encodes uint8[] as base64 strings.
    bytes = Buffer.from(dataField, "base64");
  } else {
    // Assume array-like of bytes.
    bytes = Buffer.from(dataField);
  }

  if (!width || !height) {
    throw new Error("Invalid image message: missing width/height");
  }

  const enc = encoding;
  const isMono = enc.startsWith("mono");
  const isBgr = enc.startsWith("bgr");
  const hasAlpha = enc.includes("a8");

  let srcChannels;
  if (isMono) {
    srcChannels = 1;
  } else if (hasAlpha) {
    srcChannels = 4;
  } else {
    srcChannels = 3;
  }

  const expectedSize = width * height * srcChannels;
  if (bytes.length < expectedSize) {
    throw new Error(
      `Image buffer too small: got ${bytes.length} bytes, expected at least ${expectedSize}`
    );
  }

  // Convert to tightly packed RGB buffer of size width*height*3
  const rgb = Buffer.alloc(width * height * 3);
  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      const srcIndex = (y * width + x) * srcChannels;
      let r;
      let g;
      let b;
      if (isMono) {
        const gray = bytes[srcIndex];
        r = gray;
        g = gray;
        b = gray;
      } else if (isBgr) {
        b = bytes[srcIndex + 0];
        g = bytes[srcIndex + 1];
        r = bytes[srcIndex + 2];
      } else {
        r = bytes[srcIndex + 0];
        g = bytes[srcIndex + 1];
        b = bytes[srcIndex + 2];
      }

      const dstIndex = (y * width + x) * 3;
      rgb[dstIndex + 0] = r;
      rgb[dstIndex + 1] = g;
      rgb[dstIndex + 2] = b;
    }
  }

  return { width, height, buffer: rgb };
}

/**
 * Convert an image message into a Float32 NCHW tensor suitable for YOLO.
 *
 * Handles:
 *   - Raw RGB/BGR/mono images
 *   - JPEG/data-URL encoded images
 */
function imageMsgToTensor(msg, inputSize = 640) {
  const { width, height, buffer } = decodeImageToRgbBuffer(msg);

  const channels = 3;
  const input = new Float32Array(1 * channels * inputSize * inputSize);

  // Calculate letterbox parameters to maintain aspect ratio
  const scale = Math.min(inputSize / width, inputSize / height);
  const scaledWidth = Math.round(width * scale);
  const scaledHeight = Math.round(height * scale);
  const padX = Math.floor((inputSize - scaledWidth) / 2);
  const padY = Math.floor((inputSize - scaledHeight) / 2);

  // Fill with gray background (0.5 normalized)
  input.fill(0.5);

  // Bilinear-like resize (nearest neighbor for speed)
  for (let y = 0; y < scaledHeight; y += 1) {
    const srcY = Math.floor((y / scaledHeight) * height);
    for (let x = 0; x < scaledWidth; x += 1) {
      const srcX = Math.floor((x / scaledWidth) * width);
      const dstY = y + padY;
      const dstX = x + padX;
      const dstIndex = dstY * inputSize + dstX;

      const srcIndex = (srcY * width + srcX) * channels;
      const r = buffer[srcIndex + 0];
      const g = buffer[srcIndex + 1];
      const b = buffer[srcIndex + 2];

      // NCHW: [1,3,H,W]
      input[0 * inputSize * inputSize + dstIndex] = r / 255.0;
      input[1 * inputSize * inputSize + dstIndex] = g / 255.0;
      input[2 * inputSize * inputSize + dstIndex] = b / 255.0;
    }
  }

  return {
    tensor: new ort.Tensor("float32", input, [1, channels, inputSize, inputSize]),
    scale,
    padX,
    padY
  };
}

/**
 * Improved YOLO post-processing with better handling of various output formats.
 * 
 * Enhancements:
 *   - Better detection of output format
 *   - Per-class NMS
 *   - Coordinate scaling back to original image
 */
function decodeDetections(
  output,
  tensorInfo,
  confThreshold = 0.25,
  iouThreshold = 0.45
) {
  const dims = output.dims;
  const data = output.data;

  if (dims.length !== 3 || dims[0] !== 1) {
    console.warn("Unexpected YOLO output shape:", dims);
    return [];
  }

  // Robustly detect which dimension is "attributes" (box coords + class scores)
  // and which is "num boxes". For typical YOLOv8 exports, one dim is small
  // (e.g. 84/85) and the other is large (e.g. 8400).
  const dim1 = dims[1];
  const dim2 = dims[2];

  let attributes;
  let numBoxes;
  let boxesFirst;

  if (dim1 <= dim2) {
    // Shape like [1, attributes, numBoxes] (e.g. [1, 84, 8400])
    attributes = dim1;
    numBoxes = dim2;
    boxesFirst = false; // layout [1, attributes, N]
  } else {
    // Shape like [1, numBoxes, attributes] (e.g. [1, 8400, 84])
    attributes = dim2;
    numBoxes = dim1;
    boxesFirst = true; // layout [1, N, attributes]
  }

  if (attributes < 5) {
    console.warn("Insufficient attributes in YOLO output:", dims);
    return [];
  }

  const numClasses = attributes - 4; // YOLOv8 doesn't have objectness score
  const detections = [];

  // Extract detections
  for (let i = 0; i < numBoxes; i += 1) {
    const read = (k) =>
      boxesFirst ? data[i * attributes + k] : data[k * numBoxes + i];

    const cx = read(0);
    const cy = read(1);
    const w = read(2);
    const h = read(3);

    // Find best class
    let bestClass = -1;
    let bestScore = 0;
    for (let c = 0; c < numClasses; c += 1) {
      const score = read(4 + c);
      if (score > bestScore) {
        bestScore = score;
        bestClass = c;
      }
    }

    if (bestScore < confThreshold) continue;

    // Convert to x1,y1,x2,y2 format and scale back to original image
    const x1 = (cx - w / 2 - tensorInfo.padX) / tensorInfo.scale;
    const y1 = (cy - h / 2 - tensorInfo.padY) / tensorInfo.scale;
    const x2 = (cx + w / 2 - tensorInfo.padX) / tensorInfo.scale;
    const y2 = (cy + h / 2 - tensorInfo.padY) / tensorInfo.scale;

    const label =
      bestClass >= 0 && bestClass < COCO_CLASSES.length
        ? COCO_CLASSES[bestClass]
        : `class_${bestClass}`;

    detections.push({
      x1: Math.max(0, x1),
      y1: Math.max(0, y1),
      x2,
      y2,
      score: bestScore,
      classId: bestClass,
      label
    });
  }

  // Apply NMS
  return applyNMS(detections, iouThreshold);
}

/**
 * Apply Non-Maximum Suppression with per-class handling
 */
function applyNMS(detections, iouThreshold) {
  if (detections.length === 0) return [];

  // Sort by score (descending)
  detections.sort((a, b) => b.score - a.score);

  const iou = (a, b) => {
    const x1 = Math.max(a.x1, b.x1);
    const y1 = Math.max(a.y1, b.y1);
    const x2 = Math.min(a.x2, b.x2);
    const y2 = Math.min(a.y2, b.y2);
    
    const inter = Math.max(0, x2 - x1) * Math.max(0, y2 - y1);
    const areaA = Math.max(0, a.x2 - a.x1) * Math.max(0, a.y2 - a.y1);
    const areaB = Math.max(0, b.x2 - b.x1) * Math.max(0, b.y2 - b.y1);
    const union = areaA + areaB - inter;
    
    return union > 0 ? inter / union : 0;
  };

  // Group detections by class for per-class NMS
  const byClass = {};
  for (const det of detections) {
    if (!byClass[det.classId]) byClass[det.classId] = [];
    byClass[det.classId].push(det);
  }

  const selected = [];
  
  // Apply NMS per class
  for (const classId in byClass) {
    const classDets = byClass[classId];
    const classSelected = [];
    
    for (const det of classDets) {
      let keep = true;
      for (const existing of classSelected) {
        if (iou(det, existing) > iouThreshold) {
          keep = false;
          break;
        }
      }
      if (keep) classSelected.push(det);
    }
    
    selected.push(...classSelected);
  }

  return selected;
}

/**
 * Run YOLO on a ROS image message.
 * Returns an array of detection objects (or [] if model not available).
 * 
 * Each detection contains:
 *   - x1, y1, x2, y2: bounding box coordinates in original image space
 *   - score: confidence score (0-1)
 *   - classId: detected class ID
 */
async function runYoloOnImageMsg(msg, options = {}) {
  const {
    confThreshold = 0.25,
    iouThreshold = 0.45,
    inputSize = 640
  } = options;

  let session;
  try {
    session = await getSession();
  } catch (err) {
    console.error("YOLO session not available:", err.message || err);
    return [];
  }

  try {
    const inputName = session.inputNames[0];
    const { tensor, scale, padX, padY } = imageMsgToTensor(msg, inputSize);

    const feeds = {};
    feeds[inputName] = tensor;

    const startTime = Date.now();
    const results = await session.run(feeds);
    const inferenceTime = Date.now() - startTime;

    const outputName = session.outputNames[0];
    const output = results[outputName];

    const detections = decodeDetections(
      output,
      { scale, padX, padY },
      confThreshold,
      iouThreshold
    );

    if (detections.length > 0) {
      console.log(
        `Inference completed in ${inferenceTime}ms - Found ${detections.length} detection(s)`
      );
    }

    return detections;
  } catch (err) {
    console.error("Error during YOLO inference:", err.message || err);
    return [];
  }
}

module.exports = {
  runYoloOnImageMsg,
  getSession // Export for pre-loading the model
};
