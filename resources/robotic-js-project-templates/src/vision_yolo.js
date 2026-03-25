#!/usr/bin/env node
/**
 * Image YOLO example using roslib and rosbridge.
 *
 * This script:
 *   - Subscribes to an image topic (e.g. /camera/image_raw)
 *   - Runs a CPU-only YOLO model (ONNX) in JavaScript
 *   - Republishes the (currently unmodified) image to another topic
 *   - Logs detections to the console for inspection
 *
 * Usage:
 *   bun src/vision_yolo.js
 *
 * Environment overrides:
 *   - TENSORFLEET_BASE_URL, TENSORFLEET_JWT (for proxy connection)
 *   - ROS_HOST, ROS_PORT, ROSBRIDGE_URL (for direct connection)
 *   - IMAGE_TOPIC, ANNOTATED_IMAGE_TOPIC, IMAGE_MESSAGE_TYPE
 *   - YOLO_MODEL_NAME / YOLO_MODEL, YOLO_MODEL_PATH, YOLO_MODEL_URL,
 *     YOLO_DEVICE / YOLO_BACKEND
 */

require("dotenv").config();
const ROSLIB = require("roslib");
const jpeg = require("jpeg-js");
const { getImageTopics } = require("./config");
const { runYoloOnImageMsg } = require("./yolo_inference");
const { connectToRobot } = require("./lib/robotic_utils");

const {
  imageTopic: IMAGE_TOPIC,
  annotatedImageTopic: ANNOTATED_IMAGE_TOPIC,
  messageType: IMAGE_MESSAGE_TYPE
} = getImageTopics();

/**
 * Decode a ROS sensor_msgs/Image-style message into a Buffer plus metadata.
 *
 * Supports:
 *   - msg.data as a base64 string
 *   - msg.data as an array / typed array of bytes
 */
function decodeRosImage(msg) {
  let height = msg.height;
  let width = msg.width;
  const encoding = (msg.encoding || "rgb8").toLowerCase();
  const dataField = msg.data;

  if (!dataField) {
    throw new Error("Invalid image message: missing data");
  }

  let buffer;
  let encodingKind;
  let channels;

  // Support data URLs such as "data:image/jpeg;base64,..."
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
    buffer = Buffer.from(decoded.data); // RGBA
    channels = 4;
    encodingKind = "data-url-jpeg";
  } else if (typeof dataField === "string") {
    // rosbridge commonly sends Image.data as a base64 string; decode it.
    buffer = Buffer.from(dataField, "base64");
    encodingKind = "base64";
  } else {
    // Assume array-like of bytes.
    buffer = Buffer.from(dataField);
    encodingKind = "list";
  }

  let step = msg.step;
  if (!step || step === 0) {
    // Assume tightly packed channels.
    if (!channels) {
      // Default to 3 channels when unknown.
      channels = 3;
    }
    step = width * channels;
  }

  if (!channels) {
    channels = Math.max(1, Math.floor(step / width));
  }

  const expectedMinSize = height * step;
  if (buffer.length < expectedMinSize) {
    throw new Error(
      `Image buffer too small: got ${buffer.length} bytes, expected at least ${expectedMinSize}`
    );
  }

  return {
    buffer,
    meta: {
      height,
      width,
      step,
      encoding,
      channels,
      encodingKind
    }
  };
}

function encodeRosImage(buffer, templateMsg, meta) {
  const { height, width, encoding, encodingKind, step } = meta;

  let dataField;
  if (encodingKind === "data-url-jpeg") {
    // Buffer is expected to be RGBA here.
    const rawImageData = { data: buffer, width, height };
    const jpegImage = jpeg.encode(rawImageData, 90);
    const base64Jpeg = jpegImage.data.toString("base64");
    dataField = `data:image/jpeg;base64,${base64Jpeg}`;
  } else if (encodingKind === "base64") {
    dataField = buffer.toString("base64");
  } else {
    dataField = Array.from(buffer);
  }

  return {
    ...templateMsg,
    height,
    width,
    step,
    encoding: encoding || "rgb8",
    data: dataField
  };
}

/**
 * Simple 5x7 bitmap font for digits and a few symbols, used to render
 * lightweight labels into the image buffer.
 */
const FONT_5X7 = {
  "0": [0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001, 0b01110],
  "1": [0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110],
  "2": [0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000, 0b11111],
  "3": [0b11110, 0b00001, 0b00001, 0b01110, 0b00001, 0b00001, 0b11110],
  "4": [0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010],
  "5": [0b11111, 0b10000, 0b11110, 0b00001, 0b00001, 0b10001, 0b01110],
  "6": [0b00110, 0b01000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110],
  "7": [0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000],
  "8": [0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110],
  "9": [0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00010, 0b01100],
  "#": [0b01010, 0b11111, 0b01010, 0b01010, 0b11111, 0b01010, 0b01010],
  "%": [0b11001, 0b11010, 0b00100, 0b00100, 0b01011, 0b10011, 0b00000],
  " ": [0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000],
  "-": [0b00000, 0b00000, 0b00000, 0b01110, 0b00000, 0b00000, 0b00000],
  ".": [0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00100, 0b00100],
  "·": [0b00000, 0b00000, 0b00000, 0b01100, 0b01100, 0b00000, 0b00000],
  ":": [0b00100, 0b00100, 0b00000, 0b00000, 0b00100, 0b00100, 0b00000],
  "A": [0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001],
  "B": [0b11110, 0b10001, 0b10001, 0b11110, 0b10001, 0b10001, 0b11110],
  "C": [0b01110, 0b10001, 0b10000, 0b10000, 0b10000, 0b10001, 0b01110],
  "D": [0b11110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11110],
  "E": [0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111],
  "F": [0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b10000],
  "G": [0b01110, 0b10001, 0b10000, 0b10011, 0b10001, 0b10001, 0b01110],
  "H": [0b10001, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001],
  "I": [0b01110, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110],
  "J": [0b00001, 0b00001, 0b00001, 0b00001, 0b00001, 0b10001, 0b01110],
  "K": [0b10001, 0b10010, 0b10100, 0b11000, 0b10100, 0b10010, 0b10001],
  "L": [0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b11111],
  "M": [0b10001, 0b11011, 0b10101, 0b10101, 0b10001, 0b10001, 0b10001],
  "N": [0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001, 0b10001],
  "O": [0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110],
  "P": [0b11110, 0b10001, 0b10001, 0b11110, 0b10000, 0b10000, 0b10000],
  "Q": [0b01110, 0b10001, 0b10001, 0b10001, 0b10101, 0b10010, 0b01101],
  "R": [0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001],
  "S": [0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110],
  "T": [0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100],
  "U": [0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110],
  "V": [0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01010, 0b00100],
  "W": [0b10001, 0b10001, 0b10001, 0b10101, 0b10101, 0b10101, 0b01010],
  "X": [0b10001, 0b10001, 0b01010, 0b00100, 0b01010, 0b10001, 0b10001],
  "Y": [0b10001, 0b10001, 0b01010, 0b00100, 0b00100, 0b00100, 0b00100],
  "Z": [0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0b11111]
};

function normalizeBox(box, meta) {
  const { width, height } = meta;
  let x1 = Math.max(0, Math.floor(box.x1));
  let y1 = Math.max(0, Math.floor(box.y1));
  let x2 = Math.min(width - 1, Math.ceil(box.x2));
  let y2 = Math.min(height - 1, Math.ceil(box.y2));

  if (x2 <= x1 || y2 <= y1) {
    return null;
  }
  return { x1, y1, x2, y2 };
}

function setPixel(buffer, meta, x, y, color) {
  const { width, height, step, channels, encoding } = meta;
  if (x < 0 || x >= width || y < 0 || y >= height) return;
  const offset = y * step + x * channels;
  if (offset + 2 >= buffer.length) return;

  const enc = (encoding || "").toLowerCase();
  const isBgr = enc.startsWith("bgr");
  if (isBgr) {
    buffer[offset + 0] = color.b;
    buffer[offset + 1] = color.g;
    buffer[offset + 2] = color.r;
  } else {
    buffer[offset + 0] = color.r;
    buffer[offset + 1] = color.g;
    buffer[offset + 2] = color.b;
  }
}

function blendPixel(buffer, meta, x, y, color, alpha = 1) {
  const { width, height, step, channels, encoding } = meta;
  if (x < 0 || x >= width || y < 0 || y >= height) return;
  const offset = y * step + x * channels;
  if (offset + 2 >= buffer.length) return;

  const a = Math.max(0, Math.min(1, alpha));
  const enc = (encoding || "").toLowerCase();
  const isBgr = enc.startsWith("bgr");
  const [cr, cg, cb] = isBgr
    ? [color.b, color.g, color.r]
    : [color.r, color.g, color.b];

  buffer[offset + 0] = Math.round(buffer[offset + 0] * (1 - a) + cr * a);
  buffer[offset + 1] = Math.round(buffer[offset + 1] * (1 - a) + cg * a);
  buffer[offset + 2] = Math.round(buffer[offset + 2] * (1 - a) + cb * a);
}

function fillRect(buffer, meta, x, y, w, h, color, alpha = 1) {
  const x1 = Math.max(0, Math.floor(x));
  const y1 = Math.max(0, Math.floor(y));
  const x2 = Math.min(meta.width - 1, Math.floor(x + w - 1));
  const y2 = Math.min(meta.height - 1, Math.floor(y + h - 1));
  for (let yy = y1; yy <= y2; yy += 1) {
    for (let xx = x1; xx <= x2; xx += 1) {
      blendPixel(buffer, meta, xx, yy, color, alpha);
    }
  }
}

/**
 * Draw a rectangle onto an image buffer in-place.
 */
function drawRectOnBuffer(buffer, meta, box, options = {}) {
  const { height } = meta;

  const thickness = options.thickness || 2;
  const color = options.color || { r: 92, g: 196, b: 214 };

  const norm = normalizeBox(box, meta);
  if (!norm) {
    return;
  }
  const { x1, y1, x2, y2 } = norm;

  for (let x = x1; x <= x2; x += 1) {
    for (let t = 0; t < thickness && y1 + t < height; t += 1) {
      blendPixel(buffer, meta, x, y1 + t, color, 0.88);
      blendPixel(buffer, meta, x, Math.max(y1, y2 - t), color, 0.88);
    }
  }

  for (let y = y1; y <= y2; y += 1) {
    for (let t = 0; t < thickness; t += 1) {
      blendPixel(buffer, meta, x1 + t, y, color, 0.88);
      blendPixel(buffer, meta, Math.max(x1, x2 - t), y, color, 0.88);
    }
  }
}

function drawLabelOnBuffer(buffer, meta, det, options = {}) {
  const color = options.color || { r: 92, g: 196, b: 214 };
  const scale = 1;
  const paddingX = 6;
  const paddingY = 3;
  const charWidth = 5;
  const charHeight = 7;
  const charSpacing = 1;
  const { width, height } = meta;

  const norm = normalizeBox(det, meta);
  if (!norm) return;

  const scorePercent = Math.round(((det && det.score) || 0) * 100);
  const rawLabel = ((det && det.label) || "object").replaceAll("_", " ");
  const labelWithSep = `${rawLabel} · `;
  const confText = `${scorePercent}%`;

  const measureStr = (str) =>
    str.length > 0 ? (str.length * (charWidth + charSpacing) - charSpacing) * scale : 0;

  const renderStr = (str, startX, startY, textColor) => {
    let cx = startX;
    for (let i = 0; i < str.length; i += 1) {
      const glyph = FONT_5X7[str[i].toUpperCase()] || FONT_5X7["·"] || FONT_5X7[" "];
      for (let row = 0; row < charHeight; row += 1) {
        const rowBits = glyph[row];
        for (let col = 0; col < charWidth; col += 1) {
          if (!((rowBits >> (charWidth - 1 - col)) & 1)) continue;
          for (let sy = 0; sy < scale; sy += 1) {
            for (let sx = 0; sx < scale; sx += 1) {
              const px = cx + col * scale + sx;
              const py = startY + row * scale + sy;
              if (px >= 0 && px < width && py >= 0 && py < height) {
                setPixel(buffer, meta, px, py, textColor);
              }
            }
          }
        }
      }
      cx += (charWidth + charSpacing) * scale;
    }
  };

  const labelWithSepWidth = measureStr(labelWithSep);
  const confWidth = measureStr(confText);
  const chipWidth = labelWithSepWidth + scale + confWidth + paddingX * 2;
  const chipHeight = charHeight * scale + paddingY * 2;

  let chipX = norm.x1;
  let chipY = norm.y1 - chipHeight - 2;
  if (chipY < 0) chipY = norm.y1 + 2;
  if (chipX + chipWidth > width) chipX = Math.max(0, width - chipWidth);

  fillRect(buffer, meta, chipX, chipY, chipWidth, chipHeight, { r: 10, g: 14, b: 18 }, 0.78);

  for (let bx = chipX; bx < chipX + chipWidth; bx += 1) {
    blendPixel(buffer, meta, bx, chipY, color, 0.28);
    blendPixel(buffer, meta, bx, chipY + chipHeight - 1, color, 0.28);
  }
  for (let by = chipY + 1; by < chipY + chipHeight - 1; by += 1) {
    blendPixel(buffer, meta, chipX, by, color, 0.28);
    blendPixel(buffer, meta, chipX + chipWidth - 1, by, color, 0.28);
  }

  const textY = chipY + paddingY;
  renderStr(labelWithSep, chipX + paddingX, textY, { r: 235, g: 242, b: 245 });
  renderStr(confText, chipX + paddingX + labelWithSepWidth + scale, textY, { r: 180, g: 195, b: 205 });
}

/**
 * Annotate an image message with YOLO detections: colored bounding boxes
 * plus a compact label showing class id and confidence.
 */
function annotateImageMessage(msg, detections) {
  if (!detections || detections.length === 0) {
    return msg;
  }

  try {
    const { buffer, meta } = decodeRosImage(msg);

    const colors = [
      { r: 92, g: 196, b: 214 },
      { r: 86, g: 170, b: 176 },
      { r: 232, g: 166, b: 76 },
      { r: 140, g: 160, b: 180 },
      { r: 214, g: 96, b: 96 },
    ];

    detections.forEach((det, idx) => {
      const color = colors[idx % colors.length];
      drawRectOnBuffer(buffer, meta, det, { color, thickness: 2 });
      drawLabelOnBuffer(buffer, meta, det, { color });
    });

    return encodeRosImage(buffer, msg, meta);
  } catch (err) {
    console.error("Failed to annotate image:", err.message || err);
    return msg;
  }
}

async function main() {
  let ros;

  try {
    ros = await connectToRobot();
    console.log("Connected to rosbridge.");
    console.log(
      `Subscribing to '${IMAGE_TOPIC}' (${IMAGE_MESSAGE_TYPE}) and ` +
      `republishing to '${ANNOTATED_IMAGE_TOPIC}'.`
    );

    const subscriber = new ROSLIB.Topic({
      ros,
      name: IMAGE_TOPIC,
      messageType: IMAGE_MESSAGE_TYPE,
      queue_length: 1
    });

    const publisher = new ROSLIB.Topic({
      ros,
      name: ANNOTATED_IMAGE_TOPIC,
      messageType: IMAGE_MESSAGE_TYPE
    });

    let count = 0;

    subscriber.subscribe(async (msg) => {
      count += 1;
      let detections = [];
      let annotatedMsg = msg;
      try {
        detections = await runYoloOnImageMsg(msg);
      } catch (err) {
        console.error("YOLO inference error:", err.message || err);
      }

      if (detections.length > 0) {
        console.log(
          `Image #${count}: ${detections.length} detections from YOLO (first:`,
          detections[0],
          ")"
        );
        annotatedMsg = annotateImageMessage(msg, detections);
      } else {
        console.log(`Image #${count}: no detections from YOLO.`);
      }

      // Republish the image with YOLO annotations drawn into the pixel data.
      publisher.publish(annotatedMsg);
    });

    const shutdown = () => {
      console.log("Shutting down vision yolo.");
      subscriber.unsubscribe();
      publisher.unadvertise();
      ros.close();
    };

    process.on("SIGINT", () => {
      shutdown();
    });
    process.on("SIGTERM", () => {
      shutdown();
    });
  } catch (err) {
    console.error("Failed to connect:", err);
    process.exit(1);
  }
}

if (require.main === module) {
  main();
}
