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
 *   ROS_HOST, ROS_PORT,
 *   IMAGE_TOPIC, ANNOTATED_IMAGE_TOPIC,
 *   IMAGE_MESSAGE_TYPE
 *   YOLO_MODEL_NAME / YOLO_MODEL:
 *     Logical model name to load (defaults to smallest yolov8n)
 *   YOLO_MODEL_PATH:
 *     Full path to YOLO ONNX file; overrides model name
 *   YOLO_MODEL_URL:
 *     Explicit download URL for the ONNX model; overrides the built-in default.
 *   YOLO_DEVICE / YOLO_BACKEND:
 *     Execution device; JS implementation only uses "cpu"
 */
const ROSLIB = require("roslib");
const jpeg = require("jpeg-js");
 const {
  getRosConnectionDetails,
  getImageTopics
} = require("./config");
const { runYoloOnImageMsg } = require("./yolo_inference");

const { url: ROSBRIDGE_URL } = getRosConnectionDetails();
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

/**
 * Draw a rectangle onto an image buffer in-place.
 */
function drawRectOnBuffer(buffer, meta, box, options = {}) {
  const { width, height } = meta;

  const thickness = options.thickness || 4;
  const color = options.color || { r: 0, g: 255, b: 255 };

  const norm = normalizeBox(box, meta);
  if (!norm) {
    return;
  }
  const { x1, y1, x2, y2 } = norm;

  // Top and bottom edges
  for (let x = x1; x <= x2; x += 1) {
    for (let t = 0; t < thickness && y1 + t < height; t += 1) {
      setPixel(buffer, meta, x, y1 + t, color);
      setPixel(buffer, meta, x, Math.max(y1, y2 - t), color);
    }
  }

  // Left and right edges
  for (let y = y1; y <= y2; y += 1) {
    for (let t = 0; t < thickness && x1 + t < width; t += 1) {
      setPixel(buffer, meta, x1 + t, y, color);
      setPixel(buffer, meta, Math.max(x1, x2 - t), y, color);
    }
  }
}

function drawLabelOnBuffer(buffer, meta, box, text, options = {}) {
  const { width, height } = meta;
  const baseColor = options.color || { r: 0, g: 255, b: 255 };
  const bgColor = options.bgColor || { r: baseColor.r, g: baseColor.g, b: baseColor.b };
  const textColor = options.textColor || { r: 0, g: 0, b: 0 };
  const scale = options.scale || 2;

  const norm = normalizeBox(box, meta);
  if (!norm) return;
  let { x1, y1 } = norm;

  const charWidth = 5;
  const charHeight = 7;
  const charSpacing = 1;
  const paddingX = 2 * scale;
  const paddingY = 1 * scale;

  const textLen = text.length;
  const textPixelWidth =
    textLen > 0
      ? (textLen * (charWidth + charSpacing) - charSpacing) * scale
      : 0;
  const textPixelHeight = charHeight * scale;

  const boxWidth = textPixelWidth + paddingX * 2;
  const boxHeight = textPixelHeight + paddingY * 2;

  // Position label above the box where possible.
  let labelX = x1;
  let labelY = y1 - boxHeight - 2;
  if (labelY < 0) {
    labelY = y1 + 2;
  }
  if (labelX + boxWidth > width) {
    labelX = Math.max(0, width - boxWidth);
  }

  // Draw background rectangle
  for (let y = 0; y < boxHeight; y += 1) {
    const yy = labelY + y;
    if (yy < 0 || yy >= height) continue;
    for (let x = 0; x < boxWidth; x += 1) {
      const xx = labelX + x;
      if (xx < 0 || xx >= width) continue;
      setPixel(buffer, meta, xx, yy, bgColor);
    }
  }

  // Draw text glyphs
  let cursorX = labelX + paddingX;
  const cursorY = labelY + paddingY;

  for (let i = 0; i < text.length; i += 1) {
    const chRaw = text[i];
    const chKey = chRaw.toUpperCase();
    const glyph = FONT_5X7[chKey] || FONT_5X7[" "];
    for (let row = 0; row < charHeight; row += 1) {
      const rowBits = glyph[row];
      for (let col = 0; col < charWidth; col += 1) {
        const bit = (rowBits >> (charWidth - 1 - col)) & 1;
        if (!bit) continue;
        for (let sy = 0; sy < scale; sy += 1) {
          for (let sx = 0; sx < scale; sx += 1) {
            const xx = cursorX + col * scale + sx;
            const yy = cursorY + row * scale + sy;
            if (xx < 0 || xx >= width || yy < 0 || yy >= height) continue;
            setPixel(buffer, meta, xx, yy, textColor);
          }
        }
      }
    }
    cursorX += (charWidth + charSpacing) * scale;
  }
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
      { r: 0, g: 255, b: 255 }, // Cyan
      { r: 255, g: 0, b: 255 }, // Magenta
      { r: 255, g: 255, b: 0 }, // Yellow
      { r: 0, g: 255, b: 0 },   // Green
      { r: 255, g: 128, b: 0 }  // Orange
    ];

    detections.forEach((det, idx) => {
      const color = colors[idx % colors.length];
      drawRectOnBuffer(buffer, meta, det, { color, thickness: 4 });

      const scorePct = Math.round(det.score * 100);
      const baseLabel =
        (det.label && typeof det.label === "string"
          ? det.label
          : `CLASS_${det.classId}`).toUpperCase();
      const labelText = `${baseLabel} ${scorePct}%`;
      drawLabelOnBuffer(buffer, meta, det, labelText, {
        color,
        bgColor: color,
        textColor: { r: 0, g: 0, b: 0 },
        scale: 2
      });
    });

    return encodeRosImage(buffer, msg, meta);
  } catch (err) {
    console.error("Failed to annotate image:", err.message || err);
    return msg;
  }
}
function main() {
  console.log(`Connecting to rosbridge at ${ROSBRIDGE_URL} ...`);

  const ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });

  ros.on("connection", () => {
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
  });

  ros.on("error", (err) => {
    console.error("rosbridge error:", err);
  });

  ros.on("close", () => {
    console.log("rosbridge connection closed.");
  });
}

if (require.main === module) {
  main();
}
