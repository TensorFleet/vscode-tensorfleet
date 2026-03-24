#!/usr/bin/env node
/**
 * Shared image decoding, annotation, and color-blob helpers.
 */

const jpeg = require("jpeg-js");

const FONT_5X7 = {
  " ": ["00000", "00000", "00000", "00000", "00000", "00000", "00000"],
  "%": ["11001", "11010", "00100", "01000", "10110", "00110", "00000"],
  ".": ["00000", "00000", "00000", "00000", "00000", "01100", "01100"],
  "-": ["00000", "00000", "00000", "11111", "00000", "00000", "00000"],
  "_": ["00000", "00000", "00000", "00000", "00000", "00000", "11111"],
  "0": ["01110", "10001", "10011", "10101", "11001", "10001", "01110"],
  "1": ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
  "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
  "3": ["11110", "00001", "00001", "01110", "00001", "00001", "11110"],
  "4": ["00010", "00110", "01010", "10010", "11111", "00010", "00010"],
  "5": ["11111", "10000", "10000", "11110", "00001", "00001", "11110"],
  "6": ["01110", "10000", "10000", "11110", "10001", "10001", "01110"],
  "7": ["11111", "00001", "00010", "00100", "01000", "01000", "01000"],
  "8": ["01110", "10001", "10001", "01110", "10001", "10001", "01110"],
  "9": ["01110", "10001", "10001", "01111", "00001", "00001", "01110"],
  A: ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
  B: ["11110", "10001", "10001", "11110", "10001", "10001", "11110"],
  C: ["01110", "10001", "10000", "10000", "10000", "10001", "01110"],
  D: ["11100", "10010", "10001", "10001", "10001", "10010", "11100"],
  E: ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
  F: ["11111", "10000", "10000", "11110", "10000", "10000", "10000"],
  G: ["01110", "10001", "10000", "10111", "10001", "10001", "01110"],
  H: ["10001", "10001", "10001", "11111", "10001", "10001", "10001"],
  I: ["01110", "00100", "00100", "00100", "00100", "00100", "01110"],
  J: ["00001", "00001", "00001", "00001", "10001", "10001", "01110"],
  K: ["10001", "10010", "10100", "11000", "10100", "10010", "10001"],
  L: ["10000", "10000", "10000", "10000", "10000", "10000", "11111"],
  M: ["10001", "11011", "10101", "10101", "10001", "10001", "10001"],
  N: ["10001", "11001", "10101", "10011", "10001", "10001", "10001"],
  O: ["01110", "10001", "10001", "10001", "10001", "10001", "01110"],
  P: ["11110", "10001", "10001", "11110", "10000", "10000", "10000"],
  Q: ["01110", "10001", "10001", "10001", "10101", "10010", "01101"],
  R: ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
  S: ["01111", "10000", "10000", "01110", "00001", "00001", "11110"],
  T: ["11111", "00100", "00100", "00100", "00100", "00100", "00100"],
  U: ["10001", "10001", "10001", "10001", "10001", "10001", "01110"],
  V: ["10001", "10001", "10001", "10001", "10001", "01010", "00100"],
  W: ["10001", "10001", "10001", "10101", "10101", "10101", "01010"],
  X: ["10001", "10001", "01010", "00100", "01010", "10001", "10001"],
  Y: ["10001", "10001", "01010", "00100", "00100", "00100", "00100"],
  Z: ["11111", "00001", "00010", "00100", "01000", "10000", "11111"],
};

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

  if (typeof dataField === "string" && dataField.startsWith("data:image/")) {
    const commaIndex = dataField.indexOf(",");
    const base64Part = commaIndex >= 0 ? dataField.slice(commaIndex + 1) : dataField;
    const jpegBytes = Buffer.from(base64Part, "base64");
    const decoded = jpeg.decode(jpegBytes, { useTArray: true });
    width = decoded.width;
    height = decoded.height;
    buffer = Buffer.from(decoded.data);
    channels = 4;
    encodingKind = "data-url-jpeg";
  } else if (typeof dataField === "string") {
    buffer = Buffer.from(dataField, "base64");
    encodingKind = "base64";
  } else {
    buffer = Buffer.from(dataField);
    encodingKind = "list";
  }

  let step = msg.step;
  if (!step || step === 0) {
    if (!channels) {
      channels = 3;
    }
    step = width * channels;
  }

  if (!channels) {
    channels = Math.max(1, Math.floor(step / width));
  }

  return {
    buffer,
    meta: {
      height,
      width,
      step,
      encoding,
      channels,
      encodingKind,
    },
  };
}

function encodeRosImage(buffer, templateMsg, meta) {
  const { height, width, encoding, encodingKind, step } = meta;
  let dataField;

  if (encodingKind === "data-url-jpeg") {
    const rawImageData = { data: buffer, width, height };
    const jpegImage = jpeg.encode(rawImageData, 90);
    dataField = `data:image/jpeg;base64,${jpegImage.data.toString("base64")}`;
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
    data: dataField,
  };
}

function normalizeBox(box, meta) {
  const { width, height } = meta;
  const x1 = Math.max(0, Math.floor(box.x1));
  const y1 = Math.max(0, Math.floor(box.y1));
  const x2 = Math.min(width - 1, Math.ceil(box.x2));
  const y2 = Math.min(height - 1, Math.ceil(box.y2));
  if (x2 <= x1 || y2 <= y1) {
    return null;
  }
  return { x1, y1, x2, y2 };
}

function setPixel(buffer, meta, x, y, color) {
  const { width, height, step, channels, encoding } = meta;
  if (x < 0 || x >= width || y < 0 || y >= height) {
    return;
  }

  const offset = y * step + x * channels;
  if (offset + 2 >= buffer.length) {
    return;
  }

  const isBgr = (encoding || "").toLowerCase().startsWith("bgr");
  if (isBgr) {
    buffer[offset] = color.b;
    buffer[offset + 1] = color.g;
    buffer[offset + 2] = color.r;
  } else {
    buffer[offset] = color.r;
    buffer[offset + 1] = color.g;
    buffer[offset + 2] = color.b;
  }
}

function blendPixel(buffer, meta, x, y, color, alpha = 1) {
  const { width, height, step, channels, encoding } = meta;
  if (x < 0 || x >= width || y < 0 || y >= height) {
    return;
  }

  const offset = y * step + x * channels;
  if (offset + 2 >= buffer.length) {
    return;
  }

  const a = Math.max(0, Math.min(1, alpha));
  const isBgr = (encoding || "").toLowerCase().startsWith("bgr");
  const src = isBgr
    ? [color.b, color.g, color.r]
    : [color.r, color.g, color.b];

  for (let i = 0; i < 3; i += 1) {
    buffer[offset + i] = Math.round(buffer[offset + i] * (1 - a) + src[i] * a);
  }
}

function fillRect(buffer, meta, x, y, width, height, color, alpha = 1) {
  const x1 = Math.max(0, Math.floor(x));
  const y1 = Math.max(0, Math.floor(y));
  const x2 = Math.min(meta.width - 1, Math.floor(x + width - 1));
  const y2 = Math.min(meta.height - 1, Math.floor(y + height - 1));

  for (let yy = y1; yy <= y2; yy += 1) {
    for (let xx = x1; xx <= x2; xx += 1) {
      blendPixel(buffer, meta, xx, yy, color, alpha);
    }
  }
}

function drawRectOnBuffer(buffer, meta, box, options = {}) {
  const norm = normalizeBox(box, meta);
  if (!norm) {
    return;
  }

  const thickness = options.thickness || 1;
  const color = options.color || { r: 0, g: 255, b: 255 };
  const { x1, y1, x2, y2 } = norm;

  for (let x = x1; x <= x2; x += 1) {
    for (let t = 0; t < thickness; t += 1) {
      blendPixel(buffer, meta, x, y1 + t, color, 0.9);
      blendPixel(buffer, meta, x, y2 - t, color, 0.9);
    }
  }

  for (let y = y1; y <= y2; y += 1) {
    for (let t = 0; t < thickness; t += 1) {
      blendPixel(buffer, meta, x1 + t, y, color, 0.9);
      blendPixel(buffer, meta, x2 - t, y, color, 0.9);
    }
  }
}

function getGlyph(char) {
  return FONT_5X7[char] || FONT_5X7[char.toUpperCase()] || FONT_5X7._;
}

function measureText(text, scale = 1) {
  return text.length * (5 * scale + scale) - scale;
}

function drawText(buffer, meta, text, x, y, options = {}) {
  const scale = options.scale || 1;
  const color = options.color || { r: 255, g: 255, b: 255 };
  const shadow = options.shadowColor || { r: 0, g: 0, b: 0 };
  let cursorX = x;

  for (const char of text.toUpperCase()) {
    const glyph = getGlyph(char);
    glyph.forEach((row, rowIdx) => {
      for (let colIdx = 0; colIdx < row.length; colIdx += 1) {
        if (row[colIdx] !== "1") {
          continue;
        }
        for (let sy = 0; sy < scale; sy += 1) {
          for (let sx = 0; sx < scale; sx += 1) {
            setPixel(buffer, meta, cursorX + colIdx * scale + sx + 1, y + rowIdx * scale + sy + 1, shadow);
            setPixel(buffer, meta, cursorX + colIdx * scale + sx, y + rowIdx * scale + sy, color);
          }
        }
      }
    });
    cursorX += 5 * scale + scale;
  }
}

function formatDetectionLabel(det) {
  const scorePercent = Math.round((det.score || 0) * 100);
  return `${(det.label || "object").replaceAll("_", " ")} ${scorePercent}%`;
}

function drawDetectionLabel(buffer, meta, box, text, color) {
  const scale = 1;
  const paddingX = 3;
  const paddingY = 2;
  const textWidth = measureText(text, scale);
  const labelWidth = textWidth + paddingX * 2;
  const labelHeight = 7 * scale + paddingY * 2;
  const x = Math.max(0, Math.min(meta.width - labelWidth - 1, box.x1));
  const preferredY = box.y1 - labelHeight - 2;
  const y = preferredY >= 0
    ? preferredY
    : Math.max(0, Math.min(meta.height - labelHeight - 1, box.y1 + 2));

  fillRect(buffer, meta, x, y, labelWidth, labelHeight, { r: 0, g: 0, b: 0 }, 0.72);
  fillRect(buffer, meta, x, y, labelWidth, 2, color, 1);
  drawText(buffer, meta, text, x + paddingX, y + paddingY, {
    scale,
    color: { r: 255, g: 255, b: 255 },
    shadowColor: { r: 0, g: 0, b: 0 },
  });
}

function drawCrosshair(buffer, meta, x, y, color = { r: 255, g: 255, b: 255 }) {
  for (let dx = -10; dx <= 10; dx += 1) {
    setPixel(buffer, meta, x + dx, y, color);
  }
  for (let dy = -10; dy <= 10; dy += 1) {
    setPixel(buffer, meta, x, y + dy, color);
  }
}

function annotateImageMessage(msg, detections, options = {}) {
  if (!detections || detections.length === 0) {
    return msg;
  }

  try {
    const { buffer, meta } = decodeRosImage(msg);
    const colors = options.colors || [
      { r: 0, g: 255, b: 255 },
      { r: 255, g: 0, b: 255 },
      { r: 255, g: 255, b: 0 },
      { r: 0, g: 255, b: 0 },
    ];

    detections.forEach((det, idx) => {
      const color = colors[idx % colors.length];
      const norm = normalizeBox(det, meta);
      if (!norm) {
        return;
      }
      drawRectOnBuffer(buffer, meta, norm, {
        color: colors[idx % colors.length],
        thickness: options.thickness || 4,
      });
      drawDetectionLabel(buffer, meta, norm, formatDetectionLabel(det), color);
    });

    if (options.drawCenter) {
      drawCrosshair(
        buffer,
        meta,
        Math.floor(meta.width / 2),
        Math.floor(meta.height / 2),
        { r: 255, g: 255, b: 255 }
      );
    }

    return encodeRosImage(buffer, msg, meta);
  } catch (err) {
    console.error("[VISION] Failed to annotate image:", err.message || err);
    return msg;
  }
}

function detectColorBlob(msg, options = {}) {
  const { buffer, meta } = decodeRosImage(msg);
  const targetColor = (options.targetColor || "magenta").toLowerCase();
  const minPixels = options.minBlobPixels || 100;

  let minX = meta.width;
  let minY = meta.height;
  let maxX = -1;
  let maxY = -1;
  let sumX = 0;
  let sumY = 0;
  let count = 0;

  for (let y = 0; y < meta.height; y += 1) {
    for (let x = 0; x < meta.width; x += 1) {
      const offset = y * meta.step + x * meta.channels;
      const isBgr = meta.encoding.startsWith("bgr");
      const r = isBgr ? buffer[offset + 2] : buffer[offset];
      const g = isBgr ? buffer[offset + 1] : buffer[offset + 1];
      const b = isBgr ? buffer[offset] : buffer[offset + 2];

      let match = false;
      if (targetColor === "magenta") {
        match = r > 180 && b > 180 && g < 120;
      } else if (targetColor === "cyan") {
        match = r < 120 && g > 170 && b > 170;
      } else if (targetColor === "yellow") {
        match = r > 170 && g > 170 && b < 120;
      }

      if (!match) {
        continue;
      }

      count += 1;
      sumX += x;
      sumY += y;
      if (x < minX) minX = x;
      if (y < minY) minY = y;
      if (x > maxX) maxX = x;
      if (y > maxY) maxY = y;
    }
  }

  if (count < minPixels) {
    return null;
  }

  const centroidX = sumX / count;
  const centroidY = sumY / count;
  const areaRatio = count / (meta.width * meta.height);

  return {
    count,
    areaRatio,
    centroidX,
    centroidY,
    meta,
    bounds: {
      x1: minX,
      y1: minY,
      x2: maxX,
      y2: maxY,
    },
  };
}

function annotateBlobMessage(msg, blob, options = {}) {
  if (!blob) {
    return msg;
  }

  try {
    const { buffer, meta } = decodeRosImage(msg);
    drawRectOnBuffer(buffer, meta, blob.bounds, {
      color: options.color || { r: 255, g: 255, b: 255 },
      thickness: options.thickness || 4,
    });
    drawCrosshair(buffer, meta, Math.floor(blob.centroidX), Math.floor(blob.centroidY), {
      r: 255,
      g: 255,
      b: 255,
    });
    drawCrosshair(buffer, meta, Math.floor(meta.width / 2), Math.floor(meta.height / 2), {
      r: 0,
      g: 255,
      b: 255,
    });
    return encodeRosImage(buffer, msg, meta);
  } catch (err) {
    console.error("[VISION] Failed to annotate color blob:", err.message || err);
    return msg;
  }
}

module.exports = {
  annotateBlobMessage,
  annotateImageMessage,
  decodeRosImage,
  detectColorBlob,
  drawRectOnBuffer,
  encodeRosImage,
};
