#!/usr/bin/env node
/**
 * Color-based shape detection for simulation environments.
 *
 * This script detects objects based on their distinct colors using HSV
 * color space, which is robust to lighting/shading variations.
 *
 * Usage:
 *   bun src/vision_colors.js
 *
 * Environment overrides:
 *   - TENSORFLEET_BASE_URL, TENSORFLEET_JWT (for proxy connection)
 *   - IMAGE_TOPIC, ANNOTATED_IMAGE_TOPIC
 */

require("dotenv").config();
const ROSLIB = require("roslib");
const jpeg = require("jpeg-js");
const { getImageTopics } = require("./config");
const { connectToRobot } = require("./lib/robotic_utils");

const {
    imageTopic: IMAGE_TOPIC,
    annotatedImageTopic: ANNOTATED_IMAGE_TOPIC,
    messageType: IMAGE_MESSAGE_TYPE
} = getImageTopics();

// Define target colors using HSV ranges (more robust to lighting)
// hueMin/hueMax: 0-360, satMin: 0-1, valMin: 0-1
const TARGET_COLORS = [
    { name: "red", hueMin: 0, hueMax: 15, satMin: 0.4, valMin: 0.2 },
    { name: "red2", hueMin: 345, hueMax: 360, satMin: 0.4, valMin: 0.2, displayName: "red" },
    { name: "orange", hueMin: 15, hueMax: 35, satMin: 0.4, valMin: 0.3 },
    { name: "yellow", hueMin: 35, hueMax: 70, satMin: 0.3, valMin: 0.4 },
    { name: "green", hueMin: 70, hueMax: 160, satMin: 0.3, valMin: 0.2 },
    { name: "cyan", hueMin: 160, hueMax: 200, satMin: 0.3, valMin: 0.3 },
    { name: "blue", hueMin: 200, hueMax: 260, satMin: 0.3, valMin: 0.2 },
    { name: "purple", hueMin: 260, hueMax: 290, satMin: 0.3, valMin: 0.2 },
    { name: "magenta", hueMin: 290, hueMax: 345, satMin: 0.3, valMin: 0.3 },
];

// Minimum blob size to consider (in pixels)
const MIN_BLOB_AREA = 200;

// Minimum saturation to consider (filters out gray/black/white)
const MIN_SATURATION = 0.25;

/**
 * Convert RGB to HSV
 * Returns { h: 0-360, s: 0-1, v: 0-1 }
 */
function rgbToHsv(r, g, b) {
    r /= 255; g /= 255; b /= 255;
    const max = Math.max(r, g, b);
    const min = Math.min(r, g, b);
    const d = max - min;

    let h = 0;
    const s = max === 0 ? 0 : d / max;
    const v = max;

    if (d !== 0) {
        switch (max) {
            case r: h = ((g - b) / d + (g < b ? 6 : 0)) * 60; break;
            case g: h = ((b - r) / d + 2) * 60; break;
            case b: h = ((r - g) / d + 4) * 60; break;
        }
    }

    return { h, s, v };
}

/**
 * Decode a ROS image message into raw pixel data
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
    let channels = 3;
    let encodingKind;

    // Handle data URLs (JPEG encoded)
    if (typeof dataField === "string" && dataField.startsWith("data:image/")) {
        const commaIndex = dataField.indexOf(",");
        const base64Part = commaIndex >= 0 ? dataField.slice(commaIndex + 1) : dataField;
        const jpegBytes = Buffer.from(base64Part, "base64");
        const decoded = jpeg.decode(jpegBytes, { useTArray: true });
        if (!decoded || !decoded.data) {
            throw new Error("Failed to decode JPEG image");
        }
        width = decoded.width;
        height = decoded.height;
        buffer = Buffer.from(decoded.data); // RGBA
        channels = 4;
        encodingKind = "data-url-jpeg";
    } else if (typeof dataField === "string") {
        buffer = Buffer.from(dataField, "base64");
        encodingKind = "base64";
    } else {
        buffer = Buffer.from(dataField);
        encodingKind = "list";
    }
    let step = msg.step || width * channels;
    if (!channels) {
        channels = Math.max(1, Math.floor(step / width));
    }

    return {
        buffer,
        meta: { height, width, step, encoding, channels, encodingKind }
    };
}

function encodeRosImage(buffer, templateMsg, meta) {
    const { height, width, encoding, encodingKind, step } = meta;

    let dataField;
    if (encodingKind === "data-url-jpeg") {
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
 * Get RGB pixel at (x, y) from buffer
 */
function getPixel(buffer, meta, x, y) {
    const { width, channels, encoding } = meta;
    const offset = y * width * channels + x * channels;

    const isBgr = (encoding || "").toLowerCase().startsWith("bgr");
    if (isBgr) {
        return { r: buffer[offset + 2], g: buffer[offset + 1], b: buffer[offset] };
    }
    return { r: buffer[offset], g: buffer[offset + 1], b: buffer[offset + 2] };
}

/**
 * Set RGB pixel at (x, y) in buffer
 */
function setPixel(buffer, meta, x, y, color) {
    const { width, height, channels, encoding } = meta;
    if (x < 0 || x >= width || y < 0 || y >= height) return;

    const offset = y * width * channels + x * channels;
    if (offset + 2 >= buffer.length) return;

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
    const { width, height, channels, encoding } = meta;
    if (x < 0 || x >= width || y < 0 || y >= height) return;
    const offset = y * width * channels + x * channels;
    if (offset + 2 >= buffer.length) return;
    const a = Math.max(0, Math.min(1, alpha));
    const isBgr = (encoding || "").toLowerCase().startsWith("bgr");
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
    for (let yy = y1; yy <= y2; yy++) {
        for (let xx = x1; xx <= x2; xx++) {
            blendPixel(buffer, meta, xx, yy, color, alpha);
        }
    }
}

/**
 * Calculate color distance (Euclidean in RGB space)
 */
function colorDistance(c1, c2) {
    return Math.sqrt(
        Math.pow(c1.r - c2.r, 2) +
        Math.pow(c1.g - c2.g, 2) +
        Math.pow(c1.b - c2.b, 2)
    );
}

/**
 * Create a binary mask for pixels matching a target color (using HSV)
 */
function createColorMask(buffer, meta, targetColor) {
    const { width, height } = meta;
    const mask = new Uint8Array(width * height);

    for (let y = 0; y < height; y++) {
        for (let x = 0; x < width; x++) {
            const pixel = getPixel(buffer, meta, x, y);
            const hsv = rgbToHsv(pixel.r, pixel.g, pixel.b);

            // Check if pixel is saturated enough (skip grays/blacks/whites)
            if (hsv.s < targetColor.satMin || hsv.v < targetColor.valMin) {
                continue;
            }

            // Check if hue is in range
            if (hsv.h >= targetColor.hueMin && hsv.h <= targetColor.hueMax) {
                mask[y * width + x] = 1;
            }
        }
    }

    return mask;
}

/**
 * Find connected components (blobs) in a binary mask
 * Returns array of { minX, minY, maxX, maxY, area }
 */
function findBlobs(mask, width, height) {
    const visited = new Uint8Array(width * height);
    const blobs = [];

    for (let y = 0; y < height; y++) {
        for (let x = 0; x < width; x++) {
            const idx = y * width + x;
            if (mask[idx] === 1 && visited[idx] === 0) {
                // BFS to find connected component
                const queue = [[x, y]];
                let minX = x, maxX = x, minY = y, maxY = y;
                let area = 0;

                while (queue.length > 0) {
                    const [cx, cy] = queue.shift();
                    const cidx = cy * width + cx;

                    if (cx < 0 || cx >= width || cy < 0 || cy >= height) continue;
                    if (visited[cidx] || mask[cidx] === 0) continue;

                    visited[cidx] = 1;
                    area++;

                    minX = Math.min(minX, cx);
                    maxX = Math.max(maxX, cx);
                    minY = Math.min(minY, cy);
                    maxY = Math.max(maxY, cy);

                    // 4-connectivity neighbors
                    queue.push([cx + 1, cy], [cx - 1, cy], [cx, cy + 1], [cx, cy - 1]);
                }

                if (area >= MIN_BLOB_AREA) {
                    blobs.push({ minX, minY, maxX, maxY, area });
                }
            }
        }
    }

    return blobs;
}

/**
 * Detect colored objects in an image
 */
function detectColoredObjects(buffer, meta) {
    const detections = [];

    // Map color names to RGB for drawing boxes
    const colorRgb = {
        red: { r: 255, g: 50, b: 50 },
        orange: { r: 255, g: 165, b: 0 },
        yellow: { r: 255, g: 255, b: 50 },
        green: { r: 50, g: 255, b: 50 },
        cyan: { r: 50, g: 255, b: 255 },
        blue: { r: 50, g: 100, b: 255 },
        purple: { r: 180, g: 50, b: 255 },
        magenta: { r: 255, g: 50, b: 255 },
    };

    for (const targetColor of TARGET_COLORS) {
        const mask = createColorMask(buffer, meta, targetColor);
        const blobs = findBlobs(mask, meta.width, meta.height);

        const displayName = targetColor.displayName || targetColor.name;
        const boxColor = colorRgb[displayName] || { r: 0, g: 255, b: 255 };

        for (const blob of blobs) {
            detections.push({
                x1: blob.minX,
                y1: blob.minY,
                x2: blob.maxX,
                y2: blob.maxY,
                score: Math.min(1.0, blob.area / 5000),
                label: displayName,
                color: boxColor,
                area: blob.area
            });
        }
    }

    // Sort by area (largest first)
    detections.sort((a, b) => b.area - a.area);

    return detections;
}

/**
 * Draw bounding box on image
 */
function drawRect(buffer, meta, box, color, thickness = 2) {
    const { height } = meta;
    const x1 = Math.max(0, Math.floor(box.x1));
    const y1 = Math.max(0, Math.floor(box.y1));
    const x2 = Math.min(meta.width - 1, Math.ceil(box.x2));
    const y2 = Math.min(height - 1, Math.ceil(box.y2));

    for (let x = x1; x <= x2; x++) {
        for (let t = 0; t < thickness; t++) {
            blendPixel(buffer, meta, x, y1 + t, color, 0.88);
            blendPixel(buffer, meta, x, y2 - t, color, 0.88);
        }
    }

    for (let y = y1; y <= y2; y++) {
        for (let t = 0; t < thickness; t++) {
            blendPixel(buffer, meta, x1 + t, y, color, 0.88);
            blendPixel(buffer, meta, x2 - t, y, color, 0.88);
        }
    }
}

/**
 * Simple text rendering for labels
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
    " ": [0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000],
    "-": [0b00000, 0b00000, 0b00000, 0b01110, 0b00000, 0b00000, 0b00000],
    "%": [0b11001, 0b11010, 0b00100, 0b00100, 0b01011, 0b10011, 0b00000],
    ...Object.fromEntries("ABCDEFGHIJKLMNOPQRSTUVWXYZ".split("").map((c, i) => [
        c, [
            [0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001], // A
            [0b11110, 0b10001, 0b10001, 0b11110, 0b10001, 0b10001, 0b11110], // B
            [0b01110, 0b10001, 0b10000, 0b10000, 0b10000, 0b10001, 0b01110], // C
            [0b11110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11110], // D
            [0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111], // E
            [0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b10000], // F
            [0b01110, 0b10001, 0b10000, 0b10011, 0b10001, 0b10001, 0b01110], // G
            [0b10001, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001], // H
            [0b01110, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110], // I
            [0b00001, 0b00001, 0b00001, 0b00001, 0b00001, 0b10001, 0b01110], // J
            [0b10001, 0b10010, 0b10100, 0b11000, 0b10100, 0b10010, 0b10001], // K
            [0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b11111], // L
            [0b10001, 0b11011, 0b10101, 0b10101, 0b10001, 0b10001, 0b10001], // M
            [0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001, 0b10001], // N
            [0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110], // O
            [0b11110, 0b10001, 0b10001, 0b11110, 0b10000, 0b10000, 0b10000], // P
            [0b01110, 0b10001, 0b10001, 0b10001, 0b10101, 0b10010, 0b01101], // Q
            [0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001], // R
            [0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110], // S
            [0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100], // T
            [0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110], // U
            [0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01010, 0b00100], // V
            [0b10001, 0b10001, 0b10001, 0b10101, 0b10101, 0b10101, 0b01010], // W
            [0b10001, 0b10001, 0b01010, 0b00100, 0b01010, 0b10001, 0b10001], // X
            [0b10001, 0b10001, 0b01010, 0b00100, 0b00100, 0b00100, 0b00100], // Y
            [0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0b11111], // Z
        ][i]
    ]))
};

function drawLabel(buffer, meta, box, text, accentColor) {
    const { width, height } = meta;
    const textColor = { r: 235, g: 242, b: 245 };
    const scale = 1;
    const charWidth = 5;
    const charHeight = 7;
    const charSpacing = 1;
    const paddingX = 6;
    const paddingY = 3;

    const textLen = text.length;
    const chipWidth = (textLen > 0
        ? (textLen * (charWidth + charSpacing) - charSpacing) * scale
        : 0) + paddingX * 2;
    const chipHeight = charHeight * scale + paddingY * 2;

    let labelX = Math.max(0, Math.floor(box.x1));
    let labelY = Math.floor(box.y1) - chipHeight - 2;
    if (labelY < 0) labelY = Math.floor(box.y1) + 2;
    if (labelX + chipWidth > width) labelX = Math.max(0, width - chipWidth);

    fillRect(buffer, meta, labelX, labelY, chipWidth, chipHeight, { r: 10, g: 14, b: 18 }, 0.78);

    for (let bx = labelX; bx < labelX + chipWidth; bx++) {
        blendPixel(buffer, meta, bx, labelY, accentColor, 0.28);
        blendPixel(buffer, meta, bx, labelY + chipHeight - 1, accentColor, 0.28);
    }
    for (let by = labelY + 1; by < labelY + chipHeight - 1; by++) {
        blendPixel(buffer, meta, labelX, by, accentColor, 0.28);
        blendPixel(buffer, meta, labelX + chipWidth - 1, by, accentColor, 0.28);
    }

    let cursorX = labelX + paddingX;
    const cursorY = labelY + paddingY;

    for (let i = 0; i < text.length; i++) {
        const ch = text[i].toUpperCase();
        const glyph = FONT_5X7[ch] || FONT_5X7[" "];

        for (let row = 0; row < charHeight; row++) {
            const rowBits = glyph[row];
            for (let col = 0; col < charWidth; col++) {
                const bit = (rowBits >> (charWidth - 1 - col)) & 1;
                if (!bit) continue;
                const xx = cursorX + col * scale;
                const yy = cursorY + row * scale;
                if (xx >= 0 && xx < width && yy >= 0 && yy < height) {
                    setPixel(buffer, meta, xx, yy, textColor);
                }
            }
        }
        cursorX += (charWidth + charSpacing) * scale;
    }
}

/**
 * Annotate image with detected objects
 */
function annotateImage(msg, detections) {
    if (!detections || detections.length === 0) return msg;

    try {
        const { buffer, meta } = decodeRosImage(msg);

        detections.forEach((det) => {
            const boxColor = det.color || { r: 92, g: 196, b: 214 };
            drawRect(buffer, meta, det, boxColor, 2);
            const label = (det.label || "Object").replace(/\b\w/g, (c) => c.toUpperCase());
            drawLabel(buffer, meta, det, label, boxColor);
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
            `Subscribing to '${IMAGE_TOPIC}' and republishing to '${ANNOTATED_IMAGE_TOPIC}'.`
        );
        console.log("Detecting colors:", TARGET_COLORS.map(c => c.name).join(", "));

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

        subscriber.subscribe((msg) => {
            count++;

            try {
                const { buffer, meta } = decodeRosImage(msg);
                const detections = detectColoredObjects(buffer, meta);

                if (detections.length > 0) {
                    console.log(`Image #${count}: ${detections.length} colored objects detected:`,
                        detections.map(d => `${d.label}(${d.area}px)`).join(", ")
                    );
                } else {
                    console.log(`Image #${count}: no colored objects detected.`);
                }

                const annotatedMsg = annotateImage(msg, detections);
                publisher.publish(annotatedMsg);
            } catch (err) {
                console.error("Error processing image:", err.message || err);
                publisher.publish(msg);
            }
        });

        const shutdown = () => {
            console.log("Shutting down color detection.");
            subscriber.unsubscribe();
            publisher.unadvertise();
            ros.close();
        };

        process.on("SIGINT", shutdown);
        process.on("SIGTERM", shutdown);
    } catch (err) {
        console.error("Failed to connect:", err);
        process.exit(1);
    }
}

if (require.main === module) {
    main();
}
