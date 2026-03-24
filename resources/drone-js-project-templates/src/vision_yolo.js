#!/usr/bin/env node
/**
 * Drone YOLO demo: subscribe to the front camera, run inference, and
 * republish annotated frames for the image panel.
 */

require("dotenv").config();

const { getImageTopics, getYoloConfig } = require("./config");
const { ROSLibBridgeWrapper } = require("./lib/roslib-bridge-wrapper.js");
const { annotateImageMessage } = require("./vision/image_utils");
const { getSession, runYoloOnImageMsg } = require("./vision/yolo_inference");

const {
  imageTopic: IMAGE_TOPIC,
  annotatedImageTopic: ANNOTATED_IMAGE_TOPIC,
  messageType: IMAGE_MESSAGE_TYPE,
} = getImageTopics();
const yoloConfig = getYoloConfig();
const CONNECTION_TIMEOUT_MS = 30000;
const NO_FRAME_WARNING_MS = 10000;

async function main() {
  const bridge = new ROSLibBridgeWrapper();
  await bridge.waitForConnection(CONNECTION_TIMEOUT_MS);
  await getSession();

  const imageTopics = await bridge.getAvailableImageTopics().catch(() => []);
  if (!imageTopics.some((topic) => topic.topic === IMAGE_TOPIC)) {
    const available = imageTopics.map((topic) => topic.topic).join(", ") || "none";
    console.warn(
      `[VISION] Expected image topic ${IMAGE_TOPIC} is not currently advertised. Available image topics: ${available}`
    );
  }

  console.log(`[VISION] Connected. Listening on ${IMAGE_TOPIC}`);
  console.log(`[VISION] Annotated images will be published on ${ANNOTATED_IMAGE_TOPIC}`);
  console.log(
    `[VISION] YOLO model=${yoloConfig.modelName}, conf=${yoloConfig.confThreshold}, target=${yoloConfig.targetLabel}`
  );

  let frameCount = 0;
  let processing = false;
  let lastFrameAt = 0;
  const frameWatchdog = setTimeout(() => {
    if (lastFrameAt === 0) {
      console.warn(
        `[VISION] No frames received on ${IMAGE_TOPIC} after ${NO_FRAME_WARNING_MS}ms. Verify the VM camera bridge and active world.`
      );
    }
  }, NO_FRAME_WARNING_MS);

  const unsubscribe = bridge.subscribe(
    { topic: IMAGE_TOPIC, type: IMAGE_MESSAGE_TYPE },
    async (msg) => {
      if (processing) {
        return;
      }
      processing = true;
      frameCount += 1;
      lastFrameAt = Date.now();

      try {
        const detections = await runYoloOnImageMsg(msg, yoloConfig);
        const annotatedMsg = annotateImageMessage(msg, detections, { drawCenter: true });
        bridge.publish(ANNOTATED_IMAGE_TOPIC, IMAGE_MESSAGE_TYPE, annotatedMsg);

        if (frameCount % 5 === 0 || detections.length > 0) {
          const labels = detections.map((det) => det.label).join(", ") || "none";
          console.log(`[VISION] frame=${frameCount} detections=${detections.length} labels=${labels}`);
        }
      } catch (err) {
        console.error("[VISION] Frame processing failed:", err.message || err);
      } finally {
        processing = false;
      }
    }
  );

  const shutdown = () => {
    console.log("[VISION] Shutting down YOLO demo");
    clearTimeout(frameWatchdog);
    unsubscribe();
    process.exit(0);
  };

  process.on("SIGINT", shutdown);
  process.on("SIGTERM", shutdown);
}

if (require.main === module) {
  main().catch((err) => {
    console.error("[VISION] Startup failed:", err.message || err);
    process.exit(1);
  });
}
