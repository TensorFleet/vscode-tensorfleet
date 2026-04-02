#!/usr/bin/env -S bun run
/**
 * Visual landing demo using the downward camera and a colored landing pad.
 */

require("dotenv").config();

const { initializeDroneControl } = require("./lib/drone_utils.js");
const { getImageTopics, getLandingConfig } = require("./config");
const { annotateBlobMessage, detectColorBlob } = require("./vision/image_utils");

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

async function main() {
  const landingConfig = getLandingConfig();
  const {
    landingImageTopic,
    landingAnnotatedImageTopic,
    messageType,
  } = getImageTopics();

  const { bridge, droneState, droneController } = await initializeDroneControl();

  let latestBlob = null;
  let stableFrames = 0;
  let desiredCommand = { vx: 0, vy: 0, vz: 0 };
  let processing = false;
  let modeInterval = null;
  let staleLogged = false;

  const unsubscribe = bridge.subscribe(
    { topic: landingImageTopic, type: messageType },
    async (msg) => {
      if (processing) {
        return;
      }
      processing = true;
      try {
        const blob = detectColorBlob(msg, {
          targetColor: landingConfig.padColor,
          minBlobPixels: landingConfig.minBlobPixels,
        });
        latestBlob = blob ? { ...blob, msg, receivedAt: Date.now() } : null;
        if (blob) {
          const annotated = annotateBlobMessage(msg, blob, {
            color: { r: 255, g: 255, b: 255 },
          });
          bridge.publish(landingAnnotatedImageTopic, messageType, annotated);
        }
      } catch (err) {
        console.error("[LANDING] Vision frame failed:", err.message || err);
      } finally {
        processing = false;
      }
    }
  );

  const loopIntervalMs = Math.max(50, Math.floor(1000 / landingConfig.loopHz));

  try {
    console.log("[LANDING] Taking off...");
    await droneController.requestAutoState({
      kind: "airborne",
      altMeters: landingConfig.takeoffAltitudeM,
    });
    console.log(
      `[LANDING] Using ${landingImageTopic} and pad color '${landingConfig.padColor}'`
    );

    modeInterval = setInterval(async () => {
      try {
        await droneController.setMode("OFFBOARD", 0, false);
        droneController.publishOffboardTarget({
          kind: "velocity_local",
          vx: desiredCommand.vx,
          vy: desiredCommand.vy,
          vz: desiredCommand.vz,
        });
      } catch (err) {
        console.warn("[LANDING] OFFBOARD stream warning:", err.message || err);
      }
    }, loopIntervalMs);

    while (true) {
      const blobIsFresh =
        latestBlob && Date.now() - latestBlob.receivedAt <= landingConfig.frameTimeoutMs;
      if (!blobIsFresh || !latestBlob) {
        desiredCommand = { vx: 0, vy: 0, vz: 0 };
        stableFrames = 0;
        if (!blobIsFresh && latestBlob) {
          if (!staleLogged) {
            console.warn("[LANDING] Vision data stale, hovering until the pad is seen again.");
            staleLogged = true;
          }
        } else {
          staleLogged = false;
          console.log("[LANDING] Waiting for landing pad...");
        }
        await sleep(loopIntervalMs);
        continue;
      }
      staleLogged = false;

      const { meta, centroidX, centroidY, areaRatio } = latestBlob;
      const normX = (centroidX - meta.width / 2) / (meta.width / 2);
      const normY = (centroidY - meta.height / 2) / (meta.height / 2);

      const centered =
        Math.abs(normX) < landingConfig.centeredTolerance &&
        Math.abs(normY) < landingConfig.centeredTolerance;
      const descentAligned =
        Math.abs(normX) < landingConfig.descentTolerance &&
        Math.abs(normY) < landingConfig.descentTolerance;

      desiredCommand = {
        vx: clamp(-normY * landingConfig.xyGain, -landingConfig.maxXySpeed, landingConfig.maxXySpeed),
        vy: clamp(-normX * landingConfig.xyGain, -landingConfig.maxXySpeed, landingConfig.maxXySpeed),
        vz: 0,
      };

      if (descentAligned) {
        desiredCommand.vz =
          areaRatio > landingConfig.handoffAreaRatio * 0.7
            ? -landingConfig.slowDescentSpeedMps
            : -landingConfig.descentSpeedMps;
      }

      if (centered && areaRatio >= landingConfig.handoffAreaRatio) {
        stableFrames += 1;
      } else {
        stableFrames = 0;
      }

      console.log(
        `[LANDING] area=${areaRatio.toFixed(3)} norm=(${normX.toFixed(3)}, ${normY.toFixed(3)}) cmd=(${desiredCommand.vx.toFixed(2)}, ${desiredCommand.vy.toFixed(2)}, ${desiredCommand.vz.toFixed(2)}) stable=${stableFrames}/${landingConfig.stableFramesRequired}`
      );

      if (stableFrames >= landingConfig.stableFramesRequired) {
        break;
      }

      await sleep(loopIntervalMs);
    }

    desiredCommand = { vx: 0, vy: 0, vz: 0 };
    await sleep(1000);
    console.log("[LANDING] Handoff to AUTO.LAND");
    await droneController.requestAutoState({ kind: "landed", armed: false });
    console.log("[LANDING] Complete.");
  } finally {
    if (modeInterval) {
      clearInterval(modeInterval);
    }
    unsubscribe();
    droneState.disconnect();
  }
}

if (require.main === module) {
  main().catch((err) => {
    console.error("[LANDING] Fatal error:", err.message || err);
    process.exit(1);
  });
}
