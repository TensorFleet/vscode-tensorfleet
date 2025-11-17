#!/usr/bin/env node
/**
 * Image passthrough example using roslib and rosbridge.
 *
 * This script is intentionally simple:
 *   - Subscribes to an image topic (e.g. /camera/image_raw)
 *   - Republishes the same messages on another topic (e.g. /camera/image_annotated)
 *
 * It is designed to pair well with a heavier Python perception stack
 * (for example, YOLO in vision_yolo.py) while keeping the JS side focused
 * on wiring and visualization.
 *
 * Usage:
 *   node src/image_passthrough.js
 *
 * Environment overrides:
 *   ROS_HOST, ROS_PORT,
 *   IMAGE_TOPIC, ANNOTATED_IMAGE_TOPIC,
 *   IMAGE_MESSAGE_TYPE
 */

const ROSLIB = require("roslib");

const HOST = process.env.ROS_HOST ?? "172.16.0.10";
const PORT = Number(process.env.ROS_PORT ?? 9091);

const IMAGE_TOPIC = process.env.IMAGE_TOPIC ?? "/camera/image_raw";
const ANNOTATED_IMAGE_TOPIC =
  process.env.ANNOTATED_IMAGE_TOPIC ?? "/camera/image_annotated";
const IMAGE_MESSAGE_TYPE =
  process.env.IMAGE_MESSAGE_TYPE ?? "sensor_msgs/Image";

function main() {
  const url = `ws://${HOST}:${PORT}`;
  console.log(`Connecting to rosbridge at ${url} ...`);

  const ros = new ROSLIB.Ros({ url });

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

    subscriber.subscribe((msg) => {
      count += 1;
      publisher.publish(msg);
      console.log(
        `Forwarded image message #${count} from ${IMAGE_TOPIC} to ${ANNOTATED_IMAGE_TOPIC}.`
      );
    });

    const shutdown = () => {
      console.log("Shutting down image passthrough.");
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

