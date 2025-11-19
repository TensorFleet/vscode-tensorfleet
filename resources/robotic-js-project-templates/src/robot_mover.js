#!/usr/bin/env node
/**
 * Simple movement example using roslib and rosbridge.
 *
 * This script:
 *   - Connects to the rosbridge WebSocket server
 *   - Publishes geometry_msgs/Twist messages on /cmd_vel_raw
 *   - Runs a short movement sequence (forward, backward, left, right, stop)
 *
 * Usage:
 *   bun src/robot_mover.js
 *
 * Environment overrides:
 *   ROS_HOST, ROS_PORT, CMD_VEL_TOPIC, LINEAR_SPEED, ANGULAR_SPEED
 */

const ROSLIB = require("roslib");
const {
  getRosConnectionDetails,
  getCmdVelTopic,
  getMovementSpeeds
} = require("./config");

const { url: ROSBRIDGE_URL } = getRosConnectionDetails();
const CMD_VEL_TOPIC = getCmdVelTopic();
const { linearSpeed: LINEAR_SPEED, angularSpeed: ANGULAR_SPEED } =
  getMovementSpeeds();

function makeTwist(linearX, angularZ) {
  return new ROSLIB.Message({
    linear: { x: linearX, y: 0.0, z: 0.0 },
    angular: { x: 0.0, y: 0.0, z: angularZ }
  });
}

function publishFor(ros, topic, ms, linearX, angularZ, label) {
  return new Promise((resolve) => {
    const intervalMs = 50;
    const endTime = Date.now() + ms;
    const msg = makeTwist(linearX, angularZ);

    console.log(
      `Phase: ${label} (duration ${ms / 1000}s, lin=${linearX}, ang=${angularZ})`
    );

    const timer = setInterval(() => {
      if (!ros.isConnected || Date.now() >= endTime) {
        clearInterval(timer);
        resolve();
        return;
      }
      topic.publish(msg);
    }, intervalMs);
  });
}

async function runMovement(ros) {
  console.log(`Advertising Twist publisher on '${CMD_VEL_TOPIC}' ...`);
  const cmdVel = new ROSLIB.Topic({
    ros,
    name: CMD_VEL_TOPIC,
    messageType: "geometry_msgs/Twist"
  });

  const forwardDuration = 3000;
  const backwardDuration = 3000;
  const turnDuration = 2000;
  const stopDuration = 1000;

  try {
    // 1) Drive straight forward
    await publishFor(ros, cmdVel, forwardDuration, LINEAR_SPEED, 0.0, "forward");
    await publishFor(ros, cmdVel, stopDuration, 0.0, 0.0, "stop after forward");

    // 2) Drive straight backward
    await publishFor(
      ros,
      cmdVel,
      backwardDuration,
      -LINEAR_SPEED,
      0.0,
      "backward"
    );
    await publishFor(ros, cmdVel, stopDuration, 0.0, 0.0, "stop after backward");

    // 3) Turn left in place
    await publishFor(ros, cmdVel, turnDuration, 0.0, ANGULAR_SPEED, "turn left");
    await publishFor(ros, cmdVel, stopDuration, 0.0, 0.0, "stop after left turn");

    // 4) Turn right in place
    await publishFor(
      ros,
      cmdVel,
      turnDuration,
      0.0,
      -ANGULAR_SPEED,
      "turn right"
    );
    await publishFor(ros, cmdVel, stopDuration, 0.0, 0.0, "final stop");

    console.log("Movement sequence complete.");
  } finally {
    cmdVel.unadvertise();
  }
}

function main() {
  console.log(
    `Connecting to rosbridge at ${ROSBRIDGE_URL} using roslib ...`
  );

  const ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });

  ros.on("connection", async () => {
    console.log("roslib connection established successfully.");
    try {
      await runMovement(ros);
    } catch (err) {
      console.error("Error during movement sequence:", err);
    } finally {
      ros.close();
      console.log("Connection closed.");
    }
  });

  ros.on("error", (err) => {
    console.error("rosbridge error:", err);
  });

  ros.on("close", () => {
    console.log("rosbridge connection closed.");
  });

  process.on("SIGINT", () => {
    console.log("Caught SIGINT, shutting down.");
    ros.close();
  });
}

if (require.main === module) {
  main();
}
