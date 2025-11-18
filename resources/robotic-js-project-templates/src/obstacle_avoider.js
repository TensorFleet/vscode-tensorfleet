#!/usr/bin/env node
/**
 * Obstacle avoidance using LiDAR and roslib.
 *
 * Behavior:
 *   - Moves forward by default
 *   - If an obstacle is detected ahead, chooses the best escape:
 *       TURN_LEFT, TURN_RIGHT, or BACK_UP
 *   - Resumes FORWARD when the path ahead is clear
 *
 * Usage:
 *   bun src/obstacle_avoider.js
 *
 * Environment overrides:
 *   ROS_HOST, ROS_PORT, CMD_VEL_TOPIC, SCAN_TOPIC,
 *   OBSTACLE_DISTANCE, CLEAR_DISTANCE,
 *   LINEAR_SPEED, ANGULAR_SPEED
 */
const ROSLIB = require("roslib");
const {
  getRosConnectionDetails,
  getCmdVelTopic,
  getScanTopic,
  getObstacleParams
} = require("./config");

const { url: ROSBRIDGE_URL } = getRosConnectionDetails();
const CMD_VEL_TOPIC = getCmdVelTopic();
const SCAN_TOPIC = getScanTopic();
const {
  obstacleDistance: OBSTACLE_DISTANCE,
  clearDistance: CLEAR_DISTANCE,
  linearSpeed: LINEAR_SPEED,
  angularSpeed: ANGULAR_SPEED
} = getObstacleParams();

class ObstacleAvoider {
  constructor(ros) {
    this.ros = ros;
    this.obstacleDistance = OBSTACLE_DISTANCE;
    this.clearDistance = CLEAR_DISTANCE;
    this.linearSpeed = LINEAR_SPEED;
    this.angularSpeed = ANGULAR_SPEED;

    this.cmdVelPub = new ROSLIB.Topic({
      ros,
      name: CMD_VEL_TOPIC,
      messageType: "geometry_msgs/Twist"
    });

    this.scanSub = new ROSLIB.Topic({
      ros,
      name: SCAN_TOPIC,
      messageType: "sensor_msgs/LaserScan"
    });

    this.latestScan = null;
    this.running = false;
    this.state = "FORWARD"; // FORWARD, TURNING_LEFT, TURNING_RIGHT, BACKING_UP
    this.loopTimer = null;

    this.scanSub.subscribe((msg) => {
      this.latestScan = msg;
    });

    console.log("Obstacle avoider initialized:");
    console.log(`  - Obstacle distance threshold: ${this.obstacleDistance} m`);
    console.log(`  - Clear distance required: ${this.clearDistance} m`);
    console.log(`  - Linear speed: ${this.linearSpeed} m/s`);
    console.log(`  - Angular speed: ${this.angularSpeed} rad/s`);
  }

  getMinDistanceInArc(ranges, centerIdx, arcDegrees) {
    const totalPoints = ranges.length;
    const scan = this.latestScan;
    if (!scan || typeof scan.angle_increment !== "number") {
      return Infinity;
    }

    const angleIncrement = scan.angle_increment;
    const arcRad = (arcDegrees / 2) * (Math.PI / 180);
    const pointsHalfArc = Math.max(1, Math.floor(arcRad / angleIncrement));

    const distances = [];
    for (let offset = -pointsHalfArc; offset <= pointsHalfArc; offset += 1) {
      const idx = ((centerIdx + offset) % totalPoints + totalPoints) % totalPoints;
      const value = ranges[idx];
      if (typeof value === "number" && value > 0) {
        distances.push(value);
      }
    }
    return distances.length > 0 ? Math.min(...distances) : Infinity;
  }

  analyzeSurroundings() {
    if (!this.latestScan) {
      return ["FORWARD", Infinity];
    }

    let ranges = this.latestScan.ranges;
    if (!Array.isArray(ranges)) {
      const keys = Object.keys(ranges).sort(
        (a, b) => Number(a) - Number(b)
      );
      ranges = keys.map((k) => ranges[k]);
    }

    const totalPoints = ranges.length;
    if (totalPoints === 0) {
      return ["FORWARD", Infinity];
    }

    const quarter = Math.floor(totalPoints / 4);

    const front = this.getMinDistanceInArc(ranges, 0, 90);
    const left = this.getMinDistanceInArc(ranges, quarter, 90);
    const back = this.getMinDistanceInArc(ranges, quarter * 2, 90);
    const right = this.getMinDistanceInArc(ranges, quarter * 3, 90);

    if (this.state === "FORWARD") {
      if (front < this.obstacleDistance) {
        const directions = [
          ["TURN_LEFT", left],
          ["TURN_RIGHT", right],
          ["BACK_UP", back]
        ];
        const [bestAction] = directions.reduce(
          (best, current) => (current[1] > best[1] ? current : best),
          directions[0]
        );
        return [bestAction, front];
      }
      return ["FORWARD", front];
    }

    if (front > this.clearDistance) {
      return ["FORWARD", front];
    }
    // Keep current maneuver
    const normalized = this.state
      .replace("TURNING_", "TURN_")
      .replace("BACKING_", "BACK_");
    return [normalized, front];
  }

  publishVelocity(linearX, angularZ) {
    const msg = new ROSLIB.Message({
      linear: { x: linearX, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: angularZ }
    });
    this.cmdVelPub.publish(msg);
  }

  loop() {
    if (!this.running || !this.ros.isConnected) {
      return;
    }

    if (!this.latestScan) {
      console.log("Waiting for LiDAR data...");
      this.publishVelocity(0.0, 0.0);
      return;
    }

    const [action, frontDist] = this.analyzeSurroundings();

    if (action === "FORWARD") {
      if (this.state !== "FORWARD") {
        console.log(
          `Path clear at ${frontDist.toFixed(2)} m. Resuming forward.`
        );
      }
      this.state = "FORWARD";
      this.publishVelocity(this.linearSpeed, 0.0);
      return;
    }

    if (action === "TURN_LEFT") {
      if (this.state !== "TURNING_LEFT") {
        console.log(
          `Obstacle at ${frontDist.toFixed(
            2
          )} m ahead. Turning LEFT to avoid.`
        );
      }
      this.state = "TURNING_LEFT";
      this.publishVelocity(0.0, this.angularSpeed);
      return;
    }

    if (action === "TURN_RIGHT") {
      if (this.state !== "TURNING_RIGHT") {
        console.log(
          `Obstacle at ${frontDist.toFixed(
            2
          )} m ahead. Turning RIGHT to avoid.`
        );
      }
      this.state = "TURNING_RIGHT";
      this.publishVelocity(0.0, -this.angularSpeed);
      return;
    }

    if (action === "BACK_UP") {
      if (this.state !== "BACKING_UP") {
        console.log(
          `Boxed in at ${frontDist.toFixed(
            2
          )} m ahead. Backing up to clear space.`
        );
      }
      this.state = "BACKING_UP";
      this.publishVelocity(-this.linearSpeed * 0.7, 0.0);
    }
  }

  start() {
    if (this.running) return;
    this.running = true;

    console.log();
    console.log("Starting obstacle avoidance...");
    console.log("Robot will move forward and avoid obstacles.");
    console.log("Press Ctrl+C to stop.");
    console.log();

    this.loopTimer = setInterval(() => this.loop(), 50);
  }

  stop() {
    if (!this.running) return;
    this.running = false;

    if (this.loopTimer) {
      clearInterval(this.loopTimer);
      this.loopTimer = null;
    }

    this.publishVelocity(0.0, 0.0);
    this.scanSub.unsubscribe();
    this.cmdVelPub.unadvertise();
    console.log("Robot stopped.");
  }
}

function main() {
  console.log(`Connecting to rosbridge at ${ROSBRIDGE_URL} ...`);

  const ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });

  ros.on("connection", () => {
    console.log("Connected to rosbridge.");
    const avoider = new ObstacleAvoider(ros);
    avoider.start();

    const shutdown = () => {
      console.log("Shutting down obstacle avoider.");
      avoider.stop();
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
