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
    this.turnPreference = "LEFT"; // Bias turn direction to avoid oscillation
    this.maneuverTicks = 0; // How long we've been in the same non-forward state
    this.maxManeuverTicks = 60; // ~3s at 50ms loop

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

  getArcStats(ranges, centerIdx, arcDegrees) {
    const totalPoints = ranges.length;
    const scan = this.latestScan;
    if (!scan || typeof scan.angle_increment !== "number") {
      return { min: Infinity, avg: Infinity };
    }

    const angleIncrement = scan.angle_increment;
    const arcRad = (arcDegrees / 2) * (Math.PI / 180);
    const pointsHalfArc = Math.max(1, Math.floor(arcRad / angleIncrement));

    let min = Infinity;
    let sum = 0;
    let count = 0;

    for (let offset = -pointsHalfArc; offset <= pointsHalfArc; offset += 1) {
      const idx = ((centerIdx + offset) % totalPoints + totalPoints) % totalPoints;
      const value = ranges[idx];
      if (typeof value === "number" && value > 0) {
        count += 1;
        sum += value;
        if (value < min) min = value;
      }
    }

    return {
      min,
      avg: count > 0 ? sum / count : Infinity
    };
  }

  computeForwardSpeed(frontDistance) {
    if (!Number.isFinite(frontDistance)) {
      return 0;
    }
    if (frontDistance <= this.obstacleDistance) {
      return 0;
    }
    if (frontDistance >= this.clearDistance) {
      return this.linearSpeed;
    }
    const ratio =
      (frontDistance - this.obstacleDistance) /
      (this.clearDistance - this.obstacleDistance);
    const clamped = Math.max(0.15, Math.min(1, ratio));
    return this.linearSpeed * clamped;
  }

  scoreDirection(primaryStats, secondaryStats, label) {
    const minScore = Number.isFinite(primaryStats.min) ? primaryStats.min : 0;
    const avgScore = Number.isFinite(primaryStats.avg) ? primaryStats.avg : 0;
    const farAvg = secondaryStats && Number.isFinite(secondaryStats.avg)
      ? secondaryStats.avg
      : avgScore;
    const base = minScore * 0.65 + avgScore * 0.25 + farAvg * 0.1;

    const preferenceBoost =
      label === this.turnPreference ? this.obstacleDistance * 0.1 : 0;
    return base + preferenceBoost;
  }

  analyzeSurroundings() {
    if (!this.latestScan) {
      return {
        action: "FORWARD",
        frontDist: Infinity,
        linear: this.linearSpeed,
        angular: this.angularSpeed,
        reason: "waiting for scan"
      };
    }

    const scan = this.latestScan;
    const rangeMin =
      typeof scan.range_min === "number" && scan.range_min > 0
        ? scan.range_min
        : 0.05;
    const rangeMax =
      typeof scan.range_max === "number" && scan.range_max > 0
        ? scan.range_max
        : 10.0;

    let ranges = scan.ranges;
    if (!Array.isArray(ranges)) {
      const keys = Object.keys(ranges).sort(
        (a, b) => Number(a) - Number(b)
      );
      ranges = keys.map((k) => ranges[k]);
    }

    ranges = ranges.map((val) => {
      if (typeof val === "number" && Number.isFinite(val)) {
        if (val < rangeMin) return rangeMin;
        if (val > rangeMax) return rangeMax;
        return val;
      }
      // Treat null/invalid as no-return; cap at rangeMax so we don't over-score.
      return rangeMax;
    });

    const totalPoints = ranges.length;
    if (totalPoints === 0) {
      return {
        action: "FORWARD",
        frontDist: Infinity,
        linear: this.linearSpeed,
        angular: this.angularSpeed,
        reason: "empty scan"
      };
    }

    const quarter = Math.max(1, Math.floor(totalPoints / 4));
    const eighth = Math.max(1, Math.floor(totalPoints / 8));

    const sectors = {
      front: this.getArcStats(ranges, 0, 70),
      frontLeft: this.getArcStats(ranges, eighth, 70),
      frontRight: this.getArcStats(ranges, totalPoints - eighth, 70),
      left: this.getArcStats(ranges, quarter, 70),
      right: this.getArcStats(ranges, quarter * 3, 70),
      back: this.getArcStats(ranges, quarter * 2, 80)
    };

    const frontDist = sectors.front.min;
    const forwardSpeed = this.computeForwardSpeed(frontDist);

    // If we're clear, bias to crisp forward motion.
    if (frontDist >= this.clearDistance) {
      return {
        action: "FORWARD",
        frontDist,
        linear: forwardSpeed,
        angular: 0.0,
        reason: "path wide open"
      };
    }

    // If we are close but still above obstacleDistance, creep forward.
    if (frontDist > this.obstacleDistance) {
      return {
        action: "FORWARD",
        frontDist,
        linear: forwardSpeed,
        angular: 0.0,
        reason: "cautious advance"
      };
    }

    // Too close: pick the best escape vector.
    const leftScore = this.scoreDirection(sectors.frontLeft, sectors.left, "LEFT");
    const rightScore = this.scoreDirection(
      sectors.frontRight,
      sectors.right,
      "RIGHT"
    );
    const backScore = this.scoreDirection(sectors.back, sectors.back, "BACK");

    const minFrontLeft = sectors.frontLeft.min;
    const minFrontRight = sectors.frontRight.min;

    // Back out aggressively if we're pinched very close.
    if (frontDist < this.obstacleDistance * 0.5 && backScore > 0) {
      return {
        action: "BACK_UP",
        frontDist,
        linear: -this.linearSpeed * 0.75,
        angular: 0.0,
        reason: "escape: too close"
      };
    }

    let action = "TURN_LEFT";
    let angular = this.angularSpeed;
    let winningScore = leftScore;
    let reason = `opening left (${leftScore.toFixed(2)} vs ${rightScore.toFixed(2)})`;

    if (rightScore > winningScore) {
      action = "TURN_RIGHT";
      angular = -this.angularSpeed;
      winningScore = rightScore;
      reason = `opening right (${rightScore.toFixed(2)} vs ${leftScore.toFixed(2)})`;
    }

    // If both sides are cramped, a short reverse can clear space.
    const bothTight =
      minFrontLeft < this.obstacleDistance * 0.8 &&
      minFrontRight < this.obstacleDistance * 0.8;
    if (bothTight && backScore > winningScore * 0.9) {
      return {
        action: "BACK_UP",
        frontDist,
        linear: -this.linearSpeed * 0.65,
        angular: 0.0,
        reason: "escape: boxed in"
      };
    }

    return {
      action,
      frontDist,
      linear: 0.0,
      angular,
      reason
    };
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

    let decision = this.analyzeSurroundings();
    let { action, frontDist, linear, angular, reason } = decision;

    const normalizedState = this.state
      .replace("TURNING_", "TURN_")
      .replace("BACKING_", "BACK_");

    if (action === normalizedState) {
      this.maneuverTicks += 1;
    } else {
      this.maneuverTicks = 0;
    }

    // If we've been stuck in a maneuver for too long, reset with a back-up move.
    if (
      this.state !== "FORWARD" &&
      this.maneuverTicks > this.maxManeuverTicks
    ) {
      decision = {
        action: "BACK_UP",
        frontDist,
        linear: -this.linearSpeed * 0.7,
        angular: 0.0,
        reason: "stuck override"
      };
      this.turnPreference = this.turnPreference === "LEFT" ? "RIGHT" : "LEFT";
      this.maneuverTicks = 0;
      ({ action, frontDist, linear, angular, reason } = decision);
    }

    const distLabel = Number.isFinite(frontDist) ? frontDist.toFixed(2) : "∞";

    if (action === "FORWARD") {
      if (this.state !== "FORWARD") {
        console.log(
          `Path clear at ${distLabel} m. Rolling forward (${reason}).`
        );
      }
      this.state = "FORWARD";
      this.publishVelocity(linear, 0.0);
      return;
    }

    if (action === "TURN_LEFT") {
      if (this.state !== "TURNING_LEFT") {
        console.log(
          `Obstacle at ${distLabel} m. Turning LEFT (${reason}).`
        );
      }
      this.state = "TURNING_LEFT";
      this.turnPreference = "LEFT";
      this.publishVelocity(0.0, angular);
      return;
    }

    if (action === "TURN_RIGHT") {
      if (this.state !== "TURNING_RIGHT") {
        console.log(
          `Obstacle at ${distLabel} m. Turning RIGHT (${reason}).`
        );
      }
      this.state = "TURNING_RIGHT";
      this.turnPreference = "RIGHT";
      this.publishVelocity(0.0, angular);
      return;
    }

    if (action === "BACK_UP") {
      if (this.state !== "BACKING_UP") {
        console.log(
          `Boxed in at ${distLabel} m. Backing up (${reason}).`
        );
      }
      this.state = "BACKING_UP";
      this.publishVelocity(linear, 0.0);
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
