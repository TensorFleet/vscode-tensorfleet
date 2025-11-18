#!/usr/bin/env python3
"""
Obstacle avoidance using LiDAR and roslibpy with adaptive scoring.

Behavior:
  - Moves forward by default with a speed scaled by clearance
  - Picks the best maneuver when blocked: TURN_LEFT, TURN_RIGHT, or BACK_UP
  - Resets to FORWARD when the path ahead is clear
"""

import argparse
import os
import sys
import time

try:
    import roslibpy
except ImportError:
    print("ERROR: The 'roslibpy' package is required.", file=sys.stderr)
    print("Install it with: pip install roslibpy", file=sys.stderr)
    sys.exit(1)


class ObstacleAvoider:
    """
    Obstacle avoidance logic adapted from the Node.js template.
    """

    def __init__(self, client, cmd_vel_topic, scan_topic, obstacle_distance, clear_distance, linear_speed, angular_speed):
        self.client = client
        self.obstacle_distance = obstacle_distance
        self.clear_distance = clear_distance
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed

        self.cmd_vel_pub = roslibpy.Topic(client, cmd_vel_topic, "geometry_msgs/Twist")
        self.scan_sub = roslibpy.Topic(client, scan_topic, "sensor_msgs/LaserScan")
        self.scan_sub.subscribe(self.on_scan)

        self.latest_scan = None
        self.running = False
        self.state = "FORWARD"  # FORWARD, TURNING_LEFT, TURNING_RIGHT, BACKING_UP
        self.turn_preference = "LEFT"
        self.maneuver_ticks = 0
        self.max_maneuver_ticks = 60  # ~3s at 50ms loop

        print("Obstacle avoider initialized:")
        print(f"  - Obstacle distance threshold: {obstacle_distance} m")
        print(f"  - Clear distance required: {clear_distance} m")
        print(f"  - Linear speed: {linear_speed} m/s")
        print(f"  - Angular speed: {angular_speed} rad/s")

    def on_scan(self, message):
        self.latest_scan = message

    def _ranges_to_list(self, ranges):
        if isinstance(ranges, dict):
            keys = sorted(ranges.keys(), key=lambda k: int(k))
            return [ranges[k] for k in keys]
        return list(ranges)

    def _get_arc_stats(self, ranges, center_idx, arc_degrees):
        total_points = len(ranges)
        scan = self.latest_scan or {}
        angle_increment = scan.get("angle_increment")
        if angle_increment is None or angle_increment <= 0:
            return {"min": float("inf"), "avg": float("inf")}

        arc_rad = (arc_degrees / 2.0) * (3.14159265 / 180.0)
        points_half_arc = max(1, int(arc_rad / angle_increment))

        min_val = float("inf")
        sum_vals = 0.0
        count = 0

        for offset in range(-points_half_arc, points_half_arc + 1):
            idx = (center_idx + offset) % total_points
            value = ranges[idx]
            if isinstance(value, (int, float)) and value > 0:
                count += 1
                sum_vals += value
                if value < min_val:
                    min_val = value

        avg_val = sum_vals / count if count > 0 else float("inf")
        return {"min": min_val, "avg": avg_val}

    def _compute_forward_speed(self, front_distance):
        if not isinstance(front_distance, (int, float)):
            return 0.0
        if front_distance <= self.obstacle_distance:
            return 0.0
        if front_distance >= self.clear_distance:
            return self.linear_speed

        ratio = (front_distance - self.obstacle_distance) / (self.clear_distance - self.obstacle_distance)
        clamped = max(0.15, min(1.0, ratio))
        return self.linear_speed * clamped

    def _score_direction(self, primary_stats, secondary_stats, label):
        min_score = primary_stats["min"] if isinstance(primary_stats["min"], (int, float)) else 0.0
        avg_score = primary_stats["avg"] if isinstance(primary_stats["avg"], (int, float)) else 0.0
        far_avg = secondary_stats["avg"] if secondary_stats and isinstance(secondary_stats.get("avg"), (int, float)) else avg_score
        base = min_score * 0.65 + avg_score * 0.25 + far_avg * 0.1
        preference_boost = self.obstacle_distance * 0.1 if label == self.turn_preference else 0.0
        return base + preference_boost

    def analyze_surroundings(self):
        if not self.latest_scan:
            return {"action": "FORWARD", "front_dist": float("inf"), "linear": self.linear_speed, "angular": self.angular_speed, "reason": "waiting for scan"}

        scan = self.latest_scan
        range_min = scan.get("range_min", 0.05) or 0.05
        range_max = scan.get("range_max", 10.0) or 10.0

        ranges_raw = scan.get("ranges", [])
        ranges = self._ranges_to_list(ranges_raw)

        sanitized = []
        for val in ranges:
            if isinstance(val, (int, float)) and val == val:
                if val < range_min:
                    sanitized.append(range_min)
                elif val > range_max:
                    sanitized.append(range_max)
                else:
                    sanitized.append(val)
            else:
                sanitized.append(range_max)

        total_points = len(sanitized)
        if total_points == 0:
            return {"action": "FORWARD", "front_dist": float("inf"), "linear": self.linear_speed, "angular": self.angular_speed, "reason": "empty scan"}

        quarter = max(1, total_points // 4)
        eighth = max(1, total_points // 8)

        sectors = {
            "front": self._get_arc_stats(sanitized, 0, 70),
            "front_left": self._get_arc_stats(sanitized, eighth, 70),
            "front_right": self._get_arc_stats(sanitized, total_points - eighth, 70),
            "left": self._get_arc_stats(sanitized, quarter, 70),
            "right": self._get_arc_stats(sanitized, quarter * 3, 70),
            "back": self._get_arc_stats(sanitized, quarter * 2, 80),
        }

        front_dist = sectors["front"]["min"]
        forward_speed = self._compute_forward_speed(front_dist)

        if front_dist >= self.clear_distance:
            return {"action": "FORWARD", "front_dist": front_dist, "linear": forward_speed, "angular": 0.0, "reason": "path wide open"}

        if front_dist > self.obstacle_distance:
            return {"action": "FORWARD", "front_dist": front_dist, "linear": forward_speed, "angular": 0.0, "reason": "cautious advance"}

        left_score = self._score_direction(sectors["front_left"], sectors["left"], "LEFT")
        right_score = self._score_direction(sectors["front_right"], sectors["right"], "RIGHT")
        back_score = self._score_direction(sectors["back"], sectors["back"], "BACK")

        min_front_left = sectors["front_left"]["min"]
        min_front_right = sectors["front_right"]["min"]

        if front_dist < self.obstacle_distance * 0.5 and back_score > 0:
            return {"action": "BACK_UP", "front_dist": front_dist, "linear": -self.linear_speed * 0.75, "angular": 0.0, "reason": "escape: too close"}

        action = "TURN_LEFT"
        angular = self.angular_speed
        winning_score = left_score
        reason = f"opening left ({left_score:.2f} vs {right_score:.2f})"

        if right_score > winning_score:
            action = "TURN_RIGHT"
            angular = -self.angular_speed
            winning_score = right_score
            reason = f"opening right ({right_score:.2f} vs {left_score:.2f})"

        both_tight = min_front_left < self.obstacle_distance * 0.8 and min_front_right < self.obstacle_distance * 0.8
        if both_tight and back_score > winning_score * 0.9:
            return {"action": "BACK_UP", "front_dist": front_dist, "linear": -self.linear_speed * 0.65, "angular": 0.0, "reason": "escape: boxed in"}

        return {"action": action, "front_dist": front_dist, "linear": 0.0, "angular": angular, "reason": reason}

    def publish_velocity(self, linear_x, angular_z):
        msg = roslibpy.Message({"linear": {"x": linear_x, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": angular_z}})
        self.cmd_vel_pub.publish(msg)

    def loop(self):
        if not self.running or not self.client.is_connected:
            return

        if not self.latest_scan:
            print("Waiting for LiDAR data...")
            self.publish_velocity(0.0, 0.0)
            return

        decision = self.analyze_surroundings()
        action = decision["action"]
        front_dist = decision["front_dist"]
        linear = decision["linear"]
        angular = decision["angular"]
        reason = decision["reason"]

        normalized_state = self.state.replace("TURNING_", "TURN_").replace("BACKING_", "BACK_")

        if action == normalized_state:
            self.maneuver_ticks += 1
        else:
            self.maneuver_ticks = 0

        if self.state != "FORWARD" and self.maneuver_ticks > self.max_maneuver_ticks:
            decision = {"action": "BACK_UP", "front_dist": front_dist, "linear": -self.linear_speed * 0.7, "angular": 0.0, "reason": "stuck override"}
            self.turn_preference = "RIGHT" if self.turn_preference == "LEFT" else "LEFT"
            self.maneuver_ticks = 0
            action = decision["action"]
            front_dist = decision["front_dist"]
            linear = decision["linear"]
            angular = decision["angular"]
            reason = decision["reason"]

        dist_label = f"{front_dist:.2f}" if isinstance(front_dist, (int, float)) and front_dist == front_dist else "inf"

        if action == "FORWARD":
            if self.state != "FORWARD":
                print(f"Path clear at {dist_label} m. Rolling forward ({reason}).")
            self.state = "FORWARD"
            self.publish_velocity(linear, 0.0)
            return

        if action == "TURN_LEFT":
            if self.state != "TURNING_LEFT":
                print(f"Obstacle at {dist_label} m. Turning LEFT ({reason}).")
            self.state = "TURNING_LEFT"
            self.turn_preference = "LEFT"
            self.publish_velocity(0.0, angular)
            return

        if action == "TURN_RIGHT":
            if self.state != "TURNING_RIGHT":
                print(f"Obstacle at {dist_label} m. Turning RIGHT ({reason}).")
            self.state = "TURNING_RIGHT"
            self.turn_preference = "RIGHT"
            self.publish_velocity(0.0, angular)
            return

        if action == "BACK_UP":
            if self.state != "BACKING_UP":
                print(f"Boxed in at {dist_label} m. Backing up ({reason}).")
            self.state = "BACKING_UP"
            self.publish_velocity(linear, 0.0)

    def run(self):
        if self.running:
            return
        self.running = True

        print()
        print("Starting obstacle avoidance...")
        print("Robot will move forward and avoid obstacles.")
        print("Press Ctrl+C to stop.")
        print()

        try:
            while self.running and self.client.is_connected:
                self.loop()
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\nStopping robot...")
        finally:
            self.stop()

    def stop(self):
        self.running = False
        self.publish_velocity(0.0, 0.0)
        time.sleep(0.1)
        self.scan_sub.unsubscribe()
        self.cmd_vel_pub.unadvertise()
        print("Robot stopped.")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Obstacle avoidance using LiDAR"
    )
    parser.add_argument("--host", default=os.getenv("ROS_HOST", "172.16.0.10"), help="ROS bridge host")
    parser.add_argument("--port", type=int, default=int(os.getenv("ROS_PORT", "9091")), help="ROS bridge port")
    parser.add_argument("--cmd-vel-topic", default=os.getenv("CMD_VEL_TOPIC", "/cmd_vel_raw"), help="Velocity command topic")
    parser.add_argument("--scan-topic", default=os.getenv("SCAN_TOPIC", "/scan"), help="LiDAR scan topic")
    parser.add_argument("--obstacle-distance", type=float, default=float(os.getenv("OBSTACLE_DISTANCE", "0.5")), help="Distance to trigger avoidance (m)")
    parser.add_argument("--clear-distance", type=float, default=float(os.getenv("CLEAR_DISTANCE", "1.0")), help="Distance needed to resume forward (m)")
    parser.add_argument("--linear-speed", type=float, default=float(os.getenv("LINEAR_SPEED", "3.0")), help="Forward speed in m/s")
    parser.add_argument("--angular-speed", type=float, default=float(os.getenv("ANGULAR_SPEED", "4.0")), help="Turning speed in rad/s")
    return parser.parse_args()


def main():
    args = parse_args()

    print(f"Connecting to rosbridge at {args.host}:{args.port}...")
    client = roslibpy.Ros(host=args.host, port=args.port)

    try:
        client.run()
        print("Connected to rosbridge\n")
    except Exception as e:
        print(f"ERROR: Failed to connect: {e}", file=sys.stderr)
        sys.exit(1)
    
    try:
        avoider = ObstacleAvoider(
            client=client,
            cmd_vel_topic=args.cmd_vel_topic,
            scan_topic=args.scan_topic,
            obstacle_distance=args.obstacle_distance,
            clear_distance=args.clear_distance,
            linear_speed=args.linear_speed,
            angular_speed=args.angular_speed
        )
        avoider.run()
    finally:
        client.terminate()
        print("Connection closed.")


if __name__ == "__main__":
    main()
