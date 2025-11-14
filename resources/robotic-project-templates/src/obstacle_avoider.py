#!/usr/bin/env python3
"""
Obstacle avoidance using LiDAR and roslibpy.

The robot:
- Moves forward by default
- If obstacle detected, chooses best escape direction: left, right, or backward
- Resumes forward movement when path is clear

Dependencies:
  pip install roslibpy
"""

import argparse
import sys
import time

try:
    import roslibpy
except ImportError:
    print("ERROR: The 'roslibpy' package is required.", file=sys.stderr)
    print("Install it with: pip install roslibpy", file=sys.stderr)
    sys.exit(1)


class ObstacleAvoider:
    def __init__(self, client, cmd_vel_topic, scan_topic, obstacle_distance, clear_distance, linear_speed, angular_speed):
        self.client = client
        self.obstacle_distance = obstacle_distance
        self.clear_distance = clear_distance
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        
        # Publisher for movement commands
        self.cmd_vel_pub = roslibpy.Topic(client, cmd_vel_topic, "geometry_msgs/Twist")
        
        # Subscriber for LiDAR data
        self.scan_sub = roslibpy.Topic(client, scan_topic, "sensor_msgs/LaserScan")
        self.scan_sub.subscribe(self.on_scan)
        
        self.latest_scan = None
        self.running = True
        self.state = "FORWARD"  # States: FORWARD, TURNING_LEFT, TURNING_RIGHT, BACKING_UP
        
        print(f"Obstacle avoider initialized:")
        print(f"  - Obstacle distance threshold: {obstacle_distance}m")
        print(f"  - Clear distance required: {clear_distance}m")
        print(f"  - Linear speed: {linear_speed} m/s")
        print(f"  - Angular speed: {angular_speed} rad/s")
    
    def on_scan(self, message):
        """Callback for LiDAR scan messages"""
        self.latest_scan = message
    
    def get_min_distance_in_arc(self, ranges, center_idx, arc_degrees):
        """
        Get minimum distance in an arc around center_idx.
        arc_degrees: total arc width in degrees
        """
        total_points = len(ranges)
        angle_increment = self.latest_scan['angle_increment']
        arc_rad = (arc_degrees / 2) * 3.14159 / 180
        points_half_arc = int(arc_rad / angle_increment)
        
        distances = []
        for offset in range(-points_half_arc, points_half_arc + 1):
            idx = (center_idx + offset) % total_points
            if ranges[idx] is not None and ranges[idx] > 0:
                distances.append(ranges[idx])
        
        return min(distances) if distances else float('inf')
    
    def analyze_surroundings(self):
        """
        Analyze all directions and choose best escape route.
        Returns: (action, min_front_distance)
        action: "FORWARD", "TURN_LEFT", "TURN_RIGHT", "BACK_UP"
        """
        if not self.latest_scan:
            return "FORWARD", float('inf')
        
        ranges = self.latest_scan['ranges']
        
        # Handle both dict and list formats
        if isinstance(ranges, dict):
            ranges = [ranges.get(str(i)) for i in range(len(ranges))]
        
        total_points = len(ranges)
        quarter = total_points // 4
        
        # Check all four directions
        front = self.get_min_distance_in_arc(ranges, 0, 90)           # 0° ± 45°
        left = self.get_min_distance_in_arc(ranges, quarter, 90)      # 90° ± 45°
        back = self.get_min_distance_in_arc(ranges, quarter * 2, 90)  # 180° ± 45°
        right = self.get_min_distance_in_arc(ranges, quarter * 3, 90) # 270° ± 45°
        
        # Decide action based on current state
        if self.state == "FORWARD":
            # Moving forward, check if we need to avoid
            if front < self.obstacle_distance:
                # Obstacle ahead! Choose best escape
                directions = [
                    ("TURN_LEFT", left),
                    ("TURN_RIGHT", right),
                    ("BACK_UP", back)
                ]
                # Pick direction with most space
                best_action, best_distance = max(directions, key=lambda x: x[1])
                return best_action, front
            else:
                return "FORWARD", front
        
        else:  # Currently avoiding (turning or backing up)
            # Check if path ahead is clear enough to resume
            if front > self.clear_distance:
                return "FORWARD", front
            else:
                # Keep avoiding, stick with current maneuver
                return self.state.replace("TURNING_", "TURN_").replace("BACKING_", "BACK_"), front
    
    def publish_velocity(self, linear_x, angular_z):
        """Publish velocity command"""
        msg = roslibpy.Message({
            "linear": {"x": linear_x, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_z}
        })
        self.cmd_vel_pub.publish(msg)
    
    def run(self):
        """Main control loop"""
        print("\nStarting obstacle avoidance...")
        print("Robot will move forward and avoid obstacles.")
        print("Press Ctrl+C to stop.\n")
        
        try:
            while self.running and self.client.is_connected:
                if self.latest_scan is None:
                    print("Waiting for LiDAR data...")
                    time.sleep(0.5)
                    continue
                
                action, front_dist = self.analyze_surroundings()
                
                if action == "FORWARD":
                    if self.state != "FORWARD":
                        print(f"\n✓ Path clear at {front_dist:.2f}m! Resuming forward...")
                    self.state = "FORWARD"
                    print(f"✓ Moving forward (clear: {front_dist:.2f}m)    ", end='\r')
                    self.publish_velocity(self.linear_speed, 0.0)
                
                elif action == "TURN_LEFT":
                    if self.state != "TURNING_LEFT":
                        print(f"\n⚠️ Obstacle at {front_dist:.2f}m! Turning LEFT...")
                    self.state = "TURNING_LEFT"
                    print(f"🔄 Turning LEFT... (obstacle: {front_dist:.2f}m)    ", end='\r')
                    self.publish_velocity(0.0, self.angular_speed)
                
                elif action == "TURN_RIGHT":
                    if self.state != "TURNING_RIGHT":
                        print(f"\n⚠️ Obstacle at {front_dist:.2f}m! Turning RIGHT...")
                    self.state = "TURNING_RIGHT"
                    print(f"🔄 Turning RIGHT... (obstacle: {front_dist:.2f}m)    ", end='\r')
                    self.publish_velocity(0.0, -self.angular_speed)
                
                elif action == "BACK_UP":
                    if self.state != "BACKING_UP":
                        print(f"\n🚨 Boxed in at {front_dist:.2f}m! BACKING UP...")
                    self.state = "BACKING_UP"
                    print(f"⏪ Backing up... (obstacle: {front_dist:.2f}m)    ", end='\r')
                    self.publish_velocity(-self.linear_speed * 0.7, 0.0)
                
                time.sleep(0.05)  # 20Hz control loop
                
        except KeyboardInterrupt:
            print("\n\nStopping robot...")
        finally:
            self.stop()
    
    def stop(self):
        """Stop the robot and clean up"""
        self.running = False
        self.publish_velocity(0.0, 0.0)
        time.sleep(0.5)
        self.scan_sub.unsubscribe()
        self.cmd_vel_pub.unadvertise()
        print("Robot stopped.")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Obstacle avoidance using LiDAR"
    )
    parser.add_argument("--host", default="172.16.0.10", help="ROS bridge host")
    parser.add_argument("--port", type=int, default=9091, help="ROS bridge port")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel_raw", help="Velocity command topic")
    parser.add_argument("--scan-topic", default="/scan", help="LiDAR scan topic")
    parser.add_argument("--obstacle-distance", type=float, default=0.5, 
                       help="Distance to trigger avoidance (default: 0.5m)")
    parser.add_argument("--clear-distance", type=float, default=1.0,
                       help="Distance needed to resume forward (default: 1.0m)")
    parser.add_argument("--linear-speed", type=float, default=3.0,
                       help="Forward speed in m/s (default: 3.0)")
    parser.add_argument("--angular-speed", type=float, default=4.0,
                       help="Turning speed in rad/s (default: 4.0)")
    return parser.parse_args()


def main():
    args = parse_args()
    
    print(f"Connecting to rosbridge at {args.host}:{args.port}...")
    client = roslibpy.Ros(host=args.host, port=args.port)
    
    try:
        client.run()
        print("✓ Connected to rosbridge\n")
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