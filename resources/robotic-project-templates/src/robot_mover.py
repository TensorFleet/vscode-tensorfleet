#!/usr/bin/env python3
"""
Simple movement example using roslibpy and rosbridge.

This script:
  - Connects to the rosbridge WebSocket server (via proxy or direct)
  - Publishes geometry_msgs/Twist-style messages on /cmd_vel_raw
  - Runs a short movement sequence (forward, backward, left, right, stop)

Usage:
  python src/robot_mover.py

Environment overrides:
  - TENSORFLEET_BASE_URL, TENSORFLEET_JWT (for proxy connection)
  - ROS_HOST, ROS_PORT, ROSBRIDGE_URL (for direct connection)
  - CMD_VEL_TOPIC, LINEAR_SPEED, ANGULAR_SPEED
"""

import os
import sys
import time

# Add parent directory to path for lib imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import roslibpy
except ImportError:
    print("ERROR: The 'roslibpy' package is required.", file=sys.stderr)
    print("Install it with: pip install roslibpy", file=sys.stderr)
    sys.exit(1)

try:
    from lib.robotic_utils import connect_to_robot, Topic
except ImportError:
    try:
        from robotic_utils import connect_to_robot, Topic
    except ImportError:
        connect_to_robot = None
        Topic = None


# Configuration from environment
CMD_VEL_TOPIC = os.getenv("CMD_VEL_TOPIC", "/cmd_vel_raw")
LINEAR_SPEED = float(os.getenv("LINEAR_SPEED", "0.2"))
ANGULAR_SPEED = float(os.getenv("ANGULAR_SPEED", "0.5"))


def make_twist(linear_x: float, angular_z: float) -> dict:
    """Build a geometry_msgs/Twist-style message as expected by rosbridge."""
    return {
        "linear": {"x": linear_x, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": angular_z},
    }


def publish_for(
    client,
    topic,
    duration: float,
    linear_x: float,
    angular_z: float,
    label: str,
) -> None:
    """Publish velocity messages for a specified duration."""
    print(f"Phase: {label} (duration {duration}s, lin={linear_x}, ang={angular_z})")
    end_time = time.time() + duration
    msg = make_twist(linear_x, angular_z)

    while client.is_connected and time.time() < end_time:
        topic.publish(msg)
        time.sleep(0.05)


def run_movement(client) -> None:
    """Execute the movement sequence."""
    print(f"Advertising Twist publisher on '{CMD_VEL_TOPIC}' ...")
    
    # Use Topic factory for compatibility with both roslibpy.Ros and ProxyRosClient
    if Topic:
        cmd_vel = Topic(client, CMD_VEL_TOPIC, "geometry_msgs/Twist")
    else:
        cmd_vel = roslibpy.Topic(client, CMD_VEL_TOPIC, "geometry_msgs/Twist")

    forward_duration = 3.0
    backward_duration = 3.0
    turn_duration = 2.0
    stop_duration = 1.0

    try:
        # 1) Drive straight forward
        publish_for(client, cmd_vel, forward_duration, LINEAR_SPEED, 0.0, "forward")
        publish_for(client, cmd_vel, stop_duration, 0.0, 0.0, "stop after forward")

        # 2) Drive straight backward
        publish_for(client, cmd_vel, backward_duration, -LINEAR_SPEED, 0.0, "backward")
        publish_for(client, cmd_vel, stop_duration, 0.0, 0.0, "stop after backward")

        # 3) Turn left in place
        publish_for(client, cmd_vel, turn_duration, 0.0, ANGULAR_SPEED, "turn left")
        publish_for(client, cmd_vel, stop_duration, 0.0, 0.0, "stop after left turn")

        # 4) Turn right in place
        publish_for(client, cmd_vel, turn_duration, 0.0, -ANGULAR_SPEED, "turn right")
        publish_for(client, cmd_vel, stop_duration, 0.0, 0.0, "final stop")

        print("Movement sequence complete.")
    finally:
        cmd_vel.unadvertise()


def main() -> None:
    client = None
    try:
        if connect_to_robot:
            client = connect_to_robot()
        else:
            # Fallback to direct connection
            host = os.getenv("ROS_HOST", "172.16.0.10")
            port = int(os.getenv("ROS_PORT", "9091"))
            print(f"Connecting to rosbridge at {host}:{port} using roslibpy ...")
            client = roslibpy.Ros(host=host, port=port)
            client.run()

        print("roslibpy connection established successfully.")
        run_movement(client)
    except Exception as exc:
        print(f"Error during movement sequence: {exc}", file=sys.stderr)
        sys.exit(1)
    finally:
        if client:
            client.terminate()
            print("Connection closed.")


if __name__ == "__main__":
    main()
