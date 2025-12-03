#!/usr/bin/env python3
"""
Simple movement example using roslibpy and rosbridge.

This script:
  - Connects to the rosbridge WebSocket server
  - Publishes geometry_msgs/Twist-style messages on /cmd_vel_raw
  - Runs a short movement sequence (forward, rotate, stop)

Dependencies (on the host):
  pip install roslibpy
"""

import argparse
import sys
import time

try:
    import roslibpy  # type: ignore[import]
except ImportError:  # pragma: no cover - import-time check
    print("ERROR: The 'roslibpy' package is required.", file=sys.stderr)
    print("Install it on the host with:", file=sys.stderr)
    print("  pip install roslibpy", file=sys.stderr)
    sys.exit(1)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Simple robot movement script using roslibpy.",
    )
    parser.add_argument(
        "--host",
        default="172.16.0.10",
        help="ROS bridge host (default: 172.16.0.10 inside VM).",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9091,
        help="ROS bridge WebSocket port (default: 9091).",
    )
    parser.add_argument(
        "--cmd-vel-topic",
        default="/cmd_vel_raw",
        help="Velocity command topic (default: /cmd_vel_raw).",
    )
    parser.add_argument(
        "--linear-speed",
        type=float,
        default=0.2,
        help="Forward linear.x speed in m/s (default: 0.2).",
    )
    parser.add_argument(
        "--angular-speed",
        type=float,
        default=0.5,
        help="Angular.z speed in rad/s for turning (default: 0.5).",
    )
    return parser.parse_args()


def make_twist(linear_x: float, angular_z: float) -> roslibpy.Message:
    """
    Build a geometry_msgs/Twist-style message as expected by rosbridge.
    """
    msg = {
        "linear": {"x": linear_x, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": angular_z},
    }
    return roslibpy.Message(msg)


def run_movement(client: "roslibpy.Ros", cmd_vel_topic: str, linear: float, angular: float) -> None:
    print(f"Advertising Twist publisher on '{cmd_vel_topic}' ...")
    publisher = roslibpy.Topic(
        client,
        cmd_vel_topic,
        "geometry_msgs/Twist",
    )

    forward_duration = 3.0
    backward_duration = 3.0
    turn_duration = 2.0
    stop_duration = 1.0

    def _publish_for(duration: float, lin: float, ang: float, label: str) -> None:
        print(f"Starting phase: {label} (duration {duration}s, lin={lin}, ang={ang})")
        end_time = time.time() + duration
        msg = make_twist(lin, ang)
        while client.is_connected and time.time() < end_time:
            publisher.publish(msg)
            time.sleep(0.05)

    # 1) Drive straight forward
    _publish_for(forward_duration, linear, 0.0, "forward")
    _publish_for(stop_duration, 0.0, 0.0, "stop after forward")

    # 2) Drive straight backward
    _publish_for(backward_duration, -linear, 0.0, "backward")
    _publish_for(stop_duration, 0.0, 0.0, "stop after backward")

    # 3) Turn left in place
    _publish_for(turn_duration, 0.0, angular, "turn left")
    _publish_for(stop_duration, 0.0, 0.0, "stop after left turn")

    # 4) Turn right in place
    _publish_for(turn_duration, 0.0, -angular, "turn right")
    _publish_for(stop_duration, 0.0, 0.0, "final stop")

    print("Movement sequence complete.")
    publisher.unadvertise()


def main() -> None:
    args = parse_args()

    print(f"Connecting to rosbridge at {args.host}:{args.port} using roslibpy ...")
    client = roslibpy.Ros(host=args.host, port=args.port)
    try:
        client.run()
    except Exception as exc:  # pragma: no cover - network-dependent
        print(f"ERROR: Failed to connect to rosbridge: {exc}", file=sys.stderr)
        sys.exit(1)

    print("roslibpy connection established successfully.")

    try:
        run_movement(
            client,
            cmd_vel_topic=args.cmd_vel_topic,
            linear=args.linear_speed,
            angular=args.angular_speed,
        )
    finally:
        client.terminate()
        print("Connection closed.")


if __name__ == "__main__":
    main()
