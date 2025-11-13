#!/usr/bin/env python3
"""
TensorFleet Robotic Movement Script

Basic example that publishes to cmd_vel_raw to move a robot.
"""

import math
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


@dataclass
class MotionConfig:
  cmd_vel_topic: str = "/cmd_vel_raw"
  linear_speed: float = 0.2
  angular_speed: float = 0.5


class RobotMover(Node):
  """Simple robot controller that publishes Twist on cmd_vel_raw."""

  def __init__(self, config: Optional[MotionConfig] = None) -> None:
    super().__init__("tensorfleet_robot_mover")

    self._config = config or MotionConfig()

    self._pub = self.create_publisher(
      Twist,
      self._config.cmd_vel_topic,
      10,
    )

    self.get_logger().info(
      f"RobotMover initialized. Publishing Twist on {self._config.cmd_vel_topic}"
    )

    # Timer drives the motion sequence
    self._sequence_started = False
    self._timer = self.create_timer(0.1, self._control_loop)

  def _control_loop(self) -> None:
    if self._sequence_started:
      return

    self._sequence_started = True
    self.get_logger().info("Starting basic movement sequence...")

    # Move forward
    self._move(linear=self._config.linear_speed, angular=0.0, duration=3.0)

    # Rotate in place
    self._move(linear=0.0, angular=self._config.angular_speed, duration=2.0)

    # Stop
    self._move(linear=0.0, angular=0.0, duration=1.0)

    self.get_logger().info("Movement sequence complete. Shutting down node.")
    rclpy.shutdown()

  def _move(self, linear: float, angular: float, duration: float) -> None:
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular

    end_time = self.get_clock().now().nanoseconds / 1e9 + duration

    while rclpy.ok() and (self.get_clock().now().nanoseconds / 1e9) < end_time:
      self._pub.publish(twist)
      self.get_logger().debug(
        f"Publishing cmd_vel_raw: linear={linear:.3f} m/s, angular={angular:.3f} rad/s"
      )
      time.sleep(0.05)


def main(args=None) -> None:
  rclpy.init(args=args)

  node = RobotMover()

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == "__main__":
  main()

