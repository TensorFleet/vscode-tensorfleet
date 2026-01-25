"""
SO101 Hardware Bridge for real arm communication.

This module provides a bridge between the real SO101 robot arm (via lerobot SDK)
and ROS 2 topics for integration with the TensorFleet VM.

Usage:
    from lib.so101_bridge import SO101Bridge
    
    bridge = SO101Bridge(ros_client, robot_port='/dev/ttyACM0')
    bridge.connect()
    bridge.start_publishing(rate_hz=20)
    # ... use bridge ...
    bridge.shutdown()
"""

import math
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

try:
    from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
    from lerobot.teleoperators.so101_leader import SO101Leader, SO101LeaderConfig
    LEROBOT_AVAILABLE = True
except ImportError:
    LEROBOT_AVAILABLE = False
    SO101Follower = None
    SO101FollowerConfig = None
    SO101Leader = None
    SO101LeaderConfig = None


class SO101Bridge:
    """
    Bridge between SO101 robot hardware and ROS 2 via roslibpy.
    
    This class wraps the lerobot SDK and provides:
    - ROS-compatible joint name mapping
    - Unit conversion (radians ↔ normalized values)
    - State publishing to /joint_states_raw
    - Command receiving from /joint_commands
    """

    # ROS 2 standard joint names (matches VM ros2_control config)
    JOINT_NAMES = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint',
    ]

    # Mapping from ROS 2 names to lerobot hardware SDK names
    ROS_TO_HW_MAP = {
        'shoulder_pan_joint': 'shoulder_pan',
        'shoulder_lift_joint': 'shoulder_lift',
        'elbow_joint': 'elbow_flex',
        'wrist_1_joint': 'wrist_flex',
        'wrist_2_joint': 'wrist_roll',
        'wrist_3_joint': 'gripper',
    }

    # Reverse mapping for convenience
    HW_TO_ROS_MAP = {v: k for k, v in ROS_TO_HW_MAP.items()}

    def __init__(
        self,
        ros_client: Any,
        robot_port: str = '/dev/ttyACM0',
        robot_id: str = 'so101',
        robot_type: str = 'follower',
        calibration_dir: Optional[str] = None,
        use_degrees: bool = False,
        publish_topic: str = '/joint_states_raw',
        publish_joint_names: Optional[List[str]] = None,
        command_topic: str = '/joint_commands',
        subscribe_commands: bool = True,
    ):
        """
        Initialize the SO101 bridge.

        Args:
            ros_client: Connected ROS client (roslibpy.Ros or ProxyRosClient)
            robot_port: USB serial port for the robot
            robot_id: Robot ID for calibration lookup
            robot_type: 'follower' or 'leader'
            calibration_dir: Path to calibration files (default: ~/.config/lerobot/)
            use_degrees: Use degrees instead of normalized values for hardware
            publish_topic: Topic to publish joint states to
            publish_joint_names: Joint name list to publish (defaults to ROS names)
            command_topic: Topic to subscribe for commands (follower only)
            subscribe_commands: Whether to subscribe to command topic (follower only)
        """
        if not LEROBOT_AVAILABLE:
            raise ImportError(
                "lerobot is required for SO101Bridge. "
                "Install with: pip install lerobot pyserial feetech-servo-sdk"
            )

        self.ros_client = ros_client
        self.robot_port = robot_port
        self.robot_id = robot_id
        self.robot_type = robot_type
        self.use_degrees = use_degrees
        self.publish_topic = publish_topic
        self.publish_joint_names = publish_joint_names or list(self.JOINT_NAMES)
        self.command_topic = command_topic
        self.subscribe_commands = subscribe_commands

        if len(self.publish_joint_names) != len(self.JOINT_NAMES):
            print(
                "[SO101Bridge] WARNING: publish_joint_names length mismatch, "
                "falling back to default ROS joint names."
            )
            self.publish_joint_names = list(self.JOINT_NAMES)

        # Set default calibration directory
        if calibration_dir is None:
            self.calibration_dir = Path.home() / '.config/lerobot/'
        else:
            self.calibration_dir = Path(calibration_dir)

        # Will be initialized in connect()
        self.robot = None
        self.joint_state_pub = None
        self.joint_cmd_sub = None

        # State tracking
        self.last_positions: Optional[List[float]] = None
        self.last_time: Optional[float] = None
        self._running = False
        self._connected = False

        # Thread safety
        self._lock = threading.Lock()
        self._pub_thread: Optional[threading.Thread] = None

        # Callbacks
        self.on_state_published: Optional[Callable[[List[float]], None]] = None
        self.on_command_received: Optional[Callable[[List[float]], None]] = None

    @property
    def is_connected(self) -> bool:
        """Check if connected to robot and ROS."""
        return self._connected and self.robot is not None

    def connect(self, calibrate: bool = False) -> bool:
        """
        Connect to the robot hardware.

        Args:
            calibrate: Whether to run calibration on connect

        Returns:
            True if connection successful
        """
        print(f"[SO101Bridge] Connecting to robot on {self.robot_port}...")

        try:
            if self.robot_type == 'follower':
                config = SO101FollowerConfig(
                    port=self.robot_port,
                    id=self.robot_id,
                    calibration_dir=self.calibration_dir,
                    use_degrees=self.use_degrees,
                )
                self.robot = SO101Follower(config)
            elif self.robot_type == 'leader':
                config = SO101LeaderConfig(
                    port=self.robot_port,
                    id=self.robot_id,
                    calibration_dir=self.calibration_dir,
                    use_degrees=self.use_degrees,
                )
                self.robot = SO101Leader(config)
            else:
                raise ValueError(f"Unknown robot type: {self.robot_type}")

            self.robot.connect(calibrate=calibrate)
            self._connected = True
            print(f"[SO101Bridge] Robot connected: {self.robot_type} ({self.robot_id})")
            return True

        except Exception as e:
            print(f"[SO101Bridge] ERROR: Failed to connect to robot: {e}")
            return False

    def setup_ros_topics(self, Topic: Any) -> None:
        """
        Set up ROS topic publishers and subscribers.

        Args:
            Topic: Topic factory class (roslibpy.Topic or ProxyTopic)
        """
        # Publisher: joint states to VM
        self.joint_state_pub = Topic(
            self.ros_client,
            self.publish_topic,
            'sensor_msgs/JointState',
        )
        self.joint_state_pub.advertise()
        print(f"[SO101Bridge] Publishing to {self.publish_topic}")

        # Subscriber: joint commands from VM (only for follower)
        if self.robot_type == 'follower' and self.subscribe_commands:
            self.joint_cmd_sub = Topic(
                self.ros_client,
                self.command_topic,
                'std_msgs/Float64MultiArray',
            )
            self.joint_cmd_sub.subscribe(self._on_joint_command)
            print(f"[SO101Bridge] Subscribed to {self.command_topic}")

    def _on_joint_command(self, message: dict) -> None:
        """Handle incoming joint commands from VM."""
        try:
            data = message.get('data', [])
            if len(data) != len(self.JOINT_NAMES):
                print(f"[SO101Bridge] WARNING: Expected {len(self.JOINT_NAMES)} joints, got {len(data)}")
                return

            # Convert from radians to robot format and send
            self.send_command(data)

            if self.on_command_received:
                self.on_command_received(data)

        except Exception as e:
            print(f"[SO101Bridge] ERROR in command callback: {e}")

    def send_command(self, positions_rad: List[float]) -> None:
        """
        Send joint positions to the robot.

        Args:
            positions_rad: Joint positions in radians (6 values)
        """
        if not self.is_connected:
            return

        target_positions = {}
        for i, ros_joint_name in enumerate(self.JOINT_NAMES):
            hw_joint_name = self.ROS_TO_HW_MAP[ros_joint_name]
            target_positions[f'{hw_joint_name}.pos'] = self._radians_to_robot_format(
                ros_joint_name, positions_rad[i]
            )

        with self._lock:
            self.robot.send_action(target_positions)

    def get_positions(self) -> List[float]:
        """
        Get current joint positions from the robot.

        Returns:
            Joint positions in radians (6 values)
        """
        if not self.is_connected:
            return [0.0] * len(self.JOINT_NAMES)

        with self._lock:
            if self.robot_type == 'follower':
                obs = self.robot.get_observation()
            else:  # leader
                obs = self.robot.get_action()

        positions = []
        for ros_joint_name in self.JOINT_NAMES:
            hw_joint_name = self.ROS_TO_HW_MAP[ros_joint_name]
            raw_value = obs.get(f'{hw_joint_name}.pos', 0.0)
            rad = self._robot_format_to_radians(ros_joint_name, raw_value)
            positions.append(rad)

        return positions

    def _radians_to_robot_format(self, joint_name: str, rad: float) -> float:
        """Convert radians to robot-expected format (degrees or normalized)."""
        if joint_name != 'wrist_3_joint' and self.use_degrees:
            return math.degrees(rad)
        else:
            # Convert radians to normalized range [-100, 100]
            normalized = (rad / math.pi) * 100.0
            return normalized

    def _robot_format_to_radians(self, joint_name: str, value: float) -> float:
        """Convert robot format to radians for ROS."""
        if joint_name == 'wrist_3_joint':
            # Gripper (wrist_3_joint): normalized [0, 100] -> [0, pi]
            return (value / 100.0) * math.pi
        else:
            if self.use_degrees:
                return math.radians(value)
            else:
                # Normalized range [-100, 100] -> radians
                return (value / 100.0) * math.pi

    def publish_state(self) -> None:
        """Publish current joint states to ROS."""
        if not self.is_connected or self.joint_state_pub is None:
            return

        try:
            current_time = time.time()
            positions = self.get_positions()

            # Calculate velocities from position differences
            velocities = [0.0] * len(self.JOINT_NAMES)
            if self.last_positions is not None and self.last_time is not None:
                dt = current_time - self.last_time
                if dt > 1e-6:
                    for i in range(len(self.JOINT_NAMES)):
                        velocities[i] = (positions[i] - self.last_positions[i]) / dt

            # Update tracking
            self.last_positions = positions.copy()
            self.last_time = current_time

            # Build and publish message
            secs = int(current_time)
            nsecs = int((current_time - secs) * 1e9)

            joint_state_msg = {
                'header': {
                    'stamp': {'sec': secs, 'nanosec': nsecs},
                    'frame_id': '',
                },
                'name': self.publish_joint_names,
                'position': positions,
                'velocity': velocities,
                'effort': [0.0] * len(self.JOINT_NAMES),
            }

            self.joint_state_pub.publish(joint_state_msg)

            if self.on_state_published:
                self.on_state_published(positions)

        except Exception as e:
            print(f"[SO101Bridge] ERROR publishing joint states: {e}")

    def _publisher_loop(self, rate_hz: float) -> None:
        """Main publishing loop running in separate thread."""
        interval = 1.0 / rate_hz
        next_time = time.monotonic()

        while self._running and self.ros_client.is_connected:
            self.publish_state()

            # Maintain consistent rate
            next_time += interval
            sleep_time = max(0.0, next_time - time.monotonic())
            time.sleep(sleep_time)

    def start_publishing(self, rate_hz: float = 20.0) -> None:
        """
        Start publishing joint states in a background thread.

        Args:
            rate_hz: Publishing rate in Hz
        """
        if self._running:
            return

        self._running = True
        self._pub_thread = threading.Thread(
            target=self._publisher_loop,
            args=(rate_hz,),
            daemon=True
        )
        self._pub_thread.start()
        print(f"[SO101Bridge] Publishing at {rate_hz} Hz")

    def stop_publishing(self) -> None:
        """Stop the publishing thread."""
        self._running = False
        if self._pub_thread is not None and self._pub_thread.is_alive():
            self._pub_thread.join(timeout=1.0)

    def shutdown(self) -> None:
        """Clean shutdown of all connections."""
        print("[SO101Bridge] Shutting down...")
        self.stop_publishing()

        # Disconnect robot
        if self.robot is not None:
            try:
                if hasattr(self.robot, 'is_connected') and self.robot.is_connected:
                    self.robot.disconnect()
                    print("[SO101Bridge] Robot disconnected")
            except Exception as e:
                print(f"[SO101Bridge] WARNING: Error disconnecting robot: {e}")

        # Cleanup ROS topics
        try:
            if self.joint_state_pub is not None:
                self.joint_state_pub.unadvertise()
            if self.joint_cmd_sub is not None:
                self.joint_cmd_sub.unsubscribe()
        except Exception as e:
            print(f"[SO101Bridge] WARNING: Error cleaning up topics: {e}")

        self._connected = False
        print("[SO101Bridge] Shutdown complete")


# Convenience function to check if hardware is available
def is_hardware_available() -> bool:
    """Check if lerobot SDK is available for hardware connection."""
    return LEROBOT_AVAILABLE
