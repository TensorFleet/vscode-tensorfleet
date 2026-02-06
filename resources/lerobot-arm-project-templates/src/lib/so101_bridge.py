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
    # Preferred imports for official lerobot releases (>=0.4).
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
    from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
    LEROBOT_AVAILABLE = True
except ImportError:
    try:
        # Fallback for legacy lerobot forks that exposed so101_* modules.
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

    # Joint names used by the VM ros2_control setup.
    JOINT_NAMES = [
        '1',
        '2',
        '3',
        '4',
        '5',
        '6',
    ]

    # Mapping from ROS joint names to lerobot hardware SDK names.
    ROS_TO_HW_MAP = {
        '1': 'shoulder_pan',
        '2': 'shoulder_lift',
        '3': 'elbow_flex',
        '4': 'wrist_flex',
        '5': 'wrist_roll',
        '6': 'gripper',
        # Aliases for standard ROS-style names.
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
        debug: bool = False,
        debug_interval: float = 1.0,
        debug_raw: bool = False,
        limit_map: Optional[Dict[str, tuple[float, float]]] = None,
        map_to_limits: bool = False,
        clamp_to_limits: bool = False,
        publish_to_ros: bool = True,
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
            debug: Enable periodic debug output callbacks
            debug_interval: Seconds between debug updates
            debug_raw: Read raw servo ticks for debug output (slower)
            limit_map: Mapping of joint name to (lower, upper) radians limits
            map_to_limits: Map normalized values to limits when converting to radians
            clamp_to_limits: Clamp radians to limits after conversion
            publish_to_ros: Publish joint states to ROS (disable to cut proxy traffic)
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
        self.debug = debug
        self.debug_interval = max(0.1, float(debug_interval))
        self.debug_raw = debug_raw
        self.limit_map = limit_map
        self.map_to_limits = map_to_limits
        self.clamp_to_limits = clamp_to_limits
        self.publish_to_ros = publish_to_ros

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
            self.calibration_dir = Path(calibration_dir).expanduser()

        # Will be initialized in connect()
        self.robot = None
        self.joint_state_pub = None
        self.joint_cmd_sub = None

        # State tracking
        self.last_positions: Optional[List[float]] = None
        self.last_time: Optional[float] = None
        self._last_valid_positions: Optional[List[float]] = None
        self._last_valid_norm: Optional[List[float]] = None
        self._last_raw_positions: Optional[List[float]] = None
        self._running = False
        self._connected = False
        self._last_debug_time = 0.0

        # Thread safety
        self._lock = threading.Lock()
        self._pub_thread: Optional[threading.Thread] = None

        # Callbacks
        self.on_state_published: Optional[Callable[[List[float]], None]] = None
        self.on_command_received: Optional[Callable[[List[float]], None]] = None
        self.on_state_debug: Optional[Callable[[Dict[str, Any]], None]] = None

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
        if self.publish_to_ros:
            self.joint_state_pub = Topic(
                self.ros_client,
                self.publish_topic,
                'sensor_msgs/JointState',
            )
            self.joint_state_pub.advertise()
            print(f"[SO101Bridge] Publishing to {self.publish_topic}")
        else:
            print("[SO101Bridge] ROS joint state publishing disabled")

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
        norm_positions: List[float] = []
        raw_positions: Optional[List[float]] = None
        if self.debug and self.debug_raw:
            raw_positions = self._read_raw_positions()
        for idx, ros_joint_name in enumerate(self.JOINT_NAMES):
            hw_joint_name = self.ROS_TO_HW_MAP[ros_joint_name]
            raw_value = obs.get(f'{hw_joint_name}.pos')
            if raw_value is None and self._last_valid_positions is not None:
                if self._last_valid_norm is not None:
                    norm_positions.append(self._last_valid_norm[idx])
                positions.append(self._last_valid_positions[idx])
                continue
            if raw_value is None:
                raw_value = 0.0
            norm_positions.append(float(raw_value))
            rad = self._robot_format_to_radians(ros_joint_name, raw_value)
            positions.append(rad)

        if raw_positions is not None:
            self._last_raw_positions = raw_positions
        self._last_valid_norm = norm_positions.copy()
        self._last_valid_positions = positions.copy()
        return positions

    def _read_raw_positions(self) -> Optional[List[float]]:
        if not hasattr(self.robot, 'bus'):
            return None
        try:
            raw_obs = self.robot.bus.sync_read('Present_Position', normalize=False)
        except Exception:
            return None

        raw_positions = []
        for ros_joint_name in self.JOINT_NAMES:
            hw_joint_name = self.ROS_TO_HW_MAP[ros_joint_name]
            raw_positions.append(raw_obs.get(hw_joint_name))
        return raw_positions

    def _is_gripper_joint(self, joint_name: str) -> bool:
        hw_name = self.ROS_TO_HW_MAP.get(joint_name)
        return joint_name in ('wrist_3_joint', '6') or hw_name == 'gripper'

    def _get_limits(self, joint_name: str) -> Optional[tuple[float, float]]:
        if not self.limit_map:
            return None
        if joint_name in self.limit_map:
            return self.limit_map[joint_name]
        hw_name = self.ROS_TO_HW_MAP.get(joint_name)
        if not hw_name:
            return None
        ros_name = self.HW_TO_ROS_MAP.get(hw_name)
        if ros_name in self.limit_map:
            return self.limit_map[ros_name]
        return None

    def _map_norm_to_limits(
        self,
        joint_name: str,
        norm_value: float,
        low: float,
        high: float,
    ) -> float:
        if self._is_gripper_joint(joint_name):
            bounded = max(0.0, min(100.0, norm_value))
            return low + (bounded / 100.0) * (high - low)
        bounded = max(-100.0, min(100.0, norm_value))
        return low + ((bounded + 100.0) / 200.0) * (high - low)

    def _radians_to_robot_format(self, joint_name: str, rad: float) -> float:
        """Convert radians to robot-expected format (degrees or normalized)."""
        if self._is_gripper_joint(joint_name):
            normalized = (rad / math.pi) * 100.0
            return max(0.0, min(100.0, normalized))
        if self.use_degrees:
            return math.degrees(rad)
        # Convert radians to normalized range [-100, 100]
        return (rad / math.pi) * 100.0

    def _robot_format_to_radians(self, joint_name: str, value: float) -> float:
        """Convert robot format to radians for ROS."""
        limits = self._get_limits(joint_name)

        if not self.use_degrees and self.map_to_limits and limits:
            low, high = limits
            rad = self._map_norm_to_limits(joint_name, value, low, high)
            if self.clamp_to_limits:
                return max(low, min(high, rad))
            return rad

        if self._is_gripper_joint(joint_name):
            # Gripper: normalized [0, 100] -> [0, pi]
            rad = (value / 100.0) * math.pi
        elif self.use_degrees:
            rad = math.radians(value)
        else:
            # Normalized range [-100, 100] -> radians
            rad = (value / 100.0) * math.pi

        if self.clamp_to_limits and limits:
            low, high = limits
            return max(low, min(high, rad))
        return rad

    def publish_state(self) -> None:
        """Publish current joint states to ROS."""
        if not self.is_connected:
            return

        try:
            current_time = time.time()
            positions = self.get_positions()

            if self.publish_to_ros and self.joint_state_pub is not None:
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
            else:
                # Keep timestamps updated for debug cadence.
                self.last_positions = positions.copy()
                self.last_time = current_time

            if self.on_state_published:
                self.on_state_published(positions)
            self._emit_debug(positions)

        except Exception as e:
            print(f"[SO101Bridge] ERROR publishing joint states: {e}")

    def _emit_debug(self, positions: List[float]) -> None:
        if not self.debug:
            return

        now = time.time()
        if now - self._last_debug_time < self.debug_interval:
            return

        payload = {
            'time': now,
            'joint_names': list(self.JOINT_NAMES),
            'positions_rad': positions,
            'positions_norm': self._last_valid_norm,
            'positions_raw': self._last_raw_positions,
        }

        if self.on_state_debug:
            self.on_state_debug(payload)
        else:
            print(f"[SO101Bridge][debug] {payload}")

        self._last_debug_time = now

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
