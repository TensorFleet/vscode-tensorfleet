#!/usr/bin/env python3
"""
Deploy a trained ACT policy to control the SO-ARM101 robot in Gazebo simulation.

This script loads a trained LeRobot ACT policy, subscribes to camera and joint
state topics via rosbridge, runs inference, and publishes trajectory commands
to control the robot.

Usage:
    uv run python src/deploy_act.py --policy-path <path_to_checkpoint>

Examples:
    # Deploy the latest checkpoint
    uv run python src/deploy_act.py \\
        --policy-path outputs/train/act_local_so_arm101_sim/checkpoints/last/pretrained_model

    # Smoke test (load model, check topics, run one inference)
    uv run python src/deploy_act.py --policy-path <path> --check

    # Custom FPS and action steps
    uv run python src/deploy_act.py --policy-path <path> --fps 10 --action-steps 5
"""

import argparse
import base64
import os
import sys
import threading
import time
import atexit
import select
import termios
import tty
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

try:
    import torch
except ImportError:
    print("Error: torch not installed. Run: pip install torch")
    sys.exit(1)

try:
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors
except ImportError:
    print("Error: lerobot not installed. Run: pip install lerobot")
    sys.exit(1)

try:
    import roslibpy
except ImportError:
    print("Error: roslibpy not installed. Run: pip install roslibpy")
    sys.exit(1)

try:
    from lib.robotic_utils import connect_to_robot, Topic
except ImportError:
    try:
        from robotic_utils import connect_to_robot, Topic
    except ImportError:
        connect_to_robot = None
        Topic = None

# Load .env file automatically
try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    pass

# Joint name mapping (matching record script)
JOINT_ORDER = [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
]

JOINT_NAME_MAP = {
    "1": "shoulder_pan.pos",
    "2": "shoulder_lift.pos",
    "3": "elbow_flex.pos",
    "4": "wrist_flex.pos",
    "5": "wrist_roll.pos",
    "6": "gripper.pos",
    "joint1": "shoulder_pan.pos",
    "joint2": "shoulder_lift.pos",
    "joint3": "elbow_flex.pos",
    "joint4": "wrist_flex.pos",
    "joint5": "wrist_roll.pos",
    "joint6": "gripper.pos",
    "shoulder_pan_joint": "shoulder_pan.pos",
    "shoulder_lift_joint": "shoulder_lift.pos",
    "elbow_joint": "elbow_flex.pos",
    "elbow_flex_joint": "elbow_flex.pos",
    "wrist_1_joint": "wrist_flex.pos",
    "wrist_2_joint": "wrist_roll.pos",
    "wrist_3_joint": "gripper.pos",
    "wrist_flex_joint": "wrist_flex.pos",
    "wrist_roll_joint": "wrist_roll.pos",
    "gripper_joint": "gripper.pos",
}

# URDF Limits (from teleop_so_arm101.py)
URDF_LIMITS = {
    "1": (-1.91986, 1.91986),
    "2": (-1.74533, 1.74533),
    "3": (-1.74533, 1.5708),
    "4": (-1.65806, 1.65806),
    "5": (-2.79253, 2.79253),
    "6": (-0.174533, 1.74533),
}

# ROS joint names for publishing
ROS_ARM_JOINTS = ["1", "2", "3", "4", "5"]
ROS_GRIPPER_JOINTS = ["6"]

DEFAULT_CAMERA_TOPICS = {
    "wrist": "/so_arm101/wrist_camera/image_raw",
    "agent": "/so_arm101/agent_camera/image_raw",
    "side": "/so_arm101/side_camera/image_raw",
}

# ImageNet normalization (matching training)
IMAGENET_MEAN = [0.485, 0.456, 0.406]
IMAGENET_STD = [0.229, 0.224, 0.225]


@dataclass
class DeployConfig:
    """Configuration for ACT deployment."""
    policy_path: str
    fps: int = 5
    action_steps: int = 10
    device: str = "cuda"
    state_topic: str = "/joint_states"
    arm_action_topic: str = "/arm_controller/joint_trajectory"
    gripper_action_topic: str = "/gripper_controller/joint_trajectory"
    cameras: List[str] = None
    camera_topics: Dict[str, str] = None
    rosbridge_url: Optional[str] = None
    check: bool = False
    debug: bool = False
    trajectory_time: float = 0.1

    def __post_init__(self):
        if self.cameras is None:
            self.cameras = ["wrist", "agent", "side"]
        if self.camera_topics is None:
            self.camera_topics = {
                name: DEFAULT_CAMERA_TOPICS[name]
                for name in self.cameras
            }


def _normalize_joint_name(name: str) -> Optional[str]:
    """Normalize joint name to canonical format."""
    return JOINT_NAME_MAP.get(name)


def _decode_ros_image(msg: dict) -> Optional[np.ndarray]:
    """Decode a ROS Image message to numpy array."""
    try:
        width = int(msg.get("width", 0))
        height = int(msg.get("height", 0))
        if width <= 0 or height <= 0:
            return None

        data = msg.get("data")
        if data is None:
            return None

        if isinstance(data, str):
            raw = base64.b64decode(data)
        elif isinstance(data, (bytes, bytearray)):
            raw = bytes(data)
        else:
            raw = bytes(data)

        encoding = str(msg.get("encoding", "rgb8")).lower()
        if encoding in ("rgb8", "r8g8b8"):
            channels = 3
            arr = np.frombuffer(raw, dtype=np.uint8)
            expected = height * width * channels
            if arr.size < expected:
                return None
            return arr[:expected].reshape((height, width, channels))
        if encoding in ("bgr8",):
            channels = 3
            arr = np.frombuffer(raw, dtype=np.uint8)
            expected = height * width * channels
            if arr.size < expected:
                return None
            img = arr[:expected].reshape((height, width, channels))
            return img[..., ::-1]  # BGR -> RGB
        if encoding in ("rgba8", "bgra8"):
            channels = 4
            arr = np.frombuffer(raw, dtype=np.uint8)
            expected = height * width * channels
            if arr.size < expected:
                return None
            img = arr[:expected].reshape((height, width, channels))
            if encoding == "bgra8":
                img = img[..., [2, 1, 0, 3]]
            return img[..., :3]
    except Exception:
        return None
    return None


def preprocess_image(img: np.ndarray, target_size: tuple = (240, 320)) -> torch.Tensor:
    """
    Preprocess image for ACT policy.
    
    Resizes and converts to tensor (float 0-1).
    Normalization is handled by the policy preprocessor.

    Args:
        img: RGB image as HWC numpy array (uint8)
        target_size: Target (height, width)

    Returns:
        CHW tensor (float 0-1)
    """
    import cv2

    # Resize if needed
    h, w = img.shape[:2]
    if (h, w) != target_size:
        img = cv2.resize(img, (target_size[1], target_size[0]))

    # Convert to float [0, 1]
    img = img.astype(np.float32) / 255.0

    # HWC -> CHW
    img = np.transpose(img, (2, 0, 1))

    return torch.from_numpy(img)


class ACTDeployer:
    """Deploy a trained ACT policy to control the robot."""

    def __init__(self, config: DeployConfig, client) -> None:
        self.config = config
        self.client = client
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._paused = False
        self._old_settings = None

        # Observation buffers
        self._joint_positions: Dict[str, Optional[float]] = {
            name: None for name in JOINT_ORDER
        }
        self._images: Dict[str, Optional[np.ndarray]] = {
            name: None for name in config.cameras
        }
        self._have_joint_state = False
        self._have_images = {name: False for name in config.cameras}

        # Action buffer (for chunking)
        self._action_queue: List[np.ndarray] = []
        self._action_index = 0

        # Load policy
        self._load_policy()

        # Setup ROS topics
        self._setup_topics()

    def _load_policy(self) -> None:
        """Load the trained ACT policy."""
        print(f"[DEPLOY] Loading policy from {self.config.policy_path}...")

        self.policy = ACTPolicy.from_pretrained(self.config.policy_path)

        # Move to device
        device = self.config.device
        if device == "cuda" and not torch.cuda.is_available():
            print("[DEPLOY] CUDA not available, falling back to CPU")
            device = "cpu"

        self.device = device
        self.policy.to(device)
        self.policy.eval()

        # Create pre/post processors
        preprocessor_overrides = {
            "device_processor": {"device": str(self.device)},
        }
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            policy_cfg=self.policy.config,
            pretrained_path=self.config.policy_path,
            preprocessor_overrides=preprocessor_overrides,
        )
        # Processors handle device placement internally via overrides

        print(f"[DEPLOY] Policy loaded on {device}")

    def _setup_topics(self) -> None:
        """Setup ROS topic subscriptions and publishers."""
        TopicClass = Topic if Topic else roslibpy.Topic

        # Subscribe to joint states
        self._joint_sub = TopicClass(
            self.client, self.config.state_topic, "sensor_msgs/JointState"
        )
        self._joint_sub.subscribe(self._on_joint_state)

        # Subscribe to cameras
        self._image_subs = {}
        for name, topic in self.config.camera_topics.items():
            sub = TopicClass(self.client, topic, "sensor_msgs/Image")
            sub.subscribe(lambda msg, cam=name: self._on_image(cam, msg))
            self._image_subs[name] = sub

        # Publishers for arm and gripper
        self._arm_pub = TopicClass(
            self.client,
            self.config.arm_action_topic,
            "trajectory_msgs/JointTrajectory",
        )
        self._arm_pub.advertise()

        self._gripper_pub = TopicClass(
            self.client,
            self.config.gripper_action_topic,
            "trajectory_msgs/JointTrajectory",
        )
        self._gripper_pub.advertise()

        print(f"[DEPLOY] Subscribed to {self.config.state_topic}")
        for name, topic in self.config.camera_topics.items():
            print(f"[DEPLOY] Subscribed to {topic} ({name})")
        print(f"[DEPLOY] Publishing to {self.config.arm_action_topic}")
        print(f"[DEPLOY] Publishing to {self.config.gripper_action_topic}")

    def _on_joint_state(self, msg: dict) -> None:
        """Handle incoming joint state message."""
        names = msg.get("name", [])
        positions = msg.get("position", [])
        if not names or positions is None:
            return

        with self._lock:
            for name, pos in zip(names, positions):
                canonical = _normalize_joint_name(str(name))
                if not canonical:
                    continue
                try:
                    self._joint_positions[canonical] = float(pos)
                except (TypeError, ValueError):
                    continue

            self._have_joint_state = all(
                self._joint_positions[name] is not None for name in JOINT_ORDER
            )

    def _on_image(self, camera: str, msg: dict) -> None:
        """Handle incoming image message."""
        img = _decode_ros_image(msg)
        if img is None:
            return

        with self._lock:
            self._images[camera] = img
            self._have_images[camera] = True

    def _wait_for_observations(self, timeout: float = 15.0) -> bool:
        """Wait for initial observations from all sources."""
        print("[DEPLOY] Waiting for observations...")
        start = time.time()

        while time.time() - start < timeout:
            with self._lock:
                joints_ok = self._have_joint_state
                images_ok = all(self._have_images.values())

            if joints_ok and images_ok:
                print("[DEPLOY] All observations received")
                return True

            time.sleep(0.1)

        # Report what's missing
        missing = []
        with self._lock:
            if not self._have_joint_state:
                missing.append("joint_states")
            for cam, ok in self._have_images.items():
                if not ok:
                    missing.append(f"camera:{cam}")

        print(f"[DEPLOY] Timeout waiting for: {', '.join(missing)}")
        return False



    def _restore_terminal(self) -> None:
        """Restore terminal settings."""
        if self._old_settings is not None:
             termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
             self._old_settings = None

    def _build_observation(self) -> Dict[str, torch.Tensor]:
        """Build observation dict for policy inference."""
        with self._lock:
            # Get joint state
            state = np.array(
                [self._joint_positions[name] for name in JOINT_ORDER],
                dtype=np.float32
            )

            # Get images
            images = {
                name: self._images[name].copy()
                for name in self.config.cameras
            }

        # Build observation dict
        obs = {}

        # State: shape [1, 6]
        obs["observation.state"] = torch.from_numpy(state).unsqueeze(0).to(self.device)

        # Images: shape [1, 3, H, W]
        for name, img in images.items():
            tensor = preprocess_image(img)
            obs[f"observation.images.{name}"] = tensor.unsqueeze(0).to(self.device)

        return obs

    def _run_inference(self) -> np.ndarray:
        """Run policy inference and return action chunk."""
        obs = self._build_observation()

        # Apply preprocessing (normalization)
        obs = self.preprocessor(obs)

        with torch.no_grad():
            action = self.policy.select_action(obs)

        # Apply postprocessing (unnormalization)
        action = self.postprocessor(action)

        # Handle both Tensor and Dict returns from policy
        if isinstance(action, dict):
            action_tensor = action["action"]
        else:
            action_tensor = action

        if action_tensor.dim() == 3:
            action_tensor = action_tensor.squeeze(0)  # Remove batch dim

        return action_tensor.cpu().numpy()

    def _make_trajectory_msg(
        self, joint_names: List[str], positions: List[float]
    ) -> dict:
        """Create a JointTrajectory message."""
        return {
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
            "joint_names": joint_names,
            "points": [
                {
                    "positions": positions,
                    "velocities": [],
                    "accelerations": [],
                    "effort": [],
                    "time_from_start": {
                        "sec": 0,
                        "nanosec": int(self.config.trajectory_time * 1e9),
                    },
                }
            ],
        }

    def _publish_action(self, action: np.ndarray) -> None:
        """Publish action to arm and gripper controllers."""
        # action is [6] array of joint positions
        # Clamp values to limits
        clamped_action = []
        for i, val in enumerate(action):
            name = str(i + 1)
            low, high = URDF_LIMITS.get(name, (-np.inf, np.inf))
            clamped_action.append(float(max(low, min(high, val))))
        
        arm_positions = clamped_action[:5]
        gripper_positions = clamped_action[5:]

        arm_msg = self._make_trajectory_msg(ROS_ARM_JOINTS, arm_positions)
        gripper_msg = self._make_trajectory_msg(ROS_GRIPPER_JOINTS, gripper_positions)

        self._arm_pub.publish(arm_msg)
        self._gripper_pub.publish(gripper_msg)

        if self.config.debug:
            print(f"[DEBUG] Trajectory time: {self.config.trajectory_time}s")
            print(f"[DEBUG] Arm cmd: {[f'{x:.3f}' for x in arm_positions]}")
            print(f"[DEBUG] Gripper cmd: {[f'{x:.3f}' for x in gripper_positions]}")
            with self._lock:
                current = [self._joint_positions[name] for name in JOINT_ORDER]
                print(f"[DEBUG] Current state: {[f'{x:.3f}' if x is not None else 'None' for x in current]}")

    def run_check(self) -> bool:
        """Run smoke test: load, connect, infer once."""
        print("\n[CHECK] Running smoke test...")

        if not self._wait_for_observations(timeout=10.0):
            print("[CHECK] FAILED: Could not receive observations")
            return False

        print("[CHECK] Building observation...")
        obs = self._build_observation()

        print("[CHECK] Observation shapes:")
        for key, tensor in obs.items():
            print(f"  {key}: {tuple(tensor.shape)}")

        print("[CHECK] Running inference...")
        action = self._run_inference()
        print(f"[CHECK] Action shape: {action.shape}")
        print(f"[CHECK] First action: {action[0]}")

        # Verify publishing (serialization check)
        print("[CHECK] testing publish...")
        try:
            self._publish_action(action[0])
            print("[CHECK] Publish successful")
        except Exception as e:
            print(f"[CHECK] Publish failed: {e}")
            import traceback
            traceback.print_exc()
            return False

        print("[CHECK] SUCCESS: Smoke test passed")
        return True

    def run(self) -> None:
        """Main deployment loop."""
        print("\n[DEPLOY] Starting deployment loop...")
        print("Controls: [Space] pause/resume, [q/Ctrl-C] quit\n")

        if not self._wait_for_observations():
            print("[DEPLOY] ERROR: Failed to receive observations")
            return

        interval = 1.0 / self.config.fps
        next_time = time.monotonic()
        inference_count = 0
        action_count = 0

        # Start keyboard listener in background
        self._start_keyboard_listener()

        try:
            while not self._stop.is_set():
                if self._paused:
                    time.sleep(0.1)
                    continue

                # Check if we need more actions
                if self._action_index >= len(self._action_queue):
                    # Run inference
                    actions = self._run_inference()
                    self._action_queue = list(actions[: self.config.action_steps])
                    self._action_index = 0
                    inference_count += 1

                # Get next action
                action = self._action_queue[self._action_index]
                self._action_index += 1
                action_count += 1

                # Publish action
                self._publish_action(action)

                # Rate limiting
                next_time += interval
                sleep_time = max(0.0, next_time - time.monotonic())
                time.sleep(sleep_time)

                # Status update
                if action_count % (self.config.fps * 5) == 0:
                    print(
                        f"[DEPLOY] Inferences: {inference_count}, "
                        f"Actions: {action_count}"
                    )

        except KeyboardInterrupt:
            print("\n[DEPLOY] Interrupted")
        finally:
            self._stop.set()
            print(f"\n[DEPLOY] Stopped. Total inferences: {inference_count}")

    def _start_keyboard_listener(self) -> None:
        """Start background thread to listen for keyboard input."""
        if not sys.stdin.isatty():
             return

        try:
             self._old_settings = termios.tcgetattr(sys.stdin)
             tty.setcbreak(sys.stdin.fileno())
             atexit.register(self._restore_terminal)
        except Exception:
             print("[DEPLOY] Warning: Failed to set terminal mode")
             return

        def listener():
            try:
                while not self._stop.is_set():
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.read(1)
                        self._handle_key(key)
            except Exception:
                pass
            finally:
                self._restore_terminal()

        thread = threading.Thread(target=listener, daemon=True)
        thread.start()

    def _handle_key(self, key: str) -> None:
        """Handle keyboard input."""
        if key == " ":
            self._paused = not self._paused
            status = "PAUSED" if self._paused else "RESUMED"
            print(f"[DEPLOY] {status}")
        elif key in ("q", "\x03"):  # q or Ctrl-C
            self._stop.set()

    def shutdown(self) -> None:
        """Clean shutdown."""
        self._stop.set()

        # Unsubscribe from topics
        try:
            self._joint_sub.unsubscribe()
            for sub in self._image_subs.values():
                sub.unsubscribe()
            self._arm_pub.unadvertise()
            self._gripper_pub.unadvertise()
        except Exception:
            pass
        
        self._restore_terminal()

        print("[DEPLOY] Shutdown complete")


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Deploy trained ACT policy to Gazebo simulation"
    )

    parser.add_argument(
        "--policy-path",
        type=str,
        required=True,
        help="Path to trained policy checkpoint (pretrained_model directory)",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=5,
        help="Control loop frequency in Hz (default: 5)",
    )
    parser.add_argument(
        "--action-steps",
        type=int,
        default=10,
        help="Number of actions to execute per inference (default: 10)",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda",
        choices=["cuda", "cpu", "mps"],
        help="Device for inference (default: cuda)",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Run smoke test and exit",
    )
    parser.add_argument(
        "--cameras",
        type=str,
        default="wrist,agent,side",
        help="Comma-separated list of cameras (default: wrist,agent,side)",
    )
    parser.add_argument(
        "--rosbridge-url",
        type=str,
        default=None,
        help="Rosbridge WebSocket URL (default: from .env)",
    )
    parser.add_argument(
        "--trajectory-time",
        type=float,
        default=0.2,
        help="Trajectory time_from_start in seconds (default: 0.2)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug logging of commands and states",
    )
    return parser.parse_args()


def main() -> None:
    """Main entry point."""
    args = parse_args()

    # Parse camera list
    cameras = [c.strip() for c in args.cameras.split(",") if c.strip()]

    # Create config
    config = DeployConfig(
        policy_path=args.policy_path,
        fps=args.fps,
        action_steps=args.action_steps,
        device=args.device,
        cameras=cameras,
        rosbridge_url=args.rosbridge_url,
        check=args.check,
        debug=args.debug,
        trajectory_time=args.trajectory_time,
    )

    # Validate policy path
    policy_path = Path(config.policy_path).expanduser()
    if not policy_path.exists():
        print(f"Error: Policy path not found: {policy_path}")
        sys.exit(1)

    # Check for config.json
    if not (policy_path / "config.json").exists():
        print(f"Error: config.json not found in {policy_path}")
        print("Make sure --policy-path points to the pretrained_model directory")
        sys.exit(1)

    # Connect to rosbridge
    print("[DEPLOY] Connecting to rosbridge...")
    if connect_to_robot is None:
        print("Error: lib.robotic_utils not available")
        sys.exit(1)

    try:
        client = connect_to_robot(url=config.rosbridge_url)
    except Exception as e:
        print(f"Error: Failed to connect to rosbridge: {e}")
        sys.exit(1)

    # Create deployer
    deployer = ACTDeployer(config, client)

    try:
        if config.check:
            success = deployer.run_check()
            sys.exit(0 if success else 1)
        else:
            deployer.run()
    finally:
        deployer.shutdown()
        client.terminate()


if __name__ == "__main__":
    main()
