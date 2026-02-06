#!/usr/bin/env python3
"""
Record a LeRobot-compatible dataset from the SO-ARM101 simulator over rosbridge.

Optionally run teleop in the same process (use --input keyboard or --input leader).
Lerobot-style flags (--robot.*, --teleop.*, --dataset.*) are also accepted.

Default topics are aligned with the Firecracker VM stack:
  - /joint_states
  - /arm_controller/joint_trajectory
  - /gripper_controller/joint_trajectory
  - /so_arm101/{wrist,agent,side}_camera/image_raw
  - /clock
"""
import argparse
import base64
import os
import shutil
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Iterable, Optional
from urllib.parse import urlparse

import numpy as np

try:
    import yaml
except ImportError:
    yaml = None

try:
    import roslibpy
except ImportError:
    print("Error: roslibpy not installed. Run: pip install roslibpy")
    sys.exit(1)

try:
    from lerobot.datasets.lerobot_dataset import INFO_PATH, LeRobotDataset
    from lerobot.datasets.pipeline_features import hw_to_dataset_features
    from lerobot.datasets.utils import DEFAULT_FEATURES, build_dataset_frame
    from lerobot.utils.constants import ACTION, OBS_STR
except ImportError:
    print("Error: lerobot not installed. Run: pip install lerobot")
    sys.exit(1)

try:
    from lib.robotic_utils import connect_to_robot, Topic, Service
except ImportError:
    try:
        from robotic_utils import connect_to_robot, Topic, Service
    except ImportError:
        connect_to_robot = None
        Topic = None
        Service = None

# Load .env file automatically (no need for 'source .env')
try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    pass  # dotenv is optional


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
    "joint_1": "shoulder_pan.pos",
    "joint_2": "shoulder_lift.pos",
    "joint_3": "elbow_flex.pos",
    "joint_4": "wrist_flex.pos",
    "joint_5": "wrist_roll.pos",
    "joint_6": "gripper.pos",
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
    "shoulder_pan": "shoulder_pan.pos",
    "shoulder_lift": "shoulder_lift.pos",
    "elbow_flex": "elbow_flex.pos",
    "wrist_flex": "wrist_flex.pos",
    "wrist_roll": "wrist_roll.pos",
    "gripper": "gripper.pos",
    "shoulder_pan.pos": "shoulder_pan.pos",
    "shoulder_lift.pos": "shoulder_lift.pos",
    "elbow_flex.pos": "elbow_flex.pos",
    "wrist_flex.pos": "wrist_flex.pos",
    "wrist_roll.pos": "wrist_roll.pos",
    "gripper.pos": "gripper.pos",
}

JOINT_ORDER = [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
]

DEFAULT_LEADER_ID = "awesome_leader"
DEFAULT_FOLLOWER_ID = "my_awesome_follower_arm"
DEFAULT_LEADER_CAL_DIR = "~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/"
DEFAULT_FOLLOWER_CAL_DIR = "~/.cache/huggingface/lerobot/calibration/robots/so_follower/"
DEFAULT_CAMERA_TOPICS = {
    "wrist": "/so_arm101/wrist_camera/image_raw",
    "agent": "/so_arm101/agent_camera/image_raw",
    "side": "/so_arm101/side_camera/image_raw",
    "front": "/so_arm101/agent_camera/image_raw",
}
DISPLAY_MAX_FPS = 15


def _now_ts() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def _decode_ros_image(msg: dict) -> Optional[np.ndarray]:
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
            return img[..., ::-1]
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
        if encoding in ("mono8", "8uc1"):
            arr = np.frombuffer(raw, dtype=np.uint8)
            expected = height * width
            if arr.size < expected:
                return None
            img = arr[:expected].reshape((height, width))
            return np.repeat(img[:, :, None], 3, axis=2)
    except Exception:
        return None
    return None


def _normalize_joint_name(name: str) -> Optional[str]:
    return JOINT_NAME_MAP.get(name)


def _format_feature_mismatch(expected: dict[str, dict], actual: dict[str, dict]) -> str:
    expected_keys = set(expected)
    actual_keys = set(actual)
    missing = sorted(expected_keys - actual_keys)
    extra = sorted(actual_keys - expected_keys)
    mismatched = []
    for key in sorted(expected_keys & actual_keys):
        if expected[key] != actual[key]:
            mismatched.append((key, expected[key], actual[key]))

    parts = []
    if missing:
        parts.append(f"missing: {missing}")
    if extra:
        parts.append(f"extra: {extra}")
    if mismatched:
        preview = ", ".join(
            f"{key}: expected {expected_val} got {actual_val}"
            for key, expected_val, actual_val in mismatched[:6]
        )
        if len(mismatched) > 6:
            preview += ", ..."
        parts.append(f"mismatched: {preview}")

    details = "; ".join(parts) if parts else "features differ"
    return f"Resume dataset features do not match current config. {details}"


def _coerce_bool(value: Optional[str], default: Optional[bool] = None) -> Optional[bool]:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "y", "on"}:
        return True
    if text in {"0", "false", "no", "n", "off"}:
        return False
    raise ValueError(f"Invalid boolean value: {value}")


def _dedupe_keep_order(items: Iterable[str]) -> list[str]:
    seen = set()
    result = []
    for item in items:
        if item in seen:
            continue
        seen.add(item)
        result.append(item)
    return result


def _parse_yaml_value(raw: str, label: str) -> Any:
    if yaml is None:
        raise ValueError(f"{label} requires PyYAML (pip install pyyaml).")
    try:
        return yaml.safe_load(raw)
    except Exception as exc:
        raise ValueError(f"Failed to parse {label}: {exc}") from exc


def _parse_yaml_mapping(raw: Optional[str], label: str) -> dict[str, str]:
    if not raw:
        return {}
    data = _parse_yaml_value(raw, label)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"{label} must be a mapping (got {type(data).__name__}).")
    output = {}
    for key, value in data.items():
        if value is None:
            raise ValueError(f"{label} missing value for key '{key}'.")
        output[str(key)] = str(value)
    return output


def _parse_camera_names(raw: Optional[str], label: str) -> list[str]:
    if not raw:
        return []
    data = _parse_yaml_value(raw, label)
    if isinstance(data, dict):
        names = [str(name) for name in data.keys()]
    elif isinstance(data, list):
        names = [str(name) for name in data]
    else:
        raise ValueError(f"{label} must be a mapping or list (got {type(data).__name__}).")
    names = [name.strip() for name in names if name and str(name).strip()]
    return _dedupe_keep_order(names)


def _resolve_camera_topics(
    cameras: list[str], overrides: dict[str, str]
) -> dict[str, str]:
    topics = dict(DEFAULT_CAMERA_TOPICS)
    topics.update(overrides)
    missing = [name for name in cameras if name not in topics]
    if missing:
        missing_text = ", ".join(missing)
        raise ValueError(
            "Missing camera topics for: "
            f"{missing_text}. Provide --ros.camera_topics "
            "like '{front: /so_arm101/agent_camera/image_raw}'."
        )
    return {name: topics[name] for name in cameras}


@dataclass
class RecorderConfig:
    repo_id: str
    root: str
    robot_type: str
    fps: int
    episodes: int
    episode_seconds: float
    reset_seconds: float
    task: str
    use_videos: bool
    vcodec: str
    state_topic: str
    arm_action_topic: str
    gripper_action_topic: str
    clock_topic: str
    cameras: list[str]
    camera_topics: dict[str, str]
    resume: bool
    rosbridge_url: str
    check: bool
    check_timeout: float
    wait_for_action: bool
    input_device: str
    leader_port: Optional[str]
    leader_id: str
    leader_calibration_dir: str
    leader_calibration_file: Optional[str]
    leader_rate: float
    follower_port: Optional[str]
    follower_id: str
    follower_calibration_dir: Optional[str]
    trajectory_time: Optional[float]
    display_data: bool
    debug_leader: bool
    debug_limits: bool
    debug_state: bool
    debug_interval: float
    debug_raw: bool
    debug_log_file: Optional[str]


class DisplayManager:
    def __init__(self, fps: int, window_prefix: str = "so101: ") -> None:
        self._enabled = True
        self._window_prefix = window_prefix
        self._interval = 1.0 / max(1, min(DISPLAY_MAX_FPS, fps))
        self._next_time = 0.0
        try:
            import cv2  # pylint: disable=import-error
        except Exception as exc:
            print(f"[DISPLAY] Disabled (OpenCV unavailable): {exc}")
            self._enabled = False
            self._cv2 = None
            return
        self._cv2 = cv2

    def due(self) -> bool:
        if not self._enabled:
            return False
        return time.monotonic() >= self._next_time

    def update(self, images: dict[str, Optional[np.ndarray]]) -> bool:
        if not self._enabled or self._cv2 is None:
            return True
        now = time.monotonic()
        if now < self._next_time:
            return True
        self._next_time = now + self._interval
        try:
            for name, img in images.items():
                if img is None:
                    continue
                if img.ndim == 3 and img.shape[2] == 3:
                    bgr = img[:, :, ::-1]
                else:
                    bgr = img
                self._cv2.imshow(f"{self._window_prefix}{name}", bgr)
            key = self._cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                return False
            return True
        except Exception as exc:
            print(f"[DISPLAY] Error, disabling previews: {exc}")
            self._enabled = False
            return True

    def close(self) -> None:
        if not self._enabled or self._cv2 is None:
            return
        try:
            self._cv2.destroyAllWindows()
        except Exception:
            pass


class SoArm101Recorder:
    def __init__(self, config: RecorderConfig, client) -> None:
        self.config = config
        self.client = client
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._have_joint_state = False
        self._have_action = False
        self._have_images = {name: False for name in config.cameras}
        self._clock_time = None

        self._joint_positions: dict[str, Optional[float]] = {name: None for name in JOINT_ORDER}
        self._action_positions: dict[str, Optional[float]] = {name: None for name in JOINT_ORDER}
        self._images: dict[str, Optional[np.ndarray]] = {name: None for name in config.cameras}
        self._image_shapes: dict[str, tuple[int, int, int]] = {}

        self._dataset: Optional[LeRobotDataset] = None
        self._episode_start = None
        self._episode_active = False  # Starts paused, user must press 'r' to begin
        self._episode_index = 0
        self._finalized = False
        self._stdin_settings = None
        if config.display_data and not config.cameras:
            print("[DISPLAY] No cameras enabled; previews disabled.")
        self._display = (
            DisplayManager(config.fps) if config.display_data and config.cameras else None
        )

        self._setup_topics()

    def _safe_episode_buffer(self) -> Optional[dict]:
        if self._dataset is None:
            return None
        buffer = self._dataset.episode_buffer
        # Check for valid buffer - accept either 'size' key or 'frame_index' key
        if isinstance(buffer, dict) and ("size" in buffer or "frame_index" in buffer or "task" in buffer):
            return buffer
        try:
            self._dataset.episode_buffer = self._dataset.create_episode_buffer()
            return self._dataset.episode_buffer
        except Exception:
            return None

    def _episode_buffer_size(self) -> int:
        buffer = self._safe_episode_buffer()
        if not buffer:
            return 0
        # Try 'size' key first (newer lerobot)
        size = buffer.get("size")
        if isinstance(size, (int, np.integer)):
            return int(size)
        # Fall back to frame_index list length (older lerobot)
        frame_index = buffer.get("frame_index")
        if isinstance(frame_index, list):
            return len(frame_index)
        return 0

    def _get_episode_count(self) -> int:
        if self._dataset is None:
            return 0
        try:
            return int(self._dataset.meta.total_episodes)
        except Exception:
            return 0

    def _save_episode(self, episode_index: int) -> int:
        if self._dataset is None:
            return episode_index
        frames = self._episode_buffer_size()
        if frames <= 0:
            print("Skipping empty episode.")
            self._reset_episode_buffer()
            return episode_index
        self._dataset.save_episode(parallel_encoding=False)
        new_total = self._get_episode_count()
        if new_total < episode_index + 1:
            new_total = episode_index + 1
        print(f"[s] Episode {new_total} saved ({frames} frames)")
        return new_total

    def _reset_episode_buffer(self) -> None:
        if self._dataset is None:
            return
        # Don't call clear_episode_buffer() here - it deletes image files
        # that save_episode() needs for stats computation and video encoding.
        # Just create a fresh buffer directly.
        try:
            self._dataset.episode_buffer = self._dataset.create_episode_buffer()
        except Exception:
            self._dataset.episode_buffer = None
        # Ensure 'size' key exists - some lerobot versions require it but don't create it
        if isinstance(self._dataset.episode_buffer, dict) and "size" not in self._dataset.episode_buffer:
            self._dataset.episode_buffer["size"] = 0

    def _start_episode(self) -> bool:
        """Start recording a new episode. Returns True if started successfully."""
        if self._episode_active:
            print("Already recording. Use 's' to save or 'd' to discard first.")
            return False
        if not self._have_joint_state:
            print("Cannot start: no joint_states publisher")
            return False
        if self.config.cameras and not all(self._have_images.values()):
            missing = [k for k, v in self._have_images.items() if not v]
            print(f"Cannot start: missing camera publishers: {missing}")
            return False
        # Ensure fresh episode buffer before starting
        self._reset_episode_buffer()
        self._episode_active = True
        self._episode_start = time.monotonic()
        self._episode_time_warned = False
        self._episode_index = self._get_episode_count()
        print(f"[r] Episode {self._episode_index + 1} started")
        return True

    def _save_episode_and_reset(self) -> bool:
        """Save the current episode and reset for the next one. Returns True if saved."""
        if not self._episode_active:
            print("No active episode. Press 'r' to start.")
            return False
        frames = self._episode_buffer_size()
        if frames == 0:
            print("No frames recorded in current episode. Nothing to save.")
            self._episode_active = False
            return False
        self._save_episode(self._episode_index)
        self._episode_active = False
        self._episode_index = self._get_episode_count()
        return True

    def _discard_episode(self) -> None:
        """Discard the current episode without saving."""
        if not self._episode_active:
            print("No active episode to discard.")
            return
        frames = self._episode_buffer_size()
        print(f"[d] Episode {self._episode_index + 1} discarded ({frames} frames)")
        self._reset_episode_buffer()
        self._episode_active = False

    def _finalize_dataset(self) -> None:
        """Save any pending episode, consolidate dataset, and quit."""
        # Save pending episode if frames exist
        if self._episode_active:
            pending = self._episode_buffer_size()
            if pending > 0:
                print(f"Saving pending episode ({pending} frames)...")
                self._save_episode(self._episode_index)
            self._episode_active = False

        # Consolidate dataset
        if self._dataset is not None and not self._finalized:
            print("Consolidating dataset...")
            try:
                self._dataset.consolidate(run_compute_stats=True)
                self._finalized = True
            except Exception as exc:
                print(f"Warning: consolidation failed: {exc}")

        # Print summary
        total_eps = self._get_episode_count()
        print(f"[f] Dataset finalized: {total_eps} episodes")
        print(f"Saved to: {self.config.root}")
        self._stop.set()

    def _quit_without_finalize(self) -> None:
        """Quit immediately without consolidating the dataset."""
        pending = self._episode_buffer_size()
        if pending > 0:
            print(f"Warning: {pending} frames in buffer will be lost")
        print("Quitting without finalize.")
        print("To consolidate later, load dataset with LeRobotDataset() and call .consolidate()")
        self._stop.set()

    def _setup_topics(self) -> None:
        TopicClass = Topic if Topic else roslibpy.Topic
        self._joint_sub = TopicClass(self.client, self.config.state_topic, "sensor_msgs/JointState")
        self._joint_sub.subscribe(self._on_joint_state)

        self._arm_action_sub = TopicClass(
            self.client, self.config.arm_action_topic, "trajectory_msgs/JointTrajectory"
        )
        self._arm_action_sub.subscribe(self._on_action)

        self._gripper_action_sub = TopicClass(
            self.client, self.config.gripper_action_topic, "trajectory_msgs/JointTrajectory"
        )
        self._gripper_action_sub.subscribe(self._on_action)

        self._clock_sub = TopicClass(self.client, self.config.clock_topic, "rosgraph_msgs/Clock")
        self._clock_sub.subscribe(self._on_clock)

        self._image_subs = {}
        for name, topic in self.config.camera_topics.items():
            TopicClass = Topic if Topic else roslibpy.Topic
            sub = TopicClass(self.client, topic, "sensor_msgs/Image")
            sub.subscribe(lambda msg, cam=name: self._on_image(cam, msg))
            self._image_subs[name] = sub

    def _on_clock(self, msg: dict) -> None:
        clock = msg.get("clock") or {}
        sec = clock.get("sec")
        nanosec = clock.get("nanosec")
        if sec is None or nanosec is None:
            return
        with self._lock:
            self._clock_time = float(sec) + float(nanosec) * 1e-9

    def _on_joint_state(self, msg: dict) -> None:
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
            if not self._have_action:
                for name in JOINT_ORDER:
                    if self._joint_positions[name] is not None:
                        self._action_positions[name] = self._joint_positions[name]

    def _on_action(self, msg: dict) -> None:
        joint_names = msg.get("joint_names", []) or []
        points = msg.get("points", []) or []
        if not joint_names or not points:
            return
        positions = points[-1].get("positions", []) if points else []
        if not positions:
            return
        with self._lock:
            self._have_action = True
            for name, pos in zip(joint_names, positions):
                canonical = _normalize_joint_name(str(name))
                if not canonical:
                    continue
                try:
                    self._action_positions[canonical] = float(pos)
                except (TypeError, ValueError):
                    continue

    def _on_image(self, camera: str, msg: dict) -> None:
        img = _decode_ros_image(msg)
        if img is None:
            return
        with self._lock:
            self._images[camera] = img
            self._image_shapes[camera] = img.shape
            self._have_images[camera] = True

    def _wait_for_ready(self, timeout: float = 15.0) -> None:
        start = time.time()
        while time.time() - start < timeout:
            with self._lock:
                joints_ok = self._have_joint_state
                images_ok = all(self._have_images.values()) if self.config.cameras else True
                actions_ok = self._have_action if self.config.wait_for_action else True
            if joints_ok and images_ok:
                if actions_ok:
                    return
            time.sleep(0.1)
        missing = []
        with self._lock:
            if not self._have_joint_state:
                missing.append("joint_states")
            for cam, ok in self._have_images.items():
                if not ok:
                    missing.append(f"image:{cam}")
            if self.config.wait_for_action and not self._have_action:
                missing.append("action_commands")
        hints = []
        if "joint_states" in missing:
            hints.append("use --state-topic to point at the joint state topic")
        if "action_commands" in missing:
            hints.append("start teleop or disable with --no-wait-for-action")
        if any(item.startswith("image:") for item in missing):
            hints.append(
                "use --no-images, --cameras, or --robot.cameras to match available cameras"
            )
            hints.append("override topics with --ros.camera_topics")
        hint_text = f" Hints: {'; '.join(hints)}" if hints else ""
        raise TimeoutError(f"Timeout waiting for initial data: {', '.join(missing)}.{hint_text}")

    def _create_dataset(self) -> None:
        obs_hw_features: dict[str, Any] = {name: float for name in JOINT_ORDER}
        for cam in self.config.cameras:
            shape = self._image_shapes.get(cam)
            if shape:
                obs_hw_features[cam] = shape

        obs_features = hw_to_dataset_features(obs_hw_features, OBS_STR, use_video=self.config.use_videos)
        act_features = hw_to_dataset_features(
            {name: float for name in JOINT_ORDER}, ACTION, use_video=self.config.use_videos
        )
        dataset_features = {**obs_features, **act_features}
        expected_features = {**dataset_features, **DEFAULT_FEATURES}

        if self.config.resume:
            # Try with vcodec first, fall back without for older lerobot versions
            try:
                self._dataset = LeRobotDataset(
                    self.config.repo_id,
                    root=self.config.root,
                    batch_encoding_size=1,
                    vcodec=self.config.vcodec,
                )
            except TypeError:
                self._dataset = LeRobotDataset(
                    self.config.repo_id,
                    root=self.config.root,
                    batch_encoding_size=1,
                )
            self._assert_resume_compatible(expected_features)
        else:
            # Try with vcodec first, fall back without for older lerobot versions
            try:
                self._dataset = LeRobotDataset.create(
                    self.config.repo_id,
                    fps=self.config.fps,
                    features=dataset_features,
                    root=self.config.root,
                    robot_type=self.config.robot_type,
                    use_videos=self.config.use_videos,
                    batch_encoding_size=1,
                    vcodec=self.config.vcodec,
                )
            except TypeError:
                self._dataset = LeRobotDataset.create(
                    self.config.repo_id,
                    fps=self.config.fps,
                    features=dataset_features,
                    root=self.config.root,
                    robot_type=self.config.robot_type,
                    use_videos=self.config.use_videos,
                    batch_encoding_size=1,
                )
            # Ensure tasks metadata exists so tooling can load datasets before any episodes are saved.
            self._dataset.meta.save_episode_tasks([self.config.task])

    def _assert_resume_compatible(self, expected_features: dict[str, dict]) -> None:
        if self._dataset is None:
            return
        actual_features = self._dataset.features
        if actual_features != expected_features:
            raise ValueError(_format_feature_mismatch(expected_features, actual_features))
        dataset_fps = int(self._dataset.meta.fps)
        if dataset_fps != int(self.config.fps):
            raise ValueError(
                f"Resume fps mismatch: dataset fps {dataset_fps} != --fps {self.config.fps}."
            )
        robot_type = self._dataset.meta.info.get("robot_type")
        if robot_type and robot_type != self.config.robot_type:
            raise ValueError(
                "Resume robot_type mismatch: "
                f"dataset robot_type '{robot_type}' != '{self.config.robot_type}'."
            )

    def _get_timestamp(self) -> float:
        with self._lock:
            if self._clock_time is not None:
                return self._clock_time
        return time.time()

    def _build_frame(self) -> Optional[dict]:
        with self._lock:
            if not self._have_joint_state:
                return None
            if self.config.cameras and not all(self._have_images.values()):
                return None
            if self.config.wait_for_action and not self._have_action:
                return None

            joints = {name: self._joint_positions[name] for name in JOINT_ORDER}
            actions = {name: self._action_positions[name] for name in JOINT_ORDER}
            images = {name: self._images[name] for name in self.config.cameras}

        if any(value is None for value in joints.values()):
            return None
        if any(value is None for value in actions.values()):
            return None
        if self.config.cameras and any(value is None for value in images.values()):
            return None

        obs_values = {**joints}
        for name, img in images.items():
            obs_values[name] = np.array(img, copy=True)

        action_values = {**actions}

        if self._dataset is None:
            return None

        frame = {}
        frame.update(build_dataset_frame(self._dataset.features, obs_values, prefix=OBS_STR))
        frame.update(build_dataset_frame(self._dataset.features, action_values, prefix=ACTION))
        frame["task"] = self.config.task
        # Note: DEFAULT_FEATURES (timestamp, frame_index, episode_index, etc.) are auto-populated
        # by LeRobotDataset and should NOT be included in the frame.
        return frame

    def _get_display_images(self) -> dict[str, Optional[np.ndarray]]:
        with self._lock:
            return {
                name: np.array(img, copy=True) if img is not None else None
                for name, img in self._images.items()
            }

    def _setup_keyboard(self, teleop=None) -> None:
        if not sys.stdin.isatty():
            return
        import atexit
        import termios
        import tty

        self._stdin_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin)

        # Register atexit handler to ensure terminal is ALWAYS restored
        def _restore_on_exit():
            if self._stdin_settings is not None:
                try:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._stdin_settings)
                except Exception:
                    pass

        atexit.register(_restore_on_exit)

        def _listen():
            while not self._stop.is_set():
                ch = sys.stdin.read(1)
                if not ch:
                    continue
                key = ch.lower()
                # Process recording keys first, before teleop
                if key == "r":
                    self._start_episode()
                elif key == "s":
                    self._save_episode_and_reset()
                elif key == "d":
                    self._discard_episode()
                elif key == "f":
                    self._finalize_dataset()
                    return
                elif key == "q" or ch == "\x03":
                    self._quit_without_finalize()
                    return
                elif teleop is not None and getattr(teleop, "input_device", "") == "keyboard":
                    try:
                        keep_running = teleop._handle_key(ch)
                    except Exception as exc:
                        print(f"\n[teleop] Error handling key: {exc}")
                        keep_running = True
                    if keep_running is False:
                        self._stop.set()
                        return

        thread = threading.Thread(target=_listen, daemon=True)
        thread.start()

    def _restore_keyboard(self) -> None:
        if self._stdin_settings is None:
            return
        import termios

        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._stdin_settings)
        except Exception:
            pass
        self._stdin_settings = None


    def record(self, teleop=None) -> None:
        self._setup_keyboard(teleop)
        if self.config.wait_for_action:
            print("Waiting for action commands to start recording...")
        self._wait_for_ready()
        self._create_dataset()
        if self._dataset is None:
            raise RuntimeError(
                "Failed to create dataset. If ffmpeg is missing, try --no-videos."
            )
        try:
            fps = max(1, int(self.config.fps))
            interval = 1.0 / fps
            self._episode_index = self._get_episode_count()
            next_frame_time = time.monotonic()

            # Print help message
            if self._episode_index > 0:
                print(f"Resuming dataset with {self._episode_index} existing episodes.")
            print("Recording ready.")
            print("  r = start episode")
            print("  s = save episode")
            print("  d = discard episode")
            print("  f = finalize dataset and quit")
            print("  q = quit without finalize")
            if self._display:
                print("Display: q/Esc in preview window to quit")

            self._episode_time_warned = False
            try:
                while not self._stop.is_set():
                    now = time.monotonic()
                    if now < next_frame_time:
                        time.sleep(min(0.01, next_frame_time - now))
                        continue

                    next_frame_time += interval

                    # Display update (independent of recording state)
                    if self._display and self._display.due():
                        images = self._get_display_images()
                        if not self._display.update(images):
                            self._quit_without_finalize()
                            break

                    # Frame collection (only when episode is active)
                    if not self._episode_active:
                        continue

                    frame = self._build_frame()
                    if frame is not None:
                        # Ensure episode buffer is valid before adding frame
                        buf = self._dataset.episode_buffer
                        if buf is None or not isinstance(buf, dict) or "size" not in buf:
                            self._reset_episode_buffer()
                        self._dataset.add_frame(frame)

                    # Episode time limit check (prompts user, doesn't auto-save)
                    if self.config.episode_seconds > 0 and self._episode_start:
                        elapsed = time.monotonic() - self._episode_start
                        if elapsed >= self.config.episode_seconds and not self._episode_time_warned:
                            print("Time limit reached. Press 's' to save or 'd' to discard.")
                            self._episode_time_warned = True

            except KeyboardInterrupt:
                self._quit_without_finalize()

        finally:
            # Only call finalize() if not already finalized via 'f' key
            if self._dataset is not None and not self._finalized:
                self._dataset.finalize()
            self._restore_keyboard()
            if self._display:
                self._display.close()
            self._shutdown_topics()

    def _shutdown_topics(self) -> None:
        try:
            self._joint_sub.unsubscribe()
            self._arm_action_sub.unsubscribe()
            self._gripper_action_sub.unsubscribe()
            self._clock_sub.unsubscribe()
            for sub in self._image_subs.values():
                sub.unsubscribe()
        except Exception:
            pass


def _resolve_rosbridge_url(host: str, port: Optional[int], url: str) -> str:
    if url:
        return url
    env_url = os.getenv("ROSBRIDGE_URL")
    if env_url:
        return env_url
    env_host = os.getenv("ROS_HOST")
    env_port = os.getenv("ROS_PORT") or os.getenv("ROSBRIDGE_PORT")
    env_port_val = None
    if env_port:
        try:
            env_port_val = int(env_port)
        except ValueError:
            env_port_val = None
    if host:
        port_val = port if port is not None else 9091
        return f"ws://{host}:{port_val}"
    if env_host:
        port_val = port if port is not None else env_port_val if env_port_val is not None else 9091
        return f"ws://{env_host}:{port_val}"
    return ""


def _connect_direct_rosbridge(rosbridge_url: str, timeout: float = 10.0):
    parsed = urlparse(rosbridge_url)
    host = parsed.hostname or "127.0.0.1"
    port = parsed.port or 9091

    print(f"[CONNECT] Connecting to rosbridge at {rosbridge_url}...")
    client = roslibpy.Ros(host=host, port=port)
    client.run(timeout=timeout)
    if not client.is_connected:
        raise ConnectionError(f"Failed to connect to rosbridge at {rosbridge_url}")
    print("[CONNECT] Connected to rosbridge")
    return client


def _endpoint_from_rosbridge_url(rosbridge_url: str) -> tuple[str, int]:
    if not rosbridge_url:
        return "", 0
    parsed = urlparse(rosbridge_url)
    host = parsed.hostname or ""
    port = parsed.port or 0
    return host, port


def _load_teleop_module():
    try:
        import teleop_so_arm101 as teleop_module
    except ImportError as exc:
        raise RuntimeError("teleop_so_arm101.py is required for --input modes.") from exc
    return teleop_module


def _init_teleop(config: RecorderConfig, client):
    if config.input_device == "none":
        return None, None, None

    teleop = _load_teleop_module()
    topic_class = Topic if Topic else roslibpy.Topic
    leader_bridge = None
    follower_bridge = None

    if config.input_device == "leader":
        teleop._ensure_so101_bridge()
        calib_args = argparse.Namespace(
            calibration_dir=config.leader_calibration_dir,
            leader_id=config.leader_id,
            calibration_file=config.leader_calibration_file,
        )
        leader_id, calibration_dir, calibration_file = teleop._resolve_leader_calibration(
            calib_args
        )
        print(f"Initializing leader bridge on {config.leader_port}...")
        print(f"Using calibration file: {calibration_file}")
        leader_bridge = teleop._init_leader_bridge(
            client=client,
            topic_class=topic_class,
            port=config.leader_port,
            robot_id=leader_id,
            calibration_dir=calibration_dir,
            rate_hz=config.leader_rate,
            debug=config.debug_leader,
            debug_interval=config.debug_interval,
            debug_raw=config.debug_raw,
            limit_map=teleop.URDF_LIMITS,
            map_to_limits=True,
            clamp_to_limits=True,
        )

    if config.follower_port:
        teleop._ensure_so101_bridge()
        follower_calibration_dir = (
            os.path.expanduser(config.follower_calibration_dir)
            if config.follower_calibration_dir
            else None
        )
        print(f"Initializing follower arm bridge on {config.follower_port}...")
        follower_bridge = teleop._init_follower_bridge(
            client=client,
            topic_class=topic_class,
            port=config.follower_port,
            robot_id=config.follower_id,
            calibration_dir=follower_calibration_dir,
        )

    host, port = _endpoint_from_rosbridge_url(config.rosbridge_url)
    teleop_controller = teleop.ArmKeyboardTeleop(
        client=client,
        host=host,
        port=port,
        input_device=config.input_device,
        leader_bridge=leader_bridge,
        follower_bridge=follower_bridge,
        trajectory_time=config.trajectory_time,
        debug_leader=config.debug_leader,
        debug_limits=config.debug_limits,
        debug_state=config.debug_state,
        debug_interval=config.debug_interval,
        debug_log_file=config.debug_log_file,
    )
    return teleop_controller, leader_bridge, follower_bridge


def _check_ffmpeg(use_videos: bool) -> Optional[str]:
    if not use_videos:
        return None
    if shutil.which("ffmpeg"):
        return None
    return "ffmpeg not found in PATH (use --no-videos or --dataset.video=false)"


def _rosapi_topics(client) -> Optional[dict[str, str]]:
    ServiceClass = Service if Service else roslibpy.Service
    try:
        srv = ServiceClass(client, "/rosapi/topics", "rosapi/Topics")
        response = srv.call({})
    except Exception:
        return None
    topics = response.get("topics", []) or []
    types = response.get("types", []) or []
    return {topic: types[idx] if idx < len(types) else "" for idx, topic in enumerate(topics)}


def _print_preflight(recorder: "SoArm101Recorder", timeout: float) -> bool:
    print(f"[CHECK] Waiting for topics (timeout {timeout:.1f}s)...")
    try:
        recorder._wait_for_ready(timeout=timeout)
    except TimeoutError as exc:
        print(f"[CHECK] {exc}")
        return False

    with recorder._lock:
        joints_ok = recorder._have_joint_state
        images_ok = dict(recorder._have_images)
        shapes = dict(recorder._image_shapes)
        actions_ok = recorder._have_action

    print(f"[CHECK] Joint states: {'ok' if joints_ok else 'missing'}")
    if recorder.config.wait_for_action:
        print(f"[CHECK] Action commands: {'ok' if actions_ok else 'missing'}")
    for cam, ok in images_ok.items():
        shape = shapes.get(cam)
        shape_text = f"{shape}" if shape else "unknown"
        print(f"[CHECK] Camera {cam}: {'ok' if ok else 'missing'} (shape {shape_text})")
    return True


def parse_args() -> RecorderConfig:
    parser = argparse.ArgumentParser(description="Record SO-ARM101 data into a LeRobot dataset")
    parser.add_argument("--repo-id", type=str, default="", help="LeRobot dataset repo_id")
    parser.add_argument("--root", type=str, default="", help="Dataset root directory")
    parser.add_argument("--fps", type=int, default=5, help="Dataset FPS (default: 5)")
    parser.add_argument("--episodes", type=int, default=0, help="Number of episodes (0 = unlimited)")
    parser.add_argument(
        "--episode-seconds",
        type=float,
        default=0.0,
        help="Episode length in seconds (0 = manual)",
    )
    parser.add_argument("--reset-seconds", type=float, default=2.0, help="Pause between episodes")
    parser.add_argument("--task", type=str, default="teleop", help="Task label")
    parser.add_argument("--no-videos", action="store_true", help="Store images as PNGs instead of videos")
    parser.add_argument("--vcodec", type=str, default="h264", help="Video codec")
    parser.add_argument("--dataset.repo_id", dest="dataset_repo_id", type=str, default=None)
    parser.add_argument("--dataset.root", dest="dataset_root", type=str, default=None)
    parser.add_argument("--dataset.fps", dest="dataset_fps", type=int, default=None)
    parser.add_argument("--dataset.num_episodes", dest="dataset_num_episodes", type=int, default=None)
    parser.add_argument(
        "--dataset.episode_time_s",
        dest="dataset_episode_time_s",
        type=float,
        default=None,
    )
    parser.add_argument(
        "--dataset.reset_time_s",
        dest="dataset_reset_time_s",
        type=float,
        default=None,
    )
    parser.add_argument("--dataset.single_task", dest="dataset_single_task", type=str, default=None)
    parser.add_argument(
        "--dataset.video",
        dest="dataset_video",
        nargs="?",
        const="true",
        default=None,
        help="Store images as videos (true/false)",
    )
    parser.add_argument("--dataset.vcodec", dest="dataset_vcodec", type=str, default=None)
    parser.add_argument(
        "--display_data",
        "--display-data",
        dest="display_data",
        nargs="?",
        const="true",
        default=None,
        help="Preview incoming camera feeds",
    )
    parser.add_argument(
        "--state-topic",
        "--ros.state_topic",
        dest="state_topic",
        type=str,
        default="/joint_states",
    )
    parser.add_argument(
        "--arm-action-topic",
        "--ros.arm_action_topic",
        dest="arm_action_topic",
        type=str,
        default="/arm_controller/joint_trajectory",
    )
    parser.add_argument(
        "--gripper-action-topic",
        "--ros.gripper_action_topic",
        dest="gripper_action_topic",
        type=str,
        default="/gripper_controller/joint_trajectory",
    )
    parser.add_argument(
        "--clock-topic",
        "--ros.clock_topic",
        dest="clock_topic",
        type=str,
        default="/clock",
    )
    parser.add_argument(
        "--wait-for-action",
        dest="wait_for_action",
        action="store_true",
        default=None,
        help="Wait for action commands before recording",
    )
    parser.add_argument(
        "--no-wait-for-action",
        dest="wait_for_action",
        action="store_false",
        help="Do not wait for action commands before recording",
    )
    parser.add_argument(
        "--cameras",
        type=str,
        default="wrist,agent,side",
        help="Comma-separated camera list (wrist,agent,side,front)",
    )
    parser.add_argument("--no-images", action="store_true", help="Disable image recording")
    parser.add_argument(
        "--camera-topics",
        "--ros.camera_topics",
        dest="ros_camera_topics",
        type=str,
        default=None,
        help="YAML mapping of camera name to ROS image topic",
    )
    parser.add_argument(
        "--wrist-topic", type=str, default=DEFAULT_CAMERA_TOPICS["wrist"]
    )
    parser.add_argument(
        "--agent-topic", type=str, default=DEFAULT_CAMERA_TOPICS["agent"]
    )
    parser.add_argument(
        "--side-topic", type=str, default=DEFAULT_CAMERA_TOPICS["side"]
    )
    parser.add_argument("--resume", action="store_true", help="Resume an existing dataset")
    parser.add_argument("--host", "--ros.host", dest="host", type=str, default="")
    parser.add_argument("--port", "--ros.port", dest="port", type=int, default=None)
    parser.add_argument(
        "--rosbridge-url",
        "--rosbridge.url",
        dest="rosbridge_url",
        type=str,
        default="",
        help="Rosbridge URL override",
    )
    parser.add_argument(
        "--robot.type",
        dest="robot_type",
        type=str,
        default=None,
        help="Lerobot-style robot type (e.g. so101_follower)",
    )
    parser.add_argument(
        "--robot.port",
        dest="robot_port",
        type=str,
        default=None,
        help="Follower serial port (optional)",
    )
    parser.add_argument(
        "--robot.id",
        dest="robot_id",
        type=str,
        default=None,
        help="Follower ID (maps to --follower-id)",
    )
    parser.add_argument(
        "--robot.cameras",
        dest="robot_cameras",
        type=str,
        default=None,
        help="YAML camera config dict; keys become dataset camera names",
    )
    parser.add_argument(
        "--input",
        "--input-device",
        dest="input_device",
        type=str,
        default="none",
        choices=["none", "keyboard", "leader"],
        help="Enable teleop input within the recorder (keyboard or leader)",
    )
    parser.add_argument(
        "--leader-port",
        type=str,
        default=os.getenv("SO101_LEADER_PORT"),
        help="Leader USB serial port (env: SO101_LEADER_PORT)",
    )
    parser.add_argument(
        "--teleop.type",
        dest="teleop_type",
        type=str,
        default=None,
        help="Lerobot-style teleop type (e.g. so101_leader, keyboard)",
    )
    parser.add_argument(
        "--teleop.port",
        dest="teleop_port",
        type=str,
        default=None,
        help="Lerobot-style teleop port (maps to --leader-port)",
    )
    parser.add_argument(
        "--teleop.id",
        dest="teleop_id",
        type=str,
        default=None,
        help="Lerobot-style teleop id (maps to --leader-id)",
    )
    parser.add_argument(
        "--leader-id",
        type=str,
        default=DEFAULT_LEADER_ID,
        help="Leader ID for calibration",
    )
    parser.add_argument(
        "--calibration-dir",
        dest="leader_calibration_dir",
        type=str,
        default=DEFAULT_LEADER_CAL_DIR,
        help="Path to leader calibration directory",
    )
    parser.add_argument(
        "--calibration-file",
        dest="leader_calibration_file",
        type=str,
        default=None,
        help="Path to a specific leader calibration JSON file",
    )
    parser.add_argument(
        "--leader-rate",
        type=float,
        default=100.0,
        help="Leader state publish rate in Hz (higher = lower latency, more CPU)",
    )
    parser.add_argument(
        "--trajectory-time",
        type=float,
        default=None,
        help="Trajectory time_from_start in seconds (defaults: keyboard=0.2, leader=0.02)",
    )
    parser.add_argument(
        "--follower-port",
        type=str,
        default=None,
        help="Robot USB serial port for follower output (optional)",
    )
    parser.add_argument(
        "--follower-id",
        type=str,
        default=DEFAULT_FOLLOWER_ID,
        help="Follower ID for calibration",
    )
    parser.add_argument(
        "--follower-calibration-dir",
        type=str,
        default=DEFAULT_FOLLOWER_CAL_DIR,
        help="Path to follower calibration directory",
    )
    parser.add_argument(
        "--debug-leader",
        action="store_true",
        help="Log leader input values (normalized, radians, raw if enabled)",
    )
    parser.add_argument(
        "--debug-raw",
        action="store_true",
        help="Read raw servo ticks for leader debug output (slower)",
    )
    parser.add_argument(
        "--debug-limits",
        action="store_true",
        help="Warn when leader values exceed URDF limits",
    )
    parser.add_argument(
        "--debug-state",
        action="store_true",
        help="Log sim joint states vs latest commanded targets",
    )
    parser.add_argument(
        "--debug-all",
        action="store_true",
        help="Enable all debug logs (includes raw servo ticks)",
    )
    parser.add_argument(
        "--debug-interval",
        type=float,
        default=1.0,
        help="Seconds between debug logs",
    )
    parser.add_argument(
        "--debug-log-file",
        type=str,
        default=None,
        help="Append debug logs to a file",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Preflight check: verify topics and dependencies, then exit",
    )
    parser.add_argument(
        "--check-timeout",
        type=float,
        default=15.0,
        help="Seconds to wait for initial data during --check",
    )

    args = parser.parse_args()

    if args.display_data is not None:
        try:
            args.display_data = _coerce_bool(args.display_data, default=False)
        except ValueError as exc:
            parser.error(str(exc))
    else:
        args.display_data = False

    dataset_video = None
    if args.dataset_video is not None:
        try:
            dataset_video = _coerce_bool(args.dataset_video, default=True)
        except ValueError as exc:
            parser.error(str(exc))

    if args.debug_all:
        args.debug_leader = True
        args.debug_limits = True
        args.debug_state = True
        args.debug_raw = True

    input_device = args.input_device
    if args.teleop_type:
        teleop_type = args.teleop_type.strip().lower()
        if "leader" in teleop_type:
            input_device = "leader"
        elif "keyboard" in teleop_type:
            input_device = "keyboard"
        elif teleop_type in {"none", "off", "false", "no"}:
            input_device = "none"
        else:
            parser.error("--teleop.type must be leader, keyboard, or none for this recorder.")

    leader_port = args.leader_port or args.teleop_port
    if input_device == "leader" and not leader_port:
        parser.error("--input leader requires --leader-port or --teleop.port.")

    leader_id = args.leader_id
    if args.teleop_id:
        leader_id = args.teleop_id

    follower_port = args.follower_port or args.robot_port
    follower_id = args.follower_id
    if args.robot_id:
        follower_id = args.robot_id

    timestamp = _now_ts()
    repo_id = args.dataset_repo_id or args.repo_id or f"local/so_arm101_sim_{timestamp}"
    root_value = args.dataset_root or args.root
    root = os.path.expanduser(root_value) if root_value else os.path.join(
        os.getcwd(), "datasets", f"so_arm101_sim_{timestamp}"
    )

    if args.resume and not root_value:
        parser.error("--resume requires --root or --dataset.root pointing to an existing dataset.")

    cameras = []
    if not args.no_images:
        try:
            cameras = _parse_camera_names(args.robot_cameras, "--robot.cameras")
        except ValueError as exc:
            parser.error(str(exc))
        if not cameras:
            cameras = _dedupe_keep_order(
                [name.strip() for name in args.cameras.split(",") if name.strip()]
            )

    per_camera_overrides = {}
    if args.wrist_topic != DEFAULT_CAMERA_TOPICS["wrist"]:
        per_camera_overrides["wrist"] = args.wrist_topic
    if args.agent_topic != DEFAULT_CAMERA_TOPICS["agent"]:
        per_camera_overrides["agent"] = args.agent_topic
    if args.side_topic != DEFAULT_CAMERA_TOPICS["side"]:
        per_camera_overrides["side"] = args.side_topic

    camera_topics = {}
    if cameras:
        try:
            overrides = _parse_yaml_mapping(args.ros_camera_topics, "--ros.camera_topics")
        except ValueError as exc:
            parser.error(str(exc))
        overrides = {**per_camera_overrides, **overrides}
        try:
            camera_topics = _resolve_camera_topics(cameras, overrides)
        except ValueError as exc:
            parser.error(str(exc))

    rosbridge_url = _resolve_rosbridge_url(args.host, args.port, args.rosbridge_url)
    wait_for_action = args.wait_for_action
    if wait_for_action is None:
        wait_for_action = input_device != "none"

    fps = args.dataset_fps if args.dataset_fps is not None else args.fps
    episodes = args.dataset_num_episodes if args.dataset_num_episodes is not None else args.episodes
    episode_seconds = (
        args.dataset_episode_time_s
        if args.dataset_episode_time_s is not None
        else args.episode_seconds
    )
    reset_seconds = (
        args.dataset_reset_time_s if args.dataset_reset_time_s is not None else args.reset_seconds
    )
    task = args.dataset_single_task or args.task
    use_videos = dataset_video if dataset_video is not None else not args.no_videos
    vcodec = args.dataset_vcodec or args.vcodec
    robot_type = args.robot_type or "so101_sim"

    return RecorderConfig(
        repo_id=repo_id,
        root=root,
        robot_type=robot_type,
        fps=fps,
        episodes=episodes,
        episode_seconds=episode_seconds,
        reset_seconds=reset_seconds,
        task=task,
        use_videos=use_videos,
        vcodec=vcodec,
        state_topic=args.state_topic,
        arm_action_topic=args.arm_action_topic,
        gripper_action_topic=args.gripper_action_topic,
        clock_topic=args.clock_topic,
        cameras=cameras,
        camera_topics=camera_topics,
        resume=args.resume,
        rosbridge_url=rosbridge_url,
        check=args.check,
        check_timeout=args.check_timeout,
        wait_for_action=wait_for_action,
        input_device=input_device,
        leader_port=leader_port,
        leader_id=leader_id,
        leader_calibration_dir=args.leader_calibration_dir,
        leader_calibration_file=args.leader_calibration_file,
        leader_rate=args.leader_rate,
        follower_port=follower_port,
        follower_id=follower_id,
        follower_calibration_dir=args.follower_calibration_dir,
        trajectory_time=args.trajectory_time,
        display_data=args.display_data,
        debug_leader=args.debug_leader,
        debug_limits=args.debug_limits,
        debug_state=args.debug_state,
        debug_interval=args.debug_interval,
        debug_raw=args.debug_raw,
        debug_log_file=args.debug_log_file,
    )


def main() -> None:
    config = parse_args()

    if not connect_to_robot:
        print("Error: connect_to_robot unavailable. Ensure lib.robotic_utils is installed.")
        sys.exit(1)

    if config.resume:
        info_path = os.path.join(config.root, INFO_PATH)
        if not os.path.exists(info_path):
            print(f"Error: --resume expects an existing LeRobot dataset at {config.root}")
            print(f"Missing metadata: {info_path}")
            sys.exit(1)

    if not config.check and not config.resume and os.path.exists(config.root):
        print(f"Error: dataset root exists: {config.root}")
        print("Use --resume to append or pass a new --root.")
        sys.exit(1)

    if config.rosbridge_url:
        client = _connect_direct_rosbridge(config.rosbridge_url, timeout=10.0)
    else:
        client = connect_to_robot(url=None, timeout=10.0)
    teleop_controller = None
    leader_bridge = None
    follower_bridge = None
    try:
        recorder = SoArm101Recorder(config, client)
        if config.input_device != "none":
            teleop_controller, leader_bridge, follower_bridge = _init_teleop(config, client)
        ffmpeg_error = _check_ffmpeg(config.use_videos)
        if ffmpeg_error:
            print(f"[CHECK] Error: {ffmpeg_error}")
            if not config.check:
                sys.exit(1)
        ros_topics = _rosapi_topics(client)
        if ros_topics is not None:
            print(f"[CHECK] rosapi topics: {len(ros_topics)} available")
            for topic in (
                config.state_topic,
                config.arm_action_topic,
                config.gripper_action_topic,
                config.clock_topic,
                *config.camera_topics.values(),
            ):
                if topic in ros_topics:
                    msg_type = ros_topics.get(topic, "")
                    type_text = f" ({msg_type})" if msg_type else ""
                    print(f"[CHECK] Found topic: {topic}{type_text}")
                else:
                    print(f"[CHECK] Missing topic: {topic}")
        if config.check:
            keyboard_ready = False
            if teleop_controller and getattr(teleop_controller, "input_device", "") == "keyboard":
                recorder._setup_keyboard(teleop_controller)
                keyboard_ready = True
            ok = _print_preflight(recorder, config.check_timeout)
            if keyboard_ready:
                recorder._stop.set()
                recorder._restore_keyboard()
            exit_code = 0 if ok else 2
            if ffmpeg_error:
                exit_code = 2
            sys.exit(exit_code)
        recorder.record(teleop=teleop_controller)
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        if teleop_controller:
            try:
                teleop_controller.shutdown()
            except Exception:
                pass
        if leader_bridge:
            try:
                leader_bridge.shutdown()
            except Exception:
                pass
        if follower_bridge:
            try:
                follower_bridge.shutdown()
            except Exception:
                pass
        try:
            client.terminate()
        except Exception:
            pass


if __name__ == "__main__":
    main()
