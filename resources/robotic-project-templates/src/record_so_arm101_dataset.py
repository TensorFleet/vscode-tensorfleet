#!/usr/bin/env python3
"""
Record a LeRobot-compatible dataset from the SO-ARM101 simulator over rosbridge.

Optionally run teleop in the same process (use --input keyboard or --input leader).

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
from typing import Any, Optional
from urllib.parse import urlparse

import numpy as np

try:
    import roslibpy
except ImportError:
    print("Error: roslibpy not installed. Run: pip install roslibpy")
    sys.exit(1)

try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.datasets.pipeline_features import hw_to_dataset_features
    from lerobot.datasets.utils import build_dataset_frame
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


@dataclass
class RecorderConfig:
    repo_id: str
    root: str
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
    debug_leader: bool
    debug_limits: bool
    debug_state: bool
    debug_interval: float
    debug_raw: bool
    debug_log_file: Optional[str]


class SoArm101Recorder:
    def __init__(self, config: RecorderConfig, client) -> None:
        self.config = config
        self.client = client
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._next_episode = threading.Event()
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
        self._recording = True
        self._stdin_settings = None

        self._setup_topics()

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
            for name in JOINT_ORDER:
                if self._action_positions[name] is None and self._joint_positions[name] is not None:
                    self._action_positions[name] = self._joint_positions[name]

    def _on_action(self, msg: dict) -> None:
        joint_names = msg.get("joint_names", []) or []
        points = msg.get("points", []) or []
        if not joint_names or not points:
            return
        positions = points[0].get("positions", []) if points else []
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
            hints.append("use --no-images or --cameras to match available cameras")
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

        if self.config.resume:
            self._dataset = LeRobotDataset(
                self.config.repo_id,
                root=self.config.root,
                batch_encoding_size=1,
                vcodec=self.config.vcodec,
            )
        else:
            self._dataset = LeRobotDataset.create(
                self.config.repo_id,
                fps=self.config.fps,
                features=dataset_features,
                root=self.config.root,
                robot_type="so101_sim",
                use_videos=self.config.use_videos,
                batch_encoding_size=1,
                vcodec=self.config.vcodec,
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
        return frame

    def _setup_keyboard(self, teleop=None) -> None:
        if not sys.stdin.isatty():
            return
        import termios
        import tty

        self._stdin_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin)

        def _listen():
            while not self._stop.is_set():
                ch = sys.stdin.read(1)
                if not ch:
                    continue
                if teleop is not None and getattr(teleop, "input_device", "") == "keyboard":
                    try:
                        keep_running = teleop._handle_key(ch)
                    except Exception as exc:
                        print(f"\n[teleop] Error handling key: {exc}")
                        keep_running = True
                    if keep_running is False:
                        self._stop.set()
                        return
                key = ch.lower()
                if key == "n":
                    self._next_episode.set()
                elif key == "p":
                    self._recording = not self._recording
                elif key == "q" and (
                    teleop is None or getattr(teleop, "input_device", "") != "keyboard"
                ):
                    self._stop.set()

        thread = threading.Thread(target=_listen, daemon=True)
        thread.start()

    def _restore_keyboard(self) -> None:
        if self._stdin_settings is None:
            return
        import termios

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._stdin_settings)
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
            episode_index = 0
            next_frame_time = time.monotonic()

            self._episode_start = time.monotonic()
            print("Recording started.")
            if teleop is not None and getattr(teleop, "input_device", "") == "keyboard":
                print("Recorder keys: n = next episode, p = pause, x/Ctrl-C = quit")
            else:
                print("Keys: n = next episode, p = pause, q = quit")

            while not self._stop.is_set():
                now = time.monotonic()
                if now < next_frame_time:
                    time.sleep(min(0.01, next_frame_time - now))
                    continue

                next_frame_time += interval

                if not self._recording:
                    continue

                frame = self._build_frame()
                if frame is not None:
                    self._dataset.add_frame(frame)

                episode_done = False
                if self.config.episode_seconds > 0:
                    if (time.monotonic() - self._episode_start) >= self.config.episode_seconds:
                        episode_done = True

                if self._next_episode.is_set():
                    episode_done = True
                    self._next_episode.clear()

                if episode_done:
                    if self._dataset.episode_buffer and self._dataset.episode_buffer["size"] > 0:
                        self._dataset.save_episode()
                        episode_index += 1
                        print(f"Saved episode {episode_index}.")
                    else:
                        print("Skipping empty episode.")

                    if self.config.episodes and episode_index >= self.config.episodes:
                        break

                    self._dataset.clear_episode_buffer()
                    if self.config.reset_seconds > 0:
                        time.sleep(self.config.reset_seconds)
                    self._episode_start = time.monotonic()

            if self._dataset.episode_buffer and self._dataset.episode_buffer["size"] > 0:
                self._dataset.save_episode()
                print(f"Saved episode {episode_index + 1}.")
        finally:
            if self._dataset is not None:
                self._dataset.finalize()
            self._restore_keyboard()
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
    return "ffmpeg not found in PATH (use --no-videos to store PNGs instead)"


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
    parser.add_argument("--state-topic", type=str, default="/joint_states")
    parser.add_argument("--arm-action-topic", type=str, default="/arm_controller/joint_trajectory")
    parser.add_argument("--gripper-action-topic", type=str, default="/gripper_controller/joint_trajectory")
    parser.add_argument("--clock-topic", type=str, default="/clock")
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
        help="Comma-separated camera list (wrist,agent,side)",
    )
    parser.add_argument("--no-images", action="store_true", help="Disable image recording")
    parser.add_argument(
        "--wrist-topic", type=str, default="/so_arm101/wrist_camera/image_raw"
    )
    parser.add_argument(
        "--agent-topic", type=str, default="/so_arm101/agent_camera/image_raw"
    )
    parser.add_argument(
        "--side-topic", type=str, default="/so_arm101/side_camera/image_raw"
    )
    parser.add_argument("--resume", action="store_true", help="Resume an existing dataset")
    parser.add_argument("--host", type=str, default="", help="Rosbridge host override")
    parser.add_argument("--port", type=int, default=None, help="Rosbridge port override")
    parser.add_argument("--rosbridge-url", type=str, default="", help="Rosbridge URL override")
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
        default=None,
        help="Robot USB serial port for leader input (required when input=leader)",
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

    if args.debug_all:
        args.debug_leader = True
        args.debug_limits = True
        args.debug_state = True
        args.debug_raw = True

    if args.input_device == "leader" and not args.leader_port:
        parser.error("--input leader requires --leader-port.")

    timestamp = _now_ts()
    repo_id = args.repo_id or f"local/so_arm101_sim_{timestamp}"
    root = args.root or os.path.join(os.getcwd(), "datasets", f"so_arm101_sim_{timestamp}")

    cameras = []
    if not args.no_images:
        for name in args.cameras.split(","):
            name = name.strip()
            if name:
                cameras.append(name)

    camera_topics = {}
    if "wrist" in cameras:
        camera_topics["wrist"] = args.wrist_topic
    if "agent" in cameras:
        camera_topics["agent"] = args.agent_topic
    if "side" in cameras:
        camera_topics["side"] = args.side_topic

    rosbridge_url = _resolve_rosbridge_url(args.host, args.port, args.rosbridge_url)
    wait_for_action = args.wait_for_action
    if wait_for_action is None:
        wait_for_action = args.input_device != "none"

    return RecorderConfig(
        repo_id=repo_id,
        root=root,
        fps=args.fps,
        episodes=args.episodes,
        episode_seconds=args.episode_seconds,
        reset_seconds=args.reset_seconds,
        task=args.task,
        use_videos=not args.no_videos,
        vcodec=args.vcodec,
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
        input_device=args.input_device,
        leader_port=args.leader_port,
        leader_id=args.leader_id,
        leader_calibration_dir=args.leader_calibration_dir,
        leader_calibration_file=args.leader_calibration_file,
        leader_rate=args.leader_rate,
        follower_port=args.follower_port,
        follower_id=args.follower_id,
        follower_calibration_dir=args.follower_calibration_dir,
        trajectory_time=args.trajectory_time,
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
        ffmpeg_warning = _check_ffmpeg(config.use_videos)
        if ffmpeg_warning:
            print(f"[CHECK] Warning: {ffmpeg_warning}")
        ros_topics = _rosapi_topics(client)
        if ros_topics is not None:
            print(f"[CHECK] rosapi topics: {len(ros_topics)} available")
            for topic in (
                config.state_topic,
                config.arm_action_topic,
                config.gripper_action_topic,
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
            sys.exit(0 if ok else 2)
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
