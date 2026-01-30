#!/usr/bin/env python3
"""
Record a LeRobot-compatible dataset from the SO-ARM101 simulator over rosbridge.

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
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional

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
    from lib.robotic_utils import connect_to_robot, Topic
except ImportError:
    try:
        from robotic_utils import connect_to_robot, Topic
    except ImportError:
        connect_to_robot = None
        Topic = None


JOINT_NAME_MAP = {
    "1": "shoulder_pan.pos",
    "2": "shoulder_lift.pos",
    "3": "elbow_flex.pos",
    "4": "wrist_flex.pos",
    "5": "wrist_roll.pos",
    "6": "gripper.pos",
    "joint_1": "shoulder_pan.pos",
    "joint_2": "shoulder_lift.pos",
    "joint_3": "elbow_flex.pos",
    "joint_4": "wrist_flex.pos",
    "joint_5": "wrist_roll.pos",
    "joint_6": "gripper.pos",
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


class SoArm101Recorder:
    def __init__(self, config: RecorderConfig, client) -> None:
        self.config = config
        self.client = client
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._next_episode = threading.Event()
        self._have_joint_state = False
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
            if joints_ok and images_ok:
                return
            time.sleep(0.1)
        missing = []
        with self._lock:
            if not self._have_joint_state:
                missing.append("joint_states")
            for cam, ok in self._have_images.items():
                if not ok:
                    missing.append(f"image:{cam}")
        raise TimeoutError(f"Timeout waiting for initial data: {', '.join(missing)}")

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

        if not self._dataset:
            return None

        frame = {}
        frame.update(build_dataset_frame(self._dataset.features, obs_values, prefix=OBS_STR))
        frame.update(build_dataset_frame(self._dataset.features, action_values, prefix=ACTION))
        frame["task"] = self.config.task
        frame["timestamp"] = self._get_timestamp()
        return frame

    def _setup_keyboard(self) -> None:
        if not sys.stdin.isatty():
            return
        import termios
        import tty

        self._stdin_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin)

        def _listen():
            while not self._stop.is_set():
                ch = sys.stdin.read(1)
                if ch.lower() == "n":
                    self._next_episode.set()
                elif ch.lower() == "q":
                    self._stop.set()
                elif ch.lower() == "p":
                    self._recording = not self._recording

        thread = threading.Thread(target=_listen, daemon=True)
        thread.start()

    def _restore_keyboard(self) -> None:
        if self._stdin_settings is None:
            return
        import termios

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._stdin_settings)
        self._stdin_settings = None

    def record(self) -> None:
        self._wait_for_ready()
        self._create_dataset()
        if not self._dataset:
            raise RuntimeError("Failed to create dataset.")

        self._setup_keyboard()
        try:
            fps = max(1, int(self.config.fps))
            interval = 1.0 / fps
            episode_index = 0
            next_frame_time = time.monotonic()

            self._episode_start = time.monotonic()
            print("Recording started.")
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
            if self._dataset:
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
    if host:
        port_val = port if port is not None else 9091
        return f"ws://{host}:{port_val}"
    return ""


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

    args = parser.parse_args()

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
    )


def main() -> None:
    config = parse_args()

    if not connect_to_robot:
        print("Error: connect_to_robot unavailable. Ensure lib.robotic_utils is installed.")
        sys.exit(1)

    if not config.resume and os.path.exists(config.root):
        print(f"Error: dataset root exists: {config.root}")
        print("Use --resume to append or pass a new --root.")
        sys.exit(1)

    client = connect_to_robot(url=config.rosbridge_url or None, timeout=10.0)
    try:
        recorder = SoArm101Recorder(config, client)
        recorder.record()
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        try:
            client.terminate()
        except Exception:
            pass


if __name__ == "__main__":
    main()
