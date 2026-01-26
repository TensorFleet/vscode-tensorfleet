#!/usr/bin/env python3
"""
Teleop for SO-ARM101 simulator over rosbridge.

Inputs:
- keyboard (default): control simulator with keyboard
- leader: drive simulator using a real leader arm

Usage:
    python3 teleop_so_arm101.py --input keyboard
    python3 teleop_so_arm101.py --input leader --leader-port /dev/ttyACM0

Requirements:
    pip install roslibpy websocket-client
    # For leader input:
    pip install lerobot pyserial feetech-servo-sdk

Environment overrides (for proxy connection):
    - TENSORFLEET_BASE_URL, TENSORFLEET_JWT (for proxy connection)
    - ROS_HOST, ROS_PORT, ROSBRIDGE_URL (for direct connection)
"""
import argparse
import os
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import Optional
from urllib.parse import urlparse

# Add parent directory to path for lib imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

DEFAULT_HOME = [0.0, 0.3, -0.5, 0.0, 0.0, 0.4]
URDF_LIMITS = {
    "1": (-1.91986, 1.91986),
    "2": (-1.74533, 1.74533),
    "3": (-1.74533, 1.5708),
    "4": (-1.65806, 1.65806),
    "5": (-2.79253, 2.79253),
    "6": (-0.174533, 1.74533),
}

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

# SO101 hardware bridge (optional, for leader input)
try:
    from lib.so101_bridge import SO101Bridge
except ImportError:
    try:
        from so101_bridge import SO101Bridge
    except ImportError:
        SO101Bridge = None


def resolve_rosbridge_endpoint(arg_host: str, arg_port: Optional[int]) -> tuple[str, int]:
    """Resolve rosbridge host/port using CLI flags, env vars, and defaults."""
    default_host = "172.16.0.2"
    default_port = 9091

    url_env = os.getenv("ROSBRIDGE_URL", "")
    if url_env:
        parsed = urlparse(url_env)
        host = parsed.hostname or default_host
        port = parsed.port or default_port
        return host, port

    host = arg_host or os.getenv("ROS_HOST", default_host)
    port_env = os.getenv("ROS_PORT", "")
    port = default_port
    if arg_port is not None:
        port = arg_port
    elif port_env:
        try:
            port = int(port_env)
        except ValueError:
            port = default_port
    return host, port


@dataclass
class RosEndpoint:
    host: str
    port: int
    via_proxy: bool


def parse_cli_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Keyboard or leader teleop for SO-ARM101 simulator"
    )
    parser.add_argument(
        "--host",
        type=str,
        default="",
        help="Rosbridge server host (uses proxy or ROSBRIDGE_URL if omitted)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=None,
        help="Rosbridge server port (defaults to ROS_PORT or 9091)",
    )
    parser.add_argument(
        "--input",
        "--input-device",
        dest="input_device",
        type=str,
        default="keyboard",
        choices=["keyboard", "leader"],
        help="Input device for teleoperation: 'keyboard' (default) or 'leader'",
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
        default="awesome_leader",
        help="Leader ID for calibration",
    )
    parser.add_argument(
        "--calibration-dir",
        type=str,
        default="~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/",
        help="Path to leader calibration directory",
    )
    parser.add_argument(
        "--calibration-file",
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
    return parser.parse_args()


def validate_cli_args(args: argparse.Namespace) -> None:
    if args.input_device == "leader" and not args.leader_port:
        print("Error: --input leader requires --leader-port.")
        sys.exit(1)


def build_ros_client(args: argparse.Namespace) -> tuple[roslibpy.Ros, RosEndpoint]:
    host, port = resolve_rosbridge_endpoint(args.host, args.port)
    display_host, display_port = host, port

    use_proxy_client = (
        connect_to_robot is not None
        and not args.host
        and not os.getenv("ROSBRIDGE_URL")
        and not os.getenv("ROS_HOST")
    )

    if use_proxy_client:
        client = connect_to_robot()
        endpoint = RosEndpoint(host="", port=0, via_proxy=True)
    else:
        print(f"Connecting to rosbridge at ws://{host}:{port}...")
        client = roslibpy.Ros(host=host, port=port)
        client.run()
        endpoint = RosEndpoint(host=display_host, port=display_port, via_proxy=False)

    if not client.is_connected:
        print("Failed to connect to rosbridge server.")
        sys.exit(1)

    print("✓ Connected to rosbridge!")
    return client, endpoint


def _ensure_leader_support() -> None:
    if not SO101Bridge:
        print("Error: lerobot or SO101Bridge not available.")
        print("Please install requirements: pip install lerobot pyserial feetech-servo-sdk")
        sys.exit(1)


def _init_leader_bridge(
    *,
    client,
    topic_class,
    port: str,
    robot_id: str,
    calibration_dir: str,
    rate_hz: float,
    debug: bool,
    debug_interval: float,
    debug_raw: bool,
    limit_map: Optional[dict] = None,
    map_to_limits: bool = False,
    clamp_to_limits: bool = False,
) -> "SO101Bridge":
    bridge = SO101Bridge(
        ros_client=client,
        robot_port=port,
        robot_id=robot_id,
        robot_type="leader",
        calibration_dir=calibration_dir,
        publish_topic="/leader_joint_states",
        publish_joint_names=["1", "2", "3", "4", "5", "6"],
        subscribe_commands=False,
        debug=debug,
        debug_interval=debug_interval,
        debug_raw=debug_raw,
        limit_map=limit_map,
        map_to_limits=map_to_limits,
        clamp_to_limits=clamp_to_limits,
        publish_to_ros=False,
    )
    if not bridge.connect():
        print("Failed to connect to leader arm.")
        sys.exit(1)

    bridge.setup_ros_topics(topic_class)
    bridge.start_publishing(rate_hz=rate_hz)
    return bridge


def _resolve_leader_calibration(
    args: argparse.Namespace,
) -> tuple[str, str, str]:
    calibration_dir = os.path.expanduser(args.calibration_dir)
    leader_id = args.leader_id

    if args.calibration_file:
        calibration_file = os.path.expanduser(args.calibration_file)
        if not os.path.isfile(calibration_file):
            print(f"Error: calibration file not found: {calibration_file}")
            sys.exit(1)
        calibration_dir = os.path.dirname(calibration_file)
        calibration_id = os.path.splitext(os.path.basename(calibration_file))[0]
        if leader_id and leader_id != calibration_id:
            print(
                "Warning: --leader-id does not match --calibration-file. "
                f"Using calibration id {calibration_id}."
            )
        leader_id = calibration_id
        return leader_id, calibration_dir, calibration_file

    calibration_file = os.path.join(calibration_dir, f"{leader_id}.json")
    if os.path.isfile(calibration_file):
        return leader_id, calibration_dir, calibration_file

    candidates = []
    if os.path.isdir(calibration_dir):
        for name in sorted(os.listdir(calibration_dir)):
            if name.endswith(".json"):
                candidates.append(os.path.join(calibration_dir, name))

    if len(candidates) == 1:
        calibration_file = candidates[0]
        leader_id = os.path.splitext(os.path.basename(calibration_file))[0]
        print(
            f"Using detected calibration file: {calibration_file} (leader id {leader_id})"
        )
        return leader_id, calibration_dir, calibration_file

    if candidates:
        print(
            "Error: calibration file not found for leader id "
            f"'{args.leader_id}'. Available files: "
            f"{', '.join(os.path.basename(path) for path in candidates)}"
        )
    else:
        print(
            f"Error: no calibration files found in {calibration_dir}. "
            "Run lerobot-calibrate or pass --calibration-file."
        )
    sys.exit(1)


class ArmKeyboardTeleop:
    def __init__(
        self,
        client,
        host: str = "",
        port: int = 0,
        input_device: str = "keyboard",
        leader_bridge: Optional["SO101Bridge"] = None,
        trajectory_time: Optional[float] = None,
        debug_leader: bool = False,
        debug_limits: bool = False,
        debug_state: bool = False,
        debug_interval: float = 1.0,
        debug_log_file: Optional[str] = None,
    ) -> None:
        """Initialize the teleop controller."""
        self.input_device = input_device
        self.leader_bridge = leader_bridge
        self.host = host
        self.port = port
        self.client = client
        self.step = 0.05
        self.gripper_step = 0.02
        if trajectory_time is None:
            trajectory_time = 0.02 if self.input_device == "leader" else 0.2
        if trajectory_time <= 0:
            trajectory_time = 0.02
        self.time_from_start_sec = int(trajectory_time)
        self.time_from_start_nsec = int(
            (trajectory_time - self.time_from_start_sec) * 1e9
        )
        self.home = list(DEFAULT_HOME)
        self.echo_keys = self.input_device == "keyboard"
        self._warned_no_state = False
        self.debug_leader = debug_leader
        self.debug_limits = debug_limits
        self.debug_state = debug_state
        self.debug_interval = max(0.1, float(debug_interval))
        self._last_limit_warn = 0.0
        self._last_state_debug = 0.0
        self._last_command_positions: dict[str, float] = {}
        self._last_command_time = None
        self._debug_fp = None
        self._debug_log_path = None
        if debug_log_file:
            log_path = os.path.expanduser(debug_log_file)
            log_dir = os.path.dirname(log_path)
            if log_dir:
                os.makedirs(log_dir, exist_ok=True)
            self._debug_fp = open(log_path, "a", encoding="utf-8")
            self._debug_log_path = log_path

        self.joint_names = ["1", "2", "3", "4", "5", "6"]
        self.arm_joint_names = self.joint_names[:5]
        self.gripper_joint_names = [self.joint_names[5]]
        self.limits = dict(URDF_LIMITS)

        self.positions = {name: 0.0 for name in self.joint_names}
        self.have_state = False
        self._shutdown = False
        self._stdin_settings = None

        # Use Topic factory for compatibility with both roslibpy.Ros and ProxyRosClient
        TopicClass = Topic if Topic else roslibpy.Topic

        self.arm_pub = TopicClass(
            self.client,
            "/arm_controller/joint_trajectory",
            "trajectory_msgs/JointTrajectory",
        )
        self.gripper_pub = TopicClass(
            self.client,
            "/gripper_controller/joint_trajectory",
            "trajectory_msgs/JointTrajectory",
        )

        # Subscriber for joint states from simulation
        self.joint_sub = TopicClass(
            self.client, "/joint_states", "sensor_msgs/JointState"
        )
        self.joint_sub.subscribe(self._joint_state_cb)

        if self.leader_bridge:
            self.leader_bridge.on_state_published = self._on_leader_state
            if self.debug_leader:
                self.leader_bridge.on_state_debug = self._on_leader_debug

        self._setup_publishers()

    def _setup_publishers(self) -> None:
        """Advertise topics before publishing."""
        print("Advertising simulator topics...")
        self.arm_pub.advertise()
        self.gripper_pub.advertise()
        print("✓ Simulator topics advertised")
        self._print_help()

    def _print_help(self) -> None:
        if self.host and self.port:
            conn_info = f"ws://{self.host}:{self.port}"
        else:
            conn_info = "via TensorFleet proxy" if connect_to_robot else "rosbridge"

        msg = [
            "",
            "═" * 44,
            "═" * 44,
            f"  Input: {self.input_device}",
            "═" * 44,
            f"  Connected to: {conn_info}",
            *( [f"  Debug log: {self._debug_log_path}"] if self._debug_log_path else [] ),
            "",
            "  q/a: joint1 +/-",
            "  w/s: joint2 +/-",
            "  e/d: joint3 +/-",
            "  r/f: joint4 +/-",
            "  t/g: joint5 +/-",
            "  y/h: gripper +/-",
            "  space: hold current",
            "  0: home",
            "  x or Ctrl-C: exit",
            "═" * 44,
            "",
        ]
        for line in msg:
            print(line)

    def _joint_state_cb(self, msg: dict) -> None:
        names = msg.get("name", [])
        positions = msg.get("position", [])

        if positions is None:
            return

        first_state = not self.have_state
        for name, pos in zip(names, positions):
            if pos is not None and name in self.positions:
                try:
                    self.positions[name] = float(pos)
                    self.have_state = True
                except (ValueError, TypeError):
                    pass

        if first_state and self.have_state and self.echo_keys:
            print(f"\n✓ Receiving joint states: {names}")
            valid_pos = [p for p in positions if p is not None]
            print(f"  Positions: {[f'{p:.3f}' for p in valid_pos]}\n")
        if self.debug_state:
            self._emit_state_debug(names, positions)

    def _log_debug(self, label: str, message: str) -> None:
        if not self._debug_fp:
            return
        timestamp = time.time()
        self._debug_fp.write(f"{timestamp:.3f} {label} {message}\n")
        self._debug_fp.flush()

    def _emit_state_debug(self, names: list, positions: list) -> None:
        if not self._last_command_positions:
            return
        now = time.monotonic()
        if now - self._last_state_debug < self.debug_interval:
            return

        state_positions: dict[str, float] = {}
        for name, pos in zip(names, positions):
            if pos is None or name not in self.joint_names:
                continue
            try:
                state_positions[name] = float(pos)
            except (ValueError, TypeError):
                continue

        if not state_positions:
            return

        target_positions = {
            name: self._last_command_positions.get(name)
            for name in self.joint_names
            if name in self._last_command_positions
        }

        errors: dict[str, float] = {}
        max_abs_error = 0.0
        for name in self.joint_names:
            if name in state_positions and name in target_positions:
                err = state_positions[name] - target_positions[name]
                errors[name] = err
                max_abs_error = max(max_abs_error, abs(err))

        cmd_age_ms = None
        if self._last_command_time is not None:
            cmd_age_ms = (now - self._last_command_time) * 1e3

        state_line = " ".join(
            f"{name}:{state_positions[name]:.3f}"
            for name in self.joint_names
            if name in state_positions
        )
        target_line = " ".join(
            f"{name}:{target_positions[name]:.3f}"
            for name in self.joint_names
            if name in target_positions
        )
        error_line = " ".join(
            f"{name}:{errors[name]:+.3f}"
            for name in self.joint_names
            if name in errors
        )

        print(f"[sim] state(rad): {state_line}")
        print(f"[sim] target(rad): {target_line}")
        if cmd_age_ms is None:
            print(f"[sim] error(rad): {error_line} max_abs={max_abs_error:.3f}")
        else:
            print(
                f"[sim] error(rad): {error_line} "
                f"max_abs={max_abs_error:.3f} cmd_age_ms={cmd_age_ms:.1f}"
            )

        log_summary = (
            f"state {state_line} | target {target_line} | error {error_line} "
            f"| max_abs {max_abs_error:.3f}"
        )
        if cmd_age_ms is not None:
            log_summary += f" | cmd_age_ms {cmd_age_ms:.1f}"
        self._log_debug("sim", log_summary)

        self._last_state_debug = now

    def _clamp(self, name: str, value: float) -> float:
        low, high = self.limits[name]
        return max(low, min(high, value))

    def _apply_home_positions(self) -> bool:
        if len(self.home) < 6:
            return False
        for idx, name in enumerate(self.joint_names):
            self.positions[name] = self._clamp(name, self.home[idx])
        self.have_state = True
        return True

    def _move_to_home(self) -> None:
        if not self._apply_home_positions():
            return
        self._publish_arm()
        self._publish_gripper()

    def _make_trajectory_msg(self, joint_names: list, positions: list) -> dict:
        msg = {
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
            "joint_names": joint_names,
            "points": [
                {
                    "positions": positions,
                    "velocities": [],
                    "accelerations": [],
                    "effort": [],
                    "time_from_start": {
                        "sec": self.time_from_start_sec,
                        "nanosec": self.time_from_start_nsec,
                    },
                }
            ],
        }
        if self.debug_state:
            for name, position in zip(joint_names, positions):
                self._last_command_positions[name] = position
            self._last_command_time = time.monotonic()
        if self.echo_keys:
            print(f"\rPublishing to joints {joint_names}: {[f'{p:.3f}' for p in positions]}   ")
        return msg

    def _publish_arm(self) -> None:
        positions = [self.positions[name] for name in self.arm_joint_names]
        msg = self._make_trajectory_msg(list(self.arm_joint_names), positions)
        self.arm_pub.publish(msg)

    def _publish_gripper(self) -> None:
        positions = [self.positions["6"]]
        msg = self._make_trajectory_msg(list(self.gripper_joint_names), positions)
        self.gripper_pub.publish(msg)

    def _apply_state_update(self, positions: list) -> None:
        if len(positions) != 6:
            return
        for i, pos in enumerate(positions):
            name = str(i + 1)
            try:
                self.positions[name] = self._clamp(name, float(pos))
            except (ValueError, TypeError):
                continue
        self.have_state = True

    def _on_leader_state(self, positions: list) -> None:
        self._apply_state_update(positions)
        if self.debug_limits:
            self._warn_limits(positions)
        self._publish_arm()
        self._publish_gripper()

    def _on_leader_debug(self, payload: dict) -> None:
        if not self.debug_leader:
            return

        joint_names = payload.get("joint_names", [])
        positions_rad = payload.get("positions_rad", []) or []
        positions_norm = payload.get("positions_norm", []) or []
        positions_raw = payload.get("positions_raw", [])

        norm_line = " ".join(
            f"{name}:{positions_norm[idx]:.1f}"
            for idx, name in enumerate(joint_names)
            if idx < len(positions_norm) and positions_norm[idx] is not None
        )
        rad_line = " ".join(
            f"{name}:{positions_rad[idx]:.3f}"
            for idx, name in enumerate(joint_names)
            if idx < len(positions_rad) and positions_rad[idx] is not None
        )

        print(f"[leader] norm: {norm_line}")
        print(f"[leader] rad:  {rad_line}")
        self._log_debug("leader", f"norm {norm_line}")
        self._log_debug("leader", f"rad {rad_line}")

        if positions_raw:
            raw_line = " ".join(
                f"{name}:{positions_raw[idx]}"
                for idx, name in enumerate(joint_names)
                if idx < len(positions_raw) and positions_raw[idx] is not None
            )
            print(f"[leader] raw:  {raw_line}")
            self._log_debug("leader", f"raw {raw_line}")

    def _warn_limits(self, positions: list) -> None:
        now = time.monotonic()
        if now - self._last_limit_warn < self.debug_interval:
            return

        warnings = []
        for idx, pos in enumerate(positions):
            name = self.joint_names[idx]
            low, high = self.limits[name]
            if pos < low or pos > high:
                warnings.append(
                    f"{name}={pos:.3f} limit[{low:.3f},{high:.3f}]"
                )

        if warnings:
            print("[limits] " + " | ".join(warnings))
            self._log_debug("limits", " | ".join(warnings))

        self._last_limit_warn = now

    def _handle_key(self, key: str) -> bool:
        if self.echo_keys:
            self._echo_key(key)

        if key == "\x03" or key.lower() == "x":
            return False

        if not self.have_state:
            if not self._warned_no_state:
                print("\nWaiting for /joint_states...")
                self._warned_no_state = True
            return True

        key = key.lower()
        mapping = {
            "q": ("1", +self.step),
            "a": ("1", -self.step),
            "w": ("2", +self.step),
            "s": ("2", -self.step),
            "e": ("3", +self.step),
            "d": ("3", -self.step),
            "r": ("4", +self.step),
            "f": ("4", -self.step),
            "t": ("5", +self.step),
            "g": ("5", -self.step),
            "y": ("6", +self.gripper_step),
            "h": ("6", -self.gripper_step),
        }

        if key == " ":
            self._publish_arm()
            self._publish_gripper()
            return True

        if key == "0":
            self._move_to_home()
            return True

        if key not in mapping:
            return True

        name, delta = mapping[key]
        self.positions[name] = self._clamp(name, self.positions[name] + delta)
        if name == "6":
            self._publish_gripper()
        else:
            self._publish_arm()
        return True

    def _echo_key(self, key: str) -> None:
        display = self._format_key(key)
        sys.stdout.write(f"\rKey: {display}   ")
        sys.stdout.flush()

    def _format_key(self, key: str) -> str:
        if key == " ":
            return "<space>"
        if key == "\x03":
            return "Ctrl-C"
        if key == "\x1b":
            return "Esc"
        if key in ("\r", "\n"):
            return "<enter>"
        if key.isprintable():
            return key
        return f"0x{ord(key):02x}"

    def run(self) -> None:
        if not self.client.is_connected:
            print("Error: Client is not connected.")
            return

        try:
            if self.input_device == "leader":
                self._run_leader_loop()
            else:
                self._run_keyboard_loop()
        finally:
            self.shutdown()

    def _run_leader_loop(self) -> None:
        if not self.leader_bridge or not self.leader_bridge.is_connected:
            print("Error: leader input selected but leader bridge is not connected.")
            return

        print("Starting leader arm teleop loop...")
        try:
            while not self._shutdown and self.client.is_connected:
                if not self.leader_bridge.is_connected:
                    print("\nLeader bridge disconnected.")
                    break
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass

    def _run_keyboard_loop(self) -> None:
        self._move_to_home()

        if sys.stdin.isatty():
            self._stdin_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin)

        try:
            while not self._shutdown and self.client.is_connected:
                try:
                    key = sys.stdin.read(1)
                    if key and not self._handle_key(key):
                        break
                except IOError:
                    break
        except KeyboardInterrupt:
            pass

    def shutdown(self) -> None:
        self._shutdown = True
        print("\nShutting down...")

        if self._stdin_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._stdin_settings)

        try:
            if self.joint_sub:
                self.joint_sub.unsubscribe()
            if self.arm_pub:
                self.arm_pub.unadvertise()
            if self.gripper_pub:
                self.gripper_pub.unadvertise()
        except Exception:
            pass
        finally:
            if self._debug_fp:
                try:
                    self._debug_fp.close()
                except Exception:
                    pass


def main() -> None:
    args = parse_cli_args()
    validate_cli_args(args)
    if args.debug_all:
        args.debug_leader = True
        args.debug_limits = True
        args.debug_state = True
        args.debug_raw = True

    client = None
    leader_bridge = None
    try:
        client, endpoint = build_ros_client(args)
        topic_class = Topic if Topic else roslibpy.Topic

        if args.input_device == "leader":
            _ensure_leader_support()
            leader_id, calibration_dir, calibration_file = _resolve_leader_calibration(
                args
            )
            print(f"Initializing leader bridge on {args.leader_port}...")
            print(f"Using calibration file: {calibration_file}")
            leader_bridge = _init_leader_bridge(
                client=client,
                topic_class=topic_class,
                port=args.leader_port,
                robot_id=leader_id,
                calibration_dir=calibration_dir,
                rate_hz=args.leader_rate,
                debug=args.debug_leader,
                debug_interval=args.debug_interval,
                debug_raw=args.debug_raw,
                limit_map=URDF_LIMITS,
                map_to_limits=True,
                clamp_to_limits=True,
            )

        teleop = ArmKeyboardTeleop(
            client=client,
            host=endpoint.host,
            port=endpoint.port,
            input_device=args.input_device,
            leader_bridge=leader_bridge,
            trajectory_time=args.trajectory_time,
            debug_leader=args.debug_leader,
            debug_limits=args.debug_limits,
            debug_state=args.debug_state,
            debug_interval=args.debug_interval,
            debug_log_file=args.debug_log_file,
        )
        teleop.run()

    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
    finally:
        if leader_bridge:
            try:
                leader_bridge.shutdown()
            except Exception:
                pass
        if client:
            try:
                client.terminate()
            except Exception:
                pass
            print("Connection closed.")


if __name__ == "__main__":
    main()
