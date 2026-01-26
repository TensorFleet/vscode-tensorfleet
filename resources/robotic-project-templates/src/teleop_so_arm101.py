#!/usr/bin/env python3
"""
Keyboard teleop for SO-ARM101 using rosbridge websocket.
This script runs on the HOST and connects to the VM's rosbridge server.

Supports:
- Simulation mode: Controls simulated arm via ros2_control trajectory topics
- Real mode: Controls physical SO101 arm via lerobot SDK, publishes state to VM

Usage presets:
    # Simulation via keyboard (default)
    python3 teleop_so_arm101.py --mode sim --input keyboard [--host VM_IP]

    # Simulation driven by real leader arm
    python3 teleop_so_arm101.py --mode sim --input leader --leader-port /dev/ttyACM0 [--host VM_IP]

    # Real follower + simulator mirror, keyboard input
    python3 teleop_so_arm101.py --mode real --input keyboard --mirror-sim --follower-port /dev/ttyACM0 [--host VM_IP]

    # Real follower + simulator mirror, leader input
    python3 teleop_so_arm101.py --mode real --input leader --leader-port /dev/ttyACM1 --follower-port /dev/ttyACM0 --mirror-sim [--host VM_IP]

    # State-only digital twin of the real follower
    python3 teleop_so_arm101.py --mode real --state-only --follower-port /dev/ttyACM0

Requirements:
    pip install roslibpy websocket-client
    # For real mode:
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
from typing import Optional
from urllib.parse import urlparse

# Add parent directory to path for lib imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

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

# SO101 hardware bridge (optional, for real mode)
try:
    from lib.so101_bridge import SO101Bridge, is_hardware_available
except ImportError:
    try:
        from so101_bridge import SO101Bridge, is_hardware_available
    except ImportError:
        SO101Bridge = None
        is_hardware_available = lambda: False


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


class ArmKeyboardTeleop:
    def __init__(
        self,
        client,
        host: str = "",
        port: int = 0,
        mode: str = "sim",
        input_device: str = "keyboard",
        mirror_to_sim: bool = False,
        follower_bridge: Optional["SO101Bridge"] = None,
        leader_bridge: Optional["SO101Bridge"] = None,
    ) -> None:
        """Initialize the teleop controller.
        
        Args:
            client: Connected ROS client (roslibpy.Ros or ProxyRosClient)
            host: Display host for user messages (optional)
            port: Display port for user messages (optional)
            mode: Operating mode - 'sim' for simulation, 'real' for hardware
            input_device: 'keyboard' or 'leader' for how to capture input
            mirror_to_sim: If True, also publish commands to the simulator when in real mode
            follower_bridge: SO101Bridge instance for the actuated follower (real mode)
            leader_bridge: SO101Bridge instance for leader-input hardware
        """
        self.mode = mode
        self.input_device = input_device
        self.follower_bridge = follower_bridge
        self.leader_bridge = leader_bridge
        self.host = host
        self.port = port
        self.client = client
        self.mirror_to_sim = mirror_to_sim
        self.publish_to_sim = self.mode == "sim" or self.mirror_to_sim
        self.step = 0.05
        self.gripper_step = 0.02
        self.time_from_start_sec = 0
        self.time_from_start_nsec = int(0.2 * 1e9)
        self.home = [0.0, 0.3, -0.5, 0.0, 0.0, 0.4]
        self.echo_keys = True

        self.joint_names = ["1", "2", "3", "4", "5", "6"]
        self.arm_joint_names = self.joint_names[:5]
        self.gripper_joint_names = [self.joint_names[5]]
        self.limits = {
            "1": (-1.91986, 1.91986),
            "2": (-1.74533, 1.74533),
            "3": (-1.74533, 1.5708),
            "4": (-1.65806, 1.65806),
            "5": (-2.79253, 2.79253),
            "6": (-0.174533, 1.74533),
        }

        self.positions = {name: 0.0 for name in self.joint_names}
        self.have_state = False
        self._shutdown = False
        self._stdin_settings = None

        # Use Topic factory for compatibility with both roslibpy.Ros and ProxyRosClient
        TopicClass = Topic if Topic else roslibpy.Topic

        if self.publish_to_sim:
            # Simulation outputs (used in sim mode and when mirroring hardware to the VM)
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
        else:
            self.arm_pub = None
            self.gripper_pub = None

        if self.mode == "sim":
            # Subscriber for joint states from simulation
            self.joint_sub = TopicClass(
                self.client, "/joint_states", "sensor_msgs/JointState"
            )
            self.joint_sub.subscribe(self._joint_state_cb)
        else:
            self.joint_sub = None
            
            # In real mode, we may have state from the follower bridge
            if self.follower_bridge and self.follower_bridge.is_connected:
                self._sync_positions_from_bridge(self.follower_bridge, source="follower")

        # Keep keyboard/leader view in sync with hardware feedback
        if self.follower_bridge:
            self.follower_bridge.on_state_published = self._on_follower_state
        if self.leader_bridge:
            self.leader_bridge.on_state_published = self._on_leader_state

        # Advertise publishers
        self._setup_publishers()

    def _setup_publishers(self) -> None:
        """Advertise topics before publishing."""
        if self.publish_to_sim:
            print("Advertising simulator topics...")
            if self.arm_pub:
                self.arm_pub.advertise()
            if self.gripper_pub:
                self.gripper_pub.advertise()
            print("✓ Simulator topics advertised")
        else:
            print("Simulator publishing disabled (hardware-only control)")
        self._print_help()

    def _print_help(self) -> None:
        # Determine connection info display
        if self.host and self.port:
            conn_info = f"ws://{self.host}:{self.port}"
        else:
            conn_info = "via TensorFleet proxy" if connect_to_robot else "rosbridge"

        mode_line = f"  Teleop Mode: {self.mode} | Input: {self.input_device}"
        if self.mode == "real" and self.mirror_to_sim:
            mode_line += " | mirroring to simulator"

        msg = [
            "",
            "═" * 44,
            "═" * 44,
            mode_line,
            "═" * 44,
            f"  Connected to: {conn_info}",
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
                    
        if first_state and self.have_state:
            print(f"\n✓ Receiving joint states: {names}")
            # Filter None for printing
            valid_pos = [p for p in positions if p is not None]
            print(f"  Positions: {[f'{p:.3f}' for p in valid_pos]}\n")

    def _clamp(self, name: str, value: float) -> float:
        low, high = self.limits[name]
        return max(low, min(high, value))

    def _make_trajectory_msg(self, joint_names: list, positions: list) -> dict:
        # ROS 2 JointTrajectory message with header
        # The header helps ros2_control identify the message timing
        msg = {
            "header": {
                "stamp": {"sec": 0, "nanosec": 0},  # Use sim time
                "frame_id": ""
            },
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
        if self.echo_keys:
            print(f"\rPublishing to joints {joint_names}: {[f'{p:.3f}' for p in positions]}   ")
        return msg

    def _publish_arm(self) -> None:
        if self.publish_to_sim and self.arm_pub:
            positions = [self.positions[name] for name in self.arm_joint_names]
            msg = self._make_trajectory_msg(list(self.arm_joint_names), positions)
            self.arm_pub.publish(msg)
        if self.mode == "real" and self.follower_bridge:
            # Real mode: bridge sends commands directly to hardware
            self._send_command_to_follower()

    def _publish_gripper(self) -> None:
        if self.publish_to_sim and self.gripper_pub:
            positions = [self.positions["6"]]
            msg = self._make_trajectory_msg(list(self.gripper_joint_names), positions)
            self.gripper_pub.publish(msg)
        if self.mode == "real" and self.follower_bridge:
            # Real mode: bridge handles all joints together
            self._send_command_to_follower()

    def _send_command_to_follower(self) -> None:
        """Send current positions to real robot via bridge."""
        if not self.follower_bridge or not self.follower_bridge.is_connected:
            return

        # SO101Bridge expects list of 6 values in order (rad)
        # Map our "1".."6" to the correct list order
        # Our internal mapping: 1->shoulder_pan, 2->shoulder_lift, etc.
        # matches bridge order
        target_pos = [self.positions[str(i+1)] for i in range(6)]
        
        self.follower_bridge.send_command(target_pos)
        
        if self.echo_keys:
            print(f"\rSent to robot: {[f'{p:.3f}' for p in target_pos]}   ")

    def _sync_positions_from_bridge(self, bridge: "SO101Bridge", source: str = "") -> bool:
        """Update internal positions from a hardware bridge."""
        if not bridge or not bridge.is_connected:
            return False

        current = bridge.get_positions()
        if len(current) != 6:
            return False

        for i in range(6):
            self.positions[str(i + 1)] = current[i]
        self.have_state = True

        if source and self.echo_keys:
            print(f"\rSynced from {source} hardware: {[f'{p:.2f}' for p in current]}   ", end="", flush=True)
        return True

    def _on_follower_state(self, positions: list) -> None:
        """Keep internal state aligned with follower feedback (keyboard path)."""
        if self.input_device == "leader":
            # Leader input should remain the source of commands
            return
        self._apply_state_update(positions)

    def _on_leader_state(self, positions: list) -> None:
        """Update state from leader input."""
        self._apply_state_update(positions)

    def _apply_state_update(self, positions: list) -> None:
        if len(positions) != 6:
            return
        for i, pos in enumerate(positions):
            name = str(i + 1)
            try:
                self.positions[name] = float(pos)
            except (ValueError, TypeError):
                continue
        self.have_state = True

    def _handle_key(self, key: str) -> bool:
        """Handle key press. Returns False if should exit."""
        if self.echo_keys:
            self._echo_key(key)

        if key == "\x03" or key.lower() == "x":
            return False

        if not self.have_state:
            # In real mode, we might get state immediately from bridge
            if self.mode == "real" and self.follower_bridge:
                self._sync_positions_from_bridge(self.follower_bridge, source="follower")
            
            if not self.have_state:
                print("\rWaiting for state...   ", end="", flush=True)
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
            if len(self.home) >= 6:
                for idx, name in enumerate(self.joint_names):
                    self.positions[name] = self._clamp(name, self.home[idx])
                self._publish_arm()
                self._publish_gripper()
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
        """Main loop - process keyboard input (expects already connected client)."""
        if not self.client.is_connected:
            print("Error: Client is not connected.")
            return

        try:
            if self.input_device == "leader":
                # Leader control loop
                if self.mode == "real" and not self.follower_bridge:
                    print("Error: real mode with leader input requires a follower bridge.")
                    return
                if not self.leader_bridge or not self.leader_bridge.is_connected:
                    print("Error: leader input selected but leader bridge is not connected.")
                    return

                print("Starting leader arm teleop loop...")
                try:
                    while not self._shutdown and self.client.is_connected:
                        if self.leader_bridge and self.leader_bridge.is_connected:
                            self._sync_positions_from_bridge(self.leader_bridge, source="leader")
                            # Publish to simulator (and hardware if applicable)
                            if self.publish_to_sim or self.mode == "real":
                                self._publish_arm()
                                self._publish_gripper()
                        else:
                            print("\nLeader bridge disconnected.")
                            break
                        time.sleep(0.05) # 20Hz
                except KeyboardInterrupt:
                    pass
            else:
                # Keyboard control loop
                # Set terminal to raw mode for key capture
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
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        self._shutdown = True
        print("\nShutting down...")

        # Restore terminal
        if self._stdin_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._stdin_settings)

        # Cleanup topics (connection is closed by main)
        try:
            if self.joint_sub:
                self.joint_sub.unsubscribe()
            if self.arm_pub:
                self.arm_pub.unadvertise()
            if self.gripper_pub:
                self.gripper_pub.unadvertise()
            if self.follower_bridge:
                self.follower_bridge.shutdown()
            if self.leader_bridge:
                self.leader_bridge.shutdown()
        except Exception:
            pass


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Keyboard teleop for SO-ARM101 via rosbridge"
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
        "--mode",
        type=str,
        default="sim",
        choices=["sim", "real"],
        help="Where to send commands: 'sim' (default) or 'real' follower",
    )
    parser.add_argument(
        "--calibration-dir",
        type=str,
        default="/home/shane/.config/lerobot/",
        help="Path to calibration directory (only for --mode real)",
    )
    parser.add_argument(
        "--follower-port",
        "--robot-port",
        dest="follower_port",
        type=str,
        default="/dev/ttyACM0",
        help="Robot USB serial port for follower (only for --mode real)",
    )
    parser.add_argument(
        "--follower-id",
        "--robot-id",
        dest="follower_id",
        type=str,
        default="awesome_follower",
        help="Follower ID for calibration (only for --mode real)",
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
        "--input",
        "--input-device",
        dest="input_device",
        type=str,
        default="keyboard",
        choices=["keyboard", "leader"],
        help="Input device for teleoperation: 'keyboard' (default) or 'leader'",
    )
    parser.add_argument(
        "--mirror-sim",
        "--mirror-vm",
        dest="mirror_sim",
        action="store_true",
        help="Mirror hardware control to the VM simulator (publish commands + joint states)",
    )
    parser.add_argument(
        "--state-only",
        "--digital-twin",
        dest="state_only",
        action="store_true",
        help="State-only digital twin: publish real arm joint states to VM",
    )
    args = parser.parse_args()

    client = None
    follower_bridge = None
    leader_bridge = None
    host, port = resolve_rosbridge_endpoint(args.host, args.port)
    display_host, display_port = host, port
    mirror_sim = bool(args.mirror_sim)
    state_only = bool(args.state_only)

    if state_only and args.mode != "real":
        print("Error: --state-only requires --mode real.")
        sys.exit(1)
    if args.input_device == "leader" and not args.leader_port:
        print("Error: --input leader requires --leader-port.")
        sys.exit(1)
    if args.mode == "real" and not args.follower_port:
        print("Error: --mode real requires --follower-port.")
        sys.exit(1)
    if args.mode == "sim" and mirror_sim:
        print("Note: --mirror-sim is ignored in simulation-only mode.")
        mirror_sim = False

    try:
        # Try to use connect_to_robot (supports proxy and direct connections)
        use_proxy_client = (
            connect_to_robot is not None
            and not args.host
            and not os.getenv("ROSBRIDGE_URL")
            and not os.getenv("ROS_HOST")
        )

        if use_proxy_client:
            client = connect_to_robot()
            display_host, display_port = "", 0
        else:
            # Fallback to direct roslibpy connection
            print(f"Connecting to rosbridge at ws://{host}:{port}...")
            client = roslibpy.Ros(host=host, port=port)
            client.run()

        if not client.is_connected:
            print("Failed to connect to rosbridge server.")
            sys.exit(1)

        print("✓ Connected to rosbridge!")

        needs_hardware = (
            args.mode == "real"
            or args.input_device == "leader"
            or state_only
            or mirror_sim
        )
        if needs_hardware and not SO101Bridge:
            print("Error: lerobot or SO101Bridge not available.")
            print("Please install requirements: pip install lerobot pyserial feetech-servo-sdk")
            sys.exit(1)

        TopicClass = Topic if Topic else roslibpy.Topic

        if args.input_device == "leader":
            print(f"Initializing leader bridge on {args.leader_port}...")
            leader_bridge = SO101Bridge(
                ros_client=client,
                robot_port=args.leader_port,
                robot_id=args.leader_id,
                robot_type="leader",
                calibration_dir=args.calibration_dir,
                publish_topic="/leader_joint_states",
                publish_joint_names=["1", "2", "3", "4", "5", "6"],
                subscribe_commands=False,
            )
            if not leader_bridge.connect():
                print("Failed to connect to leader arm.")
                sys.exit(1)
            leader_bridge.setup_ros_topics(TopicClass)
            leader_bridge.start_publishing(rate_hz=20)

        if args.mode == "real":
            publish_topic = "/joint_states" if (state_only or mirror_sim) else "/joint_states_raw"
            publish_joint_names = ["1", "2", "3", "4", "5", "6"]
            subscribe_commands = False  # Teleop owns command stream

            print(f"Initializing follower bridge on {args.follower_port}...")
            follower_bridge = SO101Bridge(
                ros_client=client,
                robot_port=args.follower_port,
                robot_id=args.follower_id,
                robot_type="follower",
                calibration_dir=args.calibration_dir,
                publish_topic=publish_topic,
                publish_joint_names=publish_joint_names,
                subscribe_commands=subscribe_commands,
            )

            if not follower_bridge.connect():
                print("Failed to connect to follower robot.")
                sys.exit(1)

            follower_bridge.setup_ros_topics(TopicClass)
            follower_bridge.start_publishing(rate_hz=20)

        if state_only:
            print("Digital twin mode: publishing /joint_states to VM. Press Ctrl-C to exit.")
            try:
                while client.is_connected:
                    time.sleep(1.0)
            except KeyboardInterrupt:
                pass
            if follower_bridge:
                follower_bridge.shutdown()
            if leader_bridge:
                leader_bridge.shutdown()
            return

        teleop = ArmKeyboardTeleop(
            client=client,
            host=display_host,
            port=display_port,
            mode=args.mode,
            input_device=args.input_device,
            mirror_to_sim=mirror_sim,
            follower_bridge=follower_bridge,
            leader_bridge=leader_bridge,
        )
        teleop.run()

    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
    finally:
        if client:
            client.terminate()
            print("Connection closed.")


if __name__ == "__main__":
    main()
