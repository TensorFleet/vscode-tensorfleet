#!/usr/bin/env python3
"""
Keyboard teleop for SO-ARM101 using rosbridge websocket.
This script runs on the HOST and connects to the VM's rosbridge server.

Supports:
- Simulation mode: Controls simulated arm via ros2_control trajectory topics
- Real mode: Controls physical SO101 arm via lerobot SDK, publishes state to VM

Usage:
    # Simulation mode (default)
    python3 teleop_so_arm101.py [--host VM_IP] [--port 9091]

    # Real arm mode
    python3 teleop_so_arm101.py --mode real --robot-port /dev/ttyACM0 [--host VM_IP]

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
import tty
from typing import Optional

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


class ArmKeyboardTeleop:
    def __init__(
        self,
        client,
        host: str = "",
        port: int = 0,
        mode: str = "sim",
        input_device: str = "keyboard",
        so101_bridge: Optional["SO101Bridge"] = None,
    ) -> None:
        """Initialize the teleop controller.
        
        Args:
            client: Connected ROS client (roslibpy.Ros or ProxyRosClient)
            host: Display host for user messages (optional)
            port: Display port for user messages (optional)
            mode: Operating mode - 'sim' for simulation, 'real' for hardware
            input_device: 'keyboard' or 'leader'
            so101_bridge: SO101Bridge instance for real mode (required if mode='real')
        """
        self.mode = mode
        self.input_device = input_device
        self.so101_bridge = so101_bridge
        self.host = host
        self.port = port
        self.client = client
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

        if self.mode == "sim":
            # Simulation mode: use ros2_control trajectory topics
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
        else:
            # Real mode: use SO101Bridge for hardware, no trajectory publishers
            self.arm_pub = None
            self.gripper_pub = None
            self.joint_sub = None
            
            # In real mode, we already have state from the bridge
            if self.so101_bridge and self.so101_bridge.is_connected:
                self._sync_positions_from_hardware()

        # Advertise publishers
        self._setup_publishers()

    def _setup_publishers(self) -> None:
        """Advertise topics before publishing."""
        print("Advertising topics...")
        if self.arm_pub:
            self.arm_pub.advertise()
        if self.gripper_pub:
            self.gripper_pub.advertise()
        print("✓ Topics advertised")
        self._print_help()

    def _print_help(self) -> None:
        # Determine connection info display
        if self.host and self.port:
            conn_info = f"ws://{self.host}:{self.port}"
        else:
            conn_info = "via TensorFleet proxy" if connect_to_robot else "rosbridge"

        msg = [
            "",
            "═" * 44,
            "═" * 44,
            f"  Teleop Mode: {self.mode} | Input: {self.input_device}",
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
        if self.mode == "sim":
            positions = [self.positions[name] for name in self.arm_joint_names]
            msg = self._make_trajectory_msg(list(self.arm_joint_names), positions)
            self.arm_pub.publish(msg)
        else:
            # Real mode: bridge sends commands directly to hardware
            self._send_command_to_bridge()

    def _publish_gripper(self) -> None:
        if self.mode == "sim":
            positions = [self.positions["6"]]
            msg = self._make_trajectory_msg(list(self.gripper_joint_names), positions)
            self.gripper_pub.publish(msg)
        else:
            # Real mode: bridge handles all joints together
            self._send_command_to_bridge()

    def _send_command_to_bridge(self) -> None:
        """Send current positions to real robot via bridge."""
        if not self.so101_bridge:
            return

        # Leader arm is read-only (input device)
        if self.so101_bridge.robot_type == "leader":
            if self.echo_keys:
                print(f"\rLeader arm is read-only. Ignoring command.   ", end="", flush=True)
            return

        # SO101Bridge expects list of 6 values in order (rad)
        # Map our "1".."6" to the correct list order
        # Our internal mapping: 1->shoulder_pan, 2->shoulder_lift, etc.
        # matches bridge order
        target_pos = [self.positions[str(i+1)] for i in range(6)]
        
        self.so101_bridge.send_command(target_pos)
        
        if self.echo_keys:
            print(f"\rSent to robot: {[f'{p:.3f}' for p in target_pos]}   ")

    def _sync_positions_from_hardware(self) -> None:
        """Update internal positions from current robot state."""
        if not self.so101_bridge:
            return
            
        current = self.so101_bridge.get_positions()
        if len(current) == 6:
            for i in range(6):
                self.positions[str(i+1)] = current[i]
            self.have_state = True
            print(f"Synced with hardware: {[f'{p:.2f}' for p in current]}")

    def _handle_key(self, key: str) -> bool:
        """Handle key press. Returns False if should exit."""
        if self.echo_keys:
            self._echo_key(key)

        if key == "\x03" or key.lower() == "x":
            return False

        if not self.have_state:
            # In real mode, we might get state immediately from bridge
            if self.mode == "real" and self.so101_bridge:
                self._sync_positions_from_hardware()
            
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
                import time
                print("Starting leader arm teleop loop...")
                try:
                    while not self._shutdown and self.client.is_connected:
                        if self.so101_bridge and self.so101_bridge.is_connected:
                            self._sync_positions_from_hardware()
                            # In Sim mode, publish to trajectory controller
                            if self.mode == "sim":
                                 self._publish_arm()
                                 self._publish_gripper()
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
            if self.so101_bridge:
                self.so101_bridge.shutdown()
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
        help="Rosbridge server host (uses proxy if not specified)",
    )
    parser.add_argument(
        "--port",
        type=int,
        help="Rosbridge server port (default: 9091)",
    )
    parser.add_argument(
        "--mode",
        type=str,
        default="sim",
        choices=["sim", "real"],
        help="Operating mode: 'sim' (default) or 'real'",
    )
    parser.add_argument(
        "--robot-port",
        type=str,
        default="/dev/ttyACM0",
        help="Robot USB serial port (only for --mode real)",
    )
    parser.add_argument(
        "--robot-id",
        type=str,
        default="awesome_follower",
        help="Robot ID for calibration (only for --mode real)",
    )
    parser.add_argument(
        "--calibration-dir",
        type=str,
        default="/home/shane/.config/lerobot/",
        help="Path to calibration directory (only for --mode real)",
    )
    parser.add_argument(
        "--robot-type",
        type=str,
        default="follower",
        choices=["follower", "leader"],
        help="Robot type: 'follower' (actuated) or 'leader' (passive inputs) (only for --mode real)",
    )

    parser.add_argument(
        "--input-device",
        type=str,
        default="keyboard",
        choices=["keyboard", "leader"],
        help="Input device for teleoperation: 'keyboard' (default) or 'leader'",
    )

    args = parser.parse_args()

    client = None
    host = args.host
    port = args.port
    mode = args.mode
    input_device = args.input_device
    so101_bridge = None

    try:
        # Try to use connect_to_robot (supports proxy and direct connections)
        if connect_to_robot and not args.host:
            client = connect_to_robot()
        else:
            # Fallback to direct roslibpy connection
            if not args.host:
                host = os.getenv("ROS_HOST", "172.16.0.2")
            port = int(os.getenv("ROS_PORT", str(args.port)))
            print(f"Connecting to rosbridge at ws://{host}:{port}...")
            client = roslibpy.Ros(host=host, port=port)
            client.run()

        if not client.is_connected:
            print("Failed to connect to rosbridge server.")
            sys.exit(1)

        print("✓ Connected to rosbridge!")

        # Initialize bridge if in Real mode OR if using Leader input
        if mode == "real" or input_device == "leader":
            if not SO101Bridge:
                print("Error: lerobot or SO101Bridge not available.")
                print("Please install requirements: pip install lerobot pyserial feetech-servo-sdk")
                sys.exit(1)
                
            print(f"Initializing SO101Bridge on {args.robot_port}...")
            # Use ProxyTopic if available to wrap ros_client
            TopicClass = Topic if Topic else roslibpy.Topic
            
            so101_bridge = SO101Bridge(
                ros_client=client,
                robot_port=args.robot_port,
                robot_id=args.robot_id,
                # Force leader type if input device is leader
                robot_type="leader" if input_device == "leader" else args.robot_type,
                calibration_dir=args.calibration_dir,
            )
            
            if not so101_bridge.connect():
                print("Failed to connect to real robot.")
                sys.exit(1)
                
            # Setup ROS topics for state publishing
            so101_bridge.setup_ros_topics(TopicClass)
            
            # Start background publishing thread
            so101_bridge.start_publishing(rate_hz=20)

        teleop = ArmKeyboardTeleop(
            client=client,
            host=host,
            port=port,
            mode=mode,
            input_device=input_device,
            so101_bridge=so101_bridge
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
