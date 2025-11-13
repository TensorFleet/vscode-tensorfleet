# Getting Started with Your TensorFleet Robotic Project

Welcome to your new robotic project! This template is optimized for robots that consume `cmd_vel_raw` and for setups with a VM-hosted ROS 2 + bridge stack.

## ✅ Prerequisites

- ROS 2 (Humble or newer)
- Python 3.8+
- A VM at `172.16.0.10` with:
  - rosbridge running on port `8080`
  - Foxglove bridge running on port `8765`
- TensorFleet VS Code extension

## 🚀 Quick Start

1. **Install dependencies (with uv)**

```bash
# If uv isn't installed: curl -LsSf https://astral.sh/uv/install.sh | sh
uv venv
uv pip install -r requirements.txt
```

2. **Check / update config**

Open `config/robot_config.yaml` and confirm:

```yaml
network:
  vm_ip: "172.16.0.10"
  rosbridge_url: "ws://172.16.0.10:8080"
  foxglove_bridge_url: "ws://172.16.0.10:8765"

motion:
  cmd_vel_topic: "/cmd_vel_raw"
```

3. **Run the basic motion script**

```bash
uv run python src/robot_mover.py
```

The script will:

- Initialize a ROS 2 node
- Publish to `cmd_vel_raw` using `geometry_msgs/msg/Twist`
- Execute a short sequence: move forward, rotate, then stop

## 📁 Project Layout

```
.
├── src/
│   └── robot_mover.py        # Example velocity publisher
├── config/
│   └── robot_config.yaml     # Robot + network configuration
├── launch/
│   └── (placeholder for your launch files)
├── requirements.txt
└── GETTING_STARTED.md
```

## 🧩 Customization Ideas

- Change speed or durations in `robot_mover.py`.
- Add keyboard teleop or state machines on top of `cmd_vel_raw`.
- Extend `robot_config.yaml` with your robot’s frames, wheelbase, etc.

## 🔍 Debugging Tips

- Verify topics:

```bash
ros2 topic list
ros2 topic echo /cmd_vel_raw
```

- Check connectivity from the VM to any Foxglove / visualization tools using the URLs in `robot_config.yaml`.

---

You now have a clean, robot-specific starting point that doesn’t pull in drone-specific files. Build on it however you like. 🤖
